/*
  ESP32-S3-EYE Whiteboard - Fast (Option B: Snapshot aus Puffer)
  - TinyUSB HID Touch (Android-kompatibel, Single-Finger)
  - IR-Erkennung (GRAYSCALE) mit Weighted-Centroid (Sub-Pixel), Median+EMA
  - Auto-Calibration (4 Points, Fullscreen Crosshair) mit 2s Halte-Timer
  - Calibration Preview: Camera + Board/Homography
  - Snapshot Camera Preview (Pull-based) aus JPEG-Puffer (kein 2. Kamerazugriff)
  - Click Test (Press/Release Time) -> Touch down/up
  - NVS persistent storage (calibration + vision; Legacy-kompatibel)
  - HID-Output per WebGUI schaltbar (ohne Disconnect)
  - Performance:
      * Homography auf float statt double
      * Multicore: Camera/Detection auf Core 1, HID/Server auf Core 0
      * AsyncJsonResponse für APIs
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "esp_camera.h"
extern "C" {
  #include "img_converters.h"   // frame2jpg
}

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>

#include <Preferences.h>

#include <USB.h>
#include <USBHID.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* -------------------- Kamera Pins (ESP32-S3-EYE) -------------------- */
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM     9
#define Y4_GPIO_NUM     8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM  13

/* -------------------- WLAN -------------------- */
const char* STA_SSID = "**********";
const char* STA_PASS = "**********";
const char* AP_SSID  = "S3Eye_Whiteboard";
const char* AP_PASS  = "12345678";

bool camOk = false;
AsyncWebServer server(80);

/* -------------------- Datentypen & Globals -------------------- */
struct IPoint { int x; int y; };

IPoint g_cam_pts[4]   = {{0,0},{0,0},{0,0},{0,0}};
IPoint g_board_pts[4] = {{0,0},{0,0},{0,0},{0,0}};
bool calibrated=false;

volatile bool g_calibrating=false;

volatile int latest_cam_x=-1;
volatile int latest_cam_y=-1;
volatile bool latest_has=false;
volatile int latest_cam_w=-1;
volatile int latest_cam_h=-1;

uint32_t display_w=1920;
uint32_t display_h=1080;

unsigned long last_touch_time=0;
bool touch_down=false;

/* HID-Output-Schalter (nur Senden von IR->HID) */
volatile bool g_hid_enabled = true;

/* ---- Click-Test Sequencer ---- */
volatile bool g_hid_right_active=false;
uint16_t g_right_x=0, g_right_y=0;
uint8_t  g_right_state=0;
unsigned long g_right_ts=0;

int g_right_press_ms   = 150;
int g_right_release_ms = 200;

/* -------------------- Homography (float) -------------------- */
class Homography {
  float m[9];
  static bool solve8x8(float A[8][9], float out[8]) {
    for(int i=0;i<8;i++){
      int piv=i;
      for(int r=i+1;r<8;r++)
        if (fabsf(A[r][i])>fabsf(A[piv][i])) piv=r;
      if (fabsf(A[piv][i])<1e-9f) return false;
      if (piv!=i) for(int c=i;c<=8;c++){ float t=A[i][c]; A[i][c]=A[piv][c]; A[piv][c]=t; }
      float div=A[i][i];
      for(int c=i;c<=8;c++) A[i][c]/=div;
      for(int r=0;r<8;r++){
        if (r==i) continue;
        float f=A[r][i];
        for(int c=i;c<=8;c++) A[r][c]-=f*A[i][c];
      }
    }
    for(int i=0;i<8;i++) out[i]=A[i][8];
    return true;
  }
public:
  Homography(){ setIdentity(); }
  void setIdentity(){ float I[9]={1.f,0,0, 0,1.f,0, 0,0,1.f}; memcpy(m,I,sizeof(I)); }
  void setMatrix(const float in[9]){ memcpy(m,in,9*sizeof(float)); }
  void getMatrix(float out[9]) const { memcpy(out,m,9*sizeof(float)); }
  bool compute(const float src[8], const float dst[8]){
    float A[8][9]={0};
    for(int i=0;i<4;i++){
      float x=src[2*i], y=src[2*i+1];
      float u=dst[2*i], v=dst[2*i+1];
      A[2*i][0]=x; A[2*i][1]=y; A[2*i][2]=1.f;
      A[2*i][6]=-u*x; A[2*i][7]=-u*y; A[2*i][8]=u;
      A[2*i+1][3]=x; A[2*i+1][4]=y; A[2*i+1][5]=1.f;
      A[2*i+1][6]=-v*x; A[2*i+1][7]=-v*y; A[2*i+1][8]=v;
    }
    float sol[8];
    if(!solve8x8(A, sol)) return false;
    m[0]=sol[0]; m[1]=sol[1]; m[2]=sol[2];
    m[3]=sol[3]; m[4]=sol[4]; m[5]=sol[5];
    m[6]=sol[6]; m[7]=sol[7]; m[8]=1.f;
    return true;
  }
  inline void apply(float x,float y,float &ox,float &oy) const {
    float den = m[6]*x + m[7]*y + m[8];
    if (fabsf(den)<1e-9f) { ox=oy=0; return; }
    ox = (m[0]*x + m[1]*y + m[2]) / den;
    oy = (m[3]*x + m[4]*y + m[5]) / den;
  }
};
Homography H;

/* -------------------- HID Touch (Android-kompatibel, Single-Finger) -------------------- */
USBHID HID;

typedef struct __attribute__((packed)){
  uint8_t  buttons;     // bit0 TipSwitch, bit1 InRange
  uint8_t  contactId;   // 0..127
  uint16_t x;           // 0..32767
  uint16_t y;           // 0..32767
  uint8_t  contactCount;// Anzahl aktiver Kontakte
} touch_report_t;

class HIDTouchDevice : public USBHIDDevice {
public:
  HIDTouchDevice(){ HID.addDevice(this, sizeof(reportMap)); }
  void begin(){ HID.begin(); }

  bool touchMove(uint16_t x, uint16_t y){
    if(x>32767) x=32767; if(y>32767) y=32767;
    touch_report_t r{};
    r.buttons      = 0b00000011; // Tip=1, InRange=1
    r.contactId    = 0;
    r.x            = x;
    r.y            = y;
    r.contactCount = 1;
    return HID.SendReport(1, &r, sizeof(r));
  }
  bool touchDown(uint16_t x, uint16_t y){ return touchMove(x,y); }
  bool touchUp(){
    touch_report_t r{};
    r.buttons      = 0; // Tip=0, InRange=0
    r.contactId    = 0;
    r.x            = 0;
    r.y            = 0;
    r.contactCount = 0;
    return HID.SendReport(1, &r, sizeof(r));
  }

protected:
  uint16_t _onGetDescriptor(uint8_t* dst) override {
    memcpy(dst, reportMap, sizeof(reportMap));
    return sizeof(reportMap);
  }
private:
  static constexpr uint8_t reportMap[] = {
    0x05, 0x0D, 0x09, 0x04, 0xA1, 0x01,
      0x85, 0x01,
      0x09, 0x22, 0xA1, 0x02,
        0x09, 0x42, 0x09, 0x32,
        0x15, 0x00, 0x25, 0x01,
        0x75, 0x01, 0x95, 0x02,
        0x81, 0x02,
        0x75, 0x06, 0x95, 0x01,
        0x81, 0x03,
        0x09, 0x51,
        0x15, 0x00, 0x25, 0x7F,
        0x75, 0x08, 0x95, 0x01,
        0x81, 0x02,
        0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
        0x16, 0x00, 0x00, 0x26, 0xFF, 0x7F,
        0x75, 0x10, 0x95, 0x02,
        0x81, 0x02,
      0xC0,
      0x09, 0x54,
      0x15, 0x00, 0x25, 0x0A,
      0x75, 0x08, 0x95, 0x01,
      0x81, 0x02,
    0xC0
  };
};

HIDTouchDevice Touch;

/* -------------------- NVS Calibration -------------------- */
Preferences prefs;
const char* NVS_NS  = "wbcal";
const char* NVS_KEY = "cal";

struct CalData {
  uint32_t magic   = 0x57424341;  // 'WBCA'
  uint16_t version = 1;
  uint16_t reserved= 0;
  IPoint   cam[4];
  IPoint   brd[4];
  uint32_t dispW;
  uint32_t dispH;
  float    Hm[9];  // float statt double
  uint32_t crc32   = 0;
};

uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len){
  crc = ~crc;
  for(size_t i=0;i<len;i++){
    crc ^= data[i];
    for(int b=0;b<8;b++) crc = (crc>>1) ^ (0xEDB88320U & (-(int)(crc & 1)));
  }
  return ~crc;
}

bool saveCalDataToNVS(){
  CalData d;
  for(int i=0;i<4;i++){ d.cam[i]=g_cam_pts[i]; d.brd[i]=g_board_pts[i]; }
  d.dispW = display_w; d.dispH = display_h;
  H.getMatrix(d.Hm);
  d.crc32 = 0;
  d.crc32 = crc32_update(0, (uint8_t*)&d, sizeof(CalData));

  if(!prefs.begin(NVS_NS,false)) return false;
  size_t w = prefs.putBytes(NVS_KEY, &d, sizeof(d));
  prefs.end();
  return (w == sizeof(d));
}

bool loadCalDataFromNVS(){
  if(!prefs.begin(NVS_NS,true)) return false;
  size_t sz = prefs.getBytesLength(NVS_KEY);
  if(sz != sizeof(CalData)){ prefs.end(); return false; }

  CalData d;
  size_t rd = prefs.getBytes(NVS_KEY, &d, sizeof(d));
  prefs.end();
  if(rd != sizeof(d)) return false;

  if(d.magic != 0x57424341 || d.version != 1) return false;
  uint32_t crc = d.crc32; d.crc32=0;
  if(crc32_update(0,(uint8_t*)&d,sizeof(d)) != crc) return false;

  for(int i=0;i<4;i++){ g_cam_pts[i]=d.cam[i]; g_board_pts[i]=d.brd[i]; }
  display_w=d.dispW; display_h=d.dispH;
  H.setMatrix(d.Hm);
  calibrated = true;
  return true;
}

/* -------------------- Vision/Detection Config -------------------- */
struct VisionConfig {
  bool  auto_exposure = true;
  bool  auto_gain     = true;
  int   aec_value     = 200;
  int   agc_gain      = 20;
  int   brightness    = 2;
  int   contrast      = 2;

  bool  manual_thresh = false;
  int   thresh_value  = 220;
  float percentile    = 0.995f;
  int   thresh_floor  = 180;

  int   min_blob_px   = 8;
  float ema_alpha     = 0.60f;
  int   move_px       = 2;
  int   release_ms    = 200; // 180–300 empfohlen

  int   last_thresh   = -1;
  int   last_blob_n   = 0;

  // Stroke-Splitting
  int   down_confirm_frames = 2;   // 2–3
  int   idle_up_ms          = 400; // 300–600
  int   idle_tol_hid_px     = 3;   // 2–5
} g_vis;

struct LegacyVisionConfig {
  bool  auto_exposure;
  bool  auto_gain;
  int   aec_value;
  int   agc_gain;
  int   brightness;
  int   contrast;
  bool  manual_thresh;
  int   thresh_value;
  float percentile;
  int   thresh_floor;
  int   min_blob_px;
  float ema_alpha;
  int   move_px;
  int   release_ms;
  int   last_thresh;
  int   last_blob_n;
};

static inline int   clampi(int v, int lo, int hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
static inline float clampf(float v,float lo,float hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }

bool saveVisionToNVS(){
  if(!prefs.begin("wbvis", false)) return false;
  size_t w = prefs.putBytes("vision", &g_vis, sizeof(g_vis));
  prefs.end();
  return (w == sizeof(g_vis));
}
bool loadVisionFromNVS(){
  if(!prefs.begin("wbvis", true)) return false;
  size_t sz = prefs.getBytesLength("vision");
  bool ok=false;
  if(sz == sizeof(g_vis)){
    VisionConfig tmp;
    size_t rd = prefs.getBytes("vision", &tmp, sizeof(tmp));
    if(rd == sizeof(tmp)){ g_vis = tmp; ok=true; }
  } else if (sz == sizeof(LegacyVisionConfig)) {
    LegacyVisionConfig old{};
    size_t rd = prefs.getBytes("vision", &old, sizeof(old));
    if(rd == sizeof(old)){
      g_vis.auto_exposure = old.auto_exposure;
      g_vis.auto_gain     = old.auto_gain;
      g_vis.aec_value     = old.aec_value;
      g_vis.agc_gain      = old.agc_gain;
      g_vis.brightness    = old.brightness;
      g_vis.contrast      = old.contrast;
      g_vis.manual_thresh = old.manual_thresh;
      g_vis.thresh_value  = old.thresh_value;
      g_vis.percentile    = old.percentile;
      g_vis.thresh_floor  = old.thresh_floor;
      g_vis.min_blob_px   = old.min_blob_px;
      g_vis.ema_alpha     = old.ema_alpha;
      g_vis.move_px       = old.move_px;
      g_vis.release_ms    = old.release_ms;
      g_vis.last_thresh   = old.last_thresh;
      g_vis.last_blob_n   = old.last_blob_n;

      g_vis.down_confirm_frames = 2;
      g_vis.idle_up_ms          = 400;
      g_vis.idle_tol_hid_px     = 3;
      ok=true;
    }
  }
  prefs.end();
  return ok;
}

void applySensorSettings(){
  sensor_t* s = esp_camera_sensor_get();
  if(!s) return;
  s->set_exposure_ctrl(s, g_vis.auto_exposure ? 1 : 0);
  if(!g_vis.auto_exposure){
    int v = clampi(g_vis.aec_value, 0, 1200);
    s->set_aec_value(s, v);
  }
  s->set_gain_ctrl(s, g_vis.auto_gain ? 1 : 0);
  if(!g_vis.auto_gain){
    int g = clampi(g_vis.agc_gain, 0, 30);
    s->set_agc_gain(s, g);
  }
  int b = clampi(g_vis.brightness, -2, 2);
  int c = clampi(g_vis.contrast,   -2, 2);
  s->set_brightness(s, b);
  s->set_contrast(s, c);
  s->set_whitebal(s, 0); // IR: AWB aus
}

/* -------------------- IR Blob Detection (Weighted-Centroid) -------------------- */
bool   filt_has=false;
float  filt_x=0.0f, filt_y=0.0f;

static const int MED_N = 5;
int med_x_buf[MED_N] = {0};
int med_y_buf[MED_N] = {0};
int med_idx = 0;
int med_count = 0;

static int median_buf(int *buf, int n){
  int t[MED_N];
  for(int i=0;i<n;i++) t[i]=buf[i];
  for(int i=1;i<n;i++){
    int k=t[i]; int j=i-1;
    while(j>=0 && t[j]>k){ t[j+1]=t[j]; j--; }
    t[j+1]=k;
  }
  return t[n/2];
}

/* Alpha–Beta Filter (float) */
struct ABFilter {
  float x=0, y=0;
  float vx=0, vy=0;
  bool  has=false;
  float alpha=0.35f;
  float beta =0.015f;
} ab;

unsigned long last_update_ms = 0;
unsigned long last_hid_tick  = 0;
const uint32_t HID_PERIOD_MS = 8; // ~125 Hz

/* -------------------- Multicore + Buffers -------------------- */
SemaphoreHandle_t g_cam_mutex = nullptr;
SemaphoreHandle_t g_jpeg_mutex = nullptr;

/* Puffer für Snapshot (Option B) */
uint8_t* g_jpeg_buf = nullptr;
size_t   g_jpeg_len = 0;
unsigned long g_last_jpeg_ms = 0;
const uint32_t SNAPSHOT_PERIOD_MS = 500; // 500ms Pull-Rate im Browser harmonisch

/* Vision-Ergebnis (vom Kameratask geschrieben) */
volatile bool   det_found=false;
volatile int    det_cam_x=0, det_cam_y=0;

/* Kamera-/Erkennungstask (Core 1) */
void cameraTask(void*){
  while(camOk){
    if (g_cam_mutex) xSemaphoreTake(g_cam_mutex, portMAX_DELAY);
    camera_fb_t *fb = esp_camera_fb_get();
    if (g_cam_mutex) xSemaphoreGive(g_cam_mutex);

    if (!fb){ vTaskDelay(pdMS_TO_TICKS(5)); continue; }

    latest_cam_w = fb->width;
    latest_cam_h = fb->height;

    // detectIR (weighted)
    bool found=false;
    int cx=0, cy=0;

    if (fb->format == PIXFORMAT_GRAYSCALE){
      const int w = fb->width, h = fb->height;
      const uint8_t* buf = fb->buf;
      const size_t scans = (size_t)w*h;

      // Histogram
      int hist[256]={0};
      for (size_t i=0;i<scans;i++) hist[buf[i]]++;

      // Threshold
      int th;
      if (g_vis.manual_thresh){
        th = clampi(g_vis.thresh_value, 0, 255);
      } else {
        int total = w*h, acc = 0;
        const float p = clampf(g_vis.percentile, 0.90f, 0.9999f);
        const int target = (int)lroundf(p * total);
        th = 255;
        for(int v=255; v>=0; --v){
          acc += hist[v];
          if(acc >= target){ th = v; break; }
        }
        if(th < g_vis.thresh_floor) th = g_vis.thresh_floor;
      }

      // Weighted-Centroid (Helligkeitsgewichtung)
      uint64_t swx=0, swy=0; // Sum(weight*x), Sum(weight*y)
      uint64_t wsum=0;
      int blob_px=0;
      for(int y=0;y<h;y++){
        const int row = y*w;
        for(int x=0;x<w;x++){
          const size_t idx = (size_t)row + x;
          const uint8_t v  = buf[idx];
          if (v >= th){
            const uint32_t wgt = (uint32_t)(v - th + 1); // kleine Dynamik oberhalb th
            swx += (uint64_t)wgt * (uint64_t)x;
            swy += (uint64_t)wgt * (uint64_t)y;
            wsum += wgt;
            blob_px++;
          }
        }
      }

      g_vis.last_thresh = th;
      g_vis.last_blob_n = blob_px;

      if (blob_px >= g_vis.min_blob_px && wsum > 0){
        int gx = (int)(swx / wsum);
        int gy = (int)(swy / wsum);

        // Median + EMA
        med_x_buf[med_idx] = gx;
        med_y_buf[med_idx] = gy;
        med_idx = (med_idx + 1) % MED_N;
        if (med_count < MED_N) med_count++;

        int mx = (med_count >= MED_N) ? median_buf(med_x_buf, med_count) : gx;
        int my = (med_count >= MED_N) ? median_buf(med_y_buf, med_count) : gy;

        if(!filt_has){
          filt_x = (float)mx; filt_y = (float)my;
          filt_has = true;
        } else {
          int dx = mx - (int)lroundf(filt_x);
          int dy = my - (int)lroundf(filt_y);
          if ((dx*dx + dy*dy) >= (g_vis.move_px * g_vis.move_px)){
            float a = clampf(g_vis.ema_alpha, 0.0f, 1.0f);
            filt_x = a*filt_x + (1.0f-a)*mx;
            filt_y = a*filt_y + (1.0f-a)*my;
          }
        }

        cx = (int)lroundf(filt_x);
        cy = (int)lroundf(filt_y);
        found = true;
      } else {
        found = false;
      }
    }

    // Write detection results
    det_found = found;
    if (found){
      det_cam_x = cx;
      det_cam_y = cy;
      latest_has = true;
      latest_cam_x = cx;
      latest_cam_y = cy;
    } else {
      latest_has = false;
    }

    // JPEG Snapshot Puffer (Option B)
    if (millis() - g_last_jpeg_ms >= SNAPSHOT_PERIOD_MS){
      uint8_t* jpg = nullptr; size_t jlen = 0;
      bool ok = frame2jpg(fb, 55, &jpg, &jlen); // moderate Qualität
      if (ok && jpg && jlen > 0){
        if (g_jpeg_mutex) xSemaphoreTake(g_jpeg_mutex, portMAX_DELAY);
        if (g_jpeg_buf) { free(g_jpeg_buf); g_jpeg_buf=nullptr; g_jpeg_len=0; }
        g_jpeg_buf = (uint8_t*)malloc(jlen);
        if (g_jpeg_buf){
          memcpy(g_jpeg_buf, jpg, jlen);
          g_jpeg_len = jlen;
          g_last_jpeg_ms = millis();
        }
        if (g_jpeg_mutex) xSemaphoreGive(g_jpeg_mutex);
        free(jpg);
      }
    }

    // Frame zurück
    if (g_cam_mutex) xSemaphoreTake(g_cam_mutex, portMAX_DELAY);
    esp_camera_fb_return(fb);
    if (g_cam_mutex) xSemaphoreGive(g_cam_mutex);
  }

  vTaskDelete(nullptr);
}

/* -------------------- Snapshot (aus Puffer) -------------------- */
void handleSnapshot(AsyncWebServerRequest* req){
  if (g_jpeg_mutex) xSemaphoreTake(g_jpeg_mutex, portMAX_DELAY);
  size_t len = g_jpeg_len;
  uint8_t* buf = g_jpeg_buf;
  if (len == 0 || buf == nullptr){
    if (g_jpeg_mutex) xSemaphoreGive(g_jpeg_mutex);
    req->send(503, "text/plain", "no frame");
    return;
  }
  AsyncResponseStream *res = req->beginResponseStream("image/jpeg");
  res->addHeader("Cache-Control","no-store, no-cache, must-revalidate, max-age=0");
  res->addHeader("Pragma","no-cache");
  res->addHeader("Connection","close");
  res->write(buf, len);
  if (g_jpeg_mutex) xSemaphoreGive(g_jpeg_mutex);
  req->send(res);
}

/* -------------------- WEB UI (HTML) -------------------- */
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>ESP32-S3 Whiteboard</title>
<style>
body{font-family:Arial;margin:10px}
.row{display:flex;gap:10px;flex-wrap:wrap}
.col{border:1px solid #aaa;padding:8px;max-width:460px}
canvas,img{border:1px solid #333}
table{border-collapse:collapse}
td,th{border:1px solid #ccc;padding:4px 6px}
input[type=number]{width:90px}
small{opacity:.85;display:block;margin-top:3px;line-height:1.35em}
button{margin-top:6px}
fieldset{margin-top:6px}
legend{font-weight:bold}
.badge{display:inline-block;padding:2px 6px;border:1px solid #aaa;border-radius:6px;background:#fafafa;margin-left:6px}
</style>
</head>
<body>
<h2>ESP32-S3 Whiteboard</h2>

<div class="row">

  <div class="col">
    <h3>Status</h3>
    <div><b>Display:</b> <span id="disp">-</span></div>
    <div><b>Camera:</b> <span id="cam">-</span></div>
    <div><b>Trans:</b> <span id="tr">-</span></div>
    <div><b>Calibrated:</b> <span id="cal">no</span></div>
    <div>
      <b>HID:</b> <span id="hidState">-</span>
      <label style="margin-left:8px;"><input type="checkbox" id="hidToggle"> HID Output Enable</label>
      <small>HID schaltet die Touch-Übertragung zum Host an/aus (kein USB-Disconnect).</small>
    </div>
  </div>

  <div class="col">
    <h3>Auto Calibration</h3>
    <button id="btnStartCal">Start Calibration</button>
    <button id="btnStopCal" disabled>Stop</button>
    <small>4-Punkt Autokalibrierung im Vollbild; Stift jeweils ~2 s ruhig halten.</small>
  </div>

  <div class="col">
    <h3>Click Test</h3>
    X:<input id="hidX" type="number" value="200"><br>
    Y:<input id="hidY" type="number" value="200"><br>
    Press ms:<input id="pressMs" type="number" value="150"><br>
    Release ms:<input id="releaseMs" type="number" value="200"><br>
    <button id="btnClick">Send</button>
    <div id="clickMsg"></div>
    <small>Sendet einen künstlichen Touch (Down/Up). Nützlich zum Host-Test ohne IR-Pen.</small>
  </div>

  <div class="col">
    <h3>Camera Preview</h3>
    <label><input type="checkbox" id="camToggle"> Enable</label><br>
    <img id="camView" style="max-width:300px;display:none;margin-top:8px;">
    <small>Snapshot aus Puffer, stabil & leichtgewichtig.</small>
  </div>

  <div class="col">
    <h3>Calibration Preview</h3>
    <div style="display:flex; gap:10px; align-items:flex-start; flex-wrap:wrap;">
      <div>
        <div style="font-weight:bold; margin-bottom:4px;">Camera</div>
        <canvas id="calCanvasCam" width="300" height="225"></canvas>
        <small>Blau = Cam-Punkte (roh), Gelb = IR (roh)</small>
      </div>
      <div>
        <div style="font-weight:bold; margin-bottom:4px;">Board / Homography</div>
        <canvas id="calCanvasH" width="300" height="170"></canvas>
        <small>Rot = Soll (Board), Blau = Ist (Cam->H), Grün = IR (H)</small>
      </div>
    </div>
  </div>

</div>

<div class="row">
  <div class="col">
    <h3>Manual Points</h3>
    <table>
      <thead><tr><th>#</th><th>Cam X</th><th>Cam Y</th><th>Board X</th><th>Board Y</th></tr></thead>
      <tbody id="tblBody"></tbody>
    </table>
    <button id="btnLoad">Load</button>
    <button id="btnApply">Apply & Save</button>
    <button id="btnClear">Clear Cal</button>
    <div id="manMsg"><small>-</small></div>
    <small>Manuelles Setzen der 4 Eckpunkte. Tipp: Randabstand ~80 px, gleiche Reihenfolge wie Auto-Cal.</small>
  </div>

  <div class="col">
    <h3>Display Size</h3>
    W:<input id="dW" type="number" value="1920">
    H:<input id="dH" type="number" value="1080"><br>
    <button id="btnSetDisp">Set Display</button>
    <small>Setzt die Ziel-Displaygröße für Homography & HID-Skalierung.</small>
  </div>

  <div class="col">
    <h3>Vision / IR-Erkennung</h3>

    <fieldset>
      <legend>Sensor</legend>
      <label><input type="checkbox" id="autoExp"> Auto Exposure</label>
      <label style="margin-left:8px;"><input type="checkbox" id="autoGain"> Auto Gain</label><br>
      AEC:<input id="aec" type="number" value="200" min="0" max="1200">
      AGC:<input id="agc" type="number" value="20"  min="0" max="30"><br>
      Bright:<input id="bright" type="number" value="2" min="-2" max="2">
      Contrast:<input id="contrast" type="number" value="2" min="-2" max="2">
      <small>Auto Exp/Gain: ON empfohlen; AEC ~150–400, AGC ~10–25, Bright/Contrast −2..2.</small>
    </fieldset>

    <fieldset>
      <legend>Erkennung</legend>
      <label><input type="checkbox" id="manThresh"> Manuelle Schwelle</label>
      <span id="autoBadge" class="badge">auto</span><br>
      Thresh:<input id="thresh" type="number" value="220" min="0" max="255">
      Floor:<input id="floor" type="number" value="180" min="0" max="255"><br>
      Percentile:<input id="perc" type="number" value="0.995" step="0.0005" min="0.90" max="0.9999">
      MinBlob(px):<input id="minBlob" type="number" value="8" min="1" max="500">
      <small>Auto: Perzentil 0.990–0.999; Manuell: Thresh ~180–240, Floor ~160–210; MinBlob ~5–20.</small>
    </fieldset>

    <fieldset>
      <legend>Debounce / Filter</legend>
      EMA alpha:<input id="ema" type="number" value="0.60" step="0.05" min="0" max="1">
      Move(px):<input id="move" type="number" value="2" min="0" max="50"><br>
      Release(ms):<input id="relms" type="number" value="200" min="0" max="2000">
      <small>EMA glättet (0.5–0.8), Move(px) unterdrückt Kleinstbewegungen (1–4), Release(ms) entprellt IR-Verlust (180–300).</small>
    </fieldset>

    <fieldset>
      <legend>Stroke Splitting</legend>
      Down Confirm(frames):<input id="downFrames" type="number" value="2" min="1" max="10">
      Idle Up(ms):<input id="idleMs" type="number" value="400" min="0" max="2000"><br>
      Idle Tol(HID px):<input id="idleTol" type="number" value="3" min="0" max="50">
      <small>Verhindert Linien über den Leerraum:
        <br>- Down Confirm: 2–3
        <br>- Idle Up: 300–600 ms
        <br>- Idle Tol: 2–5 HID-Einheiten</small>
    </fieldset>

    <button id="btnVisLoad">Load</button>
    <button id="btnVisApply">Apply & Save</button>
    <div id="visMsg"><small>-</small></div>
  </div>

</div>

<!-- ---------- AUTO CALIBRATION OVERLAY ---------- -->
<div id="calOverlay"
     style="position:fixed; left:0; top:0; width:100%; height:100%;
            background:white; z-index:9999; display:none; cursor:none;">
  <div id="cross"
       style="position:absolute; width:1px; height:1px; left:50%; top:50%;">
    <div style="position:absolute; left:-60px; top:0; width:120px; height:2px; background:black;"></div>
    <div style="position:absolute; left:0; top:-60px; width:2px; height:120px; background:black;"></div>
  </div>
  <div id="calMsg"
       style="position:absolute; bottom:30px; left:50%; transform:translateX(-50%);
              background:#ffffffcc; border:1px solid #aaa; padding:8px 15px;">
    Corner 1/4 - hold 0.0 / 2.0 s
  </div>
</div>

<script>
// ---------- Globale State ----------
let display_w=1920, display_h=1080;

// ---------- DOM ----------
const disp      = document.getElementById('disp');
const cam       = document.getElementById('cam');
const tr        = document.getElementById('tr');
const cal       = document.getElementById('cal');
const manMsg    = document.getElementById('manMsg');

const hidState  = document.getElementById('hidState');
const hidToggle = document.getElementById('hidToggle');

const camToggle = document.getElementById('camToggle');
const camView   = document.getElementById('camView');

const calCanvasCam = document.getElementById('calCanvasCam');
const calCanvasH   = document.getElementById('calCanvasH');
const ctxCam       = calCanvasCam.getContext('2d');
const ctxH         = calCanvasH.getContext('2d');

const btnLoad   = document.getElementById('btnLoad');
const btnApply  = document.getElementById('btnApply');
const btnClear  = document.getElementById('btnClear');

const btnSetDisp= document.getElementById('btnSetDisp');
const dW        = document.getElementById('dW');
const dH        = document.getElementById('dH');

const autoExp   = document.getElementById('autoExp');
const autoGain  = document.getElementById('autoGain');
const aec       = document.getElementById('aec');
const agc       = document.getElementById('agc');
const bright    = document.getElementById('bright');
const contrast  = document.getElementById('contrast');

const manThresh = document.getElementById('manThresh');
const thresh    = document.getElementById('thresh');
const floorV    = document.getElementById('floor');
const perc      = document.getElementById('perc');
const minBlob   = document.getElementById('minBlob');
const lastTh    = document.getElementById('lastTh');
const lastBlob  = document.getElementById('lastBlob');
const autoBadge = document.getElementById('autoBadge');

const visMsg    = document.getElementById('visMsg');
const ema       = document.getElementById('ema');
const move      = document.getElementById('move');
const relms     = document.getElementById('relms');

const downFrames = document.getElementById('downFrames');
const idleMs     = document.getElementById('idleMs');
const idleTol    = document.getElementById('idleTol');

const btnStartCal = document.getElementById('btnStartCal');
const btnStopCal  = document.getElementById('btnStopCal');
const calOverlay  = document.getElementById('calOverlay');
const calMsg      = document.getElementById('calMsg');
const cross       = document.getElementById('cross');

const btnClick  = document.getElementById('btnClick');
const hidX      = document.getElementById('hidX');
const hidY      = document.getElementById('hidY');
const pressMs   = document.getElementById('pressMs');
const releaseMs = document.getElementById('releaseMs');
const clickMsg  = document.getElementById('clickMsg');

// ---------- Hilfsflags ----------
let camTimer=null;
let calTimer=null;
let editingDisplay=false;
let last_cam_w = 320, last_cam_h = 240;

// ---------- Canvas dynamisch ans Seitenverhältnis anpassen ----------
function adjustCalCanvasSize(){
  const baseW = 300;
  calCanvasH.width = baseW;
  calCanvasH.height = Math.max(140, Math.round(baseW * (display_h / display_w)));

  const cw = last_cam_w > 0 ? last_cam_w : 320;
  const ch = last_cam_h > 0 ? last_cam_h : 240;
  calCanvasCam.width = baseW;
  calCanvasCam.height = Math.max(140, Math.round(baseW * (ch / cw)));
}
adjustCalCanvasSize();

// ---------- Display-Inputs: beim Tippen nicht überschreiben ----------
[dW, dH].forEach(inp=>{
  inp.addEventListener('focus', () => editingDisplay=true);
  inp.addEventListener('blur',  () => editingDisplay=false);
});

// ---------- Status ----------
async function updateStatus(){
  try{
    const r=await fetch('/api/coords');
    const j=await r.json();
    if(j.display){
      const sw=j.display.w, sh=j.display.h;
      if((sw!==display_w || sh!==display_h)){
        display_w=sw; display_h=sh;
        disp.innerText=`${display_w}x${display_h}`;
        if(!editingDisplay){
          dW.value=display_w; dH.value=display_h;
        }
        adjustCalCanvasSize();
      }
    }
    cal.innerText=j.calibrated?'yes':'no';
    if(j.ok){
      cam.innerText=`${j.camera.x},${j.camera.y}`;
      tr.innerText =`${j.trans.x},${j.trans.y}`;
    } else {
      cam.innerText='-'; tr.innerText='-';
    }
    if('hidEnabled' in j){
      hidState.innerText=j.hidEnabled?'on':'off';
      hidToggle.checked = !!j.hidEnabled;
    }
  }catch(e){}
}
setInterval(updateStatus,400);

// ---------- HID Toggle ----------
hidToggle.addEventListener('change', async ()=>{
  try{
    const r=await fetch('/api/set_hid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:hidToggle.checked})});
    const j=await r.json();
    hidState.innerText=j.enabled?'on':'off';
  }catch(e){}
});

// ---------- Click Test ----------
btnClick.onclick=async()=>{
  const x=+hidX.value, y=+hidY.value;
  const press=+pressMs.value, rel=+releaseMs.value;
  const r=await fetch('/api/test_click',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x,y,press,rel})});
  const j=await r.json();
  clickMsg.innerText = j.ok ? 'Sent' : 'Error';
};

// ---------- Camera Preview ----------
camToggle.onchange=()=>{
  if(camToggle.checked){
    camView.style.display='block';
    camTimer=setInterval(()=>{ camView.src='/api/cam_snapshot?ts='+Date.now(); },500);
  } else {
    if(camTimer) clearInterval(camTimer);
    camTimer=null; camView.style.display='none'; camView.src='';
  }
};

// ---------- Calibration Preview ----------
function drawCamPreview(camRaw, pen) {
  const w = calCanvasCam.width, h = calCanvasCam.height;
  const ctx = ctxCam;
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle = '#f4f4f4'; ctx.fillRect(0,0,w,h);
  ctx.strokeStyle='#000'; ctx.lineWidth=2; ctx.strokeRect(0,0,w,h);

  const camW = (pen && pen.camW) ? pen.camW : last_cam_w;
  const camH = (pen && pen.camH) ? pen.camH : last_cam_h;
  const sx = w / camW, sy = h / camH;

  if (Array.isArray(camRaw) && camRaw.length >= 3) {
    ctx.strokeStyle = 'rgba(0,51,204,1)';
    ctx.fillStyle   = 'rgba(0,51,204,0.08)';
    ctx.beginPath();
    camRaw.forEach((p,i)=>{
      const x = p.x * sx, y = p.y * sy;
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    });
    ctx.closePath();
    ctx.fill(); ctx.stroke();

    ctx.fillStyle = 'rgba(0,51,204,1)';
    camRaw.forEach(p=>{
      const x = p.x * sx, y = p.y * sy;
      ctx.beginPath(); ctx.arc(x,y,4,0,2*Math.PI); ctx.fill();
    });
  }

  if (pen && pen.has && camW && camH) {
    const rx = pen.camX * sx, ry = pen.camY * sy;
    ctx.fillStyle='gold'; ctx.beginPath(); ctx.arc(rx,ry,4,0,2*Math.PI); ctx.fill();
    ctx.strokeStyle='orange'; ctx.beginPath(); ctx.arc(rx,ry,7,0,2*Math.PI); ctx.stroke();

    ctx.fillStyle='#444'; ctx.font='10px Arial';
    ctx.fillText(`Cam: ${camW}x${camH}`, 6, 12);
  }
}

function drawHomographyPreview(camT, brd, pen){
  const w=calCanvasH.width, h=calCanvasH.height;
  const ctx = ctxH;
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle='#eee'; ctx.fillRect(0,0,w,h);
  ctx.strokeStyle='#000'; ctx.lineWidth=2; ctx.strokeRect(0,0,w,h);

  const sx=w/display_w, sy=h/display_h;

  if (Array.isArray(brd) && brd.length >= 3) {
    ctx.strokeStyle='rgba(179,0,0,1)';
    ctx.fillStyle  ='rgba(255,0,0,0.12)';
    ctx.beginPath();
    brd.forEach((p,i)=>{
      const x = p.x*sx, y = p.y*sy;
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    });
    ctx.closePath(); ctx.fill(); ctx.stroke();

    ctx.fillStyle='rgba(179,0,0,1)';
    brd.forEach(p=>{
      const x=p.x*sx, y=p.y*sy;
      ctx.beginPath(); ctx.arc(x,y,4,0,2*Math.PI); ctx.fill();
    });
  }

  if (Array.isArray(camT) && camT.length >= 3) {
    ctx.strokeStyle='rgba(0,51,204,1)';
    ctx.fillStyle  ='rgba(0,51,204,0.08)';
    ctx.beginPath();
    camT.forEach((p,i)=>{
      const x=p.x*sx, y=p.y*sy;
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    });
    ctx.closePath(); ctx.fill(); ctx.stroke();

    ctx.fillStyle='rgba(0,51,204,1)';
    camT.forEach(p=>{
      const x=p.x*sx, y=p.y*sy;
      ctx.beginPath(); ctx.arc(x,y,4,0,2*Math.PI); ctx.fill();
    });
  }

  if (pen && pen.has) {
    const px = pen.x * sx, py = pen.y * sy;
    ctx.fillStyle='limegreen'; ctx.beginPath(); ctx.arc(px,py,5,0,2*Math.PI); ctx.fill();
    ctx.strokeStyle='green'; ctx.beginPath(); ctx.arc(px,py,9,0,2*Math.PI); ctx.stroke();
  }
}

let last_cam_w = 320, last_cam_h = 240;
function adjustCalCanvasSize(){ /* (wie oben definiert) */ }

async function refreshCalPreview(){
  try{
    const r=await fetch('/api/get_cal_preview');
    if(!r.ok) return;
    const j=await r.json();

    if (j.pen && j.pen.camW && j.pen.camH) {
      last_cam_w = j.pen.camW;
      last_cam_h = j.pen.camH;
      adjustCalCanvasSize();
    }

    drawCamPreview(j.camRaw, j.pen);
    drawHomographyPreview(j.camT, j.brd, j.pen);

    if(j.penInfo){
      lastTh.innerText = (j.penInfo.lastThresh ?? '-');
      lastBlob.innerText = (j.penInfo.lastBlob ?? '-');
    }
  }catch(e){}
}
setInterval(refreshCalPreview,800);

// ---------- Manual Points ----------
btnLoad.onclick=async()=>{
  try{
    const r=await fetch('/api/get_cal_points');
    if(!r.ok){ manMsg.innerText='fail'; return; }
    const j=await r.json();
    const tb=document.getElementById('tblBody'); tb.innerHTML='';
    for(let i=0;i<4;i++){
      const c=j.camera[i], b=j.board[i];
      tb.innerHTML+=`<tr>
        <td>${i+1}</td>
        <td><input id=camx${i} type=number value=${c.x}></td>
        <td><input id=camy${i} type=number value=${c.y}></td>
        <td><input id=brdx${i} type=number value=${b.x}></td>
        <td><input id=brdy${i} type=number value=${b.y}></td>
      </tr>`;
    }
    manMsg.innerText='loaded';
  }catch(e){ manMsg.innerText='error'; }
};
btnApply.onclick=async()=>{
  let cam=[], brd=[];
  for(let i=0;i<4;i++){
    cam.push({x:+document.getElementById('camx'+i).value,y:+document.getElementById('camy'+i).value});
    brd.push({x:+document.getElementById('brdx'+i).value,y:+document.getElementById('brdy'+i).value});
  }
  const r=await fetch('/api/set_cal_points',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({camera:cam,board:brd})});
  const j=await r.json(); manMsg.innerText=j.ok?'saved':'fail';
};
btnClear.onclick=async()=>{
  const r=await fetch('/api/clear_cal',{method:'POST'}); const j=await r.json();
  manMsg.innerText=j.ok?'cleared':'fail';
};

// ---------- Display Size ----------
btnSetDisp.onclick=async()=>{
  const w=+dW.value,h=+dH.value;
  const r=await fetch('/api/set_display',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({w,h})});
  const j=await r.json(); 
  if(j.ok){ 
    display_w=j.w; display_h=j.h; 
    disp.innerText=`${j.w}x${j.h}`; 
    adjustCalCanvasSize();
  }
};

// ---------- Sensor/Erkennung UI ----------
function setSensorFieldStates() {
  const ae = autoExp.checked;
  const ag = autoGain.checked;
  aec.disabled = ae;
  agc.disabled = ag;
}
function setThreshFieldStates() {
  const man = manThresh.checked;
  thresh.disabled = !man;
  floorV.disabled = !man;
  autoBadge.textContent = man ? 'manual' : 'auto';
}
autoExp.addEventListener('change', setSensorFieldStates);
autoGain.addEventListener('change', setSensorFieldStates);
manThresh.addEventListener('change', setThreshFieldStates);

function reflectVisionUI(v) {
  autoExp.checked = !!v.auto_exposure;
  autoGain.checked = !!v.auto_gain;
  aec.value = v.aec_value ?? 200;
  agc.value = v.agc_gain  ?? 20;
  bright.value   = v.brightness ?? 2;
  contrast.value = v.contrast   ?? 2;

  manThresh.checked = !!v.manual_thresh;
  thresh.value = v.thresh_value ?? 220;
  floorV.value = v.thresh_floor ?? 180;
  perc.value   = (v.percentile ?? 0.995).toFixed(4);
  minBlob.value = v.min_blob_px ?? 8;

  ema.value  = (v.ema_alpha ?? 0.60).toFixed(2);
  move.value = v.move_px ?? 2;
  relms.value = v.release_ms ?? 200;

  downFrames.value = v.down_confirm_frames ?? 2;
  idleMs.value     = v.idle_up_ms ?? 400;
  idleTol.value    = v.idle_tol_hid_px ?? 3;

  if (typeof v.last_thresh === 'number') lastTh.textContent = v.last_thresh;
  if (typeof v.last_blob   === 'number') lastBlob.textContent = v.last_blob;

  setSensorFieldStates();
  setThreshFieldStates();
}
function collectVisionFromUI() {
  return {
    auto_exposure: autoExp.checked,
    auto_gain:     autoGain.checked,
    aec_value:     Number(aec.value),
    agc_gain:      Number(agc.value),
    brightness:    Number(bright.value),
    contrast:      Number(contrast.value),

    manual_thresh: manThresh.checked,
    thresh_value:  Number(thresh.value),
    percentile:    Number(perc.value),
    thresh_floor:  Number(floorV.value),
    min_blob_px:   Number(minBlob.value),

    ema_alpha:     Number(ema.value),
    move_px:       Number(move.value),
    release_ms:    Number(relms.value),

    down_confirm_frames: Number(downFrames.value),
    idle_up_ms:          Number(idleMs.value),
    idle_tol_hid_px:     Number(idleTol.value)
  };
}
const btnVisLoad  = document.getElementById('btnVisLoad');
const btnVisApply = document.getElementById('btnVisApply');

btnVisLoad.onclick = async () => {
  visMsg.textContent = 'Loading...';
  try {
    const r = await fetch('/api/get_vision');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const v = await r.json();
    reflectVisionUI(v);
    visMsg.textContent = 'Loaded';
  } catch (e) {
    visMsg.textContent = 'Error loading';
  }
};
btnVisApply.onclick = async () => {
  const body = collectVisionFromUI();
  visMsg.textContent = 'Applying...';
  try {
    const r = await fetch('/api/set_vision', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(body)
    });
    const j = await r.json();
    if (!j.ok) throw new Error('apply failed');
    visMsg.textContent = 'Saved';
    try {
      const r2 = await fetch('/api/get_vision');
      if (r2.ok) {
        const v2 = await r2.json();
        reflectVisionUI(v2);
      }
    } catch(_) {}
  } catch (e) {
    visMsg.textContent = 'Error saving';
  }
};
// Bei Seitenstart gleich laden
btnVisLoad.click();

// ---------- Auto Calibration ----------
const HOLD_MS = 2000;

let calStep=0;
let dwellMs = 0;
let lastTickTs = null;

document.addEventListener('fullscreenchange', async ()=>{
  if (calOverlay.style.display === 'block'){
    await new Promise(r=>setTimeout(r, 50));
  }
});
window.addEventListener('resize', async ()=>{
  if (calOverlay.style.display === 'block'){
    await new Promise(r=>setTimeout(r, 50));
  } else {
    adjustCalCanvasSize();
  }
});

function screenTarget(i){
  const m=80;
  if(i==0) return {x:m,y:m};
  if(i==1) return {x:display_w-m,y:m};
  if(i==2) return {x:display_w-m,y:display_h-m};
  return {x:m,y:display_h-m};
}
function moveCross(pt){
  const rect = calOverlay.getBoundingClientRect();
  const x = (pt.x / display_w) * rect.width;
  const y = (pt.y / display_h) * rect.height;
  cross.style.left = x + 'px';
  cross.style.top  = y + 'px';
}
function refreshCrossPosition(){
  const target = screenTarget(Math.min(calStep, 3));
  moveCross(target);
}

async function endCalibration(){
  try {
    if (calTimer) { clearInterval(calTimer); calTimer = null; }
    await fetch('/api/calib_mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({on:false})});
  } catch(e) {
  } finally {
    if (document.fullscreenElement) {
      try { await document.exitFullscreen(); } catch(_) {}
    }
    calOverlay.style.display='none';
    btnStartCal.disabled=false; btnStopCal.disabled=true;
    try { await updateStatus(); } catch(_) {}
  }
}

btnStopCal.onclick = endCalibration;

btnStartCal.onclick=async()=>{
  if(!document.fullscreenElement) await document.documentElement.requestFullscreen();
  calOverlay.style.display='block'; btnStartCal.disabled=true; btnStopCal.disabled=false;

  await new Promise(r=>setTimeout(r, 50));

  calStep=0;
  dwellMs = 0; lastTickTs = performance.now();

  await fetch('/api/calib_mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({on:true})});

  moveCross(screenTarget(0));
  calMsg.innerText='Corner 1/4 - hold 0.0 / 2.0 s';

  calTimer=setInterval(async()=>{
    const now = performance.now();
    const dt = lastTickTs ? (now - lastTickTs) : 120;
    lastTickTs = now;

    const r=await fetch('/api/coords'); const j=await r.json();
    if(!j.ok){
      dwellMs = 0;
      calMsg.innerText = `Corner ${calStep+1}/4 - hold 0.0 / 2.0 s`;
      return;
    }
    const cx=j.camera.x, cy=j.camera.y;

    // einfache Stabilitätsabschätzung
    // (hier ohne extra Puffer; reicht für 2s-Hold)
    // Bei Bewegung resetten:
    // Toleranz 4 px:
    if(Math.abs(cx - (j.camera.x))<4 && Math.abs(cy - (j.camera.y))<4){
      dwellMs += dt;
    } else {
      dwellMs = 0;
    }

    const shown = Math.min(2.0, dwellMs/1000).toFixed(1);
    calMsg.innerText = `Corner ${calStep+1}/4 - hold ${shown} / 2.0 s`;

    if(dwellMs >= HOLD_MS){
      // Punkte laden & speichern über API /save_calibration
      // Wir holen uns die aktuellen Ecke je Step:
      const boardPt = screenTarget(calStep);
      // Wir lesen live die Cam-Koords aus /api/coords => schon oben:
      const camPt = {x:cx, y:cy};

      // Zwischenspeicher im Client:
      if(!window.__calCam) window.__calCam=[];
      if(!window.__calBrd) window.__calBrd=[];
      window.__calCam.push(camPt);
      window.__calBrd.push(boardPt);

      calStep++; dwellMs=0;

      if(calStep>=4){
        try{
          await fetch('/api/save_calibration',{
            method:'POST',
            headers:{'Content-Type':'application/json'},
            body:JSON.stringify({camera:window.__calCam,board:window.__calBrd})
          });
        }catch(_){}
        await endCalibration();
        return;
      }

      moveCross(screenTarget(calStep));
      calMsg.innerText = `Corner ${calStep+1}/4 - hold 0.0 / 2.0 s`;
    }
  },120);
};
</script>
</body>
</html>
)HTML";

/* -------------------- sendJson: Template-Fix -------------------- */
template <size_t CAP>
void sendJson(AsyncWebServerRequest* request, ArduinoJson::StaticJsonDocument<CAP>& doc) {
  AsyncJsonResponse *response = new AsyncJsonResponse();
  response->addHeader("Cache-Control","no-store");
  JsonVariant root = response->getRoot();
  root.set(doc.as<JsonVariant>());
  response->setLength();
  request->send(response);
}
inline void sendJson(AsyncWebServerRequest* request, ArduinoJson::DynamicJsonDocument& doc) {
  AsyncJsonResponse *response = new AsyncJsonResponse();
  response->addHeader("Cache-Control","no-store");
  JsonVariant root = response->getRoot();
  root.set(doc.as<JsonVariant>());
  response->setLength();
  request->send(response);
}

/* -------------------- Server & APIs -------------------- */
void setupServer(){

  // Root UI
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200,"text/html",INDEX_HTML);
  });

  // Snapshot (aus Puffer)
  server.on("/api/cam_snapshot", HTTP_GET, [](AsyncWebServerRequest* request){
    handleSnapshot(request);
  });

  // Coords
  server.on("/api/coords", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<256> doc;
    if(!latest_has) doc["ok"]=false;
    else{
      doc["ok"]=true;
      JsonObject cam=doc.createNestedObject("camera");
      cam["x"]=latest_cam_x; cam["y"]=latest_cam_y;
      float tx,ty; H.apply((float)latest_cam_x,(float)latest_cam_y,tx,ty);
      JsonObject tr=doc.createNestedObject("trans");
      tr["x"]=(int)lroundf(tx); tr["y"]=(int)lroundf(ty);
    }
    JsonObject d=doc.createNestedObject("display");
    d["w"]=display_w; d["h"]=display_h;
    doc["calibrated"]=calibrated;
    doc["hidEnabled"]=g_hid_enabled;
    sendJson(request, doc);
  });

  // Display set
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/set_display",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      JsonObject o=json.as<JsonObject>();
      uint32_t w=o["w"]|0, h=o["h"]|0;
      if(w<16||h<16){ request->send(400,"application/json","{\"ok\":false}"); return; }
      display_w=w; display_h=h;
      if(calibrated) saveCalDataToNVS();
      StaticJsonDocument<64> doc; doc["ok"]=true; doc["w"]=w; doc["h"]=h;
      sendJson(request, doc);
    }
  ));

  // HID enable/disable
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/set_hid",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      bool en = json["enabled"] | true;
      g_hid_enabled = en;
      StaticJsonDocument<64> doc; doc["ok"]=true; doc["enabled"]=en;
      sendJson(request, doc);
    }
  ));

  // Click Test
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/test_click",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      int px=json["x"]|0, py=json["y"]|0;
      g_right_press_ms   = json["press"] | 150;
      g_right_release_ms = json["rel"]   | 200;

      if(px<0)px=0; if(py<0)py=0;
      if((uint32_t)px>=display_w) px=display_w-1;
      if((uint32_t)py>=display_h) py=display_h-1;

      uint16_t hx=(uint16_t)lroundf(px/(float)display_w*32767.f);
      uint16_t hy=(uint16_t)lroundf(py/(float)display_h*32767.f);

      g_right_x=hx; g_right_y=hy;
      g_hid_right_active=true;
      g_right_state=0;

      StaticJsonDocument<32> doc; doc["ok"]=true; sendJson(request, doc);
    }
  ));

  // Calibration preview data
  server.on("/api/get_cal_preview", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<1152> doc;
    JsonArray camT = doc.createNestedArray("camT");
    JsonArray brd  = doc.createNestedArray("brd");
    JsonArray camR = doc.createNestedArray("camRaw");

    for(int i=0;i<4;i++){
      JsonObject b=brd.createNestedObject(); 
      b["x"]=g_board_pts[i].x; 
      b["y"]=g_board_pts[i].y;

      float tx,ty; 
      H.apply((float)g_cam_pts[i].x,(float)g_cam_pts[i].y,tx,ty);
      JsonObject c=camT.createNestedObject(); 
      c["x"]=(int)lroundf(tx); 
      c["y"]=(int)lroundf(ty);

      JsonObject cr=camR.createNestedObject();
      cr["x"]=g_cam_pts[i].x;
      cr["y"]=g_cam_pts[i].y;
    }

    JsonObject pen = doc.createNestedObject("pen");
    pen["has"] = latest_has;
    if(latest_has){
      float tx, ty;
      H.apply((float)latest_cam_x, (float)latest_cam_y, tx, ty);
      if(tx < 0) tx = 0; if(ty < 0) ty = 0;
      if(tx > (float)display_w-1) tx = (float)display_w-1;
      if(ty > (float)display_h-1) ty = (float)display_h-1;
      pen["x"] = (int)lroundf(tx);
      pen["y"] = (int)lroundf(ty);
      pen["camX"] = latest_cam_x;
      pen["camY"] = latest_cam_y;
      pen["camW"] = latest_cam_w;
      pen["camH"] = latest_cam_h;
    }

    JsonObject pi = doc.createNestedObject("penInfo");
    pi["lastThresh"] = g_vis.last_thresh;
    pi["lastBlob"]   = g_vis.last_blob_n;

    sendJson(request, doc);
  });

  // Manual get points
  server.on("/api/get_cal_points", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<256> doc;
    JsonArray cam=doc.createNestedArray("camera");
    JsonArray brd=doc.createNestedArray("board");
    for(int i=0;i<4;i++){
      JsonObject c=cam.createNestedObject(); c["x"]=g_cam_pts[i].x; c["y"]=g_cam_pts[i].y;
      JsonObject b=brd.createNestedObject(); b["x"]=g_board_pts[i].x; b["y"]=g_board_pts[i].y;
    }
    sendJson(request, doc);
  });

  // Manual set points
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/set_cal_points",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      JsonArray cam=json["camera"];
      JsonArray brd=json["board"];
      if(cam.size()<4||brd.size()<4){ request->send(400,"application/json","{\"ok\":false}"); return; }
      float s[8],d[8];
      for(int i=0;i<4;i++){
        g_cam_pts[i].x=cam[i]["x"] | 0; g_cam_pts[i].y=cam[i]["y"] | 0;
        g_board_pts[i].x=brd[i]["x"] | 0; g_board_pts[i].y=brd[i]["y"] | 0;
        s[2*i]=(float)g_cam_pts[i].x; s[2*i+1]=(float)g_cam_pts[i].y;
        d[2*i]=(float)g_board_pts[i].x; d[2*i+1]=(float)g_board_pts[i].y;
      }
      Homography Hn; if(!Hn.compute(s,d)){ request->send(500,"application/json","{\"ok\":false}"); return; }
      float M[9]; Hn.getMatrix(M); H.setMatrix(M);
      calibrated=true; saveCalDataToNVS();
      StaticJsonDocument<32> doc; doc["ok"]=true; sendJson(request, doc);
    }
  ));

  // Auto-calibration save
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/save_calibration",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      JsonArray cam=json["camera"]; JsonArray brd=json["board"];
      if(cam.size()<4||brd.size()<4){ request->send(400,"application/json","{\"ok\":false}"); return; }
      float s[8],d[8];
      for(int i=0;i<4;i++){
        g_cam_pts[i].x=cam[i]["x"] | 0; g_cam_pts[i].y=cam[i]["y"] | 0;
        g_board_pts[i].x=brd[i]["x"] | 0; g_board_pts[i].y=brd[i]["y"] | 0;
        s[2*i]=(float)g_cam_pts[i].x; s[2*i+1]=(float)g_cam_pts[i].y;
        d[2*i]=(float)g_board_pts[i].x; d[2*i+1]=(float)g_board_pts[i].y;
      }
      Homography Hn; if(!Hn.compute(s,d)){ request->send(500,"application/json","{\"ok\":false}"); return; }
      float M[9]; Hn.getMatrix(M); H.setMatrix(M);
      calibrated=true; saveCalDataToNVS();
      StaticJsonDocument<32> doc; doc["ok"]=true; sendJson(request, doc);
    }
  ));

  // Calibration mode (Flag)
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/calib_mode",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      g_calibrating = json["on"] | false;
      StaticJsonDocument<32> doc; doc["ok"]=true; sendJson(request, doc);
    }
  ));

  // Clear calibration
  server.on("/api/clear_cal", HTTP_POST, [](AsyncWebServerRequest *request){
    if(prefs.begin(NVS_NS,false)){ prefs.remove(NVS_KEY); prefs.end(); }
    calibrated=false; H.setIdentity();
    for(int i=0;i<4;i++){ g_cam_pts[i]={0,0}; g_board_pts[i]={0,0}; }
    StaticJsonDocument<32> doc; doc["ok"]=true; sendJson(request, doc);
  });

  // Vision get
  server.on("/api/get_vision", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<512> doc;
    doc["auto_exposure"]=g_vis.auto_exposure;
    doc["auto_gain"]=g_vis.auto_gain;
    doc["aec_value"]=g_vis.aec_value;
    doc["agc_gain"]=g_vis.agc_gain;
    doc["brightness"]=g_vis.brightness;
    doc["contrast"]=g_vis.contrast;

    doc["manual_thresh"]=g_vis.manual_thresh;
    doc["thresh_value"]=g_vis.thresh_value;
    doc["percentile"]=g_vis.percentile;
    doc["thresh_floor"]=g_vis.thresh_floor;
    doc["min_blob_px"]=g_vis.min_blob_px;

    doc["ema_alpha"]=g_vis.ema_alpha;
    doc["move_px"]=g_vis.move_px;
    doc["release_ms"]=g_vis.release_ms;

    doc["down_confirm_frames"]=g_vis.down_confirm_frames;
    doc["idle_up_ms"]=g_vis.idle_up_ms;
    doc["idle_tol_hid_px"]=g_vis.idle_tol_hid_px;

    doc["last_thresh"]=g_vis.last_thresh;
    doc["last_blob"]=g_vis.last_blob_n;

    sendJson(request, doc);
  });

  // Vision set
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/set_vision",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      JsonObject o=json.as<JsonObject>();
      g_vis.auto_exposure = o["auto_exposure"] | g_vis.auto_exposure;
      g_vis.auto_gain     = o["auto_gain"]     | g_vis.auto_gain;
      g_vis.aec_value     = clampi(o["aec_value"]|g_vis.aec_value, 0, 1200);
      g_vis.agc_gain      = clampi(o["agc_gain"] |g_vis.agc_gain,  0, 30);
      g_vis.brightness    = clampi(o["brightness"]|g_vis.brightness, -2, 2);
      g_vis.contrast      = clampi(o["contrast"]  |g_vis.contrast,   -2, 2);

      g_vis.manual_thresh = o["manual_thresh"] | g_vis.manual_thresh;
      g_vis.thresh_value  = clampi(o["thresh_value"]|g_vis.thresh_value, 0, 255);
      g_vis.percentile    = clampf(o["percentile"]  |g_vis.percentile, 0.90f, 0.9999f);
      g_vis.thresh_floor  = clampi(o["thresh_floor"]|g_vis.thresh_floor, 0, 255);
      g_vis.min_blob_px   = clampi(o["min_blob_px"] |g_vis.min_blob_px, 1, 1000);

      g_vis.ema_alpha     = clampf(o["ema_alpha"]   |g_vis.ema_alpha, 0.0f, 1.0f);
      g_vis.move_px       = clampi(o["move_px"]     |g_vis.move_px, 0, 100);
      g_vis.release_ms    = clampi(o["release_ms"]  |g_vis.release_ms, 0, 2000);

      g_vis.down_confirm_frames = clampi(o["down_confirm_frames"] | g_vis.down_confirm_frames, 1, 10);
      g_vis.idle_up_ms          = clampi(o["idle_up_ms"]          | g_vis.idle_up_ms,          0, 2000);
      g_vis.idle_tol_hid_px     = clampi(o["idle_tol_hid_px"]     | g_vis.idle_tol_hid_px,     0, 50);

      applySensorSettings();
      saveVisionToNVS();

      StaticJsonDocument<32> doc; doc["ok"]=true; sendJson(request, doc);
    }
  ));

  server.begin();
}

/* -------------------- Stroke/Idle Handling -------------------- */
static int g_found_streak = 0;
static int g_last_hx = -1, g_last_hy = -1;
static unsigned long g_last_move_ms = 0;

/* -------------------- Kamera Init -------------------- */
bool initCam(){
  camera_config_t c={0};
  c.ledc_channel=LEDC_CHANNEL_0;
  c.ledc_timer  =LEDC_TIMER_0;
  c.pin_d0=Y2_GPIO_NUM; c.pin_d1=Y3_GPIO_NUM;
  c.pin_d2=Y4_GPIO_NUM; c.pin_d3=Y5_GPIO_NUM;
  c.pin_d4=Y6_GPIO_NUM; c.pin_d5=Y7_GPIO_NUM;
  c.pin_d6=Y8_GPIO_NUM; c.pin_d7=Y9_GPIO_NUM;
  c.pin_xclk=XCLK_GPIO_NUM;
  c.pin_pclk=PCLK_GPIO_NUM;
  c.pin_vsync=VSYNC_GPIO_NUM;
  c.pin_href =HREF_GPIO_NUM;
  c.pin_sccb_sda=SIOD_GPIO_NUM;
  c.pin_sccb_scl=SIOC_GPIO_NUM;
  c.pin_pwdn=PWDN_GPIO_NUM;
  c.pin_reset=RESET_GPIO_NUM;

  c.xclk_freq_hz=20000000;
  c.pixel_format=PIXFORMAT_GRAYSCALE;

  bool ps=psramFound();
  if(ps){
    c.frame_size=FRAMESIZE_QVGA;      // 320x240
    c.fb_count=2;
    c.fb_location=CAMERA_FB_IN_PSRAM;
    c.grab_mode=CAMERA_GRAB_LATEST;
  } else {
    c.frame_size=FRAMESIZE_QQVGA;     // 160x120
    c.fb_count=1;
    c.fb_location=CAMERA_FB_IN_DRAM;
    c.grab_mode=CAMERA_GRAB_WHEN_EMPTY;
  }

  framesize_t tries[]={c.frame_size, FRAMESIZE_QVGA, FRAMESIZE_QQVGA};
  for(int i=0;i<3;i++){
    c.frame_size=tries[i];
    if(esp_camera_init(&c)==ESP_OK){
      sensor_t *s=esp_camera_sensor_get();
      if(s) s->set_vflip(s,1);
      applySensorSettings();
      return true;
    }
    esp_camera_deinit();
    delay(100);
  }
  return false;
}

/* -------------------- Setup / Loop -------------------- */
void setup(){
  Serial.begin(115200);
  delay(150);

  if(!loadVisionFromNVS()){
    g_vis.auto_exposure = true;
    g_vis.auto_gain     = true;
    g_vis.aec_value     = 200;
    g_vis.agc_gain      = 20;
    g_vis.brightness    = 2;
    g_vis.contrast      = 2;

    g_vis.manual_thresh = false;
    g_vis.thresh_value  = 220;
    g_vis.percentile    = 0.995f;
    g_vis.thresh_floor  = 180;
    g_vis.min_blob_px   = 8;
    g_vis.ema_alpha     = 0.60f;
    g_vis.move_px       = 2;
    g_vis.release_ms    = 200;

    g_vis.down_confirm_frames = 2;
    g_vis.idle_up_ms          = 400;
    g_vis.idle_tol_hid_px     = 3;

    saveVisionToNVS();
  }

  camOk=initCam();
  Serial.println(camOk?"CAM OK":"CAM FAIL");

  USB.begin();
  Touch.begin();

  WiFi.begin(STA_SSID, STA_PASS);
  WiFi.setSleep(false);
  unsigned long t0=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0<8000){ delay(300); Serial.print("."); }
  Serial.println();
  if(WiFi.status()!=WL_CONNECTED){
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  }

  if(loadCalDataFromNVS()){
    Serial.println("Calibration loaded from NVS.");
  } else {
    Serial.println("No valid calibration in NVS.");
  }

  // Mutex
  g_cam_mutex = xSemaphoreCreateMutex();
  g_jpeg_mutex = xSemaphoreCreateMutex();

  // Server
  setupServer();

  // Kamera-/Erkennung auf Core 1
  xTaskCreatePinnedToCore(cameraTask, "camTask", 8192, nullptr, 2, nullptr, 1);
}

void loop() {
  if(!camOk) { delay(10); return; }

  // CLICK-TEST Sequencer -> Touch down/up
  if (g_hid_right_active) {
    uint32_t now = millis();

    if (g_right_state == 0) {
      Touch.touchDown(g_right_x, g_right_y);
      g_right_ts = now;
      g_right_state = 1;
    }
    else if (g_right_state == 1 && now - g_right_ts >= (uint32_t)g_right_press_ms) {
      Touch.touchUp();
      g_right_ts = now;
      g_right_state = 2;
    }
    else if (g_right_state == 2 && now - g_right_ts >= (uint32_t)g_right_release_ms) {
      g_hid_right_active = false;
      g_right_state = 0;
    }
  }

  // --- Vision-Ergebnis vom Kameratask lesen ---
  bool found = det_found; // volatile -> atomarer read ausreichend
  uint32_t now = millis();

  if (found) {
    latest_has = true;

    if (calibrated && !g_calibrating && display_w > 0 && display_h > 0) {
      // Measurement -> Display-Koordinaten
      float tx, ty;
      H.apply((float)det_cam_x, (float)det_cam_y, tx, ty);
      if(tx<0) tx=0; if(ty<0) ty=0;
      if(tx> (float)display_w-1) tx = (float)display_w-1;
      if(ty> (float)display_h-1) ty = (float)display_h-1;

      // Alpha–Beta Measurement-Update
      unsigned long now_ms = now;
      float dt = (last_update_ms==0) ? 0.0f : (float)(now_ms - last_update_ms)/1000.0f;
      if (!ab.has) {
        ab.x = tx; ab.y = ty;
        ab.vx = 0.0f; ab.vy = 0.0f;
        ab.has = true;
      } else {
        ab.x += ab.vx * dt;
        ab.y += ab.vy * dt;

        float rx = tx - ab.x;
        float ry = ty - ab.y;

        ab.x  += ab.alpha * rx;
        ab.y  += ab.alpha * ry;
        ab.vx += (ab.beta * rx) / (dt > 1e-6f ? dt : 1.0f);
        ab.vy += (ab.beta * ry) / (dt > 1e-6f ? dt : 1.0f);
      }
      last_update_ms = now_ms;

      if (g_hid_enabled) {
        uint16_t hx = (uint16_t)lroundf((ab.x / (float)display_w) * 32767.0f);
        uint16_t hy = (uint16_t)lroundf((ab.y / (float)display_h) * 32767.0f);

        if (!touch_down) {
          if (g_found_streak >= g_vis.down_confirm_frames) {
            // Reset AB -> kein Sprung beim Neuansetzen
            ab.x = tx; ab.y = ty; ab.vx = 0.0f; ab.vy = 0.0f; ab.has = true;
            // Explizites neues Down
            Touch.touchDown(hx, hy);
            touch_down = true;
            last_touch_time = now;
            g_last_move_ms  = now;
            g_last_hx = hx; g_last_hy = hy;
          } else {
            g_found_streak++; // Bestätigung strecken
          }
        } else {
          last_touch_time = now;
        }
      } else {
        if (touch_down) { 
          Touch.touchUp(); 
          touch_down=false; 
          ab.has=false; 
          g_last_hx=-1; g_last_hy=-1; 
          g_last_move_ms=0; 
        }
      }
    }
  }
  else {
    latest_has = false;
    if (touch_down && !g_calibrating && (now - last_touch_time > (uint32_t)g_vis.release_ms)) {
      Touch.touchUp();
      touch_down = false;
      ab.has = false;
      g_last_hx=-1; g_last_hy=-1;
      g_last_move_ms=0;
      g_found_streak = 0;
    }
  }

  // HID-Resampler
  if (ab.has && touch_down && !g_calibrating && g_hid_enabled) {
    unsigned long now_ms2 = millis();
    if (now_ms2 - last_hid_tick >= HID_PERIOD_MS) {
      last_hid_tick = now_ms2;

      float dt_pred = HID_PERIOD_MS / 1000.0f;
      float px = ab.x + ab.vx * dt_pred;
      float py = ab.y + ab.vy * dt_pred;

      if(px<0) px=0; if(py<0) py=0;
      if(px> (float)display_w-1) px = (float)display_w-1;
      if(py> (float)display_h-1) py = (float)display_h-1;

      uint16_t hx = (uint16_t)lroundf((px / (float)display_w) * 32767.0f);
      uint16_t hy = (uint16_t)lroundf((py / (float)display_h) * 32767.0f);

      Touch.touchMove(hx, hy);

      g_last_move_ms = now_ms2;
      g_last_hx = (int)hx; g_last_hy = (int)hy;
    }

    // Idle-Up
    if (g_last_hx >= 0 && g_last_hy >= 0) {
      uint16_t cur_hx = (uint16_t)lroundf((ab.x / (float)display_w) * 32767.0f);
      uint16_t cur_hy = (uint16_t)lroundf((ab.y / (float)display_h) * 32767.0f);

      int ddx = (int)cur_hx - g_last_hx;
      int ddy = (int)cur_hy - g_last_hy;

      if (abs(ddx) <= g_vis.idle_tol_hid_px && abs(ddy) <= g_vis.idle_tol_hid_px) {
        if (millis() - g_last_move_ms >= (unsigned long)g_vis.idle_up_ms) {
          Touch.touchUp();
          touch_down = false;
          ab.has = false;
          g_last_hx=-1; g_last_hy=-1;
          g_last_move_ms=0;
        }
      }
    }
  } else {
    last_hid_tick = millis();
  }

  // Loop entlasten
  delay(1);
}
