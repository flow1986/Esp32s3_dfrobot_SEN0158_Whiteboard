
/*
  ESP32-S3-EYE Whiteboard - Full (ASCII-safe)
  - TinyUSB HID Touch (Android-kompatibel, Single-Finger)
  - IR-Blob (GRAYSCALE) mit konfigurierbarer Helligkeits-/Erkennungsroutine
  - Auto-Calibration (4 Points, Fullscreen Crosshair) mit 5s Halte-Timer
  - Calibration Preview: Camera (links) + Board/Homography (rechts)
  - Snapshot Camera Preview (Pull-based, optional)
  - Click Test (Press/Release Time) -> Touch down/up
  - NVS persistent storage (calibration + vision)
  - HID-Output per WebGUI schaltbar (ohne Disconnect)
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
int g_right_release_ms = 100;

/* -------------------- Homography -------------------- */
class Homography {
  double m[9];
  static bool solve8x8(double A[8][9], double out[8]) {
    for(int i=0;i<8;i++){
      int piv=i;
      for(int r=i+1;r<8;r++)
        if (fabs(A[r][i])>fabs(A[piv][i])) piv=r;
      if (fabs(A[piv][i])<1e-12) return false;
      if (piv!=i) for(int c=i;c<=8;c++){ double t=A[i][c]; A[i][c]=A[piv][c]; A[piv][c]=t; }
      double div=A[i][i];
      for(int c=i;c<=8;c++) A[i][c]/=div;
      for(int r=0;r<8;r++){
        if (r==i) continue;
        double f=A[r][i];
        for(int c=i;c<=8;c++) A[r][c]-=f*A[i][c];
      }
    }
    for(int i=0;i<8;i++) out[i]=A[i][8];
    return true;
  }
public:
  Homography(){ setIdentity(); }
  void setIdentity(){ double I[9]={1,0,0, 0,1,0, 0,0,1}; memcpy(m,I,sizeof(I)); }
  void setMatrix(const double in[9]){ memcpy(m,in,9*sizeof(double)); }
  void getMatrix(double out[9]) const { memcpy(out,m,9*sizeof(double)); }
  bool compute(const double src[8], const double dst[8]){
    double A[8][9]={0};
    for(int i=0;i<4;i++){
      double x=src[2*i], y=src[2*i+1];
      double u=dst[2*i], v=dst[2*i+1];
      A[2*i][0]=x; A[2*i][1]=y; A[2*i][2]=1;
      A[2*i][6]=-u*x; A[2*i][7]=-u*y; A[2*i][8]=u;
      A[2*i+1][3]=x; A[2*i+1][4]=y; A[2*i+1][5]=1;
      A[2*i+1][6]=-v*x; A[2*i+1][7]=-v*y; A[2*i+1][8]=v;
    }
    double sol[8];
    if(!solve8x8(A, sol)) return false;
    m[0]=sol[0]; m[1]=sol[1]; m[2]=sol[2];
    m[3]=sol[3]; m[4]=sol[4]; m[5]=sol[5];
    m[6]=sol[6]; m[7]=sol[7]; m[8]=1.0;
    return true;
  }
  void apply(double x,double y,double &ox,double &oy) const {
    double den = m[6]*x + m[7]*y + m[8];
    if (fabs(den)<1e-12) { ox=oy=0; return; }
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
    0x05, 0x0D,              // Usage Page (Digitizer)
    0x09, 0x04,              // Usage (Touch Screen)
    0xA1, 0x01,              // Collection (Application)
      0x85, 0x01,            //   Report ID (1)
      0x09, 0x22,            //   Usage (Finger)
      0xA1, 0x02,            //   Collection (Logical)
        0x09, 0x42,          //     Usage (Tip Switch)
        0x09, 0x32,          //     Usage (In Range)
        0x15, 0x00,          //     Logical Min (0)
        0x25, 0x01,          //     Logical Max (1)
        0x75, 0x01,          //     Report Size (1)
        0x95, 0x02,          //     Report Count (2)
        0x81, 0x02,          //     Input (Data,Var,Abs)
        0x75, 0x06,          //     Report Size (6) - Padding
        0x95, 0x01,          //     Report Count (1)
        0x81, 0x03,          //     Input (Const,Var,Abs)
        0x09, 0x51,          //     Usage (Contact Identifier)
        0x15, 0x00,          //     Logical Min (0)
        0x25, 0x7F,          //     Logical Max (127)
        0x75, 0x08,          //     Report Size (8)
        0x95, 0x01,          //     Report Count (1)
        0x81, 0x02,          //     Input (Data,Var,Abs)
        0x05, 0x01,          //     Usage Page (Generic Desktop)
        0x09, 0x30,          //     Usage (X)
        0x09, 0x31,          //     Usage (Y)
        0x16, 0x00, 0x00,    //     Logical Min (0)
        0x26, 0xFF, 0x7F,    //     Logical Max (32767)
        0x75, 0x10,          //     Report Size (16)
        0x95, 0x02,          //     Report Count (2)
        0x81, 0x02,          //     Input (Data,Var,Abs)
      0xC0,                  //   End Collection (Finger)
      0x09, 0x54,            //   Usage (Contact Count)
      0x15, 0x00,            //   Logical Min (0)
      0x25, 0x0A,            //   Logical Max (10)
      0x75, 0x08,            //   Report Size (8)
      0x95, 0x01,            //   Report Count (1)
      0x81, 0x02,            //   Input (Data,Var,Abs)
    0xC0                     // End Collection
  };
};

HIDTouchDevice Touch;

/* -------------------- NVS/Preferences (Calibration) -------------------- */
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
  double   Hm[9];
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
  int   release_ms    = 50;

  int   last_thresh   = -1;
  int   last_blob_n   = 0;
} g_vis;

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
  if(sz != sizeof(g_vis)){ prefs.end(); return false; }
  VisionConfig tmp;
  size_t rd = prefs.getBytes("vision", &tmp, sizeof(tmp));
  prefs.end();
  if(rd != sizeof(tmp)) return false;
  g_vis = tmp;
  return true;
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

/* -------------------- IR Blob Detection + Smoothing -------------------- */
bool   filt_has=false;
double filt_x=0.0, filt_y=0.0;

/* Median(5) Puffer */
static const int MED_N = 5;
int med_x_buf[MED_N] = {0};
int med_y_buf[MED_N] = {0};
int med_idx = 0;
int med_count = 0;

static int median5(int *buf, int n){
  int t[MED_N];
  for(int i=0;i<n;i++) t[i]=buf[i];
  for(int i=1;i<n;i++){
    int k=t[i]; int j=i-1;
    while(j>=0 && t[j]>k){ t[j+1]=t[j]; j--; }
    t[j+1]=k;
  }
  return t[n/2];
}

/* Alpha–Beta Filter + Resampler (Display-Koordinaten) */
struct ABFilter {
  double x=0, y=0;
  double vx=0, vy=0;
  bool   has=false;
  double alpha=0.35;
  double beta =0.015;
} ab;

unsigned long last_update_ms = 0;
unsigned long last_hid_tick  = 0;
const uint32_t HID_PERIOD_MS = 8; // ~125 Hz

bool detectIR(camera_fb_t *fb,int &ox,int &oy){
  if(!fb || fb->format!=PIXFORMAT_GRAYSCALE) return false;
  int w=fb->width,h=fb->height;
  latest_cam_w=w; latest_cam_h=h;

  // Histogram
  int hist[256]={0};
  size_t scans=min((size_t)fb->len,(size_t)w*h);
  const uint8_t* buf = fb->buf;
  for(size_t i=0;i<scans;i++) hist[buf[i]]++;

  // Threshold
  int th;
  if(g_vis.manual_thresh){
    th = clampi(g_vis.thresh_value, 0, 255);
  } else {
    int total = w*h, acc = 0;
    int target = (int)roundf(clampf(g_vis.percentile, 0.90f, 0.9999f) * total);
    th = 255;
    for(int v=255; v>=0; --v){
      acc += hist[v];
      if(acc >= target){ th = v; break; }
    }
    if(th < g_vis.thresh_floor) th = g_vis.thresh_floor;
  }

  // Blob-Zentroid
  uint64_t sx=0, sy=0, cnt=0;
  for(int y=0;y<h;y++){
    int row=y*w;
    for(int x=0;x<w;x++){
      size_t idx=(size_t)row+x;
      if(idx>=scans) break;
      if(buf[idx] >= th){ sx+=x; sy+=y; cnt++; }
    }
  }

  g_vis.last_thresh = th;
  g_vis.last_blob_n = (int)cnt;

  if((int)cnt < g_vis.min_blob_px) return false;

  int cx = (int)(sx/cnt);
  int cy = (int)(sy/cnt);

  // Median(5) -> jitterfreiere Werte
  med_x_buf[med_idx] = cx;
  med_y_buf[med_idx] = cy;
  med_idx = (med_idx + 1) % MED_N;
  if (med_count < MED_N) med_count++;
  int mx = (med_count >= MED_N) ? median5(med_x_buf, med_count) : cx;
  int my = (med_count >= MED_N) ? median5(med_y_buf, med_count) : cy;

  // EMA + Bewegungsschwelle
  if(!filt_has){
    filt_x=mx; filt_y=my; filt_has=true;
  } else {
    int dx = mx - (int)lround(filt_x);
    int dy = my - (int)lround(filt_y);
    if((dx*dx + dy*dy) >= (g_vis.move_px*g_vis.move_px)){
      float a = clampf(g_vis.ema_alpha, 0.0f, 1.0f);
      filt_x = a*filt_x + (1.0f-a)*mx;
      filt_y = a*filt_y + (1.0f-a)*my;
    }
  }

  ox = (int)lround(filt_x);
  oy = (int)lround(filt_y);
  return true;
}

/* -------------------- Kamera Init (GRAYSCALE) -------------------- */
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

/* -------------------- Snapshot (JPEG) -------------------- */
void handleSnapshot(AsyncWebServerRequest* req){
  camera_fb_t *fb = esp_camera_fb_get();
  if(!fb){
    req->send(503, "text/plain", "no frame");
    return;
  }
  uint8_t* jpg = nullptr;
  size_t   jlen = 0;
  bool ok = frame2jpg(fb, 45 /*Qualitaet*/, &jpg, &jlen);
  esp_camera_fb_return(fb);
  if(!ok || !jpg || jlen == 0){
    if(jpg) free(jpg);
    req->send(500,"text/plain","jpg fail");
    return;
  }
  AsyncResponseStream *res = req->beginResponseStream("image/jpeg");
  res->write(jpg, jlen);
  req->send(res);
  free(jpg);
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
.col{border:1px solid #aaa;padding:8px}
canvas,img{border:1px solid #333}
table{border-collapse:collapse}
td,th{border:1px solid #ccc;padding:4px 6px}
input[type=number]{width:80px}
small{opacity:.8}
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
    </div>
  </div>

  <div class="col">
    <h3>Auto Calibration</h3>
    <button id="btnStartCal">Start Calibration</button>
    <button id="btnStopCal" disabled>Stop</button>
    <div><small>4-Punkt Autokalibrierung (Fullscreen). Stift 5s ruhig halten.</small></div>
  </div>

  <div class="col">
    <h3>Click Test</h3>
    X:<input id="hidX" type="number" value="200"><br>
    Y:<input id="hidY" type="number" value="200"><br>
    Press ms:<input id="pressMs" type="number" value="150"><br>
    Release ms:<input id="releaseMs" type="number" value="100"><br>
    <button id="btnClick">Send</button>
    <div id="clickMsg"></div>
  </div>

  <div class="col">
    <h3>Camera Preview</h3>
    <label><input type="checkbox" id="camToggle"> Enable</label><br>
    <img id="camView" style="max-width:300px;display:none;margin-top:8px;">
    <div><small>Snapshot-Modus. OFF = keine CPU-Last.</small></div>
  </div>

  <div class="col">
    <h3>Calibration Preview</h3>

    <div style="display:flex; gap:10px; align-items:flex-start; flex-wrap:wrap;">
      <!-- Linke Preview: Camera Space -->
      <div>
        <div style="font-weight:bold; margin-bottom:4px;">Camera</div>
        <canvas id="calCanvasCam" width="300" height="225"></canvas>
        <div style="font-size:12px; opacity:.8; margin-top:4px;">
          Blau = Cam-Punkte (roh), Gelb = IR (roh)
        </div>
      </div>

      <!-- Rechte Preview: Board/Homography Space -->
      <div>
        <div style="font-weight:bold; margin-bottom:4px;">Board / Homography</div>
        <canvas id="calCanvasH" width="300" height="170"></canvas>
        <div style="font-size:12px; opacity:.8; margin-top:4px;">
          Rot = Soll (Board), Blau = Ist (Cam->H), Grün = IR (H)
        </div>
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
  </div>

  <div class="col">
    <h3>Display Size</h3>
    W:<input id="dW" type="number" value="1920">
    H:<input id="dH" type="number" value="1080"><br>
    <button id="btnSetDisp">Set Display</button>
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
    </fieldset>
    <fieldset>
      <legend>Erkennung</legend>
      <label><input type="checkbox" id="manThresh"> Manuelle Schwelle</label>
      <span id="autoBadge" class="badge">auto</span><br>
      Thresh:<input id="thresh" type="number" value="220" min="0" max="255">
      Floor:<input id="floor" type="number" value="180" min="0" max="255"><br>
      Percentile:<input id="perc" type="number" value="0.995" step="0.0005" min="0.90" max="0.9999">
      MinBlob(px):<input id="minBlob" type="number" value="8" min="1" max="500">
      <br><small>Last: thresh=<span id="lastTh">-</span>, blobPx=<span id="lastBlob">-</span></small>
    </fieldset>
    <fieldset>
      <legend>Debounce / Filter</legend>
      EMA alpha:<input id="ema" type="number" value="0.60" step="0.05" min="0" max="1">
      Move(px):<input id="move" type="number" value="2" min="0" max="50"><br>
      Release(ms):<input id="relms" type="number" value="50" min="0" max="500">
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
    Corner 1/4 - hold 0.0 / 5.0 s
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
  // Board/Homography (rechts) -> Display-AR
  const baseW = 300;
  calCanvasH.width = baseW;
  calCanvasH.height = Math.max(140, Math.round(baseW * (display_h / display_w)));

  // Camera (links) -> Kamera-AR (falls bekannt)
  const cw = last_cam_w > 0 ? last_cam_w : 320;
  const ch = last_cam_h > 0 ? last_cam_h : 240;
  calCanvasCam.width = baseW;
  calCanvasCam.height = Math.max(140, Math.round(baseW * (ch / cw)));
}
adjustCalCanvasSize();

// ---------- Display-Inputs: beim Tippen nicht überschreiben ----------
[dW, dH].forEach(inp=>{
  inp.addEventListener('focus', ()=> editingDisplay=true);
  inp.addEventListener('blur',  ()=> editingDisplay=false);
});

// ---------- Displaygröße aus dem Overlay (Fullscreen) lesen ----------
function getOverlaySizePx(){
  const rect = calOverlay.getBoundingClientRect();
  const w = Math.round(rect.width);
  const h = Math.round(rect.height);
  return {w, h};
}
async function setDisplayFromOverlay(){
  const {w, h} = getOverlaySizePx();
  if (w > 15 && h > 15 && (w !== display_w || h !== display_h)){
    try{
      const r = await fetch('/api/set_display',{
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify({w,h})
      });
      const j = await r.json();
      if(j.ok){
        display_w=j.w; display_h=j.h;
        disp.innerText=`${j.w}x${j.h}`;
        if(!editingDisplay){ dW.value=display_w; dH.value=display_h; }
        adjustCalCanvasSize();
        refreshCrossPosition();
      }
    }catch(e){}
  }
}

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
    camTimer=setInterval(()=>{ camView.src='/api/cam_snapshot?ts='+Date.now(); },250);
  } else {
    if(camTimer) clearInterval(camTimer);
    camTimer=null; camView.style.display='none'; camView.src='';
  }
};

// ---------- Calibration Preview ----------
function drawCamPreview(camRaw, pen) {
  const w = calCanvasCam.width, h = calCanvasCam.height;
  ctxCam.clearRect(0,0,w,h);
  ctxCam.fillStyle = '#f4f4f4'; ctxCam.fillRect(0,0,w,h);
  ctxCam.strokeStyle='#000'; ctxCam.lineWidth=2; ctxCam.strokeRect(0,0,w,h);

  const camW = (pen && pen.camW) ? pen.camW : last_cam_w;
  const camH = (pen && pen.camH) ? pen.camH : last_cam_h;
  const sx = w / camW, sy = h / camH;

  if (Array.isArray(camRaw) && camRaw.length >= 3) {
    ctxCam.strokeStyle = 'rgba(0,51,204,1)';
    ctxCam.fillStyle   = 'rgba(0,51,204,0.08)';
    ctxCam.beginPath();
    camRaw.forEach((p,i)=>{
      const x = p.x * sx, y = p.y * sy;
      if(i===0) ctxCam.moveTo(x,y); else ctxCam.lineTo(x,y);
    });
    ctxCam.closePath();
    ctxCam.fill(); ctxCam.stroke();

    ctxCam.fillStyle = 'rgba(0,51,204,1)';
    camRaw.forEach(p=>{
      const x = p.x * sx, y = p.y * sy;
      ctxCam.beginPath(); ctxCam.arc(x,y,4,0,2*Math.PI); ctxCam.fill();
    });
  }

  if (pen && pen.has && camW && camH) {
    const rx = pen.camX * sx, ry = pen.camY * sy;
    ctxCam.fillStyle='gold'; ctxCam.beginPath(); ctxCam.arc(rx,ry,4,0,2*Math.PI); ctxCam.fill();
    ctxCam.strokeStyle='orange'; ctxCam.beginPath(); ctxCam.arc(rx,ry,7,0,2*Math.PI); ctxCam.stroke();

    ctxCam.fillStyle='#444'; ctxCam.font='10px Arial';
    ctxCam.fillText(`Cam: ${camW}x${camH}`, 6, 12);
  }
}

function drawHomographyPreview(camT, brd, pen){
  const w=calCanvasH.width, h=calCanvasH.height;
  ctxH.clearRect(0,0,w,h);
  ctxH.fillStyle='#eee'; ctxH.fillRect(0,0,w,h);
  ctxH.strokeStyle='#000'; ctxH.lineWidth=2; ctxH.strokeRect(0,0,w,h);

  const sx=w/display_w, sy=h/display_h;

  if (Array.isArray(brd) && brd.length >= 3) {
    ctxH.strokeStyle='rgba(179,0,0,1)';
    ctxH.fillStyle  ='rgba(255,0,0,0.12)';
    ctxH.beginPath();
    brd.forEach((p,i)=>{
      const x = p.x*sx, y = p.y*sy;
      if(i===0) ctxH.moveTo(x,y); else ctxH.lineTo(x,y);
    });
    ctxH.closePath(); ctxH.fill(); ctxH.stroke();

    ctxH.fillStyle='rgba(179,0,0,1)';
    brd.forEach(p=>{
      const x=p.x*sx, y=p.y*sy;
      ctxH.beginPath(); ctxH.arc(x,y,4,0,2*Math.PI); ctxH.fill();
    });
  }

  if (Array.isArray(camT) && camT.length >= 3) {
    ctxH.strokeStyle='rgba(0,51,204,1)';
    ctxH.fillStyle  ='rgba(0,51,204,0.08)';
    ctxH.beginPath();
    camT.forEach((p,i)=>{
      const x=p.x*sx, y=p.y*sy;
      if(i===0) ctxH.moveTo(x,y); else ctxH.lineTo(x,y);
    });
    ctxH.closePath(); ctxH.fill(); ctxH.stroke();

    ctxH.fillStyle='rgba(0,51,204,1)';
    camT.forEach(p=>{
      const x=p.x*sx, y=p.y*sy;
      ctxH.beginPath(); ctxH.arc(x,y,4,0,2*Math.PI); ctxH.fill();
    });
  }

  if (pen && pen.has) {
    const px = pen.x * sx, py = pen.y * sy;
    ctxH.fillStyle='limegreen'; ctxH.beginPath(); ctxH.arc(px,py,5,0,2*Math.PI); ctxH.fill();
    ctxH.strokeStyle='green'; ctxH.beginPath(); ctxH.arc(px,py,9,0,2*Math.PI); ctxH.stroke();
  }
}

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
btnSetDisp.onclick=async()=>{
  const w=+dW.value,h=+dH.value;
  const r=await fetch('/api/set_display',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({w,h})});
  const j=await r.json(); 
  if(j.ok){ 
    display_w=j.w; display_h=j.h; 
    disp.innerText=`${j.w}x${j.h}`; 
    adjustCalCanvasSize();
    refreshCrossPosition(); 
  }
};

// ---------- Vision: UI-Helper, Load/Apply ----------
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
  relms.value = v.release_ms ?? 50;

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
    release_ms:    Number(relms.value)
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
const HOLD_MS = 5000;

let calStep=0, calStable=0, lastCamPos={x:0,y:0};
let calCamPts=[], calBrdPts=[];
let dwellMs = 0;
let lastTickTs = null;

document.addEventListener('fullscreenchange', async ()=>{
  if (calOverlay.style.display === 'block'){
    await new Promise(r=>setTimeout(r, 50));
    await setDisplayFromOverlay();
    refreshCrossPosition();
  }
});
window.addEventListener('resize', async ()=>{
  if (calOverlay.style.display === 'block'){
    await new Promise(r=>setTimeout(r, 50));
    await setDisplayFromOverlay();
    refreshCrossPosition();
  } else {
    adjustCalCanvasSize();
  }
});

// Robustes Beenden der Kalibrierung
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

// Stop-Button
btnStopCal.onclick = endCalibration;

// Start-Button
btnStartCal.onclick=async()=>{
  if(!document.fullscreenElement) await document.documentElement.requestFullscreen();
  calOverlay.style.display='block'; btnStartCal.disabled=true; btnStopCal.disabled=false;

  await new Promise(r=>setTimeout(r, 50));
  await setDisplayFromOverlay();

  calStep=0; calStable=0; lastCamPos={x:0,y:0};
  calCamPts=[]; calBrdPts=[];
  dwellMs = 0; lastTickTs = performance.now();

  await fetch('/api/calib_mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({on:true})});

  moveCross(screenTarget(0));
  calMsg.innerText='Corner 1/4 - hold 0.0 / 5.0 s';

  calTimer=setInterval(async()=>{
    const now = performance.now();
    const dt = lastTickTs ? (now - lastTickTs) : 120;
    lastTickTs = now;

    const r=await fetch('/api/coords'); const j=await r.json();
    if(!j.ok){
      dwellMs = 0;
      calMsg.innerText = `Corner ${calStep+1}/4 - hold 0.0 / 5.0 s`;
      return;
    }
    const cx=j.camera.x, cy=j.camera.y;

    if(Math.abs(cx-lastCamPos.x)<4 && Math.abs(cy-lastCamPos.y)<4){
      dwellMs += dt;
    } else {
      dwellMs = 0;
    }
    lastCamPos={x:cx,y:cy};

    const shown = Math.min(5.0, dwellMs/1000).toFixed(1);
    calMsg.innerText = `Corner ${calStep+1}/4 - hold ${shown} / 5.0 s`;

    if(dwellMs >= HOLD_MS){
      calCamPts.push({x:cx,y:cy});
      calBrdPts.push(screenTarget(calStep));
      calStep++; dwellMs=0;

      if(calStep>=4){ await finishCalibration(); return; }

      moveCross(screenTarget(calStep));
      calMsg.innerText = `Corner ${calStep+1}/4 - hold 0.0 / 5.0 s`;
    }
  },120);
};

// Abschluss: Speichern & immer sauber beenden
async function finishCalibration(){
  if (calTimer) { clearInterval(calTimer); calTimer = null; }
  try {
    await fetch('/api/save_calibration',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({camera:calCamPts,board:calBrdPts})
    });
  } finally {
    await endCalibration();
  }
}
</script>
</body>
</html>
)HTML";

/* -------------------- Server & APIs -------------------- */
void setupServer(){

  // Root UI
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200,"text/html",INDEX_HTML);
  });

  // Snapshot
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
      double tx,ty; H.apply(latest_cam_x,latest_cam_y,tx,ty);
      JsonObject tr=doc.createNestedObject("trans");
      tr["x"]=(int)lround(tx); tr["y"]=(int)lround(ty);
    }
    JsonObject d=doc.createNestedObject("display");
    d["w"]=display_w; d["h"]=display_h;
    doc["calibrated"]=calibrated;
    doc["hidEnabled"]=g_hid_enabled;
    String out; serializeJson(doc,out);
    request->send(200,"application/json",out);
  });

  // Display set
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/set_display",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      JsonObject o=json.as<JsonObject>();
      uint32_t w=o["w"]|0, h=o["h"]|0;
      if(w<16||h<16){ request->send(400,"application/json","{\"ok\":false}"); return; }
      display_w=w; display_h=h;
      if(calibrated) saveCalDataToNVS();
      char buf[64]; snprintf(buf,sizeof(buf),"{\"ok\":true,\"w\":%u,\"h\":%u}",w,h);
      request->send(200,"application/json",buf);
    }
  ));

  // HID enable/disable
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/set_hid",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      bool en = json["enabled"] | true;
      g_hid_enabled = en;
      char buf[64]; snprintf(buf,sizeof(buf),"{\"ok\":true,\"enabled\":%s}", en?"true":"false");
      request->send(200,"application/json",buf);
    }
  ));

  // Click Test
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/test_click",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      int px=json["x"]|0, py=json["y"]|0;
      g_right_press_ms   = json["press"] | 150;
      g_right_release_ms = json["rel"]   | 100;

      if(px<0)px=0; if(py<0)py=0;
      if((uint32_t)px>=display_w) px=display_w-1;
      if((uint32_t)py>=display_h) py=display_h-1;

      uint16_t hx=(uint16_t)lround(px/(double)display_w*32767.0);
      uint16_t hy=(uint16_t)lround(py/(double)display_h*32767.0);

      g_right_x=hx; g_right_y=hy;
      g_hid_right_active=true;
      g_right_state=0;

      request->send(200,"application/json","{\"ok\":true}");
    }
  ));

  // Calibration preview data (jetzt mit camRaw)
  server.on("/api/get_cal_preview", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<1152> doc;
    JsonArray camT = doc.createNestedArray("camT");
    JsonArray brd  = doc.createNestedArray("brd");
    JsonArray camR = doc.createNestedArray("camRaw");

    for(int i=0;i<4;i++){
      JsonObject b=brd.createNestedObject(); 
      b["x"]=g_board_pts[i].x; 
      b["y"]=g_board_pts[i].y;

      double tx,ty; 
      H.apply(g_cam_pts[i].x,g_cam_pts[i].y,tx,ty);
      JsonObject c=camT.createNestedObject(); 
      c["x"]=(int)lround(tx); 
      c["y"]=(int)lround(ty);

      JsonObject cr=camR.createNestedObject();
      cr["x"]=g_cam_pts[i].x;
      cr["y"]=g_cam_pts[i].y;
    }

    JsonObject pen = doc.createNestedObject("pen");
    pen["has"] = latest_has;
    if(latest_has){
      double tx, ty;
      H.apply(latest_cam_x, latest_cam_y, tx, ty);
      if(tx < 0) tx = 0; if(ty < 0) ty = 0;
      if(tx > (double)display_w-1) tx = (double)display_w-1;
      if(ty > (double)display_h-1) ty = (double)display_h-1;
      pen["x"] = (int)lround(tx);
      pen["y"] = (int)lround(ty);
      pen["camX"] = latest_cam_x;
      pen["camY"] = latest_cam_y;
      pen["camW"] = latest_cam_w;
      pen["camH"] = latest_cam_h;
    }

    JsonObject pi = doc.createNestedObject("penInfo");
    pi["lastThresh"] = g_vis.last_thresh;
    pi["lastBlob"]   = g_vis.last_blob_n;

    String out; serializeJson(doc,out);
    request->send(200,"application/json",out);
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
    String out; serializeJson(doc,out);
    request->send(200,"application/json",out);
  });

  // Manual set points
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/set_cal_points",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      JsonArray cam=json["camera"];
      JsonArray brd=json["board"];
      if(cam.size()<4||brd.size()<4){ request->send(400,"application/json","{\"ok\":false}"); return; }
      double s[8],d[8];
      for(int i=0;i<4;i++){
        g_cam_pts[i].x=cam[i]["x"] | 0; g_cam_pts[i].y=cam[i]["y"] | 0;
        g_board_pts[i].x=brd[i]["x"] | 0; g_board_pts[i].y=brd[i]["y"] | 0;
        s[2*i]=g_cam_pts[i].x; s[2*i+1]=g_cam_pts[i].y;
        d[2*i]=g_board_pts[i].x; d[2*i+1]=g_board_pts[i].y;
      }
      Homography Hn; if(!Hn.compute(s,d)){ request->send(500,"application/json","{\"ok\":false}"); return; }
      double M[9]; Hn.getMatrix(M); H.setMatrix(M);
      calibrated=true; saveCalDataToNVS();
      request->send(200,"application/json","{\"ok\":true}");
    }
  ));

  // Auto-calibration alias
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/save_calibration",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      JsonArray cam=json["camera"]; JsonArray brd=json["board"];
      if(cam.size()<4||brd.size()<4){ request->send(400,"application/json","{\"ok\":false}"); return; }
      double s[8],d[8];
      for(int i=0;i<4;i++){
        g_cam_pts[i].x=cam[i]["x"] | 0; g_cam_pts[i].y=cam[i]["y"] | 0;
        g_board_pts[i].x=brd[i]["x"] | 0; g_board_pts[i].y=brd[i]["y"] | 0;
        s[2*i]=g_cam_pts[i].x; s[2*i+1]=g_cam_pts[i].y;
        d[2*i]=g_board_pts[i].x; d[2*i+1]=g_board_pts[i].y;
      }
      Homography Hn; if(!Hn.compute(s,d)){ request->send(500,"application/json","{\"ok\":false}"); return; }
      double M[9]; Hn.getMatrix(M); H.setMatrix(M);
      calibrated=true; saveCalDataToNVS();
      request->send(200,"application/json","{\"ok\":true}");
    }
  ));

  // Calibration mode (Flag)
  server.addHandler(new AsyncCallbackJsonWebHandler("/api/calib_mode",
    [](AsyncWebServerRequest *request, JsonVariant &json){
      g_calibrating = json["on"] | false;
      request->send(200,"application/json","{\"ok\":true}");
    }
  ));

  // Clear calibration
  server.on("/api/clear_cal", HTTP_POST, [](AsyncWebServerRequest *request){
    if(prefs.begin(NVS_NS,false)){ prefs.remove(NVS_KEY); prefs.end(); }
    calibrated=false; H.setIdentity();
    for(int i=0;i<4;i++){ g_cam_pts[i]={0,0}; g_board_pts[i]={0,0}; }
    request->send(200,"application/json","{\"ok\":true}");
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

    doc["last_thresh"]=g_vis.last_thresh;
    doc["last_blob"]=g_vis.last_blob_n;

    String out; serializeJson(doc,out);
    request->send(200,"application/json",out);
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
      g_vis.release_ms    = clampi(o["release_ms"]  |g_vis.release_ms, 0, 1000);

      applySensorSettings();
      saveVisionToNVS();

      request->send(200,"application/json","{\"ok\":true}");
    }
  ));

  server.begin();
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
    g_vis.release_ms    = 50;

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

  setupServer();
}

void loop() {
  if(!camOk) return;

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

  // FRAME HOLEN
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  latest_cam_w = fb->width;
  latest_cam_h = fb->height;

  int cx, cy;
  bool found = detectIR(fb, cx, cy);
  esp_camera_fb_return(fb);

  uint32_t now = millis();

  if (found) {
    latest_cam_x = cx;
    latest_cam_y = cy;
    latest_has   = true;

    if (calibrated && !g_calibrating && display_w > 0 && display_h > 0) {
      // Measurement -> Display-Koordinaten (mit Clamps)
      double tx, ty;
      H.apply(cx, cy, tx, ty);
      if(tx<0) tx=0; if(ty<0) ty=0;
      if(tx> (double)display_w-1) tx = (double)display_w-1;
      if(ty> (double)display_h-1) ty = (double)display_h-1;

      // Alpha–Beta Measurement-Update
      unsigned long now_ms = now;
      double dt = (last_update_ms==0) ? 0.0 : (now_ms - last_update_ms)/1000.0;
      if (!ab.has) {
        ab.x = tx; ab.y = ty;
        ab.vx = 0.0; ab.vy = 0.0;
        ab.has = true;
      } else {
        ab.x += ab.vx * dt;
        ab.y += ab.vy * dt;

        double rx = tx - ab.x;
        double ry = ty - ab.y;

        ab.x  += ab.alpha * rx;
        ab.y  += ab.alpha * ry;
        ab.vx += (ab.beta * rx) / (dt > 1e-6 ? dt : 1.0);
        ab.vy += (ab.beta * ry) / (dt > 1e-6 ? dt : 1.0);
      }
      last_update_ms = now_ms;

      if (g_hid_enabled) {
        touch_down = true;
        last_touch_time = now_ms;
      } else {
        if (touch_down) { Touch.touchUp(); touch_down=false; }
      }
    }
  }
  else {
    latest_has = false;
    if (touch_down && !g_calibrating && (now - last_touch_time > (uint32_t)g_vis.release_ms)) {
      Touch.touchUp();
      touch_down = false;
    }
  }

  // HID-Resampler: feste Rate (~125 Hz), auch ohne neues Frame
  if (ab.has && touch_down && !g_calibrating && g_hid_enabled) {
    unsigned long now_ms2 = millis();
    if (now_ms2 - last_hid_tick >= HID_PERIOD_MS) {
      last_hid_tick = now_ms2;

      double dt_pred = HID_PERIOD_MS / 1000.0;
      double px = ab.x + ab.vx * dt_pred;
      double py = ab.y + ab.vy * dt_pred;

      if(px<0) px=0; if(py<0) py=0;
      if(px> (double)display_w-1) px = (double)display_w-1;
      if(py> (double)display_h-1) py = (double)display_h-1;

      uint16_t hx = (uint16_t)lround((px / (double)display_w) * 32767.0);
      uint16_t hy = (uint16_t)lround((py / (double)display_h) * 32767.0);

      Touch.touchMove(hx, hy);
    }
  } else {
    last_hid_tick = millis();
  }
}
