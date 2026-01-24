
#include "esp_camera.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include "USB.h"
#include "USBHIDDigitizer.h"
#include <cmath>
#include <cstring>
#include <algorithm> // std::max

// -------------------- Kamera-Pins: ESP32-S3-EYE --------------------
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

USBHIDDigitizer Digitizer;
AsyncWebServer server(80);
Preferences prefs;

// -------------------- Persistente Settings --------------------
#pragma pack(push,1)
struct Settings {
  uint32_t version;          // Versionierung
  double   H[9];             // Homographie
  int32_t  screenW;          // Kalibrier-Screenbreite
  int32_t  screenH;          // Kalibrier-Screenhöhe

  // IR / Belichtung
  int32_t  irThreshold;      // Basis-Schwelle (wenn autoThreshold=0)
  int32_t  exposureValue;    // Belichtungswert (manuell)
  uint8_t  manualExposure;   // 1=manuell, 0=AEC

  // Spot & Glättung
  uint8_t  useCentroid;      // 1=Spot-Schwerpunkt aktiv
  int32_t  spotWindow;       // 3,5,7,9,...
  uint8_t  useSmoothing;     // 1=EMA aktiv
  int32_t  emaAlphaPct;      // 0..100 (EMA alpha in %)

  // Auto-Umgebungslicht & Hysterese
  uint8_t  autoThreshold;    // 1=Auto-Threshold aktiv
  int32_t  ambientOffset;    // Offset addiert zur Ambient-Basis (0..128)
  int32_t  threshFloor;      // Mindestschwelle (0..255)
  int32_t  ambAlphaPct;      // 0..100 (EMA auf Ambient)
  int32_t  hystGap;          // Gap für Hysterese (on = base+gap, off = base-gap)

  uint8_t  reserved[2];      // Reserve
  uint32_t crc32;            // CRC über die Struktur
};
#pragma pack(pop)

static const uint32_t SETTINGS_VERSION = 3;
static const char*    NVS_NS  = "whiteboard";
static const char*    NVS_KEY = "cfg";

Settings cfg;

// CRC32
static uint32_t crc32_calc(const uint8_t* data, size_t len, uint32_t crc = 0xFFFFFFFF) {
  for (size_t i = 0; i < len; ++i) {
    uint32_t c = (crc ^ data[i]) & 0xFFU;
    for (int k = 0; k < 8; ++k) c = (c & 1U) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
    crc = (crc >> 8) ^ c;
  }
  return crc ^ 0xFFFFFFFFU;
}
static void cfg_set_defaults() {
  std::memset(&cfg, 0, sizeof(cfg));
  cfg.version = SETTINGS_VERSION;
  cfg.H[0]=1; cfg.H[1]=0; cfg.H[2]=0; cfg.H[3]=0; cfg.H[4]=1; cfg.H[5]=0; cfg.H[6]=0; cfg.H[7]=0; cfg.H[8]=1;

  cfg.screenW = 1920; cfg.screenH = 1080;

  cfg.irThreshold   = 240;
  cfg.exposureValue = 10;
  cfg.manualExposure = 1;

  cfg.useCentroid  = 1;
  cfg.spotWindow   = 5;
  cfg.useSmoothing = 1;
  cfg.emaAlphaPct  = 35;

  cfg.autoThreshold = 1;
  cfg.ambientOffset = 40;   // 40 über Ambient
  cfg.threshFloor   = 120;  // nie unter 120
  cfg.ambAlphaPct   = 10;   // Ambient-EMA langsam
  cfg.hystGap       = 25;   // Hysterese-Halbgap

  cfg.crc32 = 0;
}
static void cfg_compute_crc() { cfg.crc32 = 0; cfg.crc32 = crc32_calc(reinterpret_cast<const uint8_t*>(&cfg), sizeof(cfg)); }
static bool cfg_save() {
  cfg_compute_crc();
  prefs.begin(NVS_NS, false);
  bool ok = (prefs.putBytes(NVS_KEY, &cfg, sizeof(cfg)) == sizeof(cfg));
  prefs.end();
  return ok;
}
static bool cfg_load() {
  prefs.begin(NVS_NS, true);
  size_t sz = prefs.getBytesLength(NVS_KEY);
  bool ok = false;
  if (sz == sizeof(cfg)) {
    Settings tmp;
    if (prefs.getBytes(NVS_KEY, &tmp, sizeof(tmp)) == sizeof(tmp)) {
      uint32_t saved = tmp.crc32; tmp.crc32 = 0;
      uint32_t calc = crc32_calc(reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp));
      tmp.crc32 = saved;
      if (saved == calc && tmp.version == SETTINGS_VERSION) { cfg = tmp; ok = true; }
    }
  }
  prefs.end();
  if (!ok) { cfg_set_defaults(); cfg_save(); }
  return ok;
}

// -------------------- Kalibrierung/Mathe --------------------
struct Point { double x, y; };
Point camPoints[5];
Point screenPoints[5];
int   calibStep = -1;

// Hilfsfunktionen
static inline void iswap_rows(double a[8][8], double b[8], int r1, int r2, int fromCol = 0) {
  if (r1 == r2) return;
  for (int k = fromCol; k < 8; ++k) { double t = a[r1][k]; a[r1][k] = a[r2][k]; a[r2][k] = t; }
  double tb = b[r1]; b[r1] = b[r2]; b[r2] = tb;
}
static inline double mapd(double v, double in_min, double in_max, double out_min, double out_max) {
  return out_min + (v - in_min) * (out_max - out_min) / (in_max - in_min);
}

// 5-Punkt Least-Squares Homographie -> cfg.H
void calculateHomography() {
  const int n = 5;
  double AtA[8][8] = {0};
  double Atb[8] = {0};
  auto accumulate_row = [&](const double R[8], double bval) {
    for (int r = 0; r < 8; ++r) { Atb[r] += R[r] * bval; for (int c = 0; c < 8; ++c) AtA[r][c] += R[r] * R[c]; }
  };
  for (int i = 0; i < n; ++i) {
    double X = camPoints[i].x, Y = camPoints[i].y;
    double x = screenPoints[i].x, y = screenPoints[i].y;
    double Rx[8] = { X, Y, 1, 0, 0, 0, -X * x, -Y * x };
    double Ry[8] = { 0, 0, 0, X, Y, 1, -X * y, -Y * y };
    accumulate_row(Rx, x);
    accumulate_row(Ry, y);
  }
  double a[8][8]; double b[8];
  for (int r = 0; r < 8; ++r) { b[r] = Atb[r]; for (int c = 0; c < 8; ++c) a[r][c] = AtA[r][c]; }
  for (int i = 0; i < 8; i++) {
    int piv = i; double best = std::fabs(a[i][i]);
    for (int r = i + 1; r < 8; r++) { double v = std::fabs(a[r][i]); if (v > best) { best = v; piv = r; } }
    if (best < 1e-12) { return; }
    if (piv != i) iswap_rows(a, b, i, piv, i);
    for (int j = i + 1; j < 8; j++) {
      double f = a[j][i] / a[i][i];
      b[j] -= f * b[i];
      for (int k = i; k < 8; k++) a[j][k] -= f * a[i][k];
    }
  }
  for (int i = 7; i >= 0; i--) {
    cfg.H[i] = b[i];
    for (int j = i + 1; j < 8; j++) cfg.H[i] -= a[i][j] * cfg.H[j];
    cfg.H[i] /= a[i][i];
  }
  cfg.H[8] = 1.0;
  cfg_save();
}

// -------------------- Live/Debug Variablen --------------------
volatile int    live_bx = -1, live_by = -1;   // RAW: hellster Pixel
volatile double live_cx = -1, live_cy = -1;   // CAM: ggf. Centroid
volatile double live_rx = -1, live_ry = -1;   // H-Map (ohne Glättung)
volatile double live_sx = -1, live_sy = -1;   // OUT (nach Glättung)
volatile bool   live_has = false;
volatile int    live_ton = 0, live_toff = 0;  // aktuelle Schwellen on/off
volatile int    live_vmax = 0;                // hellster Wert im Frame
volatile double live_amb = 0;                 // Ambient-EMA
volatile int    live_pressed = 0;             // 0/1

// -------------------- Web-UI --------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><title>Whiteboard Control</title>
<style>
  body { background:#222; color:#fff; font-family:sans-serif; text-align:center; padding:20px; }
  .box { background:#333; padding:20px; border-radius:10px; margin-bottom:10px; }
  button { padding:15px; background:#07f; color:#fff; border:none; border-radius:5px; cursor:pointer; }
  #canvas { position:fixed; top:0; left:0; width:100%; height:100%; display:none; background:#000; z-index:100; }
  input[type=range], select { width: 80%; }
  .row { margin: 10px 0; }
  label { user-select:none; cursor:pointer; }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace; white-space: pre-wrap; }
</style></head><body>
  <div class="box">
    <h2>Live Steuerung</h2>
    <div class="row">
      <label><input type="checkbox" id="aecToggle" onchange="setAEC(this.checked)"> Manuelle Belichtung</label>
    </div>
    <div class="row">
      <p>Threshold (Basis, falls Automatik aus): <span id="tVal">…</span><br>
      <input id="tRange" type="range" min="50" max="255" value="240" oninput="update('thresh', this.value)"></p>
    </div>
    <div class="row">
      <p>Belichtung (Kamera): <span id="eVal">…</span><br>
      <input id="eRange" type="range" min="1" max="100" value="10" oninput="update('exp', this.value)"></p>
    </div>
    <button onclick="startCalib()">Neue Kalibrierung starten</button>
  </div>

  <div class="box">
    <h3>Spot, Glättung & Automatik</h3>
    <div class="row">
      <label><input type="checkbox" id="centroidToggle" onchange="setCentroid(this.checked)"> Spot-Schwerpunkt (Centroid) aktiv</label>
    </div>
    <div class="row">
      <p>Fenstergröße (ungerade): <span id="wVal">…</span><br>
      <select id="winSelect" onchange="setWin(this.value)">
        <option>3</option><option>5</option><option>7</option><option>9</option><option>11</option>
      </select></p>
    </div>
    <div class="row">
      <label><input type="checkbox" id="smoothToggle" onchange="setSmooth(this.checked)"> Glättung (EMA) aktiv</label>
    </div>
    <div class="row">
      <p>EMA Alpha (%): <span id="aVal">…</span><br>
      <input id="aRange" type="range" min="0" max="100" value="35" oninput="setAlpha(this.value)"></p>
    </div>
    <hr>
    <div class="row">
      <label><input type="checkbox" id="autoToggle" onchange="setAuto(this.checked)"> Auto-Threshold (Umgebungslicht)</label>
    </div>
    <div class="row">
      <p>Ambient Offset (+): <span id="aoVal">…</span><br>
      <input id="aoRange" type="range" min="0" max="128" value="40" oninput="setAOffset(this.value)"></p>
    </div>
    <div class="row">
      <p>Min Floor: <span id="flVal">…</span><br>
      <input id="flRange" type="range" min="0" max="255" value="120" oninput="setFloor(this.value)"></p>
    </div>
    <div class="row">
      <p>Ambient EMA (%): <span id="amVal">…</span><br>
      <input id="amRange" type="range" min="0" max="100" value="10" oninput="setAmbAlpha(this.value)"></p>
    </div>
    <div class="row">
      <p>Hysterese Gap: <span id="hgVal">…</span><br>
      <input id="hgRange" type="range" min="0" max="100" value="25" oninput="setHGap(this.value)"></p>
    </div>
  </div>

  <div class="box">
    <h3>Live Koordinaten & Schwellen</h3>
    <div class="mono" id="coordText">Kein Punkt erkannt.</div>
  </div>

  <canvas id="canvas"></canvas>
<script>
  // Initialwerte laden
  window.addEventListener('load', () => {
    fetch('/get').then(r => r.json()).then(j => {
      // Basis
      document.getElementById('tRange').value = j.thresh; document.getElementById('tVal').innerText = j.thresh;
      document.getElementById('eRange').value = j.exp;    document.getElementById('eVal').innerText = j.exp;
      document.getElementById('aecToggle').checked = j.manual ? true : false;
      document.getElementById('eRange').disabled = j.manual ? false : true;
      // Spot/Glättung
      document.getElementById('centroidToggle').checked = j.centroid ? true : false;
      document.getElementById('winSelect').value = j.win; document.getElementById('wVal').innerText = j.win;
      document.getElementById('smoothToggle').checked = j.smooth ? true : false;
      document.getElementById('aRange').value = j.alpha;  document.getElementById('aVal').innerText = j.alpha;
      // Automatik/Hysterese
      document.getElementById('autoToggle').checked = j.auto ? true : false;
      document.getElementById('aoRange').value = j.aoffset; document.getElementById('aoVal').innerText = j.aoffset;
      document.getElementById('flRange').value = j.floor;   document.getElementById('flVal').innerText = j.floor;
      document.getElementById('amRange').value = j.aema;    document.getElementById('amVal').innerText = j.aema;
      document.getElementById('hgRange').value = j.hgap;    document.getElementById('hgVal').innerText = j.hgap;
    }).catch(()=>{});
    // Live-Polling
    setInterval(() => {
      fetch('/coords').then(r => r.json()).then(j => {
        const el = document.getElementById('coordText');
        if (!j.has) {
          el.textContent = "Kein Punkt erkannt.\n" +
                           `Pressed=${j.p} | Ambient=${j.amb.toFixed(1)} | Vmax=${j.vmax} | Ton=${j.ton} | Toff=${j.toff}`;
        } else {
          el.textContent =
            `CalibStep: ${j.step} | Pressed=${j.p}\n` +
            `RAW (bx,by) = (${j.bx}, ${j.by})\n` +
            `CAM (cx,cy) = (${j.cx.toFixed(2)}, ${j.cy.toFixed(2)})\n` +
            `H-Map (rx,ry) = (${j.rx.toFixed(1)}, ${j.ry.toFixed(1)})\n` +
            `OUT  (sx,sy) = (${j.sx.toFixed(1)}, ${j.sy.toFixed(1)})\n` +
            `Ambient=${j.amb.toFixed(1)} | Vmax=${j.vmax} | Ton=${j.ton} | Toff=${j.toff}`;
        }
      }).catch(()=>{});
    }, 150);
  });

  // Basis/Belichtung
  function setAEC(manual) { fetch(`/aec?v=${manual?1:0}`).then(() => { document.getElementById('eRange').disabled = manual ? false : true; }).catch(()=>{}); }
  function update(type, val) {
    if (type === 'thresh') document.getElementById('tVal').innerText = val;
    if (type === 'exp')    document.getElementById('eVal').innerText = val;
    fetch(`/${type}?v=${val}`).catch(()=>{});
  }

  // Spot/Glättung
  function setCentroid(on) { fetch(`/centroid?v=${on?1:0}`).catch(()=>{}); }
  function setWin(v)      { document.getElementById('wVal').innerText = v; fetch(`/win?v=${v}`).catch(()=>{}); }
  function setSmooth(on)  { fetch(`/smooth?v=${on?1:0}`).catch(()=>{}); }
  function setAlpha(v)    { document.getElementById('aVal').innerText = v; fetch(`/alpha?v=${v}`).catch(()=>{}); }

  // Automatik/Hysterese
  function setAuto(on)       { fetch(`/auto?v=${on?1:0}`).catch(()=>{}); }
  function setAOffset(v)     { document.getElementById('aoVal').innerText = v; fetch(`/aoffset?v=${v}`).catch(()=>{}); }
  function setFloor(v)       { document.getElementById('flVal').innerText = v; fetch(`/afloor?v=${v}`).catch(()=>{}); }
  function setAmbAlpha(v)    { document.getElementById('amVal').innerText = v; fetch(`/aema?v=${v}`).catch(()=>{}); }
  function setHGap(v)        { document.getElementById('hgVal').innerText = v; fetch(`/hgap?v=${v}`).catch(()=>{}); }

  // Kalibrierung mit echter Screen-Größe (Fullscreen)
  let step = 0;
  async function startCalib() {
    const cvs = document.getElementById('canvas');
    cvs.style.display = 'block';

    let fsOk = false;
    if (document.documentElement.requestFullscreen) {
      try { await document.documentElement.requestFullscreen(); fsOk = true; } catch(e) { fsOk = false; }
    }
    let w = 0, h = 0;
    if (fsOk) {
      w = (window.screen && window.screen.width)  ? window.screen.width  : window.innerWidth;
      h = (window.screen && window.screen.height) ? window.screen.height : window.innerHeight;
    } else { w = window.innerWidth; h = window.innerHeight; }
    cvs.width = w; cvs.height = h;

    try { await fetch(`/init?w=${w}&h=${h}`); runStep(); } catch(e) {}
  }
  function runStep() {
    const cvs = document.getElementById('canvas'); const ctx = cvs.getContext('2d');
    ctx.clearRect(0,0,cvs.width,cvs.height);
    const pts = [[20,20], [cvs.width-20,20], [cvs.width-20,cvs.height-20], [20,cvs.height-20], [cvs.width/2,cvs.height/2]];
    if (step < 5) {
      ctx.fillStyle='red'; ctx.beginPath(); ctx.arc(pts[step][0], pts[step][1], 20, 0, 7); ctx.fill();
      let poll = setInterval(() => {
        fetch('/status').then(r => r.text()).then(s => {
          if(parseInt(s) > step) { clearInterval(poll); step++; runStep(); }
        }).catch(()=>{});
      }, 350);
    } else {
      document.getElementById('canvas').style.display='none'; step=0;
      if (document.exitFullscreen) document.exitFullscreen().catch(()=>{});
    }
  }
</script></body></html>
)rawliteral";

// -------------------- Sensor-Apply --------------------
static void applySettingsToSensor() {
  sensor_t * s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, cfg.manualExposure ? 0 : 1); // 0=manuell, 1=AEC
  if (cfg.manualExposure) {
    s->set_aec_value(s, cfg.exposureValue);
  }
}

// -------------------- Setup --------------------
void setup() {
  // Serial.begin(115200); delay(200);
  cfg_load();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    while (true) { delay(1000); }
  }

  applySettingsToSensor();

  WiFi.softAP("Whiteboard-Setup", "");
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Aktuelle Werte
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"thresh\":" + String(cfg.irThreshold);
    json += ",\"exp\":" + String(cfg.exposureValue);
    json += ",\"manual\":" + String((int)cfg.manualExposure);
    json += ",\"centroid\":" + String((int)cfg.useCentroid);
    json += ",\"win\":" + String(cfg.spotWindow);
    json += ",\"smooth\":" + String((int)cfg.useSmoothing);
    json += ",\"alpha\":" + String(cfg.emaAlphaPct);
    json += ",\"auto\":" + String((int)cfg.autoThreshold);
    json += ",\"aoffset\":" + String(cfg.ambientOffset);
    json += ",\"floor\":" + String(cfg.threshFloor);
    json += ",\"aema\":" + String(cfg.ambAlphaPct);
    json += ",\"hgap\":" + String(cfg.hystGap);
    json += "}";
    request->send(200, "application/json", json);
  });

  // Init Kalibrierung + ScreenW/H
  server.on("/init", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("w") || !request->hasParam("h")) {
      request->send(400, "text/plain", "missing w/h"); return;
    }
    int w = request->getParam("w")->value().toInt();
    int h = request->getParam("h")->value().toInt();
    if (w <= 0 || h <= 0) { request->send(400, "text/plain", "bad size"); return; }

    cfg.screenW = w; cfg.screenH = h;
    screenPoints[0] = {20, 20};
    screenPoints[1] = {double(cfg.screenW - 20), 20};
    screenPoints[2] = {double(cfg.screenW - 20), double(cfg.screenH - 20)};
    screenPoints[3] = {20, double(cfg.screenH - 20)};
    screenPoints[4] = {double(cfg.screenW) / 2.0, double(cfg.screenH) / 2.0};
    calibStep = 0;
    cfg_save();
    request->send(200);
  });

  // Basis/Belichtung
  server.on("/thresh", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    cfg.irThreshold = request->getParam("v")->value().toInt();
    cfg_save();
    request->send(200);
  });
  server.on("/exp", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    cfg.exposureValue = request->getParam("v")->value().toInt();
    if (cfg.manualExposure) { sensor_t * s = esp_camera_sensor_get(); s->set_aec_value(s, cfg.exposureValue); }
    cfg_save();
    request->send(200);
  });
  server.on("/aec", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    cfg.manualExposure = request->getParam("v")->value().toInt() ? 1 : 0;
    applySettingsToSensor();
    cfg_save();
    request->send(200);
  });

  // Spot/Glättung
  server.on("/centroid", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    cfg.useCentroid = request->getParam("v")->value().toInt() ? 1 : 0;
    cfg_save(); request->send(200);
  });
  server.on("/win", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    int v = request->getParam("v")->value().toInt();
    if (v < 3) v = 3; if ((v % 2) == 0) v += 1; if (v > 31) v = 31;
    cfg.spotWindow = v; cfg_save(); request->send(200);
  });
  server.on("/smooth", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    cfg.useSmoothing = request->getParam("v")->value().toInt() ? 1 : 0;
    cfg_save(); request->send(200);
  });
  server.on("/alpha", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    int v = request->getParam("v")->value().toInt();
    if (v < 0) v = 0; if (v > 100) v = 100;
    cfg.emaAlphaPct = v; cfg_save(); request->send(200);
  });

  // Automatik/Hysterese
  server.on("/auto", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    cfg.autoThreshold = request->getParam("v")->value().toInt() ? 1 : 0;
    cfg_save(); request->send(200);
  });
  server.on("/aoffset", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    int v = request->getParam("v")->value().toInt();
    if (v < 0) v = 0; if (v > 128) v = 128;
    cfg.ambientOffset = v; cfg_save(); request->send(200);
  });
  server.on("/afloor", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    int v = request->getParam("v")->value().toInt();
    if (v < 0) v = 0; if (v > 255) v = 255;
    cfg.threshFloor = v; cfg_save(); request->send(200);
  });
  server.on("/aema", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    int v = request->getParam("v")->value().toInt();
    if (v < 0) v = 0; if (v > 100) v = 100;
    cfg.ambAlphaPct = v; cfg_save(); request->send(200);
  });
  server.on("/hgap", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("v")) { request->send(400); return; }
    int v = request->getParam("v")->value().toInt();
    if (v < 0) v = 0; if (v > 100) v = 100;
    cfg.hystGap = v; cfg_save(); request->send(200);
  });

  // Kalibrierstatus
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(calibStep));
  });

  // Live-Koordinaten
  server.on("/coords", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"step\":" + String(calibStep);
    json += ",\"has\":" + String(live_has ? 1 : 0);
    json += ",\"p\":" + String(live_pressed ? 1 : 0);
    json += ",\"bx\":" + String(live_bx);
    json += ",\"by\":" + String(live_by);
    json += ",\"cx\":" + String(live_cx, 3);
    json += ",\"cy\":" + String(live_cy, 3);
    json += ",\"rx\":" + String(live_rx, 2);
    json += ",\"ry\":" + String(live_ry, 2);
    json += ",\"sx\":" + String(live_sx, 2);
    json += ",\"sy\":" + String(live_sy, 2);
    json += ",\"amb\":" + String(live_amb, 2);
    json += ",\"vmax\":" + String(live_vmax);
    json += ",\"ton\":" + String(live_ton);
    json += ",\"toff\":" + String(live_toff);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.begin();

  Digitizer.begin();
  USB.begin();
}

// -------------------- Loop --------------------
void loop() {
  static double srx = 0.0, sry = 0.0; // geglättete Ausgabe
  static bool   s_init = false;

  static double ambEMA = 0.0;  // Ambient-EMA
  static bool   ambInit = false;

  static bool   pressed = false; // Hysterese-Zustand

  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) { delay(1); return; }

  const int w = fb->width, h = fb->height;
  const uint8_t* buf = fb->buf;

  // 1) Scannen: hellster Pixel + Frame-Mittelwert
  int bx = -1, by = -1;
  int vmax = 0;
  uint32_t sum = 0;

  for (int y = 0; y < h; ++y) {
    const uint8_t* row = buf + y * w;
    for (int x = 0; x < w; ++x) {
      uint8_t v = row[x];
      sum += v;
      if (v > vmax) { vmax = v; bx = x; by = y; }
    }
  }
  double frameMean = (double)sum / (double)(w * h);

  // 2) Ambient-EMA
  double ambAlpha = (cfg.ambAlphaPct <= 0) ? 0.0 : (cfg.ambAlphaPct >= 100 ? 1.0 : (cfg.ambAlphaPct / 100.0));
  if (!ambInit) { ambEMA = frameMean; ambInit = true; }
  else          { ambEMA = ambAlpha * frameMean + (1.0 - ambAlpha) * ambEMA; }

  // 3) dynamische Basis
  double base = cfg.autoThreshold
                  ? std::max((double)cfg.threshFloor, ambEMA + cfg.ambientOffset)
                  : (double)cfg.irThreshold;

  // 4) Hysterese-Schwellen
  int ton  = (int)std::lround(base + cfg.hystGap);
  int toff = (int)std::lround(base - cfg.hystGap);
  if (ton > 255) ton = 255; if (ton < 0) ton = 0;
  if (toff > 255) toff = 255; if (toff < 0) toff = 0;

  // 5) Zustandsabhängige Cutoff
  int cutoff = pressed ? toff : ton;

  // 6) Prüfen, ob wir einen Punkt über Cutoff haben
  bool found = (vmax > cutoff) && (bx >= 0) && (by >= 0);

  if (found) {
    // 7) Optional: Centroid um (bx,by)
    double px = (double)bx, py = (double)by;
    if (cfg.useCentroid) {
      int win = cfg.spotWindow;
      if (win < 3) win = 3; if ((win % 2) == 0) win += 1; if (win > 31) win = 31;
      int wh = win / 2;

      int x0 = (bx - wh < 0)  ? 0     : (bx - wh);
      int x1 = (bx + wh >= w) ? w - 1 : (bx + wh);
      int y0 = (by - wh < 0)  ? 0     : (by - wh);
      int y1 = (by + wh >= h) ? h - 1 : (by + wh);

      double sumW = 0.0, sumX = 0.0, sumY = 0.0;
      for (int y = y0; y <= y1; ++y) {
        const uint8_t* row = buf + y * w;
        for (int x = x0; x <= x1; ++x) {
          int val = row[x];
          int wgt = val - cutoff; // Gewicht relativ zur akt. Schwelle
          if (wgt <= 0) continue;
          sumW += wgt; sumX += wgt * x; sumY += wgt * y;
        }
      }
      if (sumW > 0.0) { px = sumX / sumW; py = sumY / sumW; }
    }

    // 8) Kalibrierung oder Laufzeit
    if (calibStep >= 0 && calibStep < 5) {
      camPoints[calibStep] = { px, py };

      // Live (Kalibrierphase)
      live_bx = bx; live_by = by; live_vmax = vmax;
      live_cx = px; live_cy = py;
      live_rx = live_ry = live_sx = live_sy = -1;
      live_has = true;
      live_amb = ambEMA; live_ton = ton; live_toff = toff; live_pressed = (int)pressed;

      calibStep++;
      if (calibStep == 5) { calculateHomography(); calibStep = -1; }
      s_init = false; // Glättung neu starten nach Kalibrierung
      pressed = true; // gilt als aktiv
      delay(800); // Entprellen während Kalibrierung

    } else {
      // 9) Auf Bildschirm mappen
      double z  = cfg.H[6]*px + cfg.H[7]*py + cfg.H[8];
      if (std::fabs(z) < 1e-9) z = (z >= 0 ? 1e-9 : -1e-9);
      double rx = (cfg.H[0]*px + cfg.H[1]*py + cfg.H[2]) / z;
      double ry = (cfg.H[3]*px + cfg.H[4]*py + cfg.H[5]) / z;

      // 10) EMA-Glättung
      double outx = rx, outy = ry;
      if (cfg.useSmoothing) {
        double alpha = (cfg.emaAlphaPct <= 0) ? 0.0 : (cfg.emaAlphaPct >= 100 ? 1.0 : (cfg.emaAlphaPct / 100.0));
        if (!s_init) { srx = rx; sry = ry; s_init = true; }
        srx = alpha * rx + (1.0 - alpha) * srx;
        sry = alpha * ry + (1.0 - alpha) * sry;
        outx = srx; outy = sry;
      } else {
        s_init = false;
      }

      // 11) HID-Ausgabe
      int16_t hx = (int16_t)constrain(lround(mapd(outx, 0.0, (double)cfg.screenW, 0.0, 32767.0)), 0, 32767);
      int16_t hy = (int16_t)constrain(lround(mapd(outy, 0.0, (double)cfg.screenH, 0.0, 32767.0)), 0, 32767);
      Digitizer.press(hx, hy);

      // Live
      live_bx = bx; live_by = by; live_vmax = vmax;
      live_cx = px; live_cy = py;
      live_rx = rx; live_ry = ry;
      live_sx = outx; live_sy = outy;
      live_has = true;
      live_amb = ambEMA; live_ton = ton; live_toff = toff; live_pressed = 1;

      pressed = true;
    }

  } else {
    // Kein Punkt oberhalb Cutoff
    Digitizer.release();

    live_has = false;
    live_bx = live_by = -1;
    live_cx = live_cy = -1;
    live_rx = live_ry = -1;
    live_sx = live_sy = -1;
    live_vmax = vmax;
    live_amb  = ambEMA;
    live_ton  = ton; live_toff = toff;
    live_pressed = 0;

    pressed = false;
    s_init = false; // Glättung zurücksetzen
  }

  Digitizer.update();
  esp_camera_fb_return(fb);
}
