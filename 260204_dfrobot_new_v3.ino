/**************************************************************
 * ESP32-S3 Zero - IR Whiteboard (Arduino IDE)
 * - DFRobot SEN0158 (PixArt) via I2C (4 Blobs)
 * - WiFi AP↔STA (AP nur bei fehlgeschlagener STA-Verbindung)
 * - REST + Web-GUI (neu) + WebSocket (live)
 * - 4-Punkt-Homographie
 * - USB HID Touch (Single-Touch, TinyUSB; nur Output toggelbar)
 * - HID-Test: fixer Tap
 * - NEU: Filter/Blob-Settings (persistent, Web-UI)
 *        - smoothing_on, alpha_ema + alpha_min/alpha_max (Clamp)
 *        - min_size (Blob-Mindestgröße), blobGraceMax
 **************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <algorithm>
#include <math.h>

// ==== HID (TinyUSB) - ÜBERNOMMEN ====
#include <USB.h>
#include <USBHID.h>

// ---------- Vorwärtsdeklarationen ----------
void sensorTask(void*);
static bool computeH(const float sx[4], const float sy[4], const float tx[4], const float ty[4]);
static bool applyH(float x, float y, float& X, float& Y);

// ========== SEN0158 / I2C ==========
/* ESP32-S3 Zero: bitte Pins prüfen. 
 * Viele S3-Boards verwenden:
 *  - SDA: 5
 *  - SCL: 6
 */
#define I2C_SDA       5
#define I2C_SCL       6
#define I2C_FREQ      100000     // optional: 400000 für schnellere I2C-Zugriffe
#define IR_ADDR_7BIT  0x58
#define IR_PTR_REG    0x36

// Sensorauflösung (SEN0158/WiiCam-typisch)
#define IR_X_MAX      1023
#define IR_Y_MAX      767

// ---- Konfigurierbare Parameter (persistiert) ----
uint8_t min_size      = 2;   // statt #define MIN_SIZE
int     blobGraceMax  = 3;   // statt const

// ====== Zustände/Variablen ======
bool  hid_enabled   = true;        // nur Output aktiv/inaktiv
bool  have_H        = false;
float Hm[9]         = {1,0,0, 0,1,0, 0,0,1};

uint16_t screenW = 1920, screenH = 1080;
bool ir_present = false;

// Blobs (Sensornorm 0..1 + Gültigkeit + Größe)
struct Blob { float x=0, y=0; bool valid=false; uint8_t size=0; } blobs[4];
// Touches (Displaynorm 0..1)
struct TP { float x=0, y=0; bool active=false; uint8_t id=0; } touches[4];

// Smoothing (einfach, optional – kann erweitert werden)
bool  smoothing_on  = true;
float alpha_ema     = 0.3f;
// NEU: einstellbarer Clamp-Bereich für alpha_ema
float alpha_min     = 0.05f;
float alpha_max     = 0.90f;

float filtX[4]={0}, filtY[4]={0};
bool  filtInit[4]={false,false,false,false};

// Kalibrierdaten (persistiert)
float calib_sx[4]={0}, calib_sy[4]={0};   // Sensor 0..1023 / 0..767
float calib_tx[4]={0}, calib_ty[4]={0};   // Ziel 0..W/H

// Anti-Ghosting
int blobGrace[4] = {0,0,0,0};

// Test/Live Blocker
volatile bool test_active = false;

// WiFi/AP/STA
const char* AP_SSID = "IR-Whiteboard";
const char* AP_PASS = "irwhiteboard";
IPAddress   AP_IP(192,168,4,1), AP_GW(192,168,4,1), AP_MASK(255,255,255,0);
DNSServer dnsServer;
const byte DNS_PORT = 53;

// WiFi STA-Creds
String sta_ssid="", sta_pwd="";

// Live-HID Down State
static bool live_down=false;

// Webserver / WS / NVS
WebServer server(80);
WebSocketsServer ws(81);
Preferences prefs;

// ---------- Mini JSON helpers ----------
static float jsonGetf(const String& s, const char* k, float def=0){
  int i=s.indexOf(String("\"")+k+"\""); if(i<0) return def; i=s.indexOf(":",i); if(i<0) return def;
  int j=s.indexOf(",",i+1); if(j<0) j=s.indexOf("}",i+1); return s.substring(i+1,j).toFloat();
}
static bool jsonGetb(const String& s, const char* k, bool def=false){
  int i=s.indexOf(String("\"")+k+"\""); if(i<0) return def; i=s.indexOf(":",i); if(i<0) return def;
  int j=s.indexOf(",",i+1); if(j<0) j=s.indexOf("}",i+1);
  String v = s.substring(i+1,j); v.trim(); v.toLowerCase();
  return v.indexOf("true")>=0 || v.indexOf("1")>=0;
}
static int jsonGeti(const String& s, const char* k, int def=0){
  int i=s.indexOf(String("\"")+k+"\""); if(i<0) return def; i=s.indexOf(":",i); if(i<0) return def;
  int j=s.indexOf(",",i+1); if(j<0) j=s.indexOf("}",i+1); return s.substring(i+1,j).toInt();
}
static String jsonPickStr(const String& s, const char* key){
  int i=s.indexOf(String("\"")+key+"\""); if (i<0) return "";
  i=s.indexOf(":",i); if(i<0) return "";
  int q1=s.indexOf("\"",i), q2=s.indexOf("\"",q1+1);
  if (q1<0||q2<0) return "";
  return s.substring(q1+1,q2);
}

// ========== SEN0158 Low-Level ==========
bool ir_write2(uint8_t r, uint8_t v){
  Wire.beginTransmission(IR_ADDR_7BIT);
  Wire.write(r); Wire.write(v);
  return Wire.endTransmission()==0;
}

// Striktere Decodierung: invalide bei x==1023 (Sentinel) oder y==1023, size < min_size, oder y > IR_Y_MAX
static void decodeBlobStrict(const uint8_t *buf, int base, Blob& b){
  b = Blob();
  uint8_t xl = buf[base+0], yl = buf[base+1], s  = buf[base+2];
  uint16_t x = xl | ((s & 0x30) << 4);
  uint16_t y = yl | ((s & 0xC0) << 2);
  uint8_t  sz = (s & 0x0F);

  if (x==IR_X_MAX || y==IR_X_MAX || y>IR_Y_MAX || sz < min_size) { b.valid=false; b.size=sz; return; }

  b.x = constrain((float)x/(float)IR_X_MAX, 0.0f, 1.0f);
  b.y = constrain((float)y/(float)IR_Y_MAX, 0.0f, 1.0f);
  b.valid=true; b.size=sz;
}

// ===== Wii-Style Full Sensitivity Init =====
bool ir_init_full_sensitivity() {
  bool ok = true;

  // Reset
  ok &= ir_write2(0x30, 0x01);
  delay(10);

  // Shutter / Gain / Noise (baseline ähnlich Wii)
  ir_write2(0x06, 0x90);
  ir_write2(0x08, 0xC0);
  ir_write2(0x1A, 0x40);
  ir_write2(0x33, 0x33);
  delay(10);

  // === Sensitivity Block 1 ===
  const uint8_t sens1[] = { 0x02,0x00,0x00,0x71,0x01,0x00,0xAA,0x00,0x64,0x00,0x00 };
  Wire.beginTransmission(IR_ADDR_7BIT);
  Wire.write(0x00);
  for (uint8_t b : sens1) Wire.write(b);
  ok &= (Wire.endTransmission() == 0);

  delay(10);

  // === Sensitivity Block 2 ===
  const uint8_t sens2[] = { 0x02,0x00,0x00,0x71,0x01,0x00,0xAA,0x00,0x64,0x00,0x00 };
  Wire.beginTransmission(IR_ADDR_7BIT);
  Wire.write(0x00);
  for (uint8_t b : sens2) Wire.write(b);
  ok &= (Wire.endTransmission() == 0);

  delay(10);

  // Start IR processing
  ok &= ir_write2(0x30, 0x08);
  delay(50);

  return ok;
}

// ===== Auto Sensitivity Controller (Wii-Style Levels) =====
static uint8_t currentSens = 6;   // Start: beste Reichweite
static uint8_t targetSens  = 6;
static uint32_t lastSensChange = 0;
// optional: mildes Entprellen der avg-Size
static float avgSizeSmooth = 0.0f;

void applySensitivityLevel(uint8_t lvl) {
  if (lvl == currentSens) return;

  const uint8_t* block = nullptr;

  // Sensitivity presets (WiiMote compatible)
  static const uint8_t sens1[] = {0x02,0,0,0x71,0x01,0,0x40,0,0x55,0,0}; // Level 1
  static const uint8_t sens2[] = {0x02,0,0,0x71,0x01,0,0x55,0,0x63,0,0}; // Level 2
  static const uint8_t sens3[] = {0x02,0,0,0x71,0x01,0,0x5A,0,0x6A,0,0}; // Level 3
  static const uint8_t sens4[] = {0x02,0,0,0x71,0x01,0,0x64,0,0xFE,0,0}; // Level 4
  static const uint8_t sens5[] = {0x02,0,0,0x71,0x01,0,0x96,0,0xFE,0,0}; // Level 5
  static const uint8_t sens6[] = {0x02,0,0,0x71,0x01,0,0xAA,0,0x64,0,0}; // Level 6

  switch (lvl) {
    case 1: block = sens1; break;
    case 2: block = sens2; break;
    case 3: block = sens3; break;
    case 4: block = sens4; break;
    case 5: block = sens5; break;
    case 6: block = sens6; break;
    default: return;
  }

  // Write sensitivity block
  Wire.beginTransmission(IR_ADDR_7BIT);
  Wire.write(0x00);
  for(int i=0;i<11;i++) Wire.write(block[i]);
  Wire.endTransmission();
  delay(2);

  currentSens = lvl;
  lastSensChange = millis();
}

// Durchschnittliche Blobgröße (grob)
uint8_t averageBlobSize() {
  uint16_t sum = 0; int count = 0;
  for (int i=0;i<4;i++) {
    if (blobs[i].valid) { sum += blobs[i].size; count++; }
  }
  if (!count) return 0;
  return sum / count;
}

// =========================================
//  AUTO EXPOSURE + AUTO SHUTTER (Wii-style)
// =========================================
static uint8_t currentShutter = 0x40;  
static uint32_t lastAEUpdate = 0;

// Shutter-Level-Presets (WiiMote ähnlich)
static const uint8_t shutter_low    = 0x20; // helle Umgebung / naher Stift
static const uint8_t shutter_normal = 0x40; // normal
static const uint8_t shutter_high   = 0x80; // dunkle Umgebung / große Distanz

// Hilfsfunktion: mittlere Blob-Größe
uint8_t averageBlobSizeAE() {
  int sum = 0, c = 0;
  for (int i=0; i<4; i++) {
    if (blobs[i].valid) { sum += blobs[i].size; c++; }
  }
  if (!c) return 0;
  return sum / c;
}

// Auto-Exposure / Auto-Shutter Regler
void updateAutoExposure() {
  if (!ir_present) return;               // ✅ nur wenn IR aktiv
  uint32_t now = millis();
  if (now - lastAEUpdate < 250) return;  // max. alle 250ms aktualisieren
  lastAEUpdate = now;

  uint8_t avg = averageBlobSizeAE();
  uint8_t target = currentShutter;

  // Logik:
  // großen avg  → Pen nah → weniger Belichtung
  // kleinen avg → Pen weit → längere Belichtung
  if (avg > 10)       target = shutter_low;
  else if (avg > 5)   target = shutter_normal;
  else if (avg > 0)   target = shutter_high;
  else                target = shutter_high;  // kein Blob → maximale Empfindlichkeit

  // nur schreiben, wenn nötig
  if (target == currentShutter) return;
  currentShutter = target;

  // Register 0x1A = Shutter / Exposure Control
  Wire.beginTransmission(IR_ADDR_7BIT);
  Wire.write(0x1A);
  Wire.write(target);
  Wire.endTransmission();
}

// ========== Homographie ==========
static bool solve8(float A[8][8], float b[8], float x[8]){
  for (int i=0;i<8;i++){
    int piv=i; for (int r=i+1;r<8;r++) if (fabsf(A[r][i])>fabsf(A[piv][i])) piv=r;
    if (fabsf(A[piv][i])<1e-9) return false;
    if (piv!=i){ for(int c=i;c<8;c++) std::swap(A[i][c],A[piv][c]); std::swap(b[i],b[piv]); }
    float div=A[i][i]; for(int c=i;c<8;c++) A[i][c]/=div; b[i]/=div;
    for(int r=0;r<8;r++){ if (r==i) continue; float f=A[r][i];
      for(int c=i;c<8;c++) A[r][c]-=f*A[i][c]; b[r]-=f*b[i]; }
  }
  for (int i=0;i<8;i++) x[i]=b[i]; return true;
}
static bool computeH(const float sx[4], const float sy[4], const float tx[4], const float ty[4]){
  float A[8][8]={0}; float bb[8]={0};
  for(int i=0;i<4;i++){
    float x=sx[i], y=sy[i], X=tx[i], Y=ty[i];
    A[2*i+0][0] = -x;  A[2*i+0][1] = -y;  A[2*i+0][2] = -1;
    A[2*i+0][6] =  X*x; A[2*i+0][7] =  X*y; bb[2*i+0] = -X;
    A[2*i+1][3] = -x;  A[2*i+1][4] = -y;  A[2*i+1][5] = -1;
    A[2*i+1][6] =  Y*x; A[2*i+1][7] =  Y*y; bb[2*i+1] = -Y;
  }
  float xsol[8]; if(!solve8(A,bb,xsol)) return false;
  Hm[0]=xsol[0]; Hm[1]=xsol[1]; Hm[2]=xsol[2];
  Hm[3]=xsol[3]; Hm[4]=xsol[4]; Hm[5]=xsol[5];
  Hm[6]=xsol[6]; Hm[7]=xsol[7]; Hm[8]=1.0f;
  return true;
}
static bool applyH(float x, float y, float& X, float& Y){
  float w = Hm[6]*x + Hm[7]*y + Hm[8];
  if (fabsf(w) < 1e-6f) return false;
  X = (Hm[0]*x + Hm[1]*y + Hm[2]) / w;
  Y = (Hm[3]*x + Hm[4]*y + Hm[5]) / w;
  return true;
}

// ========== NVS ==========
void loadPrefs(){
  prefs.begin("irwb", true);
  hid_enabled      = prefs.getBool ("hid", true);
  have_H           = prefs.getBool ("haveH", false);
  smoothing_on     = prefs.getBool ("smon", true);
  alpha_ema        = prefs.getFloat("alpha", 0.30f);
  alpha_min        = prefs.getFloat("a_min", 0.05f);
  alpha_max        = prefs.getFloat("a_max", 0.90f);
  if (alpha_min < 0.01f) alpha_min = 0.01f;
  if (alpha_max > 0.99f) alpha_max = 0.99f;
  if (alpha_min >= alpha_max) { alpha_min = 0.05f; alpha_max = 0.90f; }
  if (alpha_ema < alpha_min) alpha_ema = alpha_min;
  if (alpha_ema > alpha_max) alpha_ema = alpha_max;

  screenW          = prefs.getUShort("scrW", 1920);
  screenH          = prefs.getUShort("scrH", 1080);

  // Neue Blob/Grace-Settings
  min_size         = prefs.getUChar("minsz", 2);
  blobGraceMax     = prefs.getInt  ("grace",  3);

  sta_ssid         = prefs.getString("w_ssid", "");
  sta_pwd          = prefs.getString("w_pwd",  "");
  if (have_H) prefs.getBytes("H", Hm, sizeof(Hm));
  if (prefs.getBytesLength("c_sx")==sizeof(calib_sx)) prefs.getBytes("c_sx", calib_sx, sizeof(calib_sx));
  if (prefs.getBytesLength("c_sy")==sizeof(calib_sy)) prefs.getBytes("c_sy", calib_sy, sizeof(calib_sy));
  if (prefs.getBytesLength("c_tx")==sizeof(calib_tx)) prefs.getBytes("c_tx", calib_tx, sizeof(calib_tx));
  if (prefs.getBytesLength("c_ty")==sizeof(calib_ty)) prefs.getBytes("c_ty", calib_ty, sizeof(calib_ty));
  prefs.end();
}
void savePrefs(){
  prefs.begin("irwb", false);
  prefs.putBool ("hid",   hid_enabled);
  prefs.putBool ("haveH", have_H);
  prefs.putBool ("smon",  smoothing_on);
  prefs.putFloat("alpha", alpha_ema);
  prefs.putFloat("a_min", alpha_min);
  prefs.putFloat("a_max", alpha_max);
  prefs.putUShort("scrW", screenW);
  prefs.putUShort("scrH", screenH);
  prefs.putString("w_ssid", sta_ssid);
  prefs.putString("w_pwd",  sta_pwd);

  // Neue Blob/Grace-Settings
  prefs.putUChar("minsz", min_size);
  prefs.putInt  ("grace",  blobGraceMax);

  if (have_H) prefs.putBytes("H", Hm, sizeof(Hm));
  prefs.putBytes("c_sx", calib_sx, sizeof(calib_sx));
  prefs.putBytes("c_sy", calib_sy, sizeof(calib_sy));
  prefs.putBytes("c_tx", calib_tx, sizeof(calib_tx));
  prefs.putBytes("c_ty", calib_ty, sizeof(calib_ty));
  prefs.end();
}

// ========== HID (Descriptor & Touch) - ÜBERNOMMEN ==========
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
        0x75, 0x01, 0x95, 0x02, 0x81, 0x02,
        0x75, 0x06, 0x95, 0x01, 0x81, 0x03,
        0x09, 0x51, 0x15, 0x00, 0x25, 0x7F,
        0x75, 0x08, 0x95, 0x01, 0x81, 0x02,
        0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
        0x16, 0x00, 0x00, 0x26, 0xFF, 0x7F,
        0x75, 0x10, 0x95, 0x02, 0x81, 0x02,
      0xC0,
      0x09, 0x54, 0x15, 0x00, 0x25, 0x0A,
      0x75, 0x08, 0x95, 0x01, 0x81, 0x02,
    0xC0
  };
};

HIDTouchDevice Touch;
static inline uint16_t f2u16(float v){
  if (v < 0) v = 0; else if (v > 1) v = 1;
  return (uint16_t)(v * 32767.0f + 0.5f);
}

// ========== Web GUI (NEU) ==========
// ⚠️ Lass deinen vorhandenen HTML-Block (INDEX_HTML[] PROGMEM) unverändert in DEINER Datei.

// ========== Web GUI (NEU) ==========
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>IR Whiteboard</title>
<style>
:root{color-scheme:dark}
body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:0;background:#101014;color:#eaeef3}
header{display:flex;gap:10px;align-items:center;padding:12px 16px;background:#171a22;position:sticky;top:0;z-index:5}
.badge{display:inline-block;padding:2px 8px;border-radius:999px;background:#252a36;color:#9fb3c8}
.badge.ok{background:#11331d;color:#7ee2a8}
.badge.err{background:#3a1515;color:#f98e8e}
main{padding:16px}
.card{background:#171a22;border:1px solid #22283a;border-radius:10px;padding:12px;margin:10px 0}
h3{margin:6px 0 10px}
.row{display:flex;gap:12px;flex-wrap:wrap}
.btn{padding:7px 12px;border-radius:8px;border:1px solid #2b3245;background:#202636;color:#eaeef3;cursor:pointer}
.btn.red{background:#3a1c1c;border-color:#663636}
.btn:disabled{opacity:.6;cursor:not-allowed}
input[type=number],input[type=text],input[type=password],select{background:#0f131b;color:#eaeef3;border:1px solid #2b3245;border-radius:6px;padding:6px}
input[type=range]{accent-color:#6aa1ff}
table{border-collapse:collapse;width:100%;background:#0f131b}
th,td{border:1px solid #2b3245;padding:6px;text-align:center}
canvas{background:#0a0d13;border:1px solid #2b3245;border-radius:6px}
small{color:#97a0af}
.flex{display:flex;gap:10px;flex-wrap:wrap}
.kv{display:grid;grid-template-columns:auto 1fr;gap:6px 10px;align-items:center}
.overlay{position:fixed;inset:0;background:#000;z-index:9999;display:none}
.overlay canvas{position:absolute;inset:0;width:100%;height:100%}
.overlay .btn{position:fixed;top:16px;right:16px}
.help{font-size:12px;color:#97a0af}
</style></head>
<body>
<header>
  <b>ESP32‑S3 IR Whiteboard</b>
  <span id="bHID" class="badge">HID: ?</span>
  <span id="bIR" class="badge">IR: ?</span>
  <span id="bSCR" class="badge">Screen: ?</span>
  <span id="bWi" class="badge">Wi‑Fi: ?</span>
</header>
<main>

<div class="row">
  <div class="card" style="flex:1 1 360px">
    <h3>HID</h3>
    <div class="flex">
      <button id="btnHid" class="btn">HID Output EIN/AUS</button>
      <button id="btnTap" class="btn">HID Test‑Tap (fix)</button>
    </div>
    <small>Der HID‑Output wird nur ein/aus geschaltet. Das USB‑HID‑Gerät bleibt verbunden.</small>
  </div>

  <div class="card" style="flex:1 1 420px">
    <h3>Kalibrierung</h3>
    <div class="flex">
      <button id="btnCal" class="btn">Kalibrieren (Fullscreen)</button>
      <button id="btnResetCal" class="btn red">Kalibrierung zurücksetzen</button>
    </div>
    <small>Reihenfolge: links‑oben → rechts‑oben → rechts‑unten → links‑unten. Es reicht, wenn irgendein IR‑Blob sichtbar ist.</small>
  </div>

  <div class="card" style="flex:1 1 420px">
    <h3>Displayauflösung</h3>
    <div class="kv">
      <label>Breite (px)</label><input id="sw" type="number" min="200" max="16384" step="1">
      <label>Höhe (px)</label><input id="sh" type="number" min="200" max="16384" step="1">
    </div>
    <div class="flex" style="margin-top:8px">
      <button id="btnScrSave" class="btn">Speichern</button>
      <button id="btnScrFromFS" class="btn">Von Fullscreen übernehmen</button>
      <button id="btnReload" class="btn">Aktualisieren</button>
    </div>
    <small>Falls Fullscreen wegen Skalierung falsche Werte liefert, hier korrigieren.</small>
  </div>

  <div class="card" style="flex:1 1 560px">
    <h3>Kalibriertabelle (bearbeitbar)</h3>
    <small>Sensor: x 0..1023, y 0..767 — Ziel: Pixel 0..W/H</small>
    <table>
      <thead><tr><th>#</th><th>sx</th><th>sy</th><th>tx</th><th>ty</th></tr></thead>
      <tbody id="tbl"></tbody>
    </table>
    <div class="flex" style="margin-top:8px">
      <button id="cLoad" class="btn">Laden</button>
      <button id="cSave" class="btn">Speichern</button>
      <button id="cApply" class="btn">Anwenden (H berechnen)</button>
    </div>
  </div>

  <div class="card" style="flex:1 1 420px">
    <h3>Wi‑Fi</h3>
    <div class="kv">
      <label>SSID</label><input id="ssid" type="text" placeholder="SSID">
      <label>Passwort</label><input id="pwd" type="password" placeholder="Passwort">
    </div>
    <div class="flex" style="margin-top:8px">
      <button id="btnWiSave" class="btn">Speichern &amp; Auto‑Connect</button>
      <button id="btnWiStatus" class="btn">Status</button>
    </div>
    <small>Bei erfolgreichem STA wird der AP beim nächsten Start nicht mehr parallel aufgebaut.</small>
  </div>

  <div class="card" style="flex:1 1 360px">
    <h3>Live: Kamera</h3>
    <canvas id="cam" width="256" height="192"></canvas>
    <pre id="camTxt"></pre>
  </div>

  <div class="card" style="flex:1 1 420px">
    <h3>Live: Touch</h3>
    <canvas id="touch" width="320" height="180"></canvas>
    <pre id="touchTxt"></pre>
  </div>

  <!-- NEU: Filter & Blobs -->
  <div class="card" style="flex:1 1 460px">
    <h3>Filter &amp; Blobs</h3>
    <div class="kv">
      <label>Glättung aktiv</label>
      <input id="sm_on" type="checkbox">

      <label>EMA α</label>
      <div class="flex">
        <input id="sm_alpha" type="range" min="0.05" max="0.90" step="0.01" style="width:200px">
        <span id="sm_alpha_val">0.30</span>
      </div>

      <label>α min</label>
      <input id="sm_amin" type="number" min="0.01" max="0.99" step="0.01" value="0.05">

      <label>α max</label>
      <input id="sm_amax" type="number" min="0.01" max="0.99" step="0.01" value="0.90">

      <label>Blob Mindestgröße</label>
      <input id="min_size" type="number" min="0" max="15" step="1">

      <label>Ghost‑Grace (Frames)</label>
      <input id="grace" type="number" min="0" max="50" step="1">
    </div>
    <div class="flex" style="margin-top:8px">
      <button id="btnFiltSave" class="btn">Speichern</button>
      <button id="btnFiltReload" class="btn">Aktualisieren</button>
    </div>
    <div class="help">α kleiner ⇒ ruhiger, träger. α größer ⇒ reaktiver, kann zappeln. Der Slider nutzt den Bereich [αmin..αmax].</div>
  </div>

</div>

</main>

<!-- Fullscreen Overlay für Kalibrierung -->
<div id="ov" class="overlay">
  <canvas id="ovc"></canvas>
  <button id="ovAbort" class="btn red">Abbrechen</button>
</div>

<script>
const ws = new WebSocket(`ws://${location.hostname}:81/`);
const bHID = document.getElementById('bHID');
const bIR  = document.getElementById('bIR');
const bSCR = document.getElementById('bSCR');
const bWi  = document.getElementById('bWi');

const cam = document.getElementById('cam'), cr = cam.getContext('2d');
const tch = document.getElementById('touch'), tr = tch.getContext('2d');
const camTxt = document.getElementById('camTxt');
const touchTxt = document.getElementById('touchTxt');

const sw = document.getElementById('sw'), sh=document.getElementById('sh');
const tbl = document.getElementById('tbl');

const sm_on = document.getElementById('sm_on');
const sm_alpha = document.getElementById('sm_alpha');
const sm_alpha_val = document.getElementById('sm_alpha_val');
const sm_amin = document.getElementById('sm_amin');
const sm_amax = document.getElementById('sm_amax');
const min_size_in = document.getElementById('min_size');
const grace_in = document.getElementById('grace');

let C = {screenW:1920, screenH:1080, calib:{sx:[0,0,0,0],sy:[0,0,0,0],tx:[0,0,0,0],ty:[0,0,0,0]}};
let lastBlobs = [];
let irOn = false;

function badge(el, label, ok, pre=""){
  el.textContent = pre + label;
  el.className = "badge " + (ok===null ? "" : (ok ? "ok":"err"));
}

function loadConf(){
  fetch('/api/conf').then(r=>r.json()).then(c=>{
    C.screenW=c.screenW||1920; C.screenH=c.screenH||1080;
    sw.value=C.screenW; sh.value=C.screenH;
    badge(bSCR, `${C.screenW}×${C.screenH}`, true, "Screen: ");
  });
}
function loadTable(){
  fetch('/api/calib/table').then(r=>r.json()).then(ct=>{
    C.calib=ct;
    tbl.innerHTML="";
    for(let i=0;i<4;i++){
      const row=document.createElement('tr');
      row.innerHTML = `<td>${i+1}</td>
        <td><input id="sx${i}" type="number" min="0" max="1023" step="1" value="${(ct.sx[i]||0).toFixed(0)}"></td>
        <td><input id="sy${i}" type="number" min="0" max="767"  step="1" value="${(ct.sy[i]||0).toFixed(0)}"></td>
        <td><input id="tx${i}" type="number" min="0" max="${C.screenW}" step="1" value="${(ct.tx[i]||0).toFixed(0)}"></td>
        <td><input id="ty${i}" type="number" min="0" max="${C.screenH}" step="1" value="${(ct.ty[i]||0).toFixed(0)}"></td>`;
      tbl.appendChild(row);
    }
  });
}

document.getElementById('btnHid').onclick = ()=>{ fetch('/api/hid/toggle',{method:'POST'}); };
document.getElementById('btnTap').onclick = ()=>{ fetch('/api/hid/testTap',{method:'POST'}); };

document.getElementById('btnScrSave').onclick = ()=>{
  const W=+sw.value|0, H=+sh.value|0;
  fetch('/api/conf/screen',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({screenW:W, screenH:H})}).then(()=>loadConf());
};
document.getElementById('btnScrFromFS').onclick = ()=>{
  const W = Math.round(screen.width * (window.devicePixelRatio||1));
  const H = Math.round(screen.height* (window.devicePixelRatio||1));
  sw.value=W; sh.value=H;
};

document.getElementById('btnReload').onclick = ()=>{ loadConf(); loadTable(); wifiStatus(); loadFilter(); };

document.getElementById('cLoad').onclick = loadTable;
document.getElementById('cSave').onclick = ()=>{
  const sx=[],sy=[],tx=[],ty=[];
  for(let i=0;i<4;i++){
    sx.push(+document.getElementById('sx'+i).value);
    sy.push(+document.getElementById('sy'+i).value);
    tx.push(+document.getElementById('tx'+i).value);
    ty.push(+document.getElementById('ty'+i).value);
  }
  fetch('/api/calib/table',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({sx,sy,tx,ty})}).then(()=>loadTable());
};
document.getElementById('cApply').onclick = ()=>{ fetch('/api/calib/commit',{method:'POST'}); };

function wifiStatus(){
  fetch('/api/wifi/status').then(r=>r.json()).then(s=>{
    if(s.connected) badge(bWi, `${s.ssid} (${s.ip})`, true, "Wi‑Fi: ");
    else badge(bWi, 'Nicht verbunden', false, "Wi‑Fi: ");
  });
}
document.getElementById('btnWiSave').onclick=()=>{
  const ssid=document.getElementById('ssid').value;
  const pwd =document.getElementById('pwd').value;
  fetch('/api/wifi/save',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid,pwd})}).then(()=>setTimeout(wifiStatus,1200));
};
document.getElementById('btnWiStatus').onclick=wifiStatus;

// Live WebSocket
ws.onmessage = ev=>{
  const d = JSON.parse(ev.data || "{}");
  badge(bHID, d.hid ? "ON" : "OFF", d.hid, "HID: ");
  badge(bIR,  d.ir  ? "ON" : "OFF", d.ir,  "IR: ");
  irOn = !!d.ir;
  lastBlobs = Array.isArray(d.blobs)? d.blobs : [];

  // Kamera-View
  cr.clearRect(0,0,cam.width,cam.height);
  cr.strokeStyle="#3b82f6"; cr.strokeRect(0,0,cam.width,cam.height);

  const cols=["#ef4444","#f59e0b","#22c55e","#06b6d4"];
  let t="Blobs (sensor):\n";
  lastBlobs.forEach((b,i)=>{
    t += `#${i+1}: ${b.valid?(`x=${b.sx||0}|${(b.x||0).toFixed(3)}  y=${b.sy||0}|${(b.y||0).toFixed(3)}`):"--"}\n`;
    if(b.valid){
      cr.fillStyle=cols[i];
      cr.beginPath();
      cr.arc((b.x||0)*cam.width, (b.y||0)*cam.height, 4, 0, 6.28);
      cr.fill();
    }
  });
  // Kalibrier-Polygon (Sensor-Koordinaten)
  if (d.poly && d.poly.length===4){
    cr.strokeStyle="#a855f7"; cr.beginPath();
    for(let i=0;i<4;i++){
      const p=d.poly[i];
      const cx=p.x*cam.width, cy=p.y*cam.height;
      if(i===0) cr.moveTo(cx,cy); else cr.lineTo(cx,cy);
    }
    cr.closePath(); cr.stroke();
  }
  camTxt.textContent=t;

  // Touch-View
  const ar = (C.screenW>0 && C.screenH>0) ? (C.screenW/C.screenH) : (16/9);
  tch.width  = ar>=1 ? 360 : Math.round(360*ar);
  tch.height = ar>=1 ? Math.round(360/ar) : 360;

  tr.clearRect(0,0,tch.width,tch.height);
  tr.strokeStyle="#16a34a"; tr.strokeRect(0,0,tch.width,tch.height);

  if (d.touch){
    const tx = (d.touch.x||0)*tch.width;
    const ty = (d.touch.y||0)*tch.height;
    tr.fillStyle="#16a34a";
    tr.beginPath(); tr.arc(tx,ty,6,0,6.28); tr.fill();
    touchTxt.textContent = `Touch (norm): x=${(d.touch.x||0).toFixed(3)} y=${(d.touch.y||0).toFixed(3)}\n`+
                           `Touch (px):   X=${Math.round((d.touch.x||0)*C.screenW)} Y=${Math.round((d.touch.y||0)*C.screenH)}`;
  }
};

// ==== Kalibrierung Fullscreen ====
const ov = document.getElementById('ov');
const ovc = document.getElementById('ovc'), oc=ovc.getContext('2d');
const abortBtn = document.getElementById('ovAbort');

document.getElementById('btnCal').onclick = startCalibration;
document.getElementById('btnResetCal').onclick = ()=>{ fetch('/api/calib/reset',{method:'POST'}) };

function startCalibration(){
  const targets = [
    {x:0.06, y:0.06},     // TL
    {x:0.94, y:0.06},     // TR
    {x:0.94, y:0.94},     // BR
    {x:0.06, y:0.94}      // BL
  ];
  let idx=0;
  let stableSince=0;
  const holdMs = 1200;
  let lastSeen=null;
  let raf=0;

  ov.style.display='block';
  function resize(){ ovc.width=window.innerWidth; ovc.height=window.innerHeight; }
  resize(); window.addEventListener('resize', resize);
  ov.requestFullscreen && ov.requestFullscreen().catch(()=>{});

  function drawCross(cx,cy){
    oc.strokeStyle="#e5e7eb";
    const len = Math.max(16, Math.floor(Math.min(ovc.width,ovc.height)*0.03));
    const lw  = Math.max(2,  Math.floor(Math.min(ovc.width,ovc.height)*0.004));
    oc.lineWidth=lw;
    oc.beginPath();
    oc.moveTo(cx-len,cy); oc.lineTo(cx+len,cy);
    oc.moveTo(cx,cy-len); oc.lineTo(cx,cy+len);
    oc.stroke();
  }

  function loop(ts){
    oc.clearRect(0,0,ovc.width,ovc.height);
    oc.fillStyle="#fff"; oc.font="16px system-ui"; oc.fillText(`Punkt ${idx+1}/4`, 20, 32);
    const t=targets[idx];
    drawCross(t.x*ovc.width, t.y*ovc.height);

    if (irOn && Array.isArray(lastBlobs) && lastBlobs.some(b=>b && b.valid)){
      const b = lastBlobs.find(bb=>bb && bb.valid);
      if (b){
        if(!lastSeen){ lastSeen={x:b.x,y:b.y}; stableSince=ts||performance.now(); }
        const dx=b.x-lastSeen.x, dy=b.y-lastSeen.y;
        if (Math.hypot(dx,dy) <= 0.01){
          if ((ts||performance.now())-stableSince >= holdMs){
            fetch('/api/calib/point',{method:'POST',headers:{'Content-Type':'application/json'},
              body:JSON.stringify({idx:idx, sx:b.x, sy:b.y, tx:t.x, ty:t.y})
            });
            idx++; lastSeen=null; stableSince=0;
            if(idx>=4){
              const W = Math.round(screen.width * (window.devicePixelRatio||1));
              const H = Math.round(screen.height* (window.devicePixelRatio||1));
              fetch('/api/conf/screen',{method:'POST',headers:{'Content-Type':'application/json'},
                body:JSON.stringify({screenW:W, screenH:H})
              }).then(()=> fetch('/api/calib/commit',{method:'POST'}))
                .then(()=> cleanup());
              return;
            }
          }
        }else{
          lastSeen={x:b.x,y:b.y}; stableSince=ts||performance.now();
        }
      }
    }
    raf = requestAnimationFrame(loop);
  }

  function cleanup(){
    cancelAnimationFrame(raf);
    window.removeEventListener('resize', resize);
    if (document.fullscreenElement) document.exitFullscreen().catch(()=>{});
    ov.style.display='none';
    loadConf(); loadTable();
  }
  abortBtn.onclick=cleanup;

  raf=requestAnimationFrame(loop);
}

// ===== Filter laden/speichern (NEU) =====
function applyAlphaSliderBounds(amin, amax){
  // Sanity
  amin = Math.max(0.01, Math.min(0.99, amin));
  amax = Math.max(0.01, Math.min(0.99, amax));
  if (amin >= amax){ amin = 0.05; amax = 0.90; }
  sm_alpha.min = amin.toFixed(2);
  sm_alpha.max = amax.toFixed(2);
  sm_amin.value = amin.toFixed(2);
  sm_amax.value = amax.toFixed(2);
  // Wenn current alpha außerhalb liegt, in den Bereich schieben (nur Anzeige)
  const val = +sm_alpha.value;
  let nv = val;
  if (val < amin) nv = amin;
  if (val > amax) nv = amax;
  if (nv !== val){
    sm_alpha.value = nv.toFixed(2);
    sm_alpha_val.textContent = nv.toFixed(2);
  }
}

function loadFilter(){
  fetch('/api/filter').then(r=>r.json()).then(f=>{
    sm_on.checked = !!f.smoothing_on;

    const amin = (typeof f.alpha_min === 'number' ? f.alpha_min : 0.05);
    const amax = (typeof f.alpha_max === 'number' ? f.alpha_max : 0.90);
    applyAlphaSliderBounds(amin, amax);

    const a = (typeof f.alpha_ema === 'number' ? f.alpha_ema : 0.30);
    sm_alpha.value = a.toFixed(2);
    sm_alpha_val.textContent = a.toFixed(2);

    min_size_in.value = (f.min_size ?? 2);
    grace_in.value    = (f.blob_grace ?? 3);
  });
}
sm_alpha.addEventListener('input', () => {
  sm_alpha_val.textContent = (+sm_alpha.value).toFixed(2);
});
// min/max Eingaben halten sich konsistent und aktualisieren Sliderbereich
function clampAlphaInputs(){
  let amin = parseFloat(sm_amin.value), amax = parseFloat(sm_amax.value);
  if (isNaN(amin)) amin = 0.05;
  if (isNaN(amax)) amax = 0.90;
  amin = Math.max(0.01, Math.min(0.99, amin));
  amax = Math.max(0.01, Math.min(0.99, amax));
  if (amin >= amax){
    // einfache Regel: schiebe max etwas hoch
    amax = Math.min(0.99, amin + 0.05);
  }
  applyAlphaSliderBounds(amin, amax);
}
sm_amin.addEventListener('change', clampAlphaInputs);
sm_amax.addEventListener('change', clampAlphaInputs);

document.getElementById('btnFiltSave').onclick = () => {
  const body = {
    smoothing_on: sm_on.checked,
    alpha_ema: +sm_alpha.value,
    alpha_min: +sm_amin.value,
    alpha_max: +sm_amax.value,
    min_size: (+min_size_in.value)|0,
    blob_grace: (+grace_in.value)|0
  };
  fetch('/api/filter', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body)
  }).then(() => loadFilter());
};
document.getElementById('btnFiltReload').onclick = loadFilter;

// Init
loadConf(); loadTable(); wifiStatus(); loadFilter();
</script>
</body></html>)HTML";


// Hier im Snippet ausgelassen, damit die Nachricht kurz bleibt.

// ========== API / Server ==========
void handleRoot(){ server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }
void handleNotFound(){ server.sendHeader("Location", String("http://")+AP_IP.toString()); server.send(302, "text/plain", "Redirect"); }
void onWebSocketEvent(uint8_t, WStype_t, uint8_t*, size_t){}

// ---- WiFi REST ----
void api_wifi_status(){
  bool conn = WiFi.status()==WL_CONNECTED;
  String j = String("{\"connected\":") + (conn?"true":"false");
  if (conn){
    j += ",\"ssid\":\"" + WiFi.SSID() + "\"";
    j += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  }
  j += "}";
  server.send(200,"application/json", j);
}
void api_wifi_save(){
  if (server.hasArg("plain")){
    String s = server.arg("plain");
    sta_ssid = jsonPickStr(s, "ssid");
    sta_pwd  = jsonPickStr(s, "pwd");
    savePrefs();

    if (sta_ssid.length()){
      WiFi.disconnect(true, true);
      WiFi.mode(WIFI_STA);
      WiFi.setAutoReconnect(true);
      WiFi.persistent(true);
      WiFi.begin(sta_ssid.c_str(), sta_pwd.c_str());
    }
  }
  server.send(200,"text/plain","OK");
}

// ---- HID APIs ----
void api_hid_toggle(){
  hid_enabled = !hid_enabled; savePrefs();
  server.send(200,"application/json", String("{\"hid\":") + (hid_enabled?"true}":"false}"));
}
void api_hid_testTap(){
  float xn = 0.5f, yn = 0.5f;
  test_active=true;
  Touch.touchDown(f2u16(xn), f2u16(yn));
  vTaskDelay(pdMS_TO_TICKS(120));
  Touch.touchUp();
  test_active=false;
  server.send(200,"text/plain","OK");
}

// ---- Konfig ----
void api_conf_get(){
  String j = "{\"screenW\":" + String(screenW) + ",\"screenH\":" + String(screenH) + "}";
  server.send(200,"application/json", j);
}
void api_conf_screen(){
  if (server.hasArg("plain")){
    String s = server.arg("plain");
    screenW = constrain(jsonGeti(s,"screenW",screenW), 200, 16384);
    screenH = constrain(jsonGeti(s,"screenH",screenH), 200, 16384);
    savePrefs();
  }
  server.send(200,"text/plain","OK");
}

// ---- Kalibrierung ----
void api_calib_reset(){ have_H=false; savePrefs(); server.send(200,"text/plain","OK"); }
void api_calib_point(){
  if (server.hasArg("plain")){
    String s = server.arg("plain");
    int idx=(int)jsonGetf(s,"idx",-1);
    if (idx>=0 && idx<4){
      float sxn=jsonGetf(s,"sx",0), syn=jsonGetf(s,"sy",0), txn=jsonGetf(s,"tx",0), tyn=jsonGetf(s,"ty",0);
      calib_sx[idx] = constrain(sxn * IR_X_MAX, 0.0f, (float)IR_X_MAX);
      calib_sy[idx] = constrain(syn * IR_Y_MAX, 0.0f, (float)IR_Y_MAX);
      calib_tx[idx] = constrain(txn * screenW,  0.0f, (float)screenW);
      calib_ty[idx] = constrain(tyn * screenH,  0.0f, (float)screenH);
    }
  }
  server.send(200,"text/plain","OK");
}
void api_calib_commit(){
  have_H = computeH(calib_sx,calib_sy,calib_tx,calib_ty);
  savePrefs();
  server.send(200,"text/plain", have_H?"OK":"ERR");
}
void api_calib_table_get(){
  String j="{\"sx\":[";
  for(int i=0;i<4;i++){ if(i) j+=','; j+=String(calib_sx[i],0); }
  j+="],\"sy\":[";
  for(int i=0;i<4;i++){ if(i) j+=','; j+=String(calib_sy[i],0); }
  j+="],\"tx\":[";
  for(int i=0;i<4;i++){ if(i) j+=','; j+=String(calib_tx[i],0); }
  j+="],\"ty\":[";
  for(int i=0;i<4;i++){ if(i) j+=','; j+=String(calib_ty[i],0); }
  j+="]}";
  server.send(200,"application/json", j);
}
static void parseArrayInto(const String& s, const char* key, float* dst, int maxv, int count=4){
  int k = s.indexOf(String("\"") + key + "\"");
  if (k < 0) return;
  int a = s.indexOf("[", k), b = s.indexOf("]", a);
  if (a < 0 || b < 0) return;
  String arr = s.substring(a+1, b);
  for (int i=0; i<count; i++){
    int c = arr.indexOf(",", 0);
    String tok = (i < count-1 && c >= 0) ? arr.substring(0, c) : arr;
    tok.trim();
    float val = tok.toFloat();
    if (val < 0) val = 0;
    if (val > maxv) val = maxv;
    dst[i] = val;
    if (i < count-1 && c >= 0) arr = arr.substring(c+1);
  }
}
void api_calib_table_set(){
  if (server.hasArg("plain")){
    String s = server.arg("plain");
    parseArrayInto(s, "sx", calib_sx, IR_X_MAX);
    parseArrayInto(s, "sy", calib_sy, IR_Y_MAX);
    parseArrayInto(s, "tx", calib_tx, screenW);
    parseArrayInto(s, "ty", calib_ty, screenH);
    have_H=false; savePrefs();
  }
  server.send(200,"text/plain","OK");
}

// ---- Filter/Blob Settings (NEU) ----
void api_filter_get(){
  String j = "{";
  j += "\"smoothing_on\":"; j += (smoothing_on ? "true" : "false");
  j += ",\"alpha_ema\":";   j += String(alpha_ema, 3);
  j += ",\"alpha_min\":";   j += String(alpha_min, 2);
  j += ",\"alpha_max\":";   j += String(alpha_max, 2);
  j += ",\"min_size\":";    j += String(min_size);
  j += ",\"blob_grace\":";  j += String(blobGraceMax);
  j += "}";
  server.send(200, "application/json", j);
}
void api_filter_set(){
  if (server.hasArg("plain")){
    String s = server.arg("plain");

    // smoothing_on (bool)
    smoothing_on = jsonGetb(s, "smoothing_on", smoothing_on);

    // alpha_min / alpha_max zuerst lesen und sanieren
    float amin = jsonGetf(s, "alpha_min", alpha_min);
    float amax = jsonGetf(s, "alpha_max", alpha_max);
    if (amin < 0.01f) amin = 0.01f;
    if (amax > 0.99f) amax = 0.99f;
    if (amin >= amax){ amin = 0.05f; amax = 0.90f; }
    alpha_min = amin; alpha_max = amax;

    // alpha_ema (float) dann clampen auf [amin..amax]
    float a = jsonGetf(s, "alpha_ema", alpha_ema);
    if (a < alpha_min) a = alpha_min; else if (a > alpha_max) a = alpha_max;
    alpha_ema = a;

    // min_size (0..15)
    int ms = jsonGeti(s, "min_size", min_size);
    if (ms < 0) ms = 0; else if (ms > 15) ms = 15;
    min_size = (uint8_t)ms;

    // blobGraceMax (0..50)
    int bg = jsonGeti(s, "blob_grace", blobGraceMax);
    if (bg < 0) bg = 0; else if (bg > 50) bg = 50;
    blobGraceMax = bg;

    savePrefs();
  }
  server.send(200,"text/plain","OK");
}

// Webserver Setup
void setupWeb(){
  server.on("/", handleRoot);

  // HID
  server.on("/api/hid/toggle",  HTTP_POST, api_hid_toggle);
  server.on("/api/hid/testTap", HTTP_POST, api_hid_testTap);

  // Wi‑Fi
  server.on("/api/wifi/status", HTTP_GET,  api_wifi_status);
  server.on("/api/wifi/save",   HTTP_POST, api_wifi_save);

  // Kalibrierung
  server.on("/api/calib/reset",  HTTP_POST, api_calib_reset);
  server.on("/api/calib/point",  HTTP_POST, api_calib_point);
  server.on("/api/calib/commit", HTTP_POST, api_calib_commit);
  server.on("/api/calib/table",  HTTP_GET,  api_calib_table_get);
  server.on("/api/calib/table",  HTTP_POST, api_calib_table_set);

  // Konfiguration
  server.on("/api/conf",        HTTP_GET,  api_conf_get);
  server.on("/api/conf/screen", HTTP_POST, api_conf_screen);

  // Filter/Blob Settings (NEU)
  server.on("/api/filter", HTTP_GET,  api_filter_get);
  server.on("/api/filter", HTTP_POST, api_filter_set);

  server.onNotFound(handleNotFound);
  server.begin();

  ws.begin();
  ws.onEvent(onWebSocketEvent);
}

// ====== Wi‑Fi Ablauf (AP nur wenn STA nicht greift) ======
void startNetwork(){
  WiFi.setSleep(false);
  if (sta_ssid.length()){
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    WiFi.begin(sta_ssid.c_str(), sta_pwd.c_str());

    uint32_t t0=millis();
    while (WiFi.status()!=WL_CONNECTED && millis()-t0 < 5000) { delay(100); }
    if (WiFi.status()==WL_CONNECTED){
      return;
    }
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  dnsServer.start(DNS_PORT, "*", AP_IP);
}

// ====== HID Setup ======
void setupHID(){
  USB.begin();
  Touch.begin();
}

// =========================
// ====== Edge-Aware EMA ===
// =========================
static inline float smoothEMA(float prev, float curr, float alphaBase) {
  float d = fabsf(curr - prev);
  float alpha = alphaBase;
  if (d > 0.10f)      alpha *= 0.20f; // sehr großer Sprung -> sehr stark glätten
  else if (d > 0.04f) alpha *= 0.40f; // mittel -> moderat glätten
  else if (d > 0.01f) alpha *= 0.70f; // kleine Abweichung -> leicht glätten
  return prev + alpha * (curr - prev);
}

// ====== Setup / Loop ======
void setup() {
  // I2C Start
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  Wire.setTimeOut(25);

  // IR-Cam Init
  ir_present = ir_init_full_sensitivity();

  loadPrefs();
  startNetwork();
  setupWeb();
  setupHID();

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  ws.loop();

  // WS Broadcast ~40 ms
  static uint32_t lastWS = 0; uint32_t now = millis();
  if (now - lastWS > 40) {
    lastWS = now;

    // Für Kamera-Canvas zusätzlich Sensor-absolute Werte mitsenden und Kalibrier-Polygon
    String j = "{\"hid\":"; j += (hid_enabled?"true":"false");
    j += ",\"ir\":"; j += (ir_present?"true":"false");
    j += ",\"blobs\":[";
    for (int i=0;i<4;i++){
      if (i) j+=",";
      int sx = (int)round(blobs[i].x * IR_X_MAX);
      int sy = (int)round(blobs[i].y * IR_Y_MAX);
      j += "{\"x\":" + String(blobs[i].x,3)
        +  ",\"y\":" + String(blobs[i].y,3)
        +  ",\"sx\":"+ String(sx)
        +  ",\"sy\":"+ String(sy)
        +  ",\"valid\":" + String(blobs[i].valid?"true":"false") + "}";
    }
    j += "],";

    bool any=false; float xn=0, yn=0;
    for(int i=0;i<4;i++){ if(touches[i].active){ any=true; xn=touches[i].x; yn=touches[i].y; break; } }
    j += "\"touch\":{"; 
    if (any) j += "\"x\":"+String(xn,3)+",\"y\":"+String(yn,3);
    j += "},";

    j += "\"poly\":[";
    for(int i=0;i<4;i++){
      if (i) j+=",";
      float px = calib_sx[i] / (float)IR_X_MAX;
      float py = calib_sy[i] / (float)IR_Y_MAX;
      j += "{\"x\":"+String(px,3)+",\"y\":"+String(py,3)+"}";
    }
    j += "]}";

    ws.broadcastTXT(j);
  }
}

// ====== Sensor-Task ======
void sensorTask(void*) {
  const TickType_t period = 1;
  TickType_t last = xTaskGetTickCount();
  static uint16_t ir_fail = 0;

  for (;;) {
    bool got = ir_read();

    bool frameAny = false;
    for (int i=0;i<4;i++) if (blobs[i].valid) { frameAny = true; break; }

    if (got && frameAny){
      ir_fail = 0;
      for (int i=0;i<4;i++){
        if (blobs[i].valid){
          blobGrace[i] = blobGraceMax;
          if (!filtInit[i]){
            filtX[i]=blobs[i].x;
            filtY[i]=blobs[i].y;
            filtInit[i]=true;
          } else if (smoothing_on){
            // Edge-Aware EMA mit alpha_ema (geclamped über API/UI)
            filtX[i] = smoothEMA(filtX[i], blobs[i].x, alpha_ema);
            filtY[i] = smoothEMA(filtY[i], blobs[i].y, alpha_ema);
          } else {
            filtX[i]=blobs[i].x; filtY[i]=blobs[i].y;
          }
        } else {
          if (blobGrace[i] > 0) blobGrace[i]--;
          else filtInit[i] = false;
        }
      }
    } else {
      ir_fail++;
      for (int i=0;i<4;i++){
        blobs[i].valid = false;
        filtInit[i]    = false;
        blobGrace[i]   = 0;
      }
      // Auto-Recovery
      if (ir_fail > 50) {
        Wire.end(); delay(5);
        Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
        Wire.setTimeOut(25);
        ir_present = ir_init_full_sensitivity();
        ir_fail = 0;
      }
    }

    // Mapping → Touches (Displaynorm 0..1)
    for (int i=0;i<4;i++){
      touches[i].active = false;
      if (blobs[i].valid){
        float xn = (filtInit[i] ? filtX[i] : blobs[i].x);
        float yn = (filtInit[i] ? filtY[i] : blobs[i].y);

        float sx = xn * IR_X_MAX;
        float sy = yn * IR_Y_MAX;

        float Xp, Yp;
        if (have_H) {
          Xp = sx; Yp = sy;
          applyH(sx, sy, Xp, Yp); // -> Pixelraum
        } else {
          Xp = xn * screenW;
          Yp = yn * screenH;
        }

        if (Xp<0) Xp=0; if (Yp<0) Yp=0;
        if (Xp>screenW) Xp=screenW;
        if (Yp>screenH) Yp=screenH;

        touches[i].x = (screenW? Xp/screenW : 0);
        touches[i].y = (screenH? Yp/screenH : 0);
        touches[i].active = true; touches[i].id=i;
      }
    }

    // Live HID: erster aktiver Touch
    if (!test_active && hid_enabled) {
      bool any=false; float xn=0, yn=0;
      for(int i=0;i<4;i++){ if(touches[i].active){ any=true; xn=touches[i].x; yn=touches[i].y; break; } }
      if (any){
        uint16_t hx=f2u16(xn), hy=f2u16(yn);
        if (!live_down){ Touch.touchDown(hx,hy); live_down=true; }
        else           { Touch.touchMove(hx,hy); }
      } else {
        if (live_down){ Touch.touchUp(); live_down=false; }
      }
    }

    // === AUTO SENSITIVITY ADJUSTMENT (nur wenn Blobs im Frame) ===
    if (frameAny) {
      float avgf = (float)averageBlobSize();
      avgSizeSmooth = (avgSizeSmooth == 0 ? avgf : (0.8f*avgSizeSmooth + 0.2f*avgf));
      uint8_t avg = (uint8_t)(avgSizeSmooth + 0.5f);

      uint32_t now = millis();
      if (now - lastSensChange > 400) {  // mind. alle 400ms wechseln
        if      (avg > 10) targetSens = 2;  // nahe → Gain runter
        else if (avg > 5)  targetSens = 4;  // mittel
        else if (avg > 0)  targetSens = 6;  // weit → max Reichweite
        else               targetSens = 6;  // kein Pen → Suche

        if (targetSens != currentSens)
          applySensitivityLevel(targetSens);
      }
    }

    // === AUTO EXPOSURE / AUTO SHUTTER ===
    updateAutoExposure();

    vTaskDelayUntil(&last, period);
  }
}

// ====== IR Read ======
bool ir_read() {
  if (!ir_present) return false;

  uint8_t raw[16];

  // Pointer setzen + Read
  Wire.beginTransmission(IR_ADDR_7BIT);
  Wire.write(IR_PTR_REG);
  if (Wire.endTransmission(true) != 0) {
    ir_present = false; return false;
  }
  int n = Wire.requestFrom((uint16_t)IR_ADDR_7BIT, (size_t)16, (bool)true);
  if (n != 16) {
    // Fallback: Repeated Start
    Wire.beginTransmission(IR_ADDR_7BIT);
    Wire.write(IR_PTR_REG);
    if (Wire.endTransmission(false) != 0) { ir_present=false; return false; }
    n = Wire.requestFrom((uint16_t)IR_ADDR_7BIT, (size_t)16, (bool)true);
    if (n != 16) { ir_present=false; return false; }
  }
  for (int i=0;i<16;i++) raw[i]=Wire.read();

  Blob tmp[4];
  decodeBlobStrict(raw, 1,  tmp[0]);
  decodeBlobStrict(raw, 4,  tmp[1]);
  decodeBlobStrict(raw, 7,  tmp[2]);
  decodeBlobStrict(raw, 10, tmp[3]);

  bool any=false; for (int i=0;i<4;i++) if (tmp[i].valid){ any=true; break; }
  if (!any) for (int i=0;i<4;i++) blobs[i]=Blob(); else for (int i=0;i<4;i++) blobs[i]=tmp[i];

  ir_present = true;
  return true;
}