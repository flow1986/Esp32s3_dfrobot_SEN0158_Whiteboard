#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <USB.h>
#include <USBHID.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "web_ui.h"

namespace {
constexpr uint8_t kIrAddress = 0x58;
constexpr uint8_t kIrPointer = 0x36;
constexpr uint16_t kSensorWidth = 1023;
constexpr uint16_t kSensorHeight = 767;
constexpr uint8_t kSdaPin = 5;
constexpr uint8_t kSclPin = 6;
constexpr uint32_t kI2cFrequency = 400000;
constexpr uint8_t kMaxSmoothWindow = 16;
constexpr float kTrackJumpPx = 150.0f;
constexpr uint8_t kMaxReadFailures = 20;
constexpr char kApSsid[] = "IR-Whiteboard";
constexpr char kApPassword[] = "whiteboard";

struct IrPoint {
  uint16_t x = 0;
  uint16_t y = 0;
  uint8_t size = 0;
  bool valid = false;
};

struct CalibrationPoint {
  float sx = 0;
  float sy = 0;
  float tx = 0;
  float ty = 0;
};

USBHID hid;
WebServer server(80);
WebSocketsServer socketServer(81);
DNSServer dnsServer;
Preferences preferences;
IrPoint points[4];
CalibrationPoint calibration[4];
float homography[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
bool calibrated = false;
uint8_t calibrationMask = 0;
bool sensorPresent = false;
bool apStarted = false;
bool hidEnabled = true;
bool calibrationMode = false;
bool contactActive = false;
bool mouseMode = false;
bool mouseTracking = false;
bool mouseClickStarted = false;
bool mouseRightClick = false;
uint16_t longPressMs = 650;
uint32_t mousePressStartedAt = 0;
float mouseStartX = 0;
float mouseStartY = 0;
float mouseLastX = 0;
float mouseLastY = 0;
uint8_t smoothWindow = 4;
uint16_t holdMs = 80;
uint8_t minBlobSize = 1;
uint8_t sensitivityLevel = 6;
uint16_t screenWidth = 1920;
uint16_t screenHeight = 1080;
float filteredX = 0;
float filteredY = 0;
bool filterInitialized = false;
bool frameChanged = false;
volatile bool sensitivityDirty = false;
bool trackActive = false;
bool trackedValid = false;
IrPoint trackedPoint;
float trackX = 0;
float trackY = 0;
uint32_t lastSeenAt = 0;
float smoothBufferX[kMaxSmoothWindow];
float smoothBufferY[kMaxSmoothWindow];
uint8_t smoothIndex = 0;
uint8_t smoothCount = 0;
portMUX_TYPE homographyLock = portMUX_INITIALIZER_UNLOCKED;
String stationSsid;
String stationPassword;

class TouchDevice final : public USBHIDDevice {
 public:
  TouchDevice() { hid.addDevice(this, sizeof(reportMap)); }
  void begin() { hid.begin(); }

  void report(uint16_t x, uint16_t y, bool down) {
    struct __attribute__((packed)) Report {
      uint8_t buttons;
      uint8_t contactId;
      uint16_t x;
      uint16_t y;
      uint8_t contactCount;
    } value{};
    value.buttons = down ? 0x03 : 0;
    value.contactId = 0;
    value.x = x;
    value.y = y;
    value.contactCount = down ? 1 : 0;
    hid.SendReport(1, &value, sizeof(value));
  }

 protected:
  uint16_t _onGetDescriptor(uint8_t* destination) override {
    memcpy(destination, reportMap, sizeof(reportMap));
    return sizeof(reportMap);
  }

 private:
  static constexpr uint8_t reportMap[] = {
      0x05, 0x0D, 0x09, 0x04, 0xA1, 0x01, 0x85, 0x01,
      0x09, 0x22, 0xA1, 0x02, 0x09, 0x42, 0x09, 0x32,
      0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x02,
      0x81, 0x02, 0x75, 0x06, 0x95, 0x01, 0x81, 0x03,
      0x09, 0x51, 0x15, 0x00, 0x25, 0x7F, 0x75, 0x08,
      0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x30,
      0x09, 0x31, 0x16, 0x00, 0x00, 0x26, 0xFF, 0x7F,
      0x75, 0x10, 0x95, 0x02, 0x81, 0x02, 0xC0, 0x09,
      0x54, 0x15, 0x00, 0x25, 0x0A, 0x75, 0x08, 0x95,
      0x01, 0x81, 0x02, 0xC0};
};

    constexpr uint8_t TouchDevice::reportMap[];

TouchDevice touchDevice;

    class MouseDevice final : public USBHIDDevice {
     public:
      MouseDevice() { hid.addDevice(this, sizeof(reportMap)); }

      void report(uint8_t buttons, int8_t x, int8_t y) {
        struct __attribute__((packed)) Report {
          uint8_t buttons;
          int8_t x;
          int8_t y;
        } value{buttons, x, y};
        hid.SendReport(2, &value, sizeof(value));
      }

     protected:
      uint16_t _onGetDescriptor(uint8_t* destination) override {
        memcpy(destination, reportMap, sizeof(reportMap));
        return sizeof(reportMap);
      }

     private:
      static constexpr uint8_t reportMap[] = {
          0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x02,
          0x09, 0x01, 0xA1, 0x00, 0x05, 0x09, 0x19, 0x01,
          0x29, 0x03, 0x15, 0x00, 0x25, 0x01, 0x95, 0x03,
          0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
          0x81, 0x03, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
          0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02,
          0x81, 0x06, 0xC0, 0xC0};
    };

    constexpr uint8_t MouseDevice::reportMap[];

    MouseDevice mouseDevice;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kIrAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool writeSensitivity() {
  // Stufen 1-5 stammen aus dem Altprojekt; Stufe 6 ist das Preset aus Johnny Lees WiimoteWhiteboard.
  static const uint8_t block1[6][9] = {
      {0x02, 0, 0, 0x71, 0x01, 0, 0x40, 0, 0x55},
      {0x02, 0, 0, 0x71, 0x01, 0, 0x55, 0, 0x63},
      {0x02, 0, 0, 0x71, 0x01, 0, 0x5A, 0, 0x6A},
      {0x02, 0, 0, 0x71, 0x01, 0, 0x64, 0, 0xFE},
      {0x02, 0, 0, 0x71, 0x01, 0, 0x96, 0, 0xFE},
      {0x00, 0, 0, 0x00, 0x00, 0, 0x90, 0, 0x41}};
  static const uint8_t block2[2] = {0x40, 0x00};
  Wire.beginTransmission(kIrAddress);
  Wire.write(0x00);
  Wire.write(block1[constrain(sensitivityLevel, 1, 6) - 1], sizeof(block1[0]));
  const bool first = Wire.endTransmission() == 0;
  Wire.beginTransmission(kIrAddress);
  Wire.write(0x1A);
  Wire.write(block2, sizeof(block2));
  return first && Wire.endTransmission() == 0;
}

bool initializeSensor() {
  bool ok = writeRegister(0x30, 0x01);
  delay(10);
  ok &= writeRegister(0x30, 0x08);
  delay(10);
  ok &= writeSensitivity();
  delay(10);
  ok &= writeRegister(0x33, 0x33);
  delay(100);
  return ok;
}

bool readSensor() {
  Wire.beginTransmission(kIrAddress);
  Wire.write(kIrPointer);
  if (Wire.endTransmission() != 0 ||
      Wire.requestFrom(kIrAddress, static_cast<uint8_t>(16)) != 16) {
    return false;
  }
  uint8_t raw[16];
  for (uint8_t& byte : raw) byte = Wire.read();
  static uint8_t lastRaw[16];
  frameChanged = memcmp(raw, lastRaw, sizeof(raw)) != 0;
  memcpy(lastRaw, raw, sizeof(raw));
  IrPoint decoded[4];
  constexpr uint8_t offsets[] = {1, 4, 7, 10};
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t offset = offsets[i];
    const uint8_t packed = raw[offset + 2];
    decoded[i].x = raw[offset] | ((packed & 0x30) << 4);
    decoded[i].y = raw[offset + 1] | ((packed & 0xC0) << 2);
    decoded[i].size = packed & 0x0F;
    decoded[i].valid = decoded[i].x != kSensorWidth &&
                       decoded[i].y != kSensorWidth &&
                       decoded[i].y <= kSensorHeight &&
                       decoded[i].size >= minBlobSize;
  }
  memcpy(points, decoded, sizeof(points));
  return true;
}

bool solveHomography() {
  double matrix[8][9] = {};
  for (uint8_t i = 0; i < 4; ++i) {
    const double x = calibration[i].sx;
    const double y = calibration[i].sy;
    const double u = calibration[i].tx;
    const double v = calibration[i].ty;
    const uint8_t row = i * 2;
    matrix[row][0] = x;
    matrix[row][1] = y;
    matrix[row][2] = 1;
    matrix[row][6] = -x * u;
    matrix[row][7] = -y * u;
    matrix[row][8] = u;
    matrix[row + 1][3] = x;
    matrix[row + 1][4] = y;
    matrix[row + 1][5] = 1;
    matrix[row + 1][6] = -x * v;
    matrix[row + 1][7] = -y * v;
    matrix[row + 1][8] = v;
  }
  for (uint8_t column = 0; column < 8; ++column) {
    uint8_t pivot = column;
    for (uint8_t row = column + 1; row < 8; ++row) {
      if (fabs(matrix[row][column]) > fabs(matrix[pivot][column])) pivot = row;
    }
    if (fabs(matrix[pivot][column]) < 1e-8) return false;
    if (pivot != column) {
      for (uint8_t j = column; j < 9; ++j) {
        const double temporary = matrix[column][j];
        matrix[column][j] = matrix[pivot][j];
        matrix[pivot][j] = temporary;
      }
    }
    const double divisor = matrix[column][column];
    for (uint8_t j = column; j < 9; ++j) matrix[column][j] /= divisor;
    for (uint8_t row = 0; row < 8; ++row) {
      if (row == column) continue;
      const double factor = matrix[row][column];
      for (uint8_t j = column; j < 9; ++j) matrix[row][j] -= factor * matrix[column][j];
    }
  }
  float result[9];
  for (uint8_t i = 0; i < 8; ++i) result[i] = static_cast<float>(matrix[i][8]);
  result[8] = 1;
  portENTER_CRITICAL(&homographyLock);
  memcpy(homography, result, sizeof(result));
  portEXIT_CRITICAL(&homographyLock);
  return true;
}

bool mapPoint(float x, float y, float& mappedX, float& mappedY) {
  float h[9];
  portENTER_CRITICAL(&homographyLock);
  memcpy(h, homography, sizeof(h));
  portEXIT_CRITICAL(&homographyLock);
  const float denominator = h[6] * x + h[7] * y + h[8];
  if (fabsf(denominator) < 1e-6f) return false;
  mappedX = (h[0] * x + h[1] * y + h[2]) / denominator;
  mappedY = (h[3] * x + h[4] * y + h[5]) / denominator;
  return true;
}

void saveSettings();

void loadSettings() {
  preferences.begin("whiteboard", true);
  screenWidth = preferences.getUShort("width", 1920);
  screenHeight = preferences.getUShort("height", 1080);
  // Ältere Firmware kannte diese Tracking-Werte nicht; sie werden einmalig auf die neuen Standards gesetzt.
  const bool migrate = preferences.getUChar("cfgVer", 0) < 2;
  minBlobSize = migrate ? 1 : preferences.getUChar("minSize", 1);
  sensitivityLevel = migrate ? 6 : preferences.getUChar("sensitivity", 6);
  smoothWindow = migrate ? 4 : preferences.getUChar("window", 4);
  holdMs = migrate ? 80 : preferences.getUShort("hold", 80);
  hidEnabled = preferences.getBool("hid", true);
  mouseMode = preferences.getBool("mouseMode", false);
  longPressMs = preferences.getUShort("longPress", 650);
  stationSsid = preferences.getString("ssid", "");
  stationPassword = preferences.getString("password", "");
  calibrated = preferences.getBool("calibrated", false);
  calibrationMask = preferences.getUChar("calMask", 0);
  preferences.getBytes("cal", calibration, sizeof(calibration));
  preferences.end();
  if (calibrated) calibrated = calibrationMask == 0x0F && solveHomography();
  if (migrate) saveSettings();
}

void saveSettings() {
  preferences.begin("whiteboard", false);
  preferences.putUShort("width", screenWidth);
  preferences.putUShort("height", screenHeight);
  preferences.putUChar("minSize", minBlobSize);
  preferences.putUChar("sensitivity", sensitivityLevel);
  preferences.putUChar("cfgVer", 2);
  preferences.putUChar("window", smoothWindow);
  preferences.putUShort("hold", holdMs);
  preferences.putBool("hid", hidEnabled);
  preferences.putBool("mouseMode", mouseMode);
  preferences.putUShort("longPress", longPressMs);
  preferences.putString("ssid", stationSsid);
  preferences.putString("password", stationPassword);
  preferences.putBool("calibrated", calibrated);
  preferences.putUChar("calMask", calibrationMask);
  preferences.putBytes("cal", calibration, sizeof(calibration));
  preferences.end();
}

bool findPrimaryPoint(IrPoint& point) {
  if (!trackedValid) return false;
  point = trackedPoint;
  return true;
}

void sendTouch(bool down, float x, float y) {
  const uint16_t hidX = static_cast<uint16_t>(constrain(x / screenWidth, 0.0f, 1.0f) * 32767);
  const uint16_t hidY = static_cast<uint16_t>(constrain(y / screenHeight, 0.0f, 1.0f) * 32767);
  touchDevice.report(hidX, hidY, down);
}

void resetMouseGesture() {
  mouseTracking = false;
  mouseClickStarted = false;
  mouseRightClick = false;
}

void releaseMouse() {
  mouseDevice.report(0, 0, 0);
  resetMouseGesture();
}

void updateMouse(bool detected, float x, float y) {
  if (!detected) {
    if (mouseTracking) {
      if (!mouseClickStarted) {
        mouseDevice.report(1, 0, 0);
        mouseDevice.report(0, 0, 0);
      } else {
        mouseDevice.report(0, 0, 0);
      }
      resetMouseGesture();
    }
    return;
  }

  if (!mouseTracking) {
    mouseTracking = true;
    mousePressStartedAt = millis();
    mouseStartX = x;
    mouseStartY = y;
    mouseLastX = x;
    mouseLastY = y;
    return;
  }

  const int8_t dx = constrain(lroundf(x - mouseLastX), -127, 127);
  const int8_t dy = constrain(lroundf(y - mouseLastY), -127, 127);
  mouseLastX = x;
  mouseLastY = y;
  const bool dragged = fabsf(x - mouseStartX) + fabsf(y - mouseStartY) >= 6.0f;
  uint8_t buttons = 0;
  if (mouseRightClick) {
    buttons = 2;
  } else if (!dragged && millis() - mousePressStartedAt >= longPressMs) {
    mouseRightClick = true;
    mouseClickStarted = true;
    buttons = 2;
  } else if (dragged || mouseClickStarted) {
    mouseClickStarted = true;
    buttons = 1;
  }
  mouseDevice.report(buttons, dx, dy);
}

void appendLiveData(JsonDocument& document) {
  IrPoint primary;
  const bool hasPoint = findPrimaryPoint(primary);
  document["sensor"] = sensorPresent;
  document["hid"] = hidEnabled;
  document["calibrationMode"] = calibrationMode;
  document["contact"] = contactActive;
  document["mouseMode"] = mouseMode;
  document["calibrated"] = calibrated;
  document["apStarted"] = apStarted;
  document["screenWidth"] = screenWidth;
  document["screenHeight"] = screenHeight;
  document["calibrationMask"] = calibrationMask;
  document["apIp"] = WiFi.softAPIP().toString();
  document["staConnected"] = WiFi.status() == WL_CONNECTED;
  document["staIp"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  document["staSsid"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "";
  document["point"]["valid"] = hasPoint;
  if (hasPoint) {
    const float sensorX = filterInitialized ? filteredX : primary.x;
    const float sensorY = filterInitialized ? filteredY : primary.y;
    float x = sensorX;
    float y = sensorY;
    if (calibrated) mapPoint(x, y, x, y);
    document["point"]["sensorX"] = sensorX;
    document["point"]["sensorY"] = sensorY;
    document["point"]["x"] = x / screenWidth;
    document["point"]["y"] = y / screenHeight;
    document["point"]["size"] = primary.size;
  }
  JsonArray blobs = document["blobs"].to<JsonArray>();
  for (const IrPoint& point : points) {
    JsonObject blob = blobs.add<JsonObject>();
    blob["valid"] = point.valid;
    blob["x"] = static_cast<float>(point.x) / kSensorWidth;
    blob["y"] = static_cast<float>(point.y) / kSensorHeight;
  }
  JsonArray calibrationData = document["calibration"].to<JsonArray>();
  for (uint8_t i = 0; i < 4; ++i) {
    JsonObject item = calibrationData.add<JsonObject>();
    item["valid"] = calibrationMask & (1U << i);
    item["sensorX"] = calibration[i].sx / kSensorWidth;
    item["sensorY"] = calibration[i].sy / kSensorHeight;
    item["targetX"] = calibration[i].tx / screenWidth;
    item["targetY"] = calibration[i].ty / screenHeight;
  }
}

void sendStatus() {
  JsonDocument document;
  appendLiveData(document);
  String output;
  serializeJson(document, output);
  socketServer.broadcastTXT(output);
}

void handleStatus() {
  JsonDocument document;
  appendLiveData(document);
  String output;
  serializeJson(document, output);
  server.send(200, "application/json", output);
}

void handleConfigGet() {
  JsonDocument document;
  document["screenWidth"] = screenWidth;
  document["screenHeight"] = screenHeight;
  document["minBlobSize"] = minBlobSize;
  document["sensitivity"] = sensitivityLevel;
  document["smoothWindow"] = smoothWindow;
  document["holdMs"] = holdMs;
  document["hid"] = hidEnabled;
  document["mouseMode"] = mouseMode;
  document["longPressMs"] = longPressMs;
  document["ssid"] = stationSsid;
  String output;
  serializeJson(document, output);
  server.send(200, "application/json", output);
}

void handleConfigPost() {
  JsonDocument document;
  if (deserializeJson(document, server.arg("plain"))) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }
  screenWidth = constrain(document["screenWidth"] | screenWidth, 320, 16384);
  screenHeight = constrain(document["screenHeight"] | screenHeight, 240, 16384);
  minBlobSize = constrain(document["minBlobSize"] | minBlobSize, 0, 15);
  const uint8_t requestedSensitivity = constrain(document["sensitivity"] | sensitivityLevel, 1, 6);
  if (requestedSensitivity != sensitivityLevel) {
    sensitivityLevel = requestedSensitivity;
    sensitivityDirty = true;
  }
  smoothWindow = constrain(document["smoothWindow"] | smoothWindow, 1, kMaxSmoothWindow);
  holdMs = constrain(document["holdMs"] | holdMs, 0, 500);
  const bool requestedHid = document["hid"] | hidEnabled;
  if (hidEnabled && !requestedHid) {
    if (mouseMode) releaseMouse();
    else if (contactActive) sendTouch(false, 0, 0);
  }
  const bool requestedMouseMode = document["mouseMode"] | mouseMode;
  if (mouseMode != requestedMouseMode) {
    if (contactActive && !mouseMode) sendTouch(false, 0, 0);
    releaseMouse();
    contactActive = false;
    mouseMode = requestedMouseMode;
  }
  longPressMs = constrain(document["longPressMs"] | longPressMs, 350, 2000);
  hidEnabled = requestedHid;
  if (!hidEnabled) contactActive = false;
  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleCalibrationPost() {
  JsonDocument document;
  if (deserializeJson(document, server.arg("plain"))) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }
  const uint8_t index = document["index"] | 255;
  IrPoint point;
  if (index >= 4 || !findPrimaryPoint(point)) {
    server.send(409, "text/plain", "No IR point detected");
    return;
  }
  calibration[index].sx = filterInitialized ? filteredX : point.x;
  calibration[index].sy = filterInitialized ? filteredY : point.y;
  calibration[index].tx = static_cast<float>(document["targetX"] | 0.0f) * screenWidth;
  calibration[index].ty = static_cast<float>(document["targetY"] | 0.0f) * screenHeight;
  calibrationMask |= 1U << index;
  calibrated = calibrationMask == 0x0F && solveHomography();
  saveSettings();
  server.send(calibrated ? 200 : 202, "application/json",
              calibrated ? "{\"calibrated\":true}" : "{\"calibrated\":false}");
}

void handleCalibrationPointPost() {
  JsonDocument document;
  if (deserializeJson(document, server.arg("plain"))) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }
  const uint8_t index = document["index"] | 255;
  if (index >= 4 || !(calibrationMask & (1U << index))) {
    server.send(409, "text/plain", "Calibration point does not exist");
    return;
  }
  const float sensorX = document["sensorX"] | (calibration[index].sx / kSensorWidth);
  const float sensorY = document["sensorY"] | (calibration[index].sy / kSensorHeight);
  const float targetX = document["targetX"] | (calibration[index].tx / screenWidth);
  const float targetY = document["targetY"] | (calibration[index].ty / screenHeight);
  if (sensorX < 0 || sensorX > 1 || sensorY < 0 || sensorY > 1 ||
      targetX < 0 || targetX > 1 || targetY < 0 || targetY > 1) {
    server.send(400, "text/plain", "Calibration coordinates must be normalized");
    return;
  }
  calibration[index].sx = sensorX * kSensorWidth;
  calibration[index].sy = sensorY * kSensorHeight;
  calibration[index].tx = targetX * screenWidth;
  calibration[index].ty = targetY * screenHeight;
  calibrated = calibrationMask == 0x0F && solveHomography();
  saveSettings();
  server.send(200, "application/json", calibrated ? "{\"calibrated\":true}" : "{\"calibrated\":false}");
}

void handleCalibrationModePost() {
  JsonDocument document;
  if (deserializeJson(document, server.arg("plain"))) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }
  const bool enabled = document["active"] | false;
  if (enabled && !calibrationMode) {
    if (contactActive && mouseMode) releaseMouse();
    else if (contactActive) sendTouch(false, 0, 0);
    contactActive = false;
    resetMouseGesture();
  }
  calibrationMode = enabled;
  server.send(200, "application/json", enabled ? "{\"active\":true}" : "{\"active\":false}");
}

void handleCalibrationReset() {
  calibrated = false;
  calibrationMask = 0;
  memset(calibration, 0, sizeof(calibration));
  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleTouchTest() {
  if (!hidEnabled) {
    server.send(409, "text/plain", "USB Touch disabled");
    return;
  }
  if (mouseMode) {
    mouseDevice.report(1, 0, 0);
    delay(120);
    mouseDevice.report(0, 0, 0);
  } else {
    sendTouch(true, screenWidth / 2.0f, screenHeight / 2.0f);
    delay(120);
    sendTouch(false, 0, 0);
  }
  contactActive = false;
  server.send(200, "text/plain", "OK");
}

void handleWifiPost() {
  JsonDocument document;
  if (deserializeJson(document, server.arg("plain"))) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }
  stationSsid = document["ssid"] | "";
  stationPassword = document["password"] | "";
  saveSettings();
  server.send(202, "text/plain", "Connecting; access point remains active");
  WiFi.begin(stationSsid.c_str(), stationPassword.c_str());
}

void pushSample(float x, float y) {
  smoothBufferX[smoothIndex] = x;
  smoothBufferY[smoothIndex] = y;
  smoothIndex = (smoothIndex + 1) % kMaxSmoothWindow;
  if (smoothCount < kMaxSmoothWindow) ++smoothCount;
  const uint8_t window = smoothWindow < 1 ? 1 : (smoothWindow > kMaxSmoothWindow ? kMaxSmoothWindow : smoothWindow);
  const uint8_t samples = smoothCount < window ? smoothCount : window;
  float sumX = 0;
  float sumY = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    const uint8_t index = (smoothIndex + kMaxSmoothWindow - 1 - i) % kMaxSmoothWindow;
    sumX += smoothBufferX[index];
    sumY += smoothBufferY[index];
  }
  filteredX = sumX / samples;
  filteredY = sumY / samples;
  filterInitialized = true;
}

// Bei laufendem Kontakt zaehlt nur der Blob nahe der letzten Position; sonst der groesste.
void selectTrackedPoint() {
  int selected = -1;
  if (trackActive) {
    float best = kTrackJumpPx * kTrackJumpPx;
    for (uint8_t i = 0; i < 4; ++i) {
      if (!points[i].valid) continue;
      const float dx = points[i].x - trackX;
      const float dy = points[i].y - trackY;
      const float distance = dx * dx + dy * dy;
      if (distance <= best) {
        best = distance;
        selected = i;
      }
    }
  } else {
    for (uint8_t i = 0; i < 4; ++i) {
      if (points[i].valid && (selected < 0 || points[i].size > points[selected].size)) selected = i;
    }
  }
  trackedValid = selected >= 0;
  if (trackedValid) {
    trackedPoint = points[selected];
    trackX = trackedPoint.x;
    trackY = trackedPoint.y;
  }
}

void endContact() {
  if (mouseMode) updateMouse(false, 0, 0);
  else if (contactActive) sendTouch(false, 0, 0);
  contactActive = false;
  trackActive = false;
  filterInitialized = false;
  smoothCount = 0;
}

void trackStep(uint32_t now) {
  selectTrackedPoint();
  if (trackedValid) {
    if (!trackActive) smoothCount = 0;
    trackActive = true;
    lastSeenAt = now;
    if (frameChanged || !filterInitialized) pushSample(trackedPoint.x, trackedPoint.y);
    float mappedX = filteredX;
    float mappedY = filteredY;
    if (calibrated) mapPoint(filteredX, filteredY, mappedX, mappedY);
    mappedX = constrain(mappedX, 0.0f, static_cast<float>(screenWidth));
    mappedY = constrain(mappedY, 0.0f, static_cast<float>(screenHeight));
    if (calibrationMode || !hidEnabled) {
      contactActive = false;
    } else if (mouseMode) {
      updateMouse(true, mappedX, mappedY);
      contactActive = true;
    } else if (frameChanged || !contactActive) {
      sendTouch(true, mappedX, mappedY);
      contactActive = true;
    }
  } else if (trackActive && now - lastSeenAt > holdMs) {
    endContact();
  }
}

void sensorTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastRetry = 0;
  uint8_t failures = 0;
  for (;;) {
    const uint32_t now = millis();
    if (!sensorPresent) {
      if (now - lastRetry >= 5000) {
        lastRetry = now;
        failures = 0;
        sensorPresent = initializeSensor();
      }
    } else {
      if (sensitivityDirty) {
        sensitivityDirty = false;
        writeSensitivity();
      }
      if (readSensor()) {
        failures = 0;
        trackStep(now);
      } else if (++failures >= kMaxReadFailures) {
        sensorPresent = false;
        memset(points, 0, sizeof(points));
        trackedValid = false;
        endContact();
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
  }
}

void setupNetwork() {
  WiFi.setSleep(false);
  const bool apStaMode = WiFi.mode(WIFI_AP_STA);
  delay(100);
  apStarted = apStaMode && WiFi.softAP(kApSsid, kApPassword, 1, false, 4);
  if (!apStarted) {
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_AP);
    delay(100);
    apStarted = WiFi.softAP(kApSsid, kApPassword, 1, false, 4);
  }
  if (apStarted) {
    dnsServer.start(53, "*", WiFi.softAPIP());
    Serial.printf("AP started: SSID=%s IP=%s\n", kApSsid, WiFi.softAPIP().toString().c_str());
    if (stationSsid.length() && WiFi.getMode() == WIFI_AP_STA) {
      WiFi.begin(stationSsid.c_str(), stationPassword.c_str());
    }
  } else {
    Serial.printf("AP FAILED: SSID=%s\n", kApSsid);
  }
}

void setupWeb() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", WEB_UI_HTML);
  });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_GET, handleConfigGet);
  server.on("/api/config", HTTP_POST, handleConfigPost);
  server.on("/api/calibrate", HTTP_POST, handleCalibrationPost);
  server.on("/api/calibration/point", HTTP_POST, handleCalibrationPointPost);
  server.on("/api/calibration/mode", HTTP_POST, handleCalibrationModePost);
  server.on("/api/calibration/reset", HTTP_POST, handleCalibrationReset);
  server.on("/api/test", HTTP_POST, handleTouchTest);
  server.on("/api/wifi", HTTP_POST, handleWifiPost);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  socketServer.begin();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  loadSettings();
  setupNetwork();
  Wire.begin(kSdaPin, kSclPin, kI2cFrequency);
  Wire.setTimeOut(25);
  sensorPresent = initializeSensor();
  setupWeb();
  touchDevice.begin();
  USB.begin();
  Serial.printf("IR Whiteboard ready: AP=%s IP=%s sensor=%s\n", kApSsid,
                WiFi.softAPIP().toString().c_str(), sensorPresent ? "ok" : "not-found");
  xTaskCreatePinnedToCore(sensorTask, "sensor", 6144, nullptr, 3, nullptr, 1);
}

void loop() {
  static uint32_t lastBroadcast = 0;
  dnsServer.processNextRequest();
  server.handleClient();
  socketServer.loop();

  const uint32_t now = millis();
  if (now - lastBroadcast >= 50) {
    lastBroadcast = now;
    sendStatus();
  }
  delay(1);
}