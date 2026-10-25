#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <USB.h>
#include <USBHID.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint8_t kIrAddress = 0x58;
constexpr uint8_t kIrPointer = 0x36;
constexpr uint16_t kSensorWidth = 1023;
constexpr uint16_t kSensorHeight = 767;
constexpr uint8_t kSdaPin = 5;
constexpr uint8_t kSclPin = 6;
constexpr uint32_t kI2cFrequency = 100000;
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
bool smoothingEnabled = true;
float smoothing = 0.35f;
uint8_t minBlobSize = 2;
uint8_t sensitivityLevel = 6;
uint16_t screenWidth = 1920;
uint16_t screenHeight = 1080;
float filteredX = 0;
float filteredY = 0;
bool filterInitialized = false;
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
  static const uint8_t levels[6][11] = {
      {0x02, 0, 0, 0x71, 0x01, 0, 0x40, 0, 0x55, 0, 0},
      {0x02, 0, 0, 0x71, 0x01, 0, 0x55, 0, 0x63, 0, 0},
      {0x02, 0, 0, 0x71, 0x01, 0, 0x5A, 0, 0x6A, 0, 0},
      {0x02, 0, 0, 0x71, 0x01, 0, 0x64, 0, 0xFE, 0, 0},
      {0x02, 0, 0, 0x71, 0x01, 0, 0x96, 0, 0xFE, 0, 0},
      {0x02, 0, 0, 0x71, 0x01, 0, 0xAA, 0, 0x64, 0, 0}};
  Wire.beginTransmission(kIrAddress);
  Wire.write(0x00);
  Wire.write(levels[constrain(sensitivityLevel, 1, 6) - 1], sizeof(levels[0]));
  return Wire.endTransmission() == 0;
}

bool initializeSensor() {
  bool ok = writeRegister(0x30, 0x01);
  delay(10);
  ok &= writeRegister(0x30, 0x08);
  delay(10);
  ok &= writeRegister(0x06, 0x90);
  delay(10);
  ok &= writeRegister(0x08, 0xC0);
  delay(10);
  ok &= writeRegister(0x1A, 0x40);
  delay(10);
  ok &= writeRegister(0x33, 0x33);
  delay(10);
  ok &= writeSensitivity();
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
  for (uint8_t i = 0; i < 8; ++i) homography[i] = matrix[i][8];
  homography[8] = 1;
  return true;
}

bool mapPoint(float x, float y, float& mappedX, float& mappedY) {
  const float denominator = homography[6] * x + homography[7] * y + homography[8];
  if (fabsf(denominator) < 1e-6f) return false;
  mappedX = (homography[0] * x + homography[1] * y + homography[2]) / denominator;
  mappedY = (homography[3] * x + homography[4] * y + homography[5]) / denominator;
  return true;
}

void loadSettings() {
  preferences.begin("whiteboard", true);
  screenWidth = preferences.getUShort("width", 1920);
  screenHeight = preferences.getUShort("height", 1080);
  minBlobSize = preferences.getUChar("minSize", 2);
  sensitivityLevel = preferences.getUChar("sensitivity", 6);
  smoothingEnabled = preferences.getBool("smooth", true);
  smoothing = preferences.getFloat("alpha", 0.35f);
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
}

void saveSettings() {
  preferences.begin("whiteboard", false);
  preferences.putUShort("width", screenWidth);
  preferences.putUShort("height", screenHeight);
  preferences.putUChar("minSize", minBlobSize);
  preferences.putUChar("sensitivity", sensitivityLevel);
  preferences.putBool("smooth", smoothingEnabled);
  preferences.putFloat("alpha", smoothing);
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
  int selected = -1;
  for (uint8_t i = 0; i < 4; ++i) {
    if (points[i].valid && (selected < 0 || points[i].size > points[selected].size)) selected = i;
  }
  if (selected < 0) return false;
  point = points[selected];
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
  document["screenWidth"] = screenWidth;
  document["screenHeight"] = screenHeight;
  document["calibrationMask"] = calibrationMask;
  document["apIp"] = WiFi.softAPIP().toString();
  document["staConnected"] = WiFi.status() == WL_CONNECTED;
  document["staIp"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  document["staSsid"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "";
  document["point"]["valid"] = hasPoint;
  if (hasPoint) {
    const float sensorX = primary.x;
    const float sensorY = primary.y;
    float x = filterInitialized ? filteredX : sensorX;
    float y = filterInitialized ? filteredY : sensorY;
    if (calibrated) mapPoint(x, y, x, y);
    document["point"]["sensorX"] = primary.x;
    document["point"]["sensorY"] = primary.y;
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
  document["smoothing"] = smoothingEnabled;
  document["alpha"] = smoothing;
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
    if (sensorPresent) writeSensitivity();
  }
  smoothingEnabled = document["smoothing"] | smoothingEnabled;
  smoothing = constrain(document["alpha"] | smoothing, 0.05f, 0.95f);
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
  calibration[index].sx = point.x;
  calibration[index].sy = point.y;
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

void setupNetwork() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAP(kApSsid, kApPassword);
  dnsServer.start(53, "*", WiFi.softAPIP());
  if (stationSsid.length()) WiFi.begin(stationSsid.c_str(), stationPassword.c_str());
}

void setupWeb() {
  server.on("/", HTTP_GET, []() {
    File page = LittleFS.open("/index.html", "r");
    if (!page) {
      server.send(500, "text/plain", "Web UI missing; flash LittleFS image");
      return;
    }
    server.streamFile(page, "text/html");
    page.close();
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
  Wire.begin(kSdaPin, kSclPin, kI2cFrequency);
  Wire.setTimeOut(25);
  sensorPresent = initializeSensor();
  LittleFS.begin(true);
  setupNetwork();
  setupWeb();
  USB.begin();
  touchDevice.begin();
  Serial.printf("IR Whiteboard AP: %s / %s at %s\n", kApSsid, kApPassword,
                WiFi.softAPIP().toString().c_str());
}

void loop() {
  static uint32_t lastRead = 0;
  static uint32_t lastBroadcast = 0;
  static uint32_t lastSensorRetry = 0;
  const uint32_t now = millis();
  dnsServer.processNextRequest();
  server.handleClient();
  socketServer.loop();

  if (!sensorPresent && now - lastSensorRetry >= 5000) {
    lastSensorRetry = now;
    sensorPresent = initializeSensor();
  }

  if (now - lastRead >= 8) {
    lastRead = now;
    if (!readSensor()) {
      sensorPresent = false;
      memset(points, 0, sizeof(points));
      if (mouseMode) releaseMouse();
      else if (contactActive) sendTouch(false, 0, 0);
      contactActive = false;
      filterInitialized = false;
    } else {
      sensorPresent = true;
      IrPoint point;
      const bool detected = findPrimaryPoint(point);
      if (detected) {
        if (!filterInitialized) {
          filteredX = point.x;
          filteredY = point.y;
          filterInitialized = true;
        } else if (smoothingEnabled) {
          filteredX += smoothing * (point.x - filteredX);
          filteredY += smoothing * (point.y - filteredY);
        } else {
          filteredX = point.x;
          filteredY = point.y;
        }
        float mappedX = filteredX;
        float mappedY = filteredY;
        if (calibrated) mapPoint(filteredX, filteredY, mappedX, mappedY);
        mappedX = constrain(mappedX, 0.0f, static_cast<float>(screenWidth));
        mappedY = constrain(mappedY, 0.0f, static_cast<float>(screenHeight));
        if (calibrationMode) {
          contactActive = false;
        } else if (hidEnabled && mouseMode) {
          updateMouse(true, mappedX, mappedY);
          contactActive = true;
        } else if (hidEnabled) {
          sendTouch(true, mappedX, mappedY);
          contactActive = true;
        } else {
          contactActive = false;
        }
      } else {
        filterInitialized = false;
        if (mouseMode) updateMouse(false, 0, 0);
        else if (contactActive) sendTouch(false, 0, 0);
        contactActive = false;
      }
    }
  }

  if (now - lastBroadcast >= 50) {
    lastBroadcast = now;
    sendStatus();
  }
  delay(1);
}