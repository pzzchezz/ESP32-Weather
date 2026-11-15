/*
 * ESP32 Weather Receiver
 * ----------------------
 * รับข้อมูลสภาพอากาศจากแอป WeatherSender (Android) ผ่าน Bluetooth LE
 * แล้วแสดงผลทาง Serial Monitor (115200) และจอ OLED SSD1306 (ถ้าเปิดใช้)
 *
 * โปรโตคอล (ต้องตรงกับ MainActivity.java):
 *   Device name : "ESP32-Weather"
 *   Service     : 7a1c0001-2f5b-4c9e-9d3a-5e0b1a2c3d4e
 *   Current     : 7a1c0002-...  (write, 12 bytes, little-endian)
 *       [0-1]  int16  temp * 10
 *       [2-3]  uint16 humidity * 10
 *       [4-5]  int16  feels-like * 10
 *       [6-7]  uint16 pressure(hPa) * 10
 *       [8]    uint8  weather code (WMO / Open-Meteo)
 *       [9]    uint8  is_day (1/0)
 *       [10]   uint8  rain probability %
 *       [11]   uint8  wind km/h
 *   Forecast    : 7a1c0003-...  (write, 17 bytes)
 *       [0]    uint8  chunk index (0..2)
 *       [1-16] 8 x int16 temp * 10  (ชั่วโมงที่ chunk*8 .. chunk*8+7)
 *
 * จอ: OLED 1.3" SH1106 128x64 (I2C) ใช้ไลบรารี U8g2
 * ตั้ง USE_OLED เป็น 0 ถ้าไม่ต่อจอ (แสดง Serial อย่างเดียว)
 * ขาเริ่มต้น: SDA=GPIO21, SCL=GPIO22, address 0x3C
 */

#define USE_OLED 1
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDR 0x3C

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#if USE_OLED
#include <U8g2lib.h>
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
#endif

#define DEV_NAME     "ESP32-Weather"
#define SVC_UUID     "7a1c0001-2f5b-4c9e-9d3a-5e0b1a2c3d4e"
#define CUR_UUID     "7a1c0002-2f5b-4c9e-9d3a-5e0b1a2c3d4e"
#define FC_UUID      "7a1c0003-2f5b-4c9e-9d3a-5e0b1a2c3d4e"

struct Weather {
  float temp = 0, hum = 0, feels = 0, press = 0;
  uint8_t code = 0, isDay = 1, rain = 0, wind = 0;
  float fc[24];
  bool fcGot[3] = {false, false, false};
  bool haveCur = false;
} wx;

volatile bool dirty = false;
volatile bool bleConnected = false;
BLEServer *server = nullptr;

static int16_t rdI16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
static uint16_t rdU16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

const char *weatherText(uint8_t c, uint8_t day) {
  if (c == 0) return day ? "Sunny" : "Clear";
  if (c == 1) return "Mostly clear";
  if (c == 2) return "Partly cloudy";
  if (c == 3) return "Overcast";
  if (c == 45 || c == 48) return "Fog";
  if (c >= 51 && c <= 57) return "Drizzle";
  if (c >= 61 && c <= 67) return "Rain";
  if ((c >= 71 && c <= 77) || c == 85 || c == 86) return "Snow";
  if (c >= 80 && c <= 82) return "Showers";
  if (c >= 95) return "Thunderstorm";
  return "-";
}

// ---------------------------------------------------------------- BLE

class CurCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    const uint8_t *d = ch->getData();
    size_t len = ch->getLength();
    if (d == nullptr || len < 12) {
      Serial.printf("[BLE] current: ขนาดข้อมูลไม่ถูกต้อง (%u)\n", (unsigned)len);
      return;
    }
    wx.temp  = rdI16(d + 0) / 10.0f;
    wx.hum   = rdU16(d + 2) / 10.0f;
    wx.feels = rdI16(d + 4) / 10.0f;
    wx.press = rdU16(d + 6) / 10.0f;
    wx.code  = d[8];
    wx.isDay = d[9];
    wx.rain  = d[10];
    wx.wind  = d[11];
    wx.haveCur = true;
    dirty = true;
  }
};

class FcCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    const uint8_t *d = ch->getData();
    size_t len = ch->getLength();
    if (d == nullptr || len < 17) {
      Serial.printf("[BLE] forecast: ขนาดข้อมูลไม่ถูกต้อง (%u)\n", (unsigned)len);
      return;
    }
    uint8_t chunk = d[0];
    if (chunk > 2) return;
    for (int k = 0; k < 8; k++) {
      wx.fc[chunk * 8 + k] = rdI16(d + 1 + k * 2) / 10.0f;
    }
    wx.fcGot[chunk] = true;
    dirty = true;
  }
};

class ServerCallback : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    bleConnected = true;
    dirty = true;
    Serial.println("[BLE] เชื่อมต่อแล้ว");
  }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    dirty = true;
    Serial.println("[BLE] หลุดการเชื่อมต่อ กำลังโฆษณาใหม่...");
    BLEDevice::startAdvertising();
  }
};

void startBle() {
  BLEDevice::init(DEV_NAME);
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallback());

  BLEService *svc = server->createService(SVC_UUID);

  BLECharacteristic *cur = svc->createCharacteristic(
      CUR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cur->setCallbacks(new CurCallback());

  BLECharacteristic *fc = svc->createCharacteristic(
      FC_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  fc->setCallbacks(new FcCallback());

  svc->start();

  // UUID 128 บิต + ชื่ออุปกรณ์ รวมกันเกิน 31 ไบต์ จึงแยก: UUID ใน adv, ชื่อใน scan response
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  BLEAdvertisementData advData;
  advData.setFlags(0x06);  // LE General Discoverable + BR/EDR not supported
  advData.setCompleteServices(BLEUUID(SVC_UUID));
  BLEAdvertisementData scanData;
  scanData.setName(DEV_NAME);
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] กำลังโฆษณาชื่อ " DEV_NAME);
}

// ------------------------------------------------------------ output

void forecastRange(float &lo, float &hi, bool &ok) {
  ok = false;
  lo = 1000;
  hi = -1000;
  for (int c = 0; c < 3; c++) {
    if (!wx.fcGot[c]) continue;
    for (int k = 0; k < 8; k++) {
      float v = wx.fc[c * 8 + k];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
      ok = true;
    }
  }
}

void printSerial() {
  if (!wx.haveCur) return;
  Serial.println("-------- Weather --------");
  Serial.printf("Temp      : %.1f C (feels %.1f C)\n", wx.temp, wx.feels);
  Serial.printf("Humidity  : %.1f %%\n", wx.hum);
  Serial.printf("Pressure  : %.1f hPa\n", wx.press);
  Serial.printf("Wind      : %u km/h\n", wx.wind);
  Serial.printf("Rain prob : %u %%\n", wx.rain);
  Serial.printf("Condition : %s (code %u, %s)\n", weatherText(wx.code, wx.isDay), wx.code,
                wx.isDay ? "day" : "night");
  float lo, hi;
  bool ok;
  forecastRange(lo, hi, ok);
  if (ok) {
    Serial.print("Next 24h  : ");
    for (int i = 0; i < 24; i++) {
      if (wx.fcGot[i / 8]) Serial.printf("%.0f ", wx.fc[i]);
    }
    Serial.printf("\n            min %.1f / max %.1f C\n", lo, hi);
  }
  Serial.println("-------------------------");
}

#if USE_OLED
void drawOled() {
  char buf[32];
  u8g2.clearBuffer();
  if (!wx.haveCur) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "ESP32 Weather");
    u8g2.drawStr(0, 30, "Waiting for phone...");
    u8g2.drawStr(0, 62, bleConnected ? "BLE: connected" : "BLE: advertising");
    u8g2.sendBuffer();
    return;
  }
  u8g2.setFont(u8g2_font_logisoso20_tr);
  snprintf(buf, sizeof(buf), "%.1fC", wx.temp);
  u8g2.drawStr(0, 22, buf);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(100, 8, bleConnected ? "BLE" : "---");

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 33, weatherText(wx.code, wx.isDay));
  snprintf(buf, sizeof(buf), "Hum %.0f%%  %.0fhPa", wx.hum, wx.press);
  u8g2.drawStr(0, 43, buf);
  snprintf(buf, sizeof(buf), "Feels %.1f Wind %u", wx.feels, wx.wind);
  u8g2.drawStr(0, 53, buf);
  float lo, hi;
  bool ok;
  forecastRange(lo, hi, ok);
  if (ok) snprintf(buf, sizeof(buf), "Rain %u%% 24h %.0f-%.0f", wx.rain, lo, hi);
  else snprintf(buf, sizeof(buf), "Rain %u%%", wx.rain);
  u8g2.drawStr(0, 63, buf);
  u8g2.sendBuffer();
}
#endif

// -------------------------------------------------------------- main

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32 Weather Receiver");

#if USE_OLED
  u8g2.setI2CAddress(OLED_ADDR * 2);
  u8g2.begin();
  drawOled();
#endif

  startBle();
}

void loop() {
  if (dirty) {
    dirty = false;
    printSerial();
#if USE_OLED
    drawOled();
#endif
  }
  delay(50);
}
