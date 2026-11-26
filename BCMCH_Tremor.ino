/*
  BCMCH Tremor – Waveshare ESP32-S3-Touch-AMOLED-2.06
  - Shows logo, title, date/time on the AMOLED
  - Streams QMI8658 IMU (accel + gyro, 250 Hz native ODR) over BLE in binary packets
  - Time is set by the browser/phone on every connection

  Libraries (Library Manager):
    * GFX Library for Arduino   (moononournation)  >= 1.5.x
    * NimBLE-Arduino            (h2zero)           >= 2.x
  Board: ESP32S3 Dev Module, ESP32 core 3.x, USB CDC On Boot: Enabled,
         Flash 32MB, PSRAM: OPI PSRAM, Partition: any 16MB+ (e.g. "16M Flash (3MB APP/9.9MB FATFS)")

  BLE protocol
    Service  B3C10000-7A21-4E6F-9D3A-0B5C1E7A9F01
    DATA     B3C10001-...  (Notify)  IMU packets
    CTRL     B3C10002-...  (Write)   commands
      0x01 + uint32 LE : set clock (seconds since 1970 of LOCAL time, i.e. UTC + tz offset)
      0x02             : start streaming (resets sequence + timestamp)
      0x03             : stop streaming

    DATA packet (little endian), 8 + n*12 bytes (n <= 16, 200 bytes max @ MTU>=211)
      uint16 seq        packet counter (wraps) – gaps = lost packets
      uint8  n          samples in this packet
      uint8  flags      reserved (0)
      uint32 t0_us      micros since start command, time of the first sample
      n * { int16 ax, ay, az, gx, gy, gz }   raw, accel +/-4 g (8192 LSB/g), gyro +/-512 dps (64 LSB/dps)
*/
#include <Arduino.h>
#include <Wire.h>
#include <sys/time.h>
#include <time.h>
#include <Arduino_GFX_Library.h>
#include <NimBLEDevice.h>
#include "logo.h"

// ---------- Pins (Waveshare ESP32-S3-Touch-AMOLED-2.06) – verify against Waveshare pin_config.h ----------
#define LCD_CS 45
#define LCD_SCLK 47
#define LCD_SDIO0 21
#define LCD_SDIO1 48
#define LCD_SDIO2 40
#define LCD_SDIO3 39
#define LCD_RESET 8
#define LCD_W 410
#define LCD_H 502
#define IIC_SDA 15
#define IIC_SCL 14

// ---------- IMU (QMI8658) ----------
#define IMU_ODR_HZ 250
#define SPP_MAX 16                       // samples per packet
#define HDR 8
#define SAMPLE_BYTES 12

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_W, LCD_H, 22, 0, 0, 0);

#define SVC_UUID  "b3c10000-7a21-4e6f-9d3a-0b5c1e7a9f01"
#define DATA_UUID "b3c10001-7a21-4e6f-9d3a-0b5c1e7a9f01"
#define CTRL_UUID "b3c10002-7a21-4e6f-9d3a-0b5c1e7a9f01"

struct Pkt { uint16_t len; uint8_t d[HDR + SPP_MAX * SAMPLE_BYTES]; };

NimBLECharacteristic *dataChr;
QueueHandle_t pktQ;
uint8_t imuAddr = 0x6B;
volatile bool connected = false, streaming = false, timeSynced = false, resetReq = false;
volatile uint8_t spp = 1;
volatile uint32_t streamT0 = 0;

// ---------- IMU low level ----------
bool imuWrite(uint8_t r, uint8_t v) {
  Wire.beginTransmission(imuAddr); Wire.write(r); Wire.write(v);
  return Wire.endTransmission() == 0;
}
bool imuRead(uint8_t r, uint8_t *b, size_t n) {
  Wire.beginTransmission(imuAddr); Wire.write(r);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddr, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) b[i] = Wire.read();
  return true;
}
bool imuInit() {
  uint8_t id = 0;
  for (uint8_t a : {0x6B, 0x6A}) {
    imuAddr = a;
    if (imuRead(0x00, &id, 1) && id == 0x05) break;
    id = 0;
  }
  if (id != 0x05) return false;
  imuWrite(0x60, 0xB0); delay(30);       // soft reset
  imuWrite(0x02, 0x40);                  // address auto-increment, little endian
  imuWrite(0x03, (0x01 << 4) | 0x05);    // accel +/-4 g, 250 Hz
  imuWrite(0x04, (0x05 << 4) | 0x05);    // gyro +/-512 dps, 250 Hz
  imuWrite(0x06, 0x00);                  // low-pass filters off (keep full tremor bandwidth)
  imuWrite(0x08, 0x03);                  // enable accel + gyro
  delay(50);
  return true;
}

// ---------- Tasks ----------
void imuTask(void *) {
  Pkt p; uint8_t n = 0; uint16_t seq = 0; uint32_t t0 = 0; uint8_t st, r[12];
  for (;;) {
    if (resetReq) { resetReq = false; n = 0; seq = 0; xQueueReset(pktQ); }
    if (!streaming) { n = 0; vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    if (!imuRead(0x2E, &st, 1) || !(st & 0x03)) { vTaskDelay(1); continue; }   // wait for data-ready
    uint32_t now = micros() - streamT0;
    if (!imuRead(0x35, r, 12)) continue;
    if (n == 0) t0 = now;
    memcpy(p.d + HDR + n * SAMPLE_BYTES, r, 12);
    if (++n >= spp) {
      memcpy(p.d, &seq, 2); p.d[2] = n; p.d[3] = 0; memcpy(p.d + 4, &t0, 4);
      p.len = HDR + n * SAMPLE_BYTES;
      xQueueSend(pktQ, &p, 0);           // if full, packet is dropped -> seq gap is visible on the PC
      seq++; n = 0;
    }
  }
}

void sendTask(void *) {
  Pkt p;
  for (;;) {
    if (xQueueReceive(pktQ, &p, portMAX_DELAY) == pdTRUE && connected && streaming) {
      for (int tries = 0; tries < 30 && !dataChr->notify(p.d, p.len); tries++) vTaskDelay(1);
    }
  }
}

// ---------- BLE ----------
class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &ci) override {
    connected = true;
    s->updateConnParams(ci.getConnHandle(), 6, 12, 0, 400);   // 7.5–15 ms interval
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &ci, int reason) override {
    connected = false; streaming = false;
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo &ci) override {
    int n = (mtu - 3 - HDR) / SAMPLE_BYTES;
    spp = constrain(n, 1, SPP_MAX);
  }
};

class CtrlCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &ci) override {
    NimBLEAttValue v = c->getValue();
    const uint8_t *d = v.data();
    if (v.length() < 1) return;
    if (d[0] == 0x01 && v.length() >= 5) {
      uint32_t e; memcpy(&e, d + 1, 4);
      struct timeval tv = { (time_t)e, 0 };
      settimeofday(&tv, nullptr);
      timeSynced = true;
    } else if (d[0] == 0x02) {
      streamT0 = micros(); resetReq = true; streaming = true;
    } else if (d[0] == 0x03) {
      streaming = false;
    }
  }
};

// ---------- Display ----------
const uint16_t BLACK = 0x0000, WHITE = 0xFFFF, RED = 0xE125, GREY = 0x8C71, GREEN = 0x07E0;
String lastDate, lastTime, lastStatus;

void centerText(const String &s, int y, int size, uint16_t fg) {
  int w = s.length() * 6 * size, h = 8 * size;
  gfx->fillRect(0, y, LCD_W, h, BLACK);
  gfx->setTextSize(size);
  gfx->setTextColor(fg, BLACK);
  gfx->setCursor((LCD_W - w) / 2, y);
  gfx->print(s);
}

void drawStatic() {
  gfx->fillScreen(BLACK);
  gfx->draw16bitRGBBitmap((LCD_W - LOGO_W) / 2, 50, logo_img, LOGO_W, LOGO_H);
  centerText("BCMCH Tremor", 180, 3, WHITE);
}

void updateDisplay() {
  char d[24], t[16];
  String date, tm;
  if (timeSynced) {
    struct timeval tv; gettimeofday(&tv, nullptr);
    time_t now = tv.tv_sec; struct tm ti; gmtime_r(&now, &ti);   // browser sends local time as epoch
    strftime(d, sizeof d, "%a %d %b %Y", &ti);
    strftime(t, sizeof t, "%H:%M:%S", &ti);
    date = d; tm = t;
  } else { date = "Pair to sync time"; tm = "--:--:--"; }
  String status = streaming ? "RECORDING" : (connected ? "Connected" : "Waiting for pairing");

  if (date != lastDate) { centerText(date, 250, 3, GREY); lastDate = date; }
  if (tm != lastTime) { centerText(tm, 300, 5, WHITE); lastTime = tm; }
  if (status != lastStatus) {
    centerText(status, 400, 3, streaming ? RED : (connected ? GREEN : GREY));
    lastStatus = status;
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  Wire.begin(IIC_SDA, IIC_SCL); Wire.setClock(400000);

  if (!gfx->begin()) Serial.println("Display init failed");
  gfx->setBrightness(200);
  drawStatic();

  if (!imuInit()) { centerText("IMU not found", 440, 2, RED); Serial.println("QMI8658 not found"); }

  pktQ = xQueueCreate(40, sizeof(Pkt));

  NimBLEDevice::init("BCMCH-Tremor");
  NimBLEDevice::setMTU(247);
  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(new SrvCB());
  NimBLEService *svc = srv->createService(SVC_UUID);
  dataChr = svc->createCharacteristic(DATA_UUID, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic *ctrl = svc->createCharacteristic(CTRL_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  ctrl->setCallbacks(new CtrlCB());
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setName("BCMCH-Tremor");
  adv->enableScanResponse(true);
  adv->start();

  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(sendTask, "tx", 4096, nullptr, 3, nullptr, 0);
}

void loop() {
  updateDisplay();
  delay(100);
}
