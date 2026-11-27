/*
  BCMCH Tremor v2 – Waveshare ESP32-S3-Touch-AMOLED-2.06

  Features
   - Logo / title / date-time on AMOLED; screen off after 10 s, touch or BOOT wakes it for 10 s
   - QMI8658 IMU read through its FIFO (falls back to polling if the FIFO self-test fails)
   - BLE streaming of binary packets to the web dashboard (Web Bluetooth)
   - SD card CSV logging, started/stopped with the BOOT button (works without BLE)
   - Clock set by the dashboard at pairing and stored in the PCF85063 RTC

  BOOT button:  short press = wake screen (when off) / start-stop SD recording (when screen on)
                hold 1 s   = stop SD recording

  Libraries: GFX Library for Arduino (>=1.6.0), NimBLE-Arduino (2.x). SD_MMC ships with the ESP32 core.
  Board: ESP32S3 Dev Module, ESP32 core 3.x, USB CDC On Boot: Enabled, OPI PSRAM, Flash 32MB.
  SD card: FAT32, 32 GB or smaller.

  BLE  Service B3C10000-7A21-4E6F-9D3A-0B5C1E7A9F01
    DATA B3C10001-... (Notify)   packet, little endian, 8 + n*12 bytes (n = (len-8)/12, max 16)
         uint32 c0      index of the first sample (counts from IMU start; a jump = lost samples)
         uint32 t_us    micros() at the LAST sample of the packet (wraps every 71 min)
         n * int16 ax ay az gx gy gz   raw: accel +/-4 g (8192 LSB/g), gyro +/-512 dps (64 LSB/dps)
    CTRL B3C10002-... (Write)
         0x01 + uint32 sec + uint16 ms : set clock (LOCAL time as seconds since 1970 + milliseconds)
         0x02 start BLE stream   0x03 stop BLE stream
*/
#include <Arduino.h>
#include <Wire.h>
#include <sys/time.h>
#include <time.h>
#include <SD_MMC.h>
#include <Arduino_GFX_Library.h>
#include <NimBLEDevice.h>
#include "logo.h"

// ---------- Pins (Waveshare ESP32-S3-Touch-AMOLED-2.06) ----------
#define LCD_CS 12
#define LCD_SCLK 11
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_RESET 8
#define LCD_W 410
#define LCD_H 502
#define IIC_SDA 15
#define IIC_SCL 14
#define BTN_BOOT 0
#define TP_INT 38
#define TP_RST 9
#define SD_CLK 2
#define SD_CMD 1
#define SD_D0 3
#define SD_CS_D3 17

#define SCREEN_ON_MS 10000
#define FIFO_PERIOD_MS 60
#define FIFO_CFG 0x0A            // stream mode, 64-sample FIFO
#define CHUNK_MAX 16
#define SAMPLE_BYTES 12
#define HDR 8
#define RTC_ADDR 0x51

struct Pkt { uint16_t len; uint8_t d[HDR + CHUNK_MAX * SAMPLE_BYTES]; };
struct Chunk { uint32_t c0, tLast; float p; uint8_t k; uint8_t raw[CHUNK_MAX * SAMPLE_BYTES]; };

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_W, LCD_H, 22, 0, 0, 0);

#define SVC_UUID  "b3c10000-7a21-4e6f-9d3a-0b5c1e7a9f01"
#define DATA_UUID "b3c10001-7a21-4e6f-9d3a-0b5c1e7a9f01"
#define CTRL_UUID "b3c10002-7a21-4e6f-9d3a-0b5c1e7a9f01"

NimBLECharacteristic *dataChr;
QueueHandle_t bleQ, sdQ;
SemaphoreHandle_t i2cMtx;
uint8_t imuAddr = 0x6B;
bool imuOk = false, fifoMode = true;
volatile bool connected = false, bleStream = false, sdRec = false, timeSynced = false, wakeReq = false;
volatile bool sdCmdStart = false, sdCmdStop = false, sdMsgNew = false, touchFlag = false;
volatile uint8_t spp = 1;
volatile uint32_t sdDropped = 0;
char sdMsg[32] = "";
uint32_t sampleCnt = 0;
float pEst = 4500.0f;                 // estimated sample period in microseconds
bool timingHave = false;
uint32_t tmLastT = 0, tmLastEnd = 0;

// ---------- I2C (shared bus, protected by a mutex) ----------
struct I2CLock { I2CLock() { xSemaphoreTake(i2cMtx, portMAX_DELAY); } ~I2CLock() { xSemaphoreGive(i2cMtx); } };
bool i2cWrite(uint8_t a, uint8_t r, const uint8_t *b, size_t n) {
  I2CLock l; Wire.beginTransmission(a); Wire.write(r); Wire.write(b, n);
  return Wire.endTransmission() == 0;
}
bool i2cRead(uint8_t a, uint8_t r, uint8_t *b, size_t n) {
  I2CLock l; Wire.beginTransmission(a); Wire.write(r);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)a, (size_t)n, true) != n) return false;
  for (size_t i = 0; i < n; i++) b[i] = Wire.read();
  return true;
}
bool imuWrite(uint8_t r, uint8_t v) { return i2cWrite(imuAddr, r, &v, 1); }
bool imuRead(uint8_t r, uint8_t *b, size_t n) { return i2cRead(imuAddr, r, b, n); }

// ---------- RTC (PCF85063, stores LOCAL time) ----------
uint8_t bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }
uint8_t unbcd(uint8_t b) { return (b >> 4) * 10 + (b & 15); }
void rtcSet(time_t e) {
  struct tm t; gmtime_r(&e, &t);
  uint8_t b[7] = { bcd(t.tm_sec) & 0x7F, bcd(t.tm_min), bcd(t.tm_hour), bcd(t.tm_mday), (uint8_t)t.tm_wday, bcd(t.tm_mon + 1), bcd(t.tm_year - 100) };
  i2cWrite(RTC_ADDR, 0x04, b, 7);
}
bool rtcGet(time_t &e) {
  uint8_t b[7];
  if (!i2cRead(RTC_ADDR, 0x04, b, 7) || (b[0] & 0x80)) return false;   // bit7 = oscillator stopped / time invalid
  struct tm t = {};
  t.tm_sec = unbcd(b[0] & 0x7F); t.tm_min = unbcd(b[1] & 0x7F); t.tm_hour = unbcd(b[2] & 0x3F);
  t.tm_mday = unbcd(b[3] & 0x3F); t.tm_mon = unbcd(b[5] & 0x1F) - 1; t.tm_year = unbcd(b[6]) + 100;
  if (t.tm_year < 125) return false;
  e = mktime(&t);                                                       // TZ is UTC, so no offset applied
  return true;
}

// ---------- IMU (QMI8658) ----------
bool imuCmd(uint8_t cmd) {                      // CTRL9 command handshake
  imuWrite(0x0A, cmd);
  for (int i = 0; i < 50; i++) {
    uint8_t s;
    if (imuRead(0x2D, &s, 1) && (s & 0x80)) { imuWrite(0x0A, 0x00); return true; }
    vTaskDelay(2);
  }
  return false;
}
int fifoReadAll(uint8_t *out, int maxS) {       // returns number of samples, <0 on error
  if (!imuCmd(0x05)) return -1;                 // request FIFO data
  uint8_t cnt, st;
  if (!imuRead(0x15, &cnt, 1) || !imuRead(0x16, &st, 1)) return -1;
  int s = ((((st & 3) << 8) | cnt) * 2) / SAMPLE_BYTES;   // count is in 16-bit words
  if (s > maxS) s = maxS;
  if (s > 0 && !imuRead(0x17, out, s * SAMPLE_BYTES)) s = -1;
  imuWrite(0x14, FIFO_CFG);                     // leave FIFO read mode
  return s;
}
void imuEnable() {
  imuWrite(0x08, 0x00);
  if (fifoMode) { imuWrite(0x14, FIFO_CFG); imuCmd(0x04); }   // configure + reset FIFO
  imuWrite(0x08, 0x03);                         // accel + gyro on
  timingHave = false; pEst = 4500.0f;
}
void imuDisable() { imuWrite(0x08, 0x00); imuWrite(0x14, 0x00); }
bool imuInit() {
  uint8_t id = 0;
  for (uint8_t a : {0x6B, 0x6A}) { imuAddr = a; if (imuRead(0x00, &id, 1) && id == 0x05) break; id = 0; }
  if (id != 0x05) return false;
  imuWrite(0x60, 0xB0); delay(30);              // soft reset
  imuWrite(0x02, 0x40);                         // address auto-increment, little endian
  imuWrite(0x03, (0x01 << 4) | 0x05);           // accel +/-4 g, 250 Hz setting (about 222 Hz real)
  imuWrite(0x04, (0x05 << 4) | 0x05);           // gyro +/-512 dps
  imuWrite(0x06, 0x00);                         // no on-chip low-pass filtering
  // FIFO self-test: only use FIFO if it returns plausible, non-repeating data
  static uint8_t tb[64 * SAMPLE_BYTES];
  fifoMode = true; imuEnable(); delay(250);
  int k = fifoReadAll(tb, 64);
  bool ok = k >= 8;
  for (int i = 0; ok && i < k; i++) {
    int16_t a[3]; memcpy(a, tb + i * SAMPLE_BYTES, 6);
    int64_t m = (int64_t)a[0] * a[0] + (int64_t)a[1] * a[1] + (int64_t)a[2] * a[2];
    if (m < 3000LL * 3000 || m > 17000LL * 17000) ok = false;   // 0.37 g .. 2 g
  }
  if (ok && k > 9 && !memcmp(tb, tb + 9 * SAMPLE_BYTES, SAMPLE_BYTES)) ok = false;
  fifoMode = ok;
  imuDisable();
  Serial.printf("IMU %s (self-test samples: %d)\n", fifoMode ? "FIFO mode" : "polling fallback", k);
  return true;
}

void emitChunk(const uint8_t *raw, int k, uint32_t tLast) {
  Chunk c; c.c0 = sampleCnt; c.tLast = tLast; c.k = k;
  if (timingHave) {
    uint32_t dt = tLast - tmLastT, dc = (sampleCnt + k) - tmLastEnd;
    if (dc > 0 && dt > 0) { float q = (float)dt / dc; if (q > 3000 && q < 6000) pEst = 0.95f * pEst + 0.05f * q; }
  }
  tmLastT = tLast; tmLastEnd = sampleCnt + k; timingHave = true;
  c.p = pEst; memcpy(c.raw, raw, k * SAMPLE_BYTES);
  if (sdRec && xQueueSend(sdQ, &c, 0) != pdTRUE) sdDropped++;
  if (bleStream) {
    for (int off = 0; off < k;) {
      int n = min((int)spp, k - off);
      Pkt q; uint32_t c0 = sampleCnt + off, t = tLast - (uint32_t)((k - off - n) * pEst);
      memcpy(q.d, &c0, 4); memcpy(q.d + 4, &t, 4); memcpy(q.d + HDR, raw + off * SAMPLE_BYTES, n * SAMPLE_BYTES);
      q.len = HDR + n * SAMPLE_BYTES;
      xQueueSend(bleQ, &q, 0);                  // full queue = dropped packet, visible as a counter gap
      off += n;
    }
  }
  sampleCnt += k;
}

void imuTask(void *) {
  static uint8_t fb[64 * SAMPLE_BYTES];
  uint8_t pb[CHUNK_MAX * SAMPLE_BYTES], st, r[12];
  int pn = 0; bool on = false;
  for (;;) {
    bool want = imuOk && (bleStream || sdRec);
    if (!want) { if (on) { imuDisable(); on = false; } vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if (!on) { imuEnable(); on = true; sampleCnt = 0; pn = 0; vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if (fifoMode) {
      vTaskDelay(pdMS_TO_TICKS(FIFO_PERIOD_MS));
      int k = fifoReadAll(fb, 64); uint32_t tr = micros();
      if (k <= 0) continue;
      for (int off = 0; off < k; off += CHUNK_MAX) {
        int n = min(CHUNK_MAX, k - off);
        emitChunk(fb + off * SAMPLE_BYTES, n, tr - (uint32_t)((k - off - n) * pEst));
      }
    } else {
      if (!imuRead(0x2E, &st, 1) || !(st & 0x03)) { vTaskDelay(1); continue; }
      uint32_t t = micros();
      if (!imuRead(0x35, r, 12)) continue;
      memcpy(pb + pn * SAMPLE_BYTES, r, 12);
      if (++pn >= CHUNK_MAX) { emitChunk(pb, pn, t); pn = 0; }
    }
  }
}

void sendTask(void *) {
  Pkt p;
  for (;;) {
    if (xQueueReceive(bleQ, &p, portMAX_DELAY) == pdTRUE && connected && bleStream) {
      for (int tries = 0; tries < 30 && !dataChr->notify(p.d, p.len); tries++) vTaskDelay(1);
    }
  }
}

// ---------- SD logging ----------
File sdFile; bool sdMounted = false, sdFirst = true, sdWallOk = false;
char wbuf[4608]; size_t wlen = 0;
uint32_t sdC0, sdT0; int64_t sdWall0;
void setMsg(const char *m) { strlcpy(sdMsg, m, sizeof sdMsg); sdMsgNew = true; }

void sdFail() {
  sdRec = false;
  if (sdFile) sdFile.close();
  SD_MMC.end(); sdMounted = false; wlen = 0;
  setMsg("SD write error");
}
void sdFlushBuf() {
  if (!sdFile || !wlen) return;
  size_t n = wlen; wlen = 0;
  if (sdFile.write((const uint8_t *)wbuf, n) != n) sdFail();
}
bool sdOpen() {
  if (!sdMounted) {
    pinMode(SD_CS_D3, OUTPUT); digitalWrite(SD_CS_D3, HIGH);   // DAT3 high for 1-bit mode
    SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
    if (!SD_MMC.begin("/sdcard", true, false, 20000)) return false;
    if (SD_MMC.cardType() == CARD_NONE) { SD_MMC.end(); return false; }
    sdMounted = true;
  }
  char name[40];
  if (timeSynced) {
    struct timeval tv; gettimeofday(&tv, nullptr); time_t t = tv.tv_sec; struct tm ti; gmtime_r(&t, &ti);
    strftime(name, sizeof name, "/TREMOR_%Y%m%d_%H%M%S.csv", &ti);
  } else {
    for (int i = 1; i < 1000; i++) { snprintf(name, sizeof name, "/TREMOR_%03d.csv", i); if (!SD_MMC.exists(name)) break; }
  }
  sdFile = SD_MMC.open(name, FILE_WRITE);
  if (!sdFile) return false;
  sdFile.print("sample,time_s,datetime_local,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n");
  Serial.printf("SD file %s\n", name);
  return true;
}
void sdRows(const Chunk &c) {
  if (!sdFile) return;
  if (sdFirst) {
    sdFirst = false; sdC0 = c.c0; sdT0 = c.tLast - (uint32_t)((c.k - 1) * c.p);
    struct timeval tv; gettimeofday(&tv, nullptr);
    sdWallOk = timeSynced;
    sdWall0 = (int64_t)tv.tv_sec * 1000000 + tv.tv_usec - (int64_t)(uint32_t)(micros() - sdT0);
  }
  static int64_t cacheSec = -1; static char cacheStr[24] = "";
  for (int i = 0; i < c.k && sdFile; i++) {
    int16_t v[6]; memcpy(v, c.raw + i * SAMPLE_BYTES, 12);
    uint32_t rel = (c.tLast - (uint32_t)((c.k - 1 - i) * c.p)) - sdT0;
    char dt[32] = "";
    if (sdWallOk) {
      int64_t wall = sdWall0 + rel, sec = wall / 1000000;
      if (sec != cacheSec) { time_t tt = (time_t)sec; struct tm ti; gmtime_r(&tt, &ti); strftime(cacheStr, sizeof cacheStr, "%Y-%m-%dT%H:%M:%S", &ti); cacheSec = sec; }
      snprintf(dt, sizeof dt, "%s.%03d", cacheStr, (int)((wall / 1000) % 1000));
    }
    if (wlen > sizeof(wbuf) - 160) sdFlushBuf();
    wlen += snprintf(wbuf + wlen, sizeof(wbuf) - wlen, "%lu,%.4f,%s,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n",
                     (unsigned long)(c.c0 + i - sdC0), rel / 1e6, dt,
                     v[0] / 8192.0, v[1] / 8192.0, v[2] / 8192.0, v[3] / 64.0, v[4] / 64.0, v[5] / 64.0);
  }
}
void sdTask(void *) {
  Chunk c; uint32_t lastFlush = 0;
  for (;;) {
    if (sdCmdStart) {
      sdCmdStart = false;
      if (!sdRec) {
        if (sdOpen()) { sdFirst = true; wlen = 0; sdDropped = 0; xQueueReset(sdQ); sdRec = true; lastFlush = millis(); setMsg("SD recording started"); }
        else setMsg(sdMounted ? "SD file error" : "No SD card");
      }
    }
    if (sdCmdStop) {
      sdCmdStop = false;
      if (sdRec && sdFile) {
        sdRec = false;
        while (xQueueReceive(sdQ, &c, 0) == pdTRUE) sdRows(c);
        sdFlushBuf();
        if (sdFile) { sdFile.flush(); sdFile.close(); setMsg(sdDropped ? "SD saved (gaps)" : "SD saved"); }
      }
    }
    if (sdFile && xQueueReceive(sdQ, &c, pdMS_TO_TICKS(50)) == pdTRUE) sdRows(c);
    else if (!sdFile) vTaskDelay(pdMS_TO_TICKS(50));
    if (sdFile && sdRec && millis() - lastFlush > 2000) { sdFlushBuf(); if (sdFile) sdFile.flush(); lastFlush = millis(); }
  }
}

// ---------- BLE ----------
class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &ci) override {
    connected = true; wakeReq = true;
    s->updateConnParams(ci.getConnHandle(), 6, 12, 0, 400);
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &ci, int reason) override {
    connected = false; bleStream = false; wakeReq = true;
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo &ci) override {
    spp = constrain((int)((mtu - 3 - HDR) / SAMPLE_BYTES), 1, CHUNK_MAX);
  }
};
class CtrlCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &ci) override {
    NimBLEAttValue v = c->getValue(); const uint8_t *d = v.data();
    if (v.length() < 1) return;
    if (d[0] == 0x01 && v.length() >= 7) {
      uint32_t s; uint16_t ms; memcpy(&s, d + 1, 4); memcpy(&ms, d + 5, 2);
      struct timeval tv = { (time_t)s, (suseconds_t)ms * 1000 };
      settimeofday(&tv, nullptr);
      rtcSet((time_t)s);
      timeSynced = true;
    } else if (d[0] == 0x02) bleStream = true;
    else if (d[0] == 0x03) bleStream = false;
  }
};

// ---------- Display ----------
const uint16_t BLACK = 0x0000, WHITE = 0xFFFF, RED = 0xE125, GREY = 0x8C71, GREEN = 0x07E0;
String lastDate, lastTime, lastStatus, lastMsg;
bool screenOn = true; uint32_t offAt = 0;

void IRAM_ATTR touchISR() { touchFlag = true; }

void centerText(const String &s, int y, int size, uint16_t fg) {
  gfx->fillRect(0, y, LCD_W, 8 * size, BLACK);
  gfx->setTextSize(size); gfx->setTextColor(fg, BLACK);
  gfx->setCursor((LCD_W - (int)s.length() * 6 * size) / 2, y);
  gfx->print(s);
}
void drawStatic() {
  gfx->fillScreen(BLACK);
  gfx->draw16bitRGBBitmap((LCD_W - LOGO_W) / 2, 50, logo_img, LOGO_W, LOGO_H);
  centerText("BCMCH Tremor", 180, 3, WHITE);
}
void updateDisplay() {
  char d[24], t[16]; String date, tm;
  if (timeSynced) {
    struct timeval tv; gettimeofday(&tv, nullptr);
    time_t now = tv.tv_sec; struct tm ti; gmtime_r(&now, &ti);
    strftime(d, sizeof d, "%a %d %b %Y", &ti); strftime(t, sizeof t, "%H:%M:%S", &ti);
    date = d; tm = t;
  } else { date = "Pair to sync time"; tm = "--:--:--"; }
  bool ble = bleStream, sd = sdRec;
  String status = (ble && sd) ? "BLE + SD REC" : sd ? "SD REC" : ble ? "BLE REC" : connected ? "Connected" : "Waiting for pairing";
  uint16_t col = (ble || sd) ? RED : connected ? GREEN : GREY;
  String msg = sdMsg;
  if (date != lastDate) { centerText(date, 250, 3, GREY); lastDate = date; }
  if (tm != lastTime) { centerText(tm, 300, 5, WHITE); lastTime = tm; }
  if (status != lastStatus) { centerText(status, 400, 3, col); lastStatus = status; }
  if (msg != lastMsg) { centerText(msg, 450, 2, GREY); lastMsg = msg; }
}
void wake() {
  offAt = millis() + SCREEN_ON_MS;
  if (!screenOn) {
    lastDate = lastTime = lastStatus = lastMsg = "#";   // force a full refresh
    updateDisplay();
    gfx->setBrightness(200);
    screenOn = true;
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(80);                       // plenty for this job, saves power
  i2cMtx = xSemaphoreCreateMutex();
  Wire.setBufferSize(1024);                     // IMU FIFO burst reads
  Wire.begin(IIC_SDA, IIC_SCL); Wire.setClock(400000);
  pinMode(BTN_BOOT, INPUT_PULLUP);
  pinMode(TP_RST, OUTPUT); digitalWrite(TP_RST, LOW); delay(10); digitalWrite(TP_RST, HIGH); delay(50);
  pinMode(TP_INT, INPUT); attachInterrupt(TP_INT, touchISR, FALLING);

  time_t e;
  if (rtcGet(e)) { struct timeval tv = { e, 0 }; settimeofday(&tv, nullptr); timeSynced = true; }

  if (!gfx->begin()) Serial.println("Display init failed");
  gfx->setBrightness(200);
  drawStatic();
  offAt = millis() + SCREEN_ON_MS;

  imuOk = imuInit();
  if (!imuOk) { setMsg("IMU not found"); Serial.println("QMI8658 not found"); }

  bleQ = xQueueCreate(40, sizeof(Pkt));
  sdQ = xQueueCreate(100, sizeof(Chunk));       // about 6 s of buffering for SD write stalls

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
  adv->addServiceUUID(SVC_UUID); adv->setName("BCMCH-Tremor"); adv->enableScanResponse(true); adv->start();

  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(sendTask, "tx", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(sdTask, "sd", 8192, nullptr, 2, nullptr, 1);
}

void loop() {
  uint32_t now = millis();
  static bool down = false, longDone = false; static uint32_t pressT = 0, lastBtn = 0, lastTouch = 0, lastDraw = 0;

  if (now - lastBtn >= 20) {                    // BOOT button, 20 ms debounce
    lastBtn = now;
    bool pr = !digitalRead(BTN_BOOT);
    if (pr && !down) { down = true; pressT = now; longDone = false; }
    else if (pr && down && !longDone && now - pressT >= 1000) {
      longDone = true; if (sdRec) sdCmdStop = true; wake();
    } else if (!pr && down) {
      down = false;
      if (!longDone && now - pressT >= 30) {
        if (!screenOn) wake();
        else { if (sdRec) sdCmdStop = true; else sdCmdStart = true; wake(); }
      }
    }
  }
  if (touchFlag) { touchFlag = false; if (now - lastTouch > 200) { lastTouch = now; wake(); } }
  if (wakeReq) { wakeReq = false; wake(); }
  if (sdMsgNew) { sdMsgNew = false; wake(); }

  if (screenOn) {
    if (now - lastDraw >= 200) { lastDraw = now; updateDisplay(); }
    if ((int32_t)(now - offAt) > 0) { gfx->setBrightness(0); screenOn = false; }
  }
  delay(10);
}
