wave/*
 * BCMCH Tremor Monitor - Firmware for Waveshare ESP32-S3-Touch-AMOLED-2.06
 *
 * Features:
 *  - CO5300 410x502 QSPI AMOLED display showing the BCMCH shield logo,
 *    "BCMCH Tremor", live PCF85063 RTC date/time, BLE/Recording/IMU/Battery status.
 *  - Onboard QMI8658C 6-axis IMU sampled at native ~208-224 Hz (6-DOF ODR).
 *  - High-efficiency binary BLE streaming (14 samples / 206-byte MTU-aligned packets).
 *  - Automatic RTC time synchronization from Android/PC upon BLE pairing.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#include <Arduino_GFX_Library.h>
#include <SensorQMI8658.hpp>
#include <SensorPCF85063.hpp>

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ===================== BOARD PINOUT (Waveshare ESP32-S3-Touch-AMOLED-2.06) =====================
#define LCD_SDIO0   4
#define LCD_SDIO1   5
#define LCD_SDIO2   6
#define LCD_SDIO3   7
#define LCD_SCLK    11
#define LCD_CS      12
#define LCD_RESET   8
#define LCD_TE      13
#define LCD_WIDTH   410
#define LCD_HEIGHT  502

#define IIC_SDA     15
#define IIC_SCL     14
#define TP_RST      9
#define TP_INT      38

// ===================== RGB565 COLORS =====================
#define COL_BLACK     0x0000
#define COL_WHITE     0xFFFF
#define COL_CARD_BG   0x10A2  // Dark slate panel (#101418)
#define COL_DIVIDER   0x31A6  // Muted slate line
#define COL_SHIELD    0x5B0C  // #5A6064 grey
#define COL_CROSS_RED 0xE104  // #E52224 red
#define COL_CYAN      0x07FF
#define COL_GREEN     0x262B  // #22C55E
#define COL_AMBER     0xFCE0  // #F59E0B
#define COL_RED       0xF208  // #EF4444
#define COL_MUTED     0x9CD3  // #94A3B8

// ===================== BLE UUIDS =====================
#define BLE_DEVICE_NAME       "BCMCH-Tremor"
#define SERVICE_UUID          "7e570001-b0c0-4a1e-9c0d-bc11c4000001"
#define CHAR_IMU_STREAM_UUID  "7e570002-b0c0-4a1e-9c0d-bc11c4000001"
#define CHAR_COMMAND_UUID     "7e570003-b0c0-4a1e-9c0d-bc11c4000001"
#define CHAR_STATUS_UUID      "7e570004-b0c0-4a1e-9c0d-bc11c4000001"

// ===================== BINARY PACKET FORMAT =====================
#define MAX_SAMPLES_PER_PKT   14
#define ACC_SCALE_FACTOR      4096.0f  // +/- 8G  -> 32768 / 8 = 4096 LSB/g
#define GYR_SCALE_FACTOR      64.0f    // +/- 512 dps -> 32768 / 512 = 64 LSB/(deg/s)

struct __attribute__((packed)) ImuSampleBin {
  uint16_t dt_us;
  int16_t  ax;
  int16_t  ay;
  int16_t  az;
  int16_t  gx;
  int16_t  gy;
  int16_t  gz;
};

struct __attribute__((packed)) ImuPacketBin {
  uint32_t     seq;
  uint32_t     startMs;
  uint8_t      count;
  uint8_t      flags;  // bit 0: recording active
  ImuSampleBin samples[MAX_SAMPLES_PER_PKT];
};

// ===================== HARDWARE OBJECTS =====================
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3
);
Arduino_CO5300 *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0 /* rotation */, LCD_WIDTH, LCD_HEIGHT,
  22 /* col_offset1 */, 0, 0, 0
);

SensorQMI8658  qmi;
SensorPCF85063 rtc;
XPowersAXP2101 power;

// ===================== BLE OBJECTS =====================
BLEServer         *pServer     = nullptr;
BLECharacteristic *pImuChar    = nullptr;
BLECharacteristic *pCmdChar    = nullptr;
BLECharacteristic *pStatusChar = nullptr;

// ===================== SHARED STATE =====================
SemaphoreHandle_t i2cMutex = nullptr;

volatile bool     bleConnected    = false;
volatile uint16_t bleConnId       = 0;
volatile bool     isRecording     = false;
volatile uint32_t packetSeq       = 0;
volatile uint32_t recPacketsSent  = 0;
volatile float    measuredHz      = 0.0f;
volatile int      batteryPct      = -1;
volatile bool     vbusPresent     = false;

volatile bool     uiDirtyTime     = true;
volatile bool     uiDirtyStatus   = true;
volatile bool     sendStatusNow   = false;

bool imuReady = false;
bool rtcReady = false;
bool pmuReady = false;

// 116x116 logo sprite buffer (even dimensions for CO5300)
#define LOGO_SIZE 116
static uint16_t logoBuf[LOGO_SIZE * LOGO_SIZE];

// ===================== HELPER FUNCTIONS =====================
static inline int16_t clampInt16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lrintf(v);
}

static uint16_t blendRGB565(uint8_t r0, uint8_t g0, uint8_t b0,
                            uint8_t r1, uint8_t g1, uint8_t b1, float t) {
  uint8_t r = (uint8_t)(r0 + (r1 - r0) * t);
  uint8_t g = (uint8_t)(g0 + (g1 - g0) * t);
  uint8_t b = (uint8_t)(b0 + (b1 - b0) * t);
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Distance to a line segment (for stylized rounded cross arms)
static float distToSegment(float px, float py, float ax, float ay, float bx, float by) {
  float vx = bx - ax, vy = by - ay;
  float wx = px - ax, wy = py - ay;
  float len2 = vx * vx + vy * vy;
  float t = (len2 > 1e-6f) ? ((wx * vx + wy * vy) / len2) : 0.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  float dx = px - (ax + t * vx);
  float dy = py - (ay + t * vy);
  return sqrtf(dx * dx + dy * dy);
}

// Distance to a convex polygon / capsule-tapered cross matching the uploaded BCMCH logo
static bool insideCross(float u, float v) {
  // Top vertical arm (slightly tapered upward)
  if (v >= -0.56f && v <= 0.0f) {
    float t = (v + 0.56f) / 0.56f;
    float hw = 0.095f + 0.055f * t;
    if (fabsf(u) <= hw) return true;
  }
  // Bottom stem
  if (v >= 0.0f && v <= 0.52f) {
    float t = v / 0.52f;
    float hw = 0.15f - 0.01f * t;
    if (fabsf(u) <= hw) return true;
  }
  // Left horizontal arm (slightly tilted upward on outer top, angled toward stem)
  float dLeft = distToSegment(u, v, -0.37f, -0.17f, 0.02f, -0.12f);
  if (dLeft <= 0.095f) return true;
  // Right horizontal arm (slightly lower outer tip matching the uploaded emblem)
  float dRight = distToSegment(u, v, -0.02f, -0.11f, 0.38f, -0.07f);
  if (dRight <= 0.105f) return true;

  // Rounded caps on top and bottom of vertical stem
  if (distToSegment(u, v, 0.0f, -0.50f, 0.0f, -0.10f) <= 0.095f) return true;
  if (distToSegment(u, v, 0.0f, -0.10f, 0.0f, 0.45f) <= 0.135f) return true;
  return false;
}

// Generates a 4x4 supersampled anti-aliased shield + stylized red cross sprite
static void generateLogoBitmap() {
  for (int py = 0; py < LOGO_SIZE; py++) {
    for (int px = 0; px < LOGO_SIZE; px++) {
      int sumR = 0, sumG = 0, sumB = 0;

      for (int sy = 0; sy < 4; sy++) {
        for (int sx = 0; sx < 4; sx++) {
          float u = ((px + (sx + 0.5f) * 0.25f) / (float)LOGO_SIZE) * 2.0f - 1.0f;
          float v = ((py + (sy + 0.5f) * 0.25f) / (float)LOGO_SIZE) * 2.0f - 1.0f;

          // Shield outer half-width as a function of vertical coordinate v
          bool inOuter = false;
          bool inInner = false;

          if (v >= -0.88f && v <= 0.90f) {
            float hwOuter = 0.86f;
            if (v > -0.18f) {
              float t = (v + 0.18f) / 1.08f;
              if (t > 1.0f) t = 1.0f;
              hwOuter = 0.86f * powf(1.0f - powf(t, 2.05f), 0.72f);
            }
            // Top corner rounding
            if (v < -0.76f && fabsf(u) > 0.74f) {
              float dx = fabsf(u) - 0.74f;
              float dy = -0.76f - v;
              inOuter = (dx * dx + dy * dy) <= (0.12f * 0.12f);
            } else {
              inOuter = (fabsf(u) <= hwOuter);
            }

            // Inner white region (inset border ~0.14)
            if (v >= -0.74f && v <= 0.74f) {
              float hwInner = 0.72f;
              if (v > -0.18f) {
                float t = (v + 0.18f) / 0.92f;
                if (t > 1.0f) t = 1.0f;
                hwInner = 0.72f * powf(1.0f - powf(t, 2.05f), 0.72f);
              }
              if (v < -0.66f && fabsf(u) > 0.64f) {
                float dx = fabsf(u) - 0.64f;
                float dy = -0.66f - v;
                inInner = (dx * dx + dy * dy) <= (0.08f * 0.08f);
              } else {
                inInner = (fabsf(u) <= hwInner);
              }
            }
          }

          uint8_t r = 0, g = 0, b = 0;  // Black AMOLED background
          if (inInner) {
            if (insideCross(u, v)) {
              r = 229; g = 34; b = 36;  // Red cross
            } else {
              r = 255; g = 255; b = 255; // White shield interior
            }
          } else if (inOuter) {
            r = 90; g = 96; b = 100;    // Slate grey shield border
          }
          sumR += r; sumG += g; sumB += b;
        }
      }

      uint8_t ar = sumR >> 4;
      uint8_t ag = sumG >> 4;
      uint8_t ab = sumB >> 4;
      logoBuf[py * LOGO_SIZE + px] = ((ar & 0xF8) << 8) | ((ag & 0xFC) << 3) | (ab >> 3);
    }
  }
}

// Helper to draw horizontally centered text with even-aligned clear box
static void drawCenteredText(int16_t y, const char *txt, uint8_t size,
                             uint16_t fg, uint16_t bg, int16_t clearHeight) {
  // Even-aligned clear strip across screen width
  int16_t cy = y & ~1;
  int16_t ch = (clearHeight + 1) & ~1;
  gfx->fillRect(0, cy, LCD_WIDTH, ch, bg);

  int16_t len = (int16_t)strlen(txt);
  int16_t textW = len * 6 * size;
  int16_t x = (LCD_WIDTH - textW) / 2;
  if (x < 0) x = 0;

  gfx->setTextSize(size);
  gfx->setTextColor(fg, bg);
  gfx->setCursor(x, y);
  gfx->print(txt);
}

static void drawStaticHeader() {
  gfx->fillScreen(COL_BLACK);

  // 1. Uploaded Logo at top-center (even coordinates for CO5300)
  int16_t logoX = ((LCD_WIDTH - LOGO_SIZE) / 2) & ~1; // 146
  gfx->draw16bitRGBBitmap(logoX, 20, logoBuf, LOGO_SIZE, LOGO_SIZE);

  // 2. "BCMCH Tremor" below the logo
  drawCenteredText(152, "BCMCH Tremor", 3, COL_WHITE, COL_BLACK, 28);

  // Subtle divider line (even height = 2px)
  gfx->fillRect(40, 192, LCD_WIDTH - 80, 2, COL_DIVIDER);

  // Bottom status panel background (even coordinates: x=24, y=312, w=362, h=168)
  gfx->fillRect(24, 312, 362, 168, COL_CARD_BG);
  // Top accent bar on card
  gfx->fillRect(24, 312, 362, 4, COL_CROSS_RED);
}

static void updateTimeOnScreen() {
  RTC_DateTime dt;
  bool ok = false;

  if (rtcReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    dt = rtc.getDateTime();
    xSemaphoreGive(i2cMutex);
    ok = true;
  }

  char timeStr[16];
  char dateStr[28];

  if (ok) {
    snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u",
             dt.getHour(), dt.getMinute(), dt.getSecond());
    snprintf(dateStr, sizeof(dateStr), "%04u-%02u-%02u",
             dt.getYear(), dt.getMonth(), dt.getDay());
  } else {
    snprintf(timeStr, sizeof(timeStr), "--:--:--");
    snprintf(dateStr, sizeof(dateStr), "RTC Initializing");
  }

  // Current Time & Date below "BCMCH Tremor"
  drawCenteredText(210, timeStr, 5, COL_CYAN, COL_BLACK, 44);
  drawCenteredText(264, dateStr, 2, COL_MUTED, COL_BLACK, 22);
}

static void updateStatusOnScreen() {
  char line1[40], line2[40], line3[40], line4[40];

  uint16_t bleCol = bleConnected ? COL_GREEN : COL_AMBER;
  snprintf(line1, sizeof(line1), "BLE: %s",
           bleConnected ? "Paired (Connected)" : "Advertising...");

  uint16_t recCol = isRecording ? COL_RED : COL_MUTED;
  snprintf(line2, sizeof(line2), "REC: %s",
           isRecording ? "RECORDING ACTIVE" : "IDLE / STANDBY");

  snprintf(line3, sizeof(line3), "IMU: %.1f Hz | Pkts: %lu",
           measuredHz, (unsigned long)recPacketsSent);

  if (batteryPct >= 0) {
    snprintf(line4, sizeof(line4), "BAT: %d%%%s",
             batteryPct, vbusPresent ? " (USB)" : "");
  } else {
    snprintf(line4, sizeof(line4), "PWR: %s", vbusPresent ? "USB Powered" : "Ready");
  }

  auto drawCardLine = [](int16_t y, const char *txt, uint16_t color) {
    gfx->fillRect(32, y & ~1, 346, 24, COL_CARD_BG);
    int16_t textW = (int16_t)strlen(txt) * 12; // size 2 = 12px/char
    int16_t x = (LCD_WIDTH - textW) / 2;
    if (x < 32) x = 32;
    gfx->setTextSize(2);
    gfx->setTextColor(color, COL_CARD_BG);
    gfx->setCursor(x, y);
    gfx->print(txt);
  };

  drawCardLine(332, line1, bleCol);
  drawCardLine(368, line2, recCol);
  drawCardLine(404, line3, COL_WHITE);
  drawCardLine(440, line4, COL_MUTED);
}

static void sendBleStatus() {
  if (!bleConnected || pStatusChar == nullptr) return;
  char json[128];
  snprintf(json, sizeof(json),
           "{\"rec\":%d,\"hz\":%.2f,\"pkts\":%lu,\"bat\":%d,\"usb\":%d}",
           isRecording ? 1 : 0,
           measuredHz,
           (unsigned long)recPacketsSent,
           batteryPct,
           vbusPresent ? 1 : 0);
  pStatusChar->setValue((uint8_t *)json, strlen(json));
  pStatusChar->notify();
}

// ===================== BLE CALLBACKS =====================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    bleConnected = true;
    bleConnId = s->getConnId();
    uiDirtyStatus = true;
    sendStatusNow = true;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    isRecording = false;
    uiDirtyStatus = true;
    Serial.println("[BLE] Client disconnected, restarting advertising");
    BLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChr) override {
    String cmd = pChr->getValue().c_str();
    cmd.trim();
    if (cmd.length() == 0) return;

    Serial.printf("[BLE CMD] %s\n", cmd.c_str());

    if (cmd.startsWith("T:")) {
      int yr, mo, dy, hr, mn, sc;
      if (sscanf(cmd.c_str() + 2, "%d,%d,%d,%d,%d,%d", &yr, &mo, &dy, &hr, &mn, &sc) == 6) {
        if (yr >= 2020 && yr <= 2099 && mo >= 1 && mo <= 12 && dy >= 1 && dy <= 31 &&
            hr >= 0 && hr <= 23 && mn >= 0 && mn <= 59 && sc >= 0 && sc <= 59) {
          if (rtcReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            rtc.setDateTime(yr, mo, dy, hr, mn, sc);
            xSemaphoreGive(i2cMutex);
          }
          uiDirtyTime = true;
          sendStatusNow = true;
          Serial.printf("[RTC] Synced to %04d-%02d-%02d %02d:%02d:%02d\n",
                        yr, mo, dy, hr, mn, sc);
        }
      }
    } else if (cmd == "START") {
      packetSeq = 0;
      recPacketsSent = 0;
      isRecording = true;
      uiDirtyStatus = true;
      sendStatusNow = true;
    } else if (cmd == "STOP") {
      isRecording = false;
      uiDirtyStatus = true;
      sendStatusNow = true;
    } else if (cmd == "REFRESH") {
      uiDirtyTime = true;
      uiDirtyStatus = true;
      sendStatusNow = true;
    }
  }
};

// ===================== IMU SAMPLING & BLE STREAM TASK =====================
void imuStreamTask(void *pvParameters) {
  ImuPacketBin pkt;
  uint8_t sampleIdx = 0;
  uint32_t lastSampleUs = micros();
  uint32_t hzWindowStartMs = millis();
  uint32_t hzSampleCount = 0;

  for (;;) {
    bool dataReady = false;
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;

    if (imuReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (qmi.getDataReady()) {
        qmi.getAccelerometer(ax, ay, az);
        qmi.getGyroscope(gx, gy, gz);
        dataReady = true;
      }
      xSemaphoreGive(i2cMutex);
    }

    if (dataReady) {
      uint32_t nowUs = micros();
      uint32_t nowMs = millis();
      uint32_t dt = nowUs - lastSampleUs;
      lastSampleUs = nowUs;
      if (dt > 65535UL) dt = 65535UL;

      hzSampleCount++;
      uint32_t elapsedHzMs = nowMs - hzWindowStartMs;
      if (elapsedHzMs >= 1000) {
        measuredHz = (hzSampleCount * 1000.0f) / (float)elapsedHzMs;
        hzSampleCount = 0;
        hzWindowStartMs = nowMs;
      }

      if (bleConnected && pImuChar != nullptr) {
        if (sampleIdx == 0) {
          pkt.seq = packetSeq++;
          pkt.startMs = nowMs;
          pkt.flags = isRecording ? 0x01 : 0x00;
        }

        pkt.samples[sampleIdx].dt_us = (uint16_t)dt;
        pkt.samples[sampleIdx].ax = clampInt16(ax * ACC_SCALE_FACTOR);
        pkt.samples[sampleIdx].ay = clampInt16(ay * ACC_SCALE_FACTOR);
        pkt.samples[sampleIdx].az = clampInt16(az * ACC_SCALE_FACTOR);
        pkt.samples[sampleIdx].gx = clampInt16(gx * GYR_SCALE_FACTOR);
        pkt.samples[sampleIdx].gy = clampInt16(gy * GYR_SCALE_FACTOR);
        pkt.samples[sampleIdx].gz = clampInt16(gz * GYR_SCALE_FACTOR);
        sampleIdx++;

        // Adapt packet sample count to negotiated BLE MTU (default 14 samples = 206 bytes)
        uint16_t mtu = pServer->getPeerMTU(bleConnId);
        uint8_t targetSamples = MAX_SAMPLES_PER_PKT;
        if (mtu >= 27 && mtu < 209) {
          uint8_t fit = (mtu - 13) / sizeof(ImuSampleBin);
          if (fit >= 1 && fit < targetSamples) targetSamples = fit;
        } else if (mtu > 0 && mtu < 27) {
          targetSamples = 1;
        }

        if (sampleIdx >= targetSamples) {
          pkt.count = sampleIdx;
          size_t pktBytes = 10 + (size_t)sampleIdx * sizeof(ImuSampleBin);
          pImuChar->setValue((uint8_t *)&pkt, pktBytes);
          pImuChar->notify();

          if (isRecording) {
            recPacketsSent++;
          }
          sampleIdx = 0;
        }
      } else {
        sampleIdx = 0;
      }
    } else {
      // Yield 1 ms when next IMU sample is not yet ready (~4.5 ms period at 224 Hz)
      vTaskDelay(1);
    }
  }
}

// ===================== SETUP & MAIN LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== BCMCH Tremor Monitor Booting ===");

  i2cMutex = xSemaphoreCreateMutex();

  // Keep touch controller out of reset so I2C bus remains clean
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, HIGH);

  // Initialize shared I2C at 400 kHz
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);

  // Initialize AXP2101 PMIC for battery telemetry
  if (power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    pmuReady = true;
    power.enableBattDetection();
    power.enableBattVoltageMeasure();
    power.enableVbusVoltageMeasure();
    batteryPct = power.getBatteryPercent();
    vbusPresent = power.isVbusIn();
    Serial.println("[PMU] AXP2101 initialized");
  } else {
    Serial.println("[PMU] AXP2101 not detected");
  }

  // Initialize PCF85063 RTC
  if (rtc.begin(Wire, PCF85063_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    rtcReady = true;
    RTC_DateTime dt = rtc.getDateTime();
    if (dt.getYear() < 2024 || dt.getYear() > 2099) {
      rtc.setDateTime(2026, 1, 1, 0, 0, 0);
    }
    Serial.println("[RTC] PCF85063 initialized");
  } else {
    Serial.println("[RTC] PCF85063 not detected");
  }

  // Initialize QMI8658C IMU at ~208-224 Hz native 6-DOF ODR
  if (qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_8G,
      SensorQMI8658::ACC_ODR_250Hz,
      SensorQMI8658::LPF_MODE_3  // 13.37% of ODR (~30 Hz cutoff, preserves tremor band)
    );
    qmi.configGyroscope(
      SensorQMI8658::GYR_RANGE_512DPS,
      SensorQMI8658::GYR_ODR_224_2Hz, // Drives 6-DOF ODR (~208-224 Hz native)
      SensorQMI8658::LPF_MODE_3
    );
    qmi.enableGyroscope();
    qmi.enableAccelerometer();
    imuReady = true;
    Serial.println("[IMU] QMI8658C initialized (6-DOF ~224.2 Hz nominal)");
  } else {
    Serial.println("[IMU] QMI8658C not detected");
  }

  // Initialize CO5300 AMOLED Display
  generateLogoBitmap();
  if (!gfx->begin()) {
    Serial.println("[LCD] gfx->begin() failed!");
  }
  gfx->setBrightness(210);
  drawStaticHeader();
  updateTimeOnScreen();
  updateStatusOnScreen();

  // Initialize BLE Server
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(256);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pImuChar = pService->createCharacteristic(
    CHAR_IMU_STREAM_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pImuChar->addDescriptor(new BLE2902());

  pCmdChar = pService->createCharacteristic(
    CHAR_COMMAND_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdChar->setCallbacks(new CommandCallbacks());

  pStatusChar = pService->createCharacteristic(
    CHAR_STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatusChar->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);  // Helps fast connection interval (7.5 ms)
  pAdv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising as BCMCH-Tremor");

  // Launch high-priority IMU + BLE streaming task on Core 1
  xTaskCreatePinnedToCore(
    imuStreamTask,
    "IMUStreamTask",
    4096,
    nullptr,
    2,
    nullptr,
    1
  );
}

void loop() {
  static uint32_t lastSecTick = 0;
  static uint32_t lastStatusTick = 0;
  uint32_t now = millis();

  // Refresh clock once per second or immediately after BLE time sync
  if (uiDirtyTime || (now - lastSecTick >= 1000)) {
    lastSecTick = now;
    uiDirtyTime = false;
    updateTimeOnScreen();
  }

  // Refresh telemetry & status every 1.5s or on state change
  if (uiDirtyStatus || (now - lastStatusTick >= 1500)) {
    lastStatusTick = now;
    uiDirtyStatus = false;

    if (pmuReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      batteryPct = power.getBatteryPercent();
      vbusPresent = power.isVbusIn();
      xSemaphoreGive(i2cMutex);
    }

    updateStatusOnScreen();
    sendBleStatus();
  }

  if (sendStatusNow) {
    sendStatusNow = false;
    sendBleStatus();
  }

  delay(25);
}