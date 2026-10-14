/*
 * TLE5012B Power-Cycle Drift Test (5s + 1000-SMA) — XIAO ESP32-S3 Plus
 * Board: XIAO ESP32S3 Plus
 *
 * Pairs with: monitor_tle5012_powercycles_5s_sma.py
 *
 * Timing: 2s boot, read raw + 999 more samples for 1000-SMA filtered, 5s off
 * D8=SCK  D9=MISO  D10=MOSI  D0=CS  D2=power (HIGH=ON)
 */

#include <Arduino.h>
#include <SPI.h>

static const int PIN_SCK  = D8;
static const int PIN_MISO = D9;
static const int PIN_MOSI = D10;
static const int PIN_CS   = D0;
static const int PIN_PWR  = D2;

static const uint32_t BOOT_WAIT_MS  = 2000;
static const uint32_t POWER_OFF_MS  = 5000;
static const int SMA_N = 1000;

static const uint16_t CMD_READ_ANGLE = 0x8020;

SPIClass spi(FSPI);
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

enum State { ST_POWER_ON, ST_WAIT_BOOT, ST_READ, ST_POWER_OFF, ST_WAIT_OFF };

static State state = ST_POWER_ON;
static uint32_t cycleNum = 0;
static uint32_t stateStartMs = 0;

static void sensorPower(bool on) {
  digitalWrite(PIN_PWR, on ? HIGH : LOW);
}

static bool readEncoder(uint16_t &rawOut) {
  spi.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);

  spi.transfer16(CMD_READ_ANGLE);
  uint16_t data = spi.transfer16(0x0000);
  spi.transfer16(0x0000);

  digitalWrite(PIN_CS, HIGH);
  spi.endTransaction();
  delayMicroseconds(5);

  if (data == 0x0000 || data == 0xFFFF) {
    return false;
  }

  rawOut = data & 0x7FFF;
  return true;
}

static inline float rawToDeg(uint16_t raw) {
  return (float)raw * 360.0f / 32768.0f;
}

static bool captureRawAndFiltered(uint16_t &rawOut, float &rawDegOut, float &filteredDegOut) {
  uint16_t raw = 0;
  if (!readEncoder(raw)) {
    return false;
  }

  rawOut = raw;
  rawDegOut = rawToDeg(raw);

  double sum = rawDegOut;
  for (int i = 1; i < SMA_N; ++i) {
    uint16_t sample = 0;
    if (!readEncoder(sample)) {
      return false;
    }
    sum += rawToDeg(sample);
  }

  filteredDegOut = (float)(sum / (double)SMA_N);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_PWR, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  sensorPower(false);

  Serial.println("--- TLE5012B Power-Cycle Test (5s off, 1000-SMA) ---");
  Serial.println("PINS SCK=D8 MISO=D9 MOSI=D10 CS=D0 PWR=D2");
  Serial.println("TIMING boot=2s off=5s filter=1000-SMA");

  state = ST_POWER_ON;
  stateStartMs = millis();
}

void loop() {
  uint32_t now = millis();

  switch (state) {
    case ST_POWER_ON:
      cycleNum++;
      sensorPower(true);
      spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
      digitalWrite(PIN_CS, HIGH);
      Serial.printf("POWER_ON cycle=%lu\n", cycleNum);
      stateStartMs = now;
      state = ST_WAIT_BOOT;
      break;

    case ST_WAIT_BOOT:
      if (now - stateStartMs >= BOOT_WAIT_MS) {
        state = ST_READ;
      }
      break;

    case ST_READ: {
      uint16_t raw = 0;
      float rawDeg = 0.0f;
      float filteredDeg = 0.0f;

      if (captureRawAndFiltered(raw, rawDeg, filteredDeg)) {
        Serial.printf(
          "CYCLE=%lu ANGLE=%.6f FILTERED=%.6f RAW=%u STATUS=OK\n",
          cycleNum, rawDeg, filteredDeg, raw
        );
      } else {
        Serial.printf(
          "CYCLE=%lu ANGLE=nan FILTERED=nan RAW=0 STATUS=ERR\n",
          cycleNum
        );
      }
      state = ST_POWER_OFF;
      break;
    }

    case ST_POWER_OFF:
      spi.end();
      sensorPower(false);
      Serial.printf("POWER_OFF cycle=%lu off_for=%lus\n",
                    cycleNum, POWER_OFF_MS / 1000UL);
      stateStartMs = now;
      state = ST_WAIT_OFF;
      break;

    case ST_WAIT_OFF:
      if (now - stateStartMs >= POWER_OFF_MS) {
        state = ST_POWER_ON;
      }
      break;
  }
}
