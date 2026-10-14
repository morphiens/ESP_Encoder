/*
 * TLE5012B Power-Cycle Drift Test (3min + 1000-SMA) — XIAO ESP32-S3 Plus
 *
 * Pairs with: monitor_tle5012_powercycles_3min_sma.py
 *
 * Timing: 2s boot, 1000-sample burst (1 raw + 999 for SMA), 3min off
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
static const uint32_t POWER_OFF_MS  = 180000;
static const uint32_t COUNTDOWN_S   = 30;
static const int SMA_SAMPLES        = 1000;

static const uint16_t CMD_READ_ANGLE = 0x8020;

SPIClass spi(FSPI);
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

enum State { ST_POWER_ON, ST_WAIT_BOOT, ST_READ, ST_POWER_OFF, ST_WAIT_OFF };

static State state = ST_POWER_ON;
static uint32_t cycleNum = 0;
static uint32_t stateStartMs = 0;
static uint32_t lastCountdownMs = 0;

static inline float rawToDeg(uint16_t raw) {
  return (float)raw * 360.0f / 32768.0f;
}

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

static bool captureRawAndSma(float &rawAngle, uint16_t &rawCount, float &filteredAngle) {
  uint16_t raw = 0;
  if (!readEncoder(raw)) {
    return false;
  }

  rawCount = raw;
  rawAngle = rawToDeg(raw);

  double sum = rawAngle;
  for (int i = 1; i < SMA_SAMPLES; ++i) {
    if (!readEncoder(raw)) {
      return false;
    }
    sum += rawToDeg(raw);
  }

  filteredAngle = (float)(sum / SMA_SAMPLES);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_PWR, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  sensorPower(false);

  Serial.println("--- TLE5012B Power-Cycle Test (3min off, 1000-SMA) ---");
  Serial.println("PINS SCK=D8 MISO=D9 MOSI=D10 CS=D0 PWR=D2");
  Serial.println("TIMING boot=2s off=180s sma=1000");

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
      float rawAngle = 0.0f;
      float filteredAngle = 0.0f;
      uint16_t rawCount = 0;

      if (captureRawAndSma(rawAngle, rawCount, filteredAngle)) {
        Serial.printf(
          "CYCLE=%lu ANGLE=%.6f FILTERED=%.6f RAW=%u STATUS=OK\n",
          cycleNum, rawAngle, filteredAngle, rawCount);
      } else {
        Serial.printf(
          "CYCLE=%lu ANGLE=nan FILTERED=nan RAW=0 STATUS=ERR\n",
          cycleNum);
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
      lastCountdownMs = now;
      state = ST_WAIT_OFF;
      break;

    case ST_WAIT_OFF:
      if (now - stateStartMs >= POWER_OFF_MS) {
        state = ST_POWER_ON;
      } else if (COUNTDOWN_S > 0 &&
                 (now - lastCountdownMs) >= COUNTDOWN_S * 1000UL) {
        lastCountdownMs = now;
        uint32_t remaining = (POWER_OFF_MS - (now - stateStartMs)) / 1000UL;
        Serial.printf("WAIT=%lus\n", remaining);
      }
      break;
  }
}
