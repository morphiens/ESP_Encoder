/*
 * Dual TLE5012B — XIAO ESP32-S3 Plus (PCB pins)
 * Direction via sensor MOD_2.ANG_DIR (rewritten each boot)
 *
 * Encoder 1 (FSPI): D8/D9/D10/D3
 * Encoder 2 (HSPI): D5/D0/D1/D2
 *
 * true  = Clockwise     (ANG_DIR = 1)
 * false = Counter-CW    (ANG_DIR = 0, factory default)
 */

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ========== Direction (change these) ==========
#define ENC1_DIR_CW  false
#define ENC2_DIR_CW  false

// ========== PCB pins ==========

#define SCK1_PIN   D1  // GPIO7
#define MISO1_PIN  D0  // GPIO8
#define MOSI1_PIN  D5  // GPIO9
#define CS1_PIN    D4  // GPIO4

#define SCK2_PIN   D8    // GPIO6
#define MISO2_PIN  D9    // GPIO1
#define MOSI2_PIN  D10   // GPIO2
#define CS2_PIN    D3    // GPIO3

SPIClass spi1(FSPI);
SPIClass spi2(HSPI);

const uint16_t CMD_READ_ANGLE = 0x8020;
// MOD_2 config access: write=0x5081, read=0xD081
const uint16_t CMD_WRITE_MOD2 = 0x5081;
const uint16_t CMD_READ_MOD2  = 0xD081;
const uint16_t MOD2_DEFAULT   = 0x0801;  // 360°, CCW, autocal on
const uint16_t MOD2_ANG_DIR   = 0x0008;  // bit 3

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

const int SMA_WINDOW = 500;

struct VectorSMA {
  float sinBuffer[SMA_WINDOW] = {0};
  float cosBuffer[SMA_WINDOW] = {0};
  int index = 0;
  float sinSum = 0, cosSum = 0;
  bool full = false;

  float update(float deg) {
    float rad = deg * (M_PI / 180.0f);
    float s = sinf(rad), c = cosf(rad);
    if (full) { sinSum -= sinBuffer[index]; cosSum -= cosBuffer[index]; }
    sinBuffer[index] = s; cosBuffer[index] = c;
    sinSum += s; cosSum += c;
    if (++index >= SMA_WINDOW) { index = 0; full = true; }
    float avgDeg = atan2f(sinSum, cosSum) * (180.0f / M_PI);
    if (avgDeg < 0.0f) avgDeg += 360.0f;
    return avgDeg;
  }
};

VectorSMA sma1, sma2;
float latest1 = 0, latest2 = 0;
bool ok1 = false, ok2 = false;
uint32_t lastPrint = 0;

bool readEncoderHW(SPIClass &bus, uint8_t cs, uint16_t &rawOut) {
  bus.beginTransaction(spiSettings);
  digitalWrite(cs, LOW);
  delayMicroseconds(2);

  bus.transfer16(CMD_READ_ANGLE);
  uint16_t data = bus.transfer16(0x0000);
  bus.transfer16(0x0000);  // safety word

  digitalWrite(cs, HIGH);
  bus.endTransaction();
  delayMicroseconds(5);

  if (data == 0x0000 || data == 0xFFFF) return false;
  rawOut = data & 0x7FFF;
  return true;
}

bool writeMod2(SPIClass &bus, uint8_t cs, uint16_t value) {
  bus.beginTransaction(spiSettings);
  digitalWrite(cs, LOW);
  delayMicroseconds(2);

  bus.transfer16(CMD_WRITE_MOD2);
  bus.transfer16(value);
  uint16_t safety = bus.transfer16(0x0000);

  digitalWrite(cs, HIGH);
  bus.endTransaction();
  delayMicroseconds(5);

  return (safety != 0x0000 && safety != 0xFFFF);
}

bool readMod2(SPIClass &bus, uint8_t cs, uint16_t &value) {
  bus.beginTransaction(spiSettings);
  digitalWrite(cs, LOW);
  delayMicroseconds(2);

  bus.transfer16(CMD_READ_MOD2);
  value = bus.transfer16(0x0000);
  bus.transfer16(0x0000);

  digitalWrite(cs, HIGH);
  bus.endTransaction();
  delayMicroseconds(5);

  return (value != 0x0000 && value != 0xFFFF);
}

// Option 1: set ANG_DIR in MOD_2 (must run every boot)
bool setAngleDirection(SPIClass &bus, uint8_t cs, bool clockwise) {
  uint16_t mod2 = MOD2_DEFAULT;

  // Prefer read-modify-write so other MOD_2 bits stay intact
  uint16_t current = 0;
  if (readMod2(bus, cs, current)) {
    mod2 = current & 0x7FFF;  // drop any status/valid bit if present
  }

  if (clockwise) mod2 |= MOD2_ANG_DIR;
  else           mod2 &= ~MOD2_ANG_DIR;

  if (!writeMod2(bus, cs, mod2)) return false;

  // Verify
  uint16_t check = 0;
  if (!readMod2(bus, cs, check)) return false;
  bool bitSet = (check & MOD2_ANG_DIR) != 0;
  return bitSet == clockwise;
}

inline float rawToDeg(uint16_t raw) {
  return ((float)raw * 360.0f) / 32768.0f;
}

static double norm3(double d) {
  d = fmod(d, 360.0);
  if (d < 0.0) d += 360.0;
  return round(d * 1000.0) / 1000.0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CS1_PIN, OUTPUT);
  pinMode(CS2_PIN, OUTPUT);
  digitalWrite(CS1_PIN, HIGH);
  digitalWrite(CS2_PIN, HIGH);

  spi1.begin(SCK1_PIN, MISO1_PIN, MOSI1_PIN, CS1_PIN);
  spi2.begin(SCK2_PIN, MISO2_PIN, MOSI2_PIN, CS2_PIN);

  delay(5);  // let sensors settle after power-up

  bool d1 = setAngleDirection(spi1, CS1_PIN, ENC1_DIR_CW);
  bool d2 = setAngleDirection(spi2, CS2_PIN, ENC2_DIR_CW);

  Serial.println("--- Dual SPI + ANG_DIR Ready ---");
  Serial.printf("ENC1 DIR: %s (%s)\n",
                ENC1_DIR_CW ? "CW" : "CCW", d1 ? "OK" : "WRITE FAIL");
  Serial.printf("ENC2 DIR: %s (%s)\n",
                ENC2_DIR_CW ? "CW" : "CCW", d2 ? "OK" : "WRITE FAIL");
}

void loop() {
  uint16_t r1 = 0, r2 = 0;

  ok1 = readEncoderHW(spi1, CS1_PIN, r1);
  if (ok1) latest1 = sma1.update(rawToDeg(r1));

  ok2 = readEncoderHW(spi2, CS2_PIN, r2);
  if (ok2) latest2 = sma2.update(rawToDeg(r2));

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    if (ok1 && ok2) {
      Serial.printf("ENC1: %.3f deg | ENC2: %.3f deg\n",
                    norm3(latest1), norm3(latest2));
    } else if (!ok1 && !ok2) {
      Serial.println("ENC1: ERR | ENC2: ERR");
    } else if (!ok1) {
      Serial.printf("ENC1: ERR | ENC2: %.3f deg\n", norm3(latest2));
    } else {
      Serial.printf("ENC1: %.3f deg | ENC2: ERR\n", norm3(latest1));
    }
  }
}