
/*
 * Dual TLE5012B True Hardware SSC Driver
 * Board: Seeed Studio XIAO ESP32-S3
 * Pin Configuration: Shared DATA node on MOSI/MISO with 470R isolation
 */

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ========== Direction Control ==========
#define ENC1_DIR_CW  false
#define ENC2_DIR_CW  false

// ========== EXACT PIN CONFIGURATION ==========
static const int PIN_SCK  = D8;   // GPIO7
static const int PIN_MISO = D9;   // GPIO8
static const int PIN_MOSI = D10;  // GPIO9
static const int PIN_CS1  = D0;   // GPIO1
static const int PIN_CS2  = D1;   // GPIO2

SPIClass spiBus(FSPI);
SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE1); // 4 MHz for clean half-duplex transitions

// Register Commands
const uint16_t CMD_READ_ANGLE = 0x8020;
const uint16_t CMD_WRITE_MOD2 = 0x5081;
const uint16_t CMD_READ_MOD2  = 0xD081;
const uint16_t MOD2_DEFAULT   = 0x0801;
const uint16_t MOD2_ANG_DIR   = 0x0008;

// ========== TRIGONOMETRIC SMA FILTER ==========
const int SMA_WINDOW = 500;

struct VectorSMA {
  float sinBuffer[SMA_WINDOW] = {0};
  float cosBuffer[SMA_WINDOW] = {0};
  int index = 0;
  float sinSum = 0.0f;
  float cosSum = 0.0f;
  bool full = false;

  float update(float deg) {
    float rad = deg * (M_PI / 180.0f);
    float s = sinf(rad);
    float c = cosf(rad);

    if (full) {
      sinSum -= sinBuffer[index];
      cosSum -= cosBuffer[index];
    }

    sinBuffer[index] = s;
    cosBuffer[index] = c;
    sinSum += s;
    cosSum += c;

    if (++index >= SMA_WINDOW) {
      index = 0;
      full = true;
    }

    float avgDeg = atan2f(sinSum, cosSum) * (180.0f / M_PI);
    if (avgDeg < 0.0f) avgDeg += 360.0f;
    return avgDeg;
  }
};

VectorSMA sma1, sma2;
float latest1 = 0.0f, latest2 = 0.0f;
bool ok1 = false, ok2 = false;
uint32_t lastPrint = 0;

// Converts 15-bit two's complement raw value to clean [0, 360) degrees
inline float rawToDeg(uint16_t raw) {
  uint16_t raw15 = raw & 0x7FFF;
  int16_t signedVal = (int16_t)raw15;
  
  if (raw15 & 0x4000) {
    signedVal |= 0x8000; // Sign extension
  }

  float deg = ((float)signedVal * 360.0f) / 32768.0f;
  
  while (deg < 0.0f)   deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;

  return deg;
}

// Low-level helper: Tri-state MOSI pin to float
inline void disableMOSIDriver() {
  pinMode(PIN_MOSI, INPUT); // Removes ESP32 driver from line completely
}

// Low-level helper: Re-attach MOSI to SPI peripheral hardware
inline void enableMOSIDriver() {
  pinMode(PIN_MOSI, OUTPUT);
  // Connect PIN_MOSI back to FSPI MOSI signal (FSPID_OUT = 40 on ESP32-S3)
  esp_rom_gpio_connect_out_signal(PIN_MOSI, FSPID_OUT_IDX, false, false);
}

// ========== PROTOCOL-CORRECT TRUE SSC READ ==========
bool readEncoderSSCHW(uint8_t csPin, uint16_t &rawOut) {
  spiBus.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);

  // 1. PHASE 1: Master TX Command Word (16 bits)
  spiBus.transfer16(CMD_READ_ANGLE);

  // 2. PHASE 2: Bus Turnaround (t_wr_s)
  // Release MOSI line so the TLE5012B push-pull driver can take control
  disableMOSIDriver();
  delayMicroseconds(2); // Sensor switches from input to output

  // 3. PHASE 3: Slave RX Response Words (32 bits = Angle Data + Safety)
  uint16_t data   = spiBus.transfer16(0x0000);
  uint16_t safety = spiBus.transfer16(0x0000);

  // 4. Clean up bus state
  digitalWrite(csPin, HIGH);
  enableMOSIDriver(); // Restore MOSI for the next command phase
  spiBus.endTransaction();

  if (data == 0x0000 || data == 0xFFFF) return false;

  rawOut = data;
  return true;
}

// ========== REGISTER CONFIGURATION ==========
bool writeMod2(uint8_t csPin, uint16_t value) {
  spiBus.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);

  spiBus.transfer16(CMD_WRITE_MOD2);
  spiBus.transfer16(value);

  // Read back safety word
  disableMOSIDriver();
  delayMicroseconds(2);
  uint16_t safety = spiBus.transfer16(0x0000);

  digitalWrite(csPin, HIGH);
  enableMOSIDriver();
  spiBus.endTransaction();

  return (safety != 0x0000 && safety != 0xFFFF);
}

bool readMod2(uint8_t csPin, uint16_t &value) {
  spiBus.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);

  spiBus.transfer16(CMD_READ_MOD2);

  disableMOSIDriver();
  delayMicroseconds(2);
  value = spiBus.transfer16(0x0000);
  spiBus.transfer16(0x0000); // Safety word

  digitalWrite(csPin, HIGH);
  enableMOSIDriver();
  spiBus.endTransaction();

  return (value != 0x0000 && value != 0xFFFF);
}

bool setAngleDirection(uint8_t csPin, bool clockwise) {
  uint16_t mod2 = MOD2_DEFAULT;
  uint16_t current = 0;

  if (readMod2(csPin, current)) {
    mod2 = current & 0x7FFF;
  }

  if (clockwise) mod2 |= MOD2_ANG_DIR;
  else           mod2 &= ~MOD2_ANG_DIR;

  if (!writeMod2(csPin, mod2)) return false;

  uint16_t check = 0;
  if (!readMod2(csPin, check)) return false;
  return ((check & MOD2_ANG_DIR) != 0) == clockwise;
}

// ========== SETUP & LOOP ==========
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_CS1, OUTPUT);
  pinMode(PIN_CS2, OUTPUT);
  digitalWrite(PIN_CS1, HIGH);
  digitalWrite(PIN_CS2, HIGH);

  spiBus.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);

  delay(10); 

  bool d1 = setAngleDirection(PIN_CS1, ENC1_DIR_CW);
  bool d2 = setAngleDirection(PIN_CS2, ENC2_DIR_CW);

  Serial.println("\n--- True Hardware SSC Dual TLE5012B Driver ---");
  Serial.printf("ENC1 Init: %s\n", d1 ? "OK" : "FAIL");
  Serial.printf("ENC2 Init: %s\n", d2 ? "OK" : "FAIL");
}

void loop() {
  uint16_t r1 = 0, r2 = 0;

  ok1 = readEncoderSSCHW(PIN_CS1, r1);
  if (ok1) latest1 = sma1.update(rawToDeg(r1));

  ok2 = readEncoderSSCHW(PIN_CS2, r2);
  if (ok2) latest2 = sma2.update(rawToDeg(r2));

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    if (ok1 && ok2) {
      Serial.printf("ENC1: %7.4f deg | ENC2: %7.4f deg\n", latest1, latest2);
    } else {
      Serial.printf("ENC1: %s | ENC2: %s\n", 
                    ok1 ? "OK" : "ERR", 
                    ok2 ? "OK" : "ERR");
    }
  }
}