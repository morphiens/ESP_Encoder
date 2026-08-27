// /*
//  * Dual TLE5012B Hardware SPI Reader (Seeed Studio XIAO ESP32-S3)
//  * Feature: 1000-Sample Circular Vector SMA Filter 
//  * Solution: Eliminates 0/360 degree wraparound glitches completely via Trigonometric Decomposition
//  */

// #include <Arduino.h>
// #include <SPI.h>
// #include <math.h>

// // ==========================================
// // PIN DEFINITIONS (XIAO ESP32-S3 GPIOs)
// // ==========================================
// // SPI Bus 1 (FSPI / SPI2)
// #define SCK1_PIN   7  // D8
// #define MISO1_PIN  8  // D9
// #define MOSI1_PIN  9  // D10
// #define CS1_PIN    4  // D3

// // SPI Bus 2 (HSPI / SPI3)
// #define SCK2_PIN   5  // D4
// #define MISO2_PIN  1  // D0
// #define MOSI2_PIN  6  // D5
// #define CS2_PIN    2  // D1

// const uint16_t CMD_READ_ANGLE = 0x8020; 

// // Hardware SPI settings (4 MHz, Mode 1)
// SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE1);

// // Hardware SPI Instances
// SPIClass spi1(FSPI);
// SPIClass spi2(HSPI);

// // ==========================================
// // VECTOR SMA FILTER (GLITCH-FREE AT 0/360°)
// // ==========================================
// const int SMA_WINDOW = 1000; 

// struct VectorSMA {
//   float sinBuffer[SMA_WINDOW] = {0};
//   float cosBuffer[SMA_WINDOW] = {0};
//   int index = 0;
//   float sinSum = 0;
//   float cosSum = 0;
//   bool full = false;

//   float update(float deg) {
//     // 1. Convert incoming angle to unit vector components
//     float rad = deg * (M_PI / 180.0f);
//     float s = sinf(rad);
//     float c = cosf(rad);

//     // 2. Subtract outgoing sample from running sums
//     if (full) {
//       sinSum -= sinBuffer[index];
//       cosSum -= cosBuffer[index];
//     }

//     // 3. Add new sample to buffers and running sums
//     sinBuffer[index] = s;
//     cosBuffer[index] = c;
//     sinSum += s;
//     cosSum += c;

//     index++;
//     if (index >= SMA_WINDOW) {
//       index = 0;
//       full = true;
//     }

//     // 4. Reconstruct mean vector back to degrees using four-quadrant arctan
//     float avgRad = atan2f(sinSum, cosSum);
//     float avgDeg = avgRad * (180.0f / M_PI);

//     // 5. Wrap safely into positive range [0, 360)
//     if (avgDeg < 0.0f) {
//       avgDeg += 360.0f;
//     }
    
//     return avgDeg;
//   }
// };

// VectorSMA sma1;
// VectorSMA sma2;

// // Output State Variables
// float latestFiltered1 = 0.0f;
// float latestFiltered2 = 0.0f;
// bool enc1Ok = false;
// bool enc2Ok = false;

// static uint32_t lastPrintTime = 0;

// // ==========================================
// // HARDWARE SPI READ FUNCTION
// // ==========================================
// bool readEncoderHW(SPIClass &bus, uint8_t csPin, uint16_t &rawOut) {
//   bus.beginTransaction(spiSettings);
//   digitalWrite(csPin, LOW);
//   delayMicroseconds(1);

//   // Send Read Angle Command
//   bus.transfer16(CMD_READ_ANGLE);

//   // Receive Response
//   uint16_t response = bus.transfer16(0x0000);

//   digitalWrite(csPin, HIGH);
//   bus.endTransaction();

//   // Basic floating bus check
//   if (response == 0x0000 || response == 0xFFFF) {
//     return false;
//   }

//   rawOut = response & 0x7FFF; // Extract 15-bit raw angle value
//   return true;
// }

// // ==========================================
// // UTILITY FUNCTIONS
// // ==========================================
// inline float rawToDegrees(uint16_t raw) {
//   return ((float)raw * 360.0f) / 32768.0f;
// }

// static double normalizeDeg3(double deg) {
//   deg = fmod(deg, 360.0);
//   if (deg < 0.0) deg += 360.0;
//   return round(deg * 1000.0) / 1000.0;
// }

// // ==========================================
// // SETUP
// // ==========================================
// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   // Chip Select Setup
//   pinMode(CS1_PIN, OUTPUT);
//   pinMode(CS2_PIN, OUTPUT);
//   digitalWrite(CS1_PIN, HIGH);
//   digitalWrite(CS2_PIN, HIGH);

//   // Hardware SPI Bus Initialization
//   spi1.begin(SCK1_PIN, MISO1_PIN, MOSI1_PIN, CS1_PIN);
//   spi2.begin(SCK2_PIN, MISO2_PIN, MOSI2_PIN, CS2_PIN);

//   Serial.println("\n--- Vector SMA (1000 Samples) Dual SPI Ready ---");
// }

// // ==========================================
// // MAIN LOOP
// // ==========================================
// void loop() {
//   uint16_t raw1 = 0;
//   uint16_t raw2 = 0;

//   // 1. HIGH-SPEED BACKGROUND VECTOR FILTERING
//   enc1Ok = readEncoderHW(spi1, CS1_PIN, raw1);
//   if (enc1Ok) {
//     latestFiltered1 = sma1.update(rawToDegrees(raw1));
//   }

//   enc2Ok = readEncoderHW(spi2, CS2_PIN, raw2);
//   if (enc2Ok) {
//     latestFiltered2 = sma2.update(rawToDegrees(raw2));
//   }

//   // 2. SERIAL OUTPUT PRINTING (Every 200ms)
//   if (millis() - lastPrintTime >= 100) {
//     lastPrintTime = millis();

//     if (enc1Ok && enc2Ok) {
//       Serial.printf("ENC1: %.3f deg | ENC2: %.3f deg\n", 
//                     normalizeDeg3(latestFiltered1), 
//                     normalizeDeg3(latestFiltered2));
//     } else if (!enc1Ok && !enc2Ok) {
//       Serial.println("ENC1: ERR | ENC2: ERR");
//     } else if (!enc1Ok) {
//       Serial.printf("ENC1: ERR | ENC2: %.3f deg\n", normalizeDeg3(latestFiltered2));
//     } else {
//       Serial.printf("ENC1: %.3f deg | ENC2: ERR\n", normalizeDeg3(latestFiltered1));
//     }
//   }
// }


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
#define MISO1_PIN  D9  // GPIO8
#define MOSI1_PIN  D5  // GPIO9
#define CS1_PIN    D4  // GPIO4

#define SCK2_PIN   D8    // GPIO6
#define MISO2_PIN  D0    // GPIO1
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