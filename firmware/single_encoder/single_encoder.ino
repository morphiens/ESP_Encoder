#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLESecurity.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <SPI.h>
#include <algorithm> // For std::sort

/**
 * ============================================================
 *  Single Encoder BLE Firmware Template   [firmware/single_encoder/]
 *  Board: XIAO ESP32C3 / ESP32C6
 *  Sensor: AS5047D 14-bit Magnetic Rotary Encoder (SPI)
 *
 *  For multi-encoder (master+slave) projects, see:
 *    firmware/master/master.ino  — ESP32C6 master
 *    firmware/slave/slave.ino    — ESP32C3 slave
 * ============================================================
 *
 *  CONFIGURE BEFORE FLASHING:
 *  Change ESP_NAME, SERVICE_UUID, and CHARACTERISTIC_UUID below.
 *  Everything else (SPI, sampling algorithm, BLE logic) does not need editing.
 *
 *  Strategy: Hybrid Robust Filtering
 *   1. 4096 Samples total.
 *   2. Divided into 16 blocks of 256 samples each.
 *   3. Each block: Robust Mean (discards samples > 1.5 LSB from mean).
 *   4. Final Result: Median of the 16 block-means.
 * ============================================================
 * Author: Swaraj Dangare
 */

// ============================================================
//  CONFIGURE BEFORE FLASHING
// ============================================================
#define ESP_NAME "YOUR_DEVICE_NAME"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// ============================================================

// ---------------- AS5047D SETTINGS ----------------
#define ANGLECOM 0x3FFF
#define DIAAGC 0x3FFC // Diagnostic and AGC register
#define MAG 0x3FFD    // CORDIC magnitude register
#define RD 0x40
#define NUM_BLOCKS 16
#define SAMPLES_PER_BLOCK 256
#define TOTAL_SAMPLES (NUM_BLOCKS * SAMPLES_PER_BLOCK) // 4096

// SPI Pins (XIAO ESP32C3/C6)
// SCK=D1, MISO=D0, MOSI=D10, CS=D7
const int PIN_CS = D7;
const int PIN_SCK = D1;
const int PIN_MISO = D0;
const int PIN_MOSI = D10;

SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE1);

// ---------------- BLE STATE ----------------
BLECharacteristic *pChar;
bool deviceConnected = false;
bool readRequested = false;
bool zeroRequest = false;
double zeroOffset = 0.0;
uint32_t packetIdx = 0; // Global packet counter

// ---------------- BLE CALLBACKS ----------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Connected");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("[BLE] Disconnected");
    BLEDevice::getAdvertising()->start();
  }
};

class WriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String value = pChar->getValue().c_str();
    if (value == "READ") {
      readRequested = true;
    } else if (value == "ZERO") {
      zeroRequest = true;
      Serial.println("[CMD] Zero reset requested");
    }
  }
};

// ---------------- UTILS ----------------
uint16_t evenParityBit(uint16_t x) {
  x &= 0x7FFF;
  return __builtin_parity(x);
}

uint16_t makeReadCmd(uint16_t addr) {
  uint16_t cmd = (1 << 14) | (addr & 0x3FFF);
  cmd |= (evenParityBit(cmd) << 15);
  return cmd;
}

uint16_t AS5047D_Read() {
  uint16_t result;
  SPI.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer16(makeReadCmd(ANGLECOM));
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_CS, LOW);
  result = SPI.transfer16(0x0000);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return result;
}

uint16_t AS5047D_ReadRegister(uint16_t address) {
  uint16_t result;
  SPI.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer16(makeReadCmd(address));
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_CS, LOW);
  result = SPI.transfer16(0x0000);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return result;
}

// ---------------- STATISTICS ----------------
double getRobustMean(uint16_t *samples, int size) {
  double sum = 0;
  for (int i = 0; i < size; i++)
    sum += (samples[i] & 0x3FFF);
  double initialMean = sum / size;

  double robustSum = 0;
  int count = 0;
  for (int i = 0; i < size; i++) {
    uint16_t val = samples[i] & 0x3FFF;
    if (abs((double)val - initialMean) < 1.5) {
      robustSum += val;
      count++;
    }
  }
  return (count > 0) ? (robustSum / count) : initialMean;
}

double getMedian(double *values, int size) {
  std::sort(values, values + size);
  return values[size / 2];
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  BLEDevice::init(ESP_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pChar = pService->createCharacteristic(CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                             BLECharacteristic::PROPERTY_WRITE);
  pChar->addDescriptor(new BLE2902());
  pChar->setCallbacks(new WriteCallback());
  pService->start();
  BLEDevice::getAdvertising()->start();
  Serial.println("[BLE] Advertising as '" ESP_NAME "' — awaiting READ command");
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // On-demand read: only fires when PC writes "READ" over BLE
  if (readRequested) {
    readRequested = false;
    packetIdx++;

    Serial.printf("[READ #%u] Sampling...\n", packetIdx);
    double blockMeans[NUM_BLOCKS];
    uint16_t blockSamples[SAMPLES_PER_BLOCK];

    for (int b = 0; b < NUM_BLOCKS; b++) {
      for (int s = 0; s < SAMPLES_PER_BLOCK; s++) {
        uint16_t raw = AS5047D_Read();
        if (((raw >> 15) & 1) == evenParityBit(raw)) {
          blockSamples[s] = raw;
        } else {
          s--; // Retry on parity error
        }
      }
      blockMeans[b] = getRobustMean(blockSamples, SAMPLES_PER_BLOCK);
      delayMicroseconds(50); // Brief yield to keep BLE alive
    }

    double finalCounts = getMedian(blockMeans, NUM_BLOCKS);
    double rawAngle    = (finalCounts * 360.0) / 16384.0;

    // Handle zero reset (applied on next READ after ZERO command)
    if (zeroRequest) {
      zeroOffset  = rawAngle;
      zeroRequest = false;
      Serial.printf("[ZERO] Offset set to %.5f\n", zeroOffset);
    }

    // Apply zero offset and normalize to [0, 360)
    double angle = rawAngle - zeroOffset;
    if (angle < 0)      angle += 360.0;
    if (angle >= 360.0) angle -= 360.0;

    // Angle as integer x 10000 (matches Oliver master format)
    int angleInt = (int)(angle * 10000.0);

    // Read diagnostics
    uint16_t diaagc    = AS5047D_ReadRegister(DIAAGC);
    uint16_t mag       = AS5047D_ReadRegister(MAG);
    uint8_t  agc       = diaagc & 0xFF;
    uint8_t  cof       = (diaagc >> 9)  & 1;
    uint8_t  magl      = (diaagc >> 10) & 1;
    uint8_t  magh      = (diaagc >> 11) & 1;
    uint16_t magnitude = mag & 0x3FFF;

    // BLE message: same 7-field format as Oliver master, single encoder only
    // <packetIdx>|M0:<angleInt>,<pkt>,<agc>,<mag>,<magl>,<magh>,<cof>
    String bleMsg = String(packetIdx);
    bleMsg += "|M0:" + String(angleInt)  + ","
                      + String(packetIdx) + ","
                      + String(agc)       + ","
                      + String(magnitude) + ","
                      + String(magl)      + ","
                      + String(magh)      + ","
                      + String(cof);

    if (deviceConnected && pChar) {
      pChar->setValue(bleMsg.c_str());
      pChar->notify();
    }

    Serial.printf("[READ #%u] M0=%d (%.5f deg) | AGC:%u MAG:%u MAGL:%u MAGH:%u COF:%u\n",
                  packetIdx, angleInt, angle, agc, magnitude, magl, magh, cof);
  }

  delay(1);
}
