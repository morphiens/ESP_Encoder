#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLESecurity.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <SPI.h>
#include <algorithm> // For std::sort

/**
 * ============================================================
 *  Dual Encoder BLE Firmware Template     [firmware/dual_encoder/]
 *  Board: XIAO ESP32C3 / ESP32C6
 *  Sensor: 2x AS5047D 14-bit Magnetic Rotary Encoder (SPI)
 *
 *  Features:
 *  - Reads two encoders via SPI using two separate CS pins.
 *  - Sends data via BLE when connected (responds to "READ").
 *  - Automatically reads and outputs to Serial every 500ms
 *    when BLE is disconnected.
 * ============================================================
 */

// ============================================================
//  CONFIGURE BEFORE FLASHING
// ============================================================
#define ESP_NAME "DUAL_ENCODER_ESP"
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
// SCK=D1, MISO=D0, MOSI=D10
// Two CS pins for the two encoders
const int PIN_CS1 = D7; // CS for Encoder 1
const int PIN_CS2 = D6; // CS for Encoder 2

const int PIN_SCK = D8;
const int PIN_MISO = D9;
const int PIN_MOSI = D10;

SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE1);

// ---------------- BLE STATE ----------------
BLECharacteristic *pChar;
bool deviceConnected = false;
bool readRequested = false;
bool zeroRequest = false;
double zeroOffset1 = 0.0;
double zeroOffset2 = 0.0;
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

uint16_t AS5047D_Read(int csPin) {
  uint16_t result;
  SPI.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);
  SPI.transfer16(makeReadCmd(ANGLECOM));
  digitalWrite(csPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(csPin, LOW);
  result = SPI.transfer16(0x0000);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return result;
}

uint16_t AS5047D_ReadRegister(int csPin, uint16_t address) {
  uint16_t result;
  SPI.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);
  SPI.transfer16(makeReadCmd(address));
  digitalWrite(csPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(csPin, LOW);
  result = SPI.transfer16(0x0000);
  digitalWrite(csPin, HIGH);
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

double getUltraPrecisionReading(int csPin) {
  double blockMeans[NUM_BLOCKS];
  uint16_t blockSamples[SAMPLES_PER_BLOCK];

  for (int b = 0; b < NUM_BLOCKS; b++) {
    for (int s = 0; s < SAMPLES_PER_BLOCK; s++) {
      uint16_t raw = AS5047D_Read(csPin);
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
  double rawAngle = (finalCounts * 360.0) / 16384.0;
  return rawAngle;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_CS1, OUTPUT);
  digitalWrite(PIN_CS1, HIGH);
  pinMode(PIN_CS2, OUTPUT);
  digitalWrite(PIN_CS2, HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS1);

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
  Serial.println("[BLE] Advertising as '" ESP_NAME "'");
}

// ---------------- MAIN LOOP ----------------
void loop() {
  
  // Auto-trigger read every 500ms if BLE is not connected
  if (!deviceConnected) {
    static unsigned long lastAutoReadTime = 0;
    if (millis() - lastAutoReadTime >= 500) {
      lastAutoReadTime = millis();
      readRequested = true;
    }
  }

  // On-demand read from BLE, or auto read from loop above
  if (readRequested) {
    readRequested = false;
    packetIdx++;

    double rawAngle1 = getUltraPrecisionReading(PIN_CS1);
    double rawAngle2 = getUltraPrecisionReading(PIN_CS2);

    // Handle zero reset (applied on next READ after ZERO command)
    if (zeroRequest) {
      zeroOffset1 = rawAngle1;
      zeroOffset2 = rawAngle2;
      zeroRequest = false;
      Serial.printf("[ZERO] Offsets set to %.5f and %.5f\n", zeroOffset1, zeroOffset2);
    }

    // Apply zero offset and normalize to [0, 360) for Enc 1
    double angle1 = rawAngle1 - zeroOffset1;
    if (angle1 < 0)      angle1 += 360.0;
    if (angle1 >= 360.0) angle1 -= 360.0;
    int angleInt1 = (int)(angle1 * 10000.0);

    // Apply zero offset and normalize to [0, 360) for Enc 2
    double angle2 = rawAngle2 - zeroOffset2;
    if (angle2 < 0)      angle2 += 360.0;
    if (angle2 >= 360.0) angle2 -= 360.0;
    int angleInt2 = (int)(angle2 * 10000.0);

    // Read diagnostics Enc1
    uint16_t diaagc1    = AS5047D_ReadRegister(PIN_CS1, DIAAGC);
    uint16_t mag1_reg   = AS5047D_ReadRegister(PIN_CS1, MAG);
    uint8_t  agc1       = diaagc1 & 0xFF;
    uint8_t  cof1       = (diaagc1 >> 9)  & 1;
    uint8_t  magl1      = (diaagc1 >> 10) & 1;
    uint8_t  magh1      = (diaagc1 >> 11) & 1;
    uint16_t magnitude1 = mag1_reg & 0x3FFF;

    // Read diagnostics Enc2
    uint16_t diaagc2    = AS5047D_ReadRegister(PIN_CS2, DIAAGC);
    uint16_t mag2_reg   = AS5047D_ReadRegister(PIN_CS2, MAG);
    uint8_t  agc2       = diaagc2 & 0xFF;
    uint8_t  cof2       = (diaagc2 >> 9)  & 1;
    uint8_t  magl2      = (diaagc2 >> 10) & 1;
    uint8_t  magh2      = (diaagc2 >> 11) & 1;
    uint16_t magnitude2 = mag2_reg & 0x3FFF;

    // BLE message: single encoder uses same format, here we concatenate M0 and M1
    String bleMsg = String(packetIdx) 
                  + "|M0:" + String(angleInt1) + "," + String(packetIdx) + "," + String(agc1) + "," + String(magnitude1) + "," + String(magl1) + "," + String(magh1) + "," + String(cof1)
                  + "|M1:" + String(angleInt2) + "," + String(packetIdx) + "," + String(agc2) + "," + String(magnitude2) + "," + String(magl2) + "," + String(magh2) + "," + String(cof2);

    if (deviceConnected && pChar) {
      pChar->setValue(bleMsg.c_str());
      pChar->notify();
    }

    Serial.printf("[READ #%u] M0=%d (%.5f deg) | M1=%d (%.5f deg) \n",
                  packetIdx, angleInt1, angle1, angleInt2, angle2);
  }

  delay(1);
}
