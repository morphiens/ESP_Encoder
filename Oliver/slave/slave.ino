#include <SPI.h>
#include <WiFi.h>
#include <algorithm>
#include <esp_now.h>
#include <esp_wifi.h>

/**
 * Project: Wireless Ultra-Precision Slave (On-Demand)
 * Protocol: ESP-NOW (Request -> Response)
 * Sensor: AS5047D (SPI)
 * Logic: Wait for Request -> Measure (~15ms) -> Reply
 * Author: Swaraj Dangare
 */

#define SLAVE_ID 9 //<--- CHANGE THIS FOR EACH BOARD: 0, 1, 2, 3, 4
#define WIFI_CHANNEL                                                           \
  11 // Must match master's WIFI_CHANNEL (use 1, 6, or 11) 11 is for Oliver 4
     // set

// ---------------- COMMUNICATION ----------------
typedef struct __attribute__((packed)) {
  uint8_t id;
  int value; // Angle * 10000
  uint32_t packetIdx;
  uint8_t agc;  // AS5047D AGC value (0-255)
  uint16_t mag; // AS5047D CORDIC magnitude (14-bit)
  uint8_t magl; // Magnetic field too low  (0 or 1)
  uint8_t magh; // Magnetic field too high (0 or 1)
  uint8_t cof;  // CORDIC overflow         (0 or 1)
} Payload;

Payload pkt;
uint32_t packet_counter = 0;

// ---------------- AS5047D SETTINGS ----------------
#define ANGLECOM 0x3FFF
#define DIAAGC_REG 0x3FFC
#define MAG_REG 0x3FFD
#define RD 0x40
#define NUM_BLOCKS 16
#define SAMPLES_PER_BLOCK 256
#define TOTAL_SAMPLES (NUM_BLOCKS * SAMPLES_PER_BLOCK) // 4096

// XIAO ESP32C3 SPI Pins (Standard)
const int PIN_CS = D7;
const int PIN_SCK = D1;
const int PIN_MISO = D0;
const int PIN_MOSI = D10;

SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE1);

// ---------------- UTILS & ENCODER ----------------
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

uint16_t readRegister(uint16_t addr) {
  uint16_t result;
  SPI.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer16(makeReadCmd(addr));
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_CS, LOW);
  result = SPI.transfer16(0x0000);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return result & 0x3FFF;
}

// Robust Mean: Calculate mean of samples within 1.5 LSB distance from initial
// median
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

double getUltraPrecisionReading() {
  double blockMeans[NUM_BLOCKS];
  uint16_t blockSamples[SAMPLES_PER_BLOCK];

  for (int b = 0; b < NUM_BLOCKS; b++) {
    for (int s = 0; s < SAMPLES_PER_BLOCK; s++) {
      uint16_t raw = AS5047D_Read();
      if (((raw >> 15) & 1) == evenParityBit(raw)) {
        blockSamples[s] = raw;
      } else {
        s--;
      }
    }
    blockMeans[b] = getRobustMean(blockSamples, SAMPLES_PER_BLOCK);
    delayMicroseconds(50); // Small yield
  }
  double finalCounts = getMedian(blockMeans, NUM_BLOCKS);
  return (finalCounts * 360.0) / 16384.0;
}

// ---------------- ESP-NOW CALLBACK ----------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // 1. Discovery Response (Master sends 0xFF)
  if (len == 1 && data[0] == 0xFF) {
    // Add small random delay to prevent collision when multiple slaves respond
    // This staggers responses across 10-30ms window
    delayMicroseconds(random(10000, 30000)); // 10-30ms

    uint8_t reply = SLAVE_ID;
    // Add master as peer dynamically so we can reply
    if (!esp_now_is_peer_exist(info->src_addr)) {
      esp_now_peer_info_t peer{};
      memcpy(peer.peer_addr, info->src_addr, 6);
      peer.channel = WIFI_CHANNEL;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
    }
    esp_now_send(info->src_addr, &reply, 1);
  }

  // 2. Data Request (Master sends Slave ID)
  if (len == 1 && data[0] == SLAVE_ID) {
    // Perform Measurement (Block-Blocking but fast enough ~15-20ms)
    double angle = getUltraPrecisionReading();

    // Read diagnostic registers
    uint16_t diaagc = readRegister(DIAAGC_REG);
    uint16_t mag = readRegister(MAG_REG);

    pkt.id = SLAVE_ID;
    pkt.value = (int)(angle * 10000.0);
    pkt.packetIdx = packet_counter++;
    pkt.agc = diaagc & 0xFF;
    pkt.mag = mag & 0x3FFF;
    pkt.magl = (diaagc >> 8) & 0x01;
    pkt.magh = (diaagc >> 10) & 0x01;
    pkt.cof = (diaagc >> 9) & 0x01;

    esp_now_send(info->src_addr, (uint8_t *)&pkt, sizeof(pkt));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("init");
  // SPI Setup
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // WiFi / ESP-NOW Setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Force WiFi channel (must match master)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.printf("Slave %d Ready | Channel %d | MAC: %s\n", SLAVE_ID,
                WIFI_CHANNEL, WiFi.macAddress().c_str());
}

void loop() {
  // Nothing here - totally interrupt driven
  delay(100);
}