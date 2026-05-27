#include <SPI.h>
#include <WiFi.h>
#include <algorithm>
#include <esp_now.h>
#include <esp_wifi.h>

/*
 * Oliver Master Gateway - Serial-Only Edition
 * Hardware: XIAO ESP32C6
 * Features:
 * - 30s blocking discovery for all 5 slaves
 * - Continuous re-discovery for missing slaves
 * - Serial command interface for manual re-discovery and on-demand reading
 * - Detailed status reporting
 * Author: Swaraj Dangare (Original) / Antigravity (Serial adaptation)
 */

#define NUM_SLAVES 1
#define WIFI_CHANNEL 11 // is for Oliver 4 set

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

uint8_t slaveMACs[NUM_SLAVES][6];
bool slaveFound[NUM_SLAVES] = {false};
Payload slaves[NUM_SLAVES];
uint32_t gatewayPacketIdx = 0;
uint32_t lastSeenTime[NUM_SLAVES] = {0}; // Track last response time

bool rediscoverRequested = false;
bool readRequested = false;
bool slaveResponded[NUM_SLAVES] = {false};

// ---------------- MASTER ENCODER SETTINGS ----------------
#define ANGLECOM 0x3FFF
#define DIAAGC_REG 0x3FFC
#define MAG_REG 0x3FFD
#define RD 0x40
#define NUM_BLOCKS 16
#define SAMPLES_PER_BLOCK 256

const int PIN_CS = D7;
const int PIN_SCK = D1;
const int PIN_MISO = D0;
const int PIN_MOSI = D10;

SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE1);

// Master encoder data
Payload masterData = {255, 0, 0, 0, 0, 0, 0, 0}; // id=255 for master

// ESP-NOW Receive Callback
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data,
                  int len) {
  if (len == 1) { // Discovery Response
    uint8_t id = data[0];
    if (id < NUM_SLAVES) {
      if (!slaveFound[id]) {
        memcpy(slaveMACs[id], info->src_addr, 6);
        slaveFound[id] = true;
        esp_now_peer_info_t peer{};
        memcpy(peer.peer_addr, info->src_addr, 6);
        peer.channel = WIFI_CHANNEL;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
        // Safe to print here - quick message
        Serial.printf(
            "[DISCOVERY] Found Slave %d: %02X:%02X:%02X:%02X:%02X:%02X\n", id,
            info->src_addr[0], info->src_addr[1], info->src_addr[2],
            info->src_addr[3], info->src_addr[4], info->src_addr[5]);
      }
      lastSeenTime[id] = millis();
    }
  } else if (len == sizeof(Payload)) { // Data Response
    Payload p;
    memcpy(&p, data, sizeof(p));
    if (p.id < NUM_SLAVES) {
      slaves[p.id] = p;
      lastSeenTime[p.id] = millis();
      slaveResponded[p.id] = true;
    }
  }
}

// ---------------- MASTER ENCODER FUNCTIONS ----------------
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
    delayMicroseconds(50);
  }
  double finalCounts = getMedian(blockMeans, NUM_BLOCKS);
  return (finalCounts * 360.0) / 16384.0;
}

// Discovery Function
void discoverSlaves(uint32_t timeoutMs) {
  uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint32_t startTime = millis();

  Serial.printf(
      "[DISCOVERY] Starting discovery for %d slaves (timeout: %dms)...\n",
      NUM_SLAVES, timeoutMs);

  while (millis() - startTime < timeoutMs) {
    // Count found slaves
    int foundCount = 0;
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (slaveFound[i])
        foundCount++;
    }

    // Exit early if all found
    if (foundCount == NUM_SLAVES) {
      Serial.println("[DISCOVERY] All slaves found!");
      break;
    }

    // Send broadcast
    uint8_t ping = 0xFF;
    esp_now_send(broadcastAddr, &ping, 1);

    // Print status every 2 seconds
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 2000) {
      lastPrint = millis();
      Serial.printf("[DISCOVERY] Progress: %d/%d slaves found | Missing: ",
                    foundCount, NUM_SLAVES);
      for (int i = 0; i < NUM_SLAVES; i++) {
        if (!slaveFound[i])
          Serial.printf("S%d ", i);
      }
      Serial.println();
    }

    delay(200); // Broadcast every 200ms
  }

  // Final report
  int finalCount = 0;
  for (int i = 0; i < NUM_SLAVES; i++) {
    if (slaveFound[i])
      finalCount++;
  }

  Serial.println("\n" + String('=', 50));
  Serial.printf("[DISCOVERY] Complete: %d/%d slaves discovered\n", finalCount,
                NUM_SLAVES);
  if (finalCount < NUM_SLAVES) {
    Serial.print("[WARNING] Missing slaves: ");
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (!slaveFound[i])
        Serial.printf("S%d ", i);
    }
    Serial.println("\n[INFO] Will retry in background...");
  }
  Serial.println(String('=', 50) + "\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize

  Serial.println("\n\n=== OLIVER MASTER GATEWAY (SERIAL-ONLY) ===");
  Serial.println("Hardware: XIAO ESP32C6");
  Serial.printf("Firmware: Robust Discovery v2.0 (Serial adaptation)\n\n");

  // WiFi Init
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("[WIFI] MAC Address: %s\n", WiFi.macAddress().c_str());

  // Master Encoder SPI Init
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  Serial.println("[SPI] Master encoder initialized");

  // Force WiFi channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.printf("[WIFI] Channel %d locked\n", WIFI_CHANNEL);

  // ESP-NOW Init
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW Init Failed!");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  Serial.println("[ESP-NOW] Initialized");

  // Add Broadcast Peer
  uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t bcast{};
  memcpy(bcast.peer_addr, broadcastAddr, 6);
  bcast.channel = WIFI_CHANNEL;
  bcast.encrypt = false;
  esp_now_add_peer(&bcast);
  Serial.println("[ESP-NOW] Broadcast peer added\n");

  // Initial Discovery (30 seconds, blocking)
  discoverSlaves(30000);

  Serial.println("\n=== GATEWAY READY (SERIAL COMMANDS ENABLED) ===\n");
}

void loop() {
  static uint32_t lastRediscover = 0;

  // ── Serial Command Handler ──
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "READ") {
      readRequested = true;
      for (int i = 0; i < NUM_SLAVES; i++)
        slaveResponded[i] = false;
    } else if (cmd == "REDISCOVER") {
      rediscoverRequested = true;
      Serial.println("[CMD] Re-discovery requested from Serial");
    }
  }

  // Handle manual re-discovery request
  if (rediscoverRequested) {
    rediscoverRequested = false;
    Serial.println("\n[CMD] Manual re-discovery triggered!");
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (!slaveFound[i]) {
        Serial.printf("[REDISCOVER] Will search for S%d\n", i);
      }
    }
    discoverSlaves(15000);
  }

  // Automatic re-discovery every 10 seconds for missing slaves
  if (millis() - lastRediscover > 10000) {
    lastRediscover = millis();
    int missingCount = 0;
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (!slaveFound[i])
        missingCount++;
    }

    if (missingCount > 0) {
      Serial.printf("[AUTO-REDISCOVER] Searching for %d missing slaves...\n",
                    missingCount);
      uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      for (int attempt = 0; attempt < 5; attempt++) {
        uint8_t ping = 0xFF;
        esp_now_send(broadcastAddr, &ping, 1);
        delay(200);
      }
    }
  }

  // ── On-demand read: triggered by Serial "READ" command from PC ──
  if (readRequested) {
    readRequested = false;
    gatewayPacketIdx++;

    // 1. Read master's own encoder + diagnostics
    double masterAngle = getUltraPrecisionReading();
    masterData.value = (int)(masterAngle * 10000.0);
    masterData.packetIdx = gatewayPacketIdx;

    uint16_t diaagc = readRegister(DIAAGC_REG);
    uint16_t mag = readRegister(MAG_REG);
    masterData.agc = diaagc & 0xFF;
    masterData.mag = mag & 0x3FFF;
    masterData.magl = (diaagc >> 8) & 0x01;
    masterData.magh = (diaagc >> 10) & 0x01;
    masterData.cof = (diaagc >> 9) & 0x01;

    // 2. Request data from all discovered slaves
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (slaveFound[i]) {
        uint8_t req = i;
        esp_now_send(slaveMACs[i], &req, 1);
      }
    }

    // 3. Wait for slave responses (up to 200ms timeout)
    uint32_t waitStart = millis();
    while (millis() - waitStart < 200) {
      bool allResponded = true;
      for (int i = 0; i < NUM_SLAVES; i++) {
        if (slaveFound[i] && !slaveResponded[i]) {
          allResponded = false;
          break;
        }
      }
      if (allResponded)
        break;
      delay(1);
    }

    // 4. Build Data message (same 7-field format)
    String bleMsg = String(gatewayPacketIdx);

    bleMsg += "|M0:" + String(masterData.value) + "," +
              String(masterData.packetIdx) + "," + String(masterData.agc) +
              "," + String(masterData.mag) + "," + String(masterData.magl) +
              "," + String(masterData.magh) + "," + String(masterData.cof);

    for (int i = 0; i < NUM_SLAVES; i++) {
      if (slaveFound[i] && slaveResponded[i]) {
        bleMsg += "|S" + String(i) + ":" + String(slaves[i].value) + "," +
                  String(slaves[i].packetIdx) + "," + String(slaves[i].agc) +
                  "," + String(slaves[i].mag) + "," + String(slaves[i].magl) +
                  "," + String(slaves[i].magh) + "," + String(slaves[i].cof);
      } else {
        bleMsg += "|S" + String(i) + ":OFFLINE,0";
      }
    }

    // 5. Send to Serial for USB logging
    Serial.println("DATA:" + bleMsg);

    // 6. Serial log (original style)
    Serial.printf("[READ #%u] M0=%d", gatewayPacketIdx, masterData.value);
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (slaveFound[i] && slaveResponded[i])
        Serial.printf(" | S%d=%d", i, slaves[i].value);
      else
        Serial.printf(" | S%d=OFFLINE", i);
    }
    Serial.println();
  }

  delay(1);
}
