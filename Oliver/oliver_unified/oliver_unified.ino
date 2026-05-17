/*
 * Oliver Unified Firmware
 * ------------------------
 * One firmware for every XIAO ESP32-C6 in the Oliver cluster.
 *
 * Boots into BLE SETUP mode when NVS is empty. The PC scans for
 * "Oliver_SETUP_<MAC_LAST3>" advertisers, blink-identifies the physical
 * board, and writes role + identity over GATT. After provisioning the board
 * reboots into one of three runtime modes:
 *
 *   - master      : BLE GATT gateway + ESP-NOW master, notifies the PC.
 *   - slave       : ESP-NOW only, owns one AS5047D encoder, no BLE.
 *   - standalone  : BLE GATT gateway, single on-board encoder, no ESP-NOW.
 *
 * Pair logic lives on the PC, not in firmware. The master accepts
 * "READ_MASK:<uint32>"  where bit 0 = M0 (master encoder),
 * bit 1 = S0, bit 2 = S1, ..., so the PC can read just the encoders
 * a given pair (blade / slide / chassis_drift / ...) needs.
 *
 * Hardware: XIAO ESP32-C6 (every board, master/slave/standalone)
 * Sensor:   AS5047D (SPI)
 * Author:   Oliver project
 */

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <SPI.h>
#include <WiFi.h>
#include <algorithm>
#include <esp_now.h>
#include <esp_wifi.h>

// =============================================================================
// Build-time constants
// =============================================================================

#define OLIVER_FW_VERSION "oliver-unified-1.0.0"
#define OLIVER_NVS_NAMESPACE "oliver"

// Setup-mode GATT (shared by every unprovisioned board).
#define OLIVER_SETUP_SERVICE_UUID  "5e740001-7b9c-4c2e-9a0f-3c5d6e7f8a90"
#define OLIVER_SETUP_INFO_UUID     "5e740002-7b9c-4c2e-9a0f-3c5d6e7f8a90"
#define OLIVER_SETUP_IDENTIFY_UUID "5e740003-7b9c-4c2e-9a0f-3c5d6e7f8a90"
#define OLIVER_SETUP_CONFIG_UUID   "5e740004-7b9c-4c2e-9a0f-3c5d6e7f8a90"

// Runtime GATT (used by master + standalone). Matches the Oliver-1 defaults
// already encoded in the gantry's Python client.
#define OLIVER_RUNTIME_SERVICE_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define OLIVER_RUNTIME_NOTIFY_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define OLIVER_RUNTIME_COMMAND_UUID  "8d53dc1d-1db7-4cd3-868b-8a527460aa84"

// Cluster limits.
#define OLIVER_MAX_SLAVES 31  // bit 0 = master, bits 1..31 = slaves S0..S30

// AS5047D registers + sampling.
#define ANGLECOM 0x3FFF
#define DIAAGC_REG 0x3FFC
#define MAG_REG 0x3FFD
#define NUM_BLOCKS 16
#define SAMPLES_PER_BLOCK 256

const int PIN_CS   = D7;
const int PIN_SCK  = D1;
const int PIN_MISO = D0;
const int PIN_MOSI = D10;

#ifndef LED_BUILTIN
#define LED_BUILTIN D15
#endif

// XIAO ESP32-C6 BOOT button (held LOW at power-up = wipe NVS / re-enter setup).
#ifndef BOOT_BUTTON_PIN
#define BOOT_BUTTON_PIN 9
#endif
#define BOOT_HOLD_MS 3000

SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE1);

// =============================================================================
// Types
// =============================================================================

enum class OliverRole : uint8_t {
  Unset      = 0,
  Master     = 1,
  Slave      = 2,
  Standalone = 3,
};

typedef struct __attribute__((packed)) {
  uint8_t  id;        // 255 = master, otherwise slave_id
  int32_t  value;     // angle * 10000
  uint32_t packetIdx;
  uint8_t  agc;
  uint16_t mag;
  uint8_t  magl;
  uint8_t  magh;
  uint8_t  cof;
} OliverPayload;

// Tagged ESP-NOW protocol. First byte is the tag.
enum OliverEspNowTag : uint8_t {
  OLI_TAG_DISC_PING       = 0x01,  // master broadcast, len=1
  OLI_TAG_DISC_REPLY      = 0x02,  // slave -> master, len=2  : [tag, slave_id]
  OLI_TAG_READ_REQ        = 0x03,  // master -> slave, len=2  : [tag, slave_id]
  OLI_TAG_DATA_REPLY      = 0x04,  // slave -> master, len=1+sizeof(OliverPayload)
  OLI_TAG_IDENTIFY        = 0x05,  // master -> slave, len=4  : [tag, slave_id, ms_lo, ms_hi]
  OLI_TAG_FACTORY_RESET   = 0x06,  // master -> slave, len=2  : [tag, slave_id]
};

// =============================================================================
// Globals
// =============================================================================

Preferences nvs;

OliverRole g_role        = OliverRole::Unset;
uint8_t    g_slave_id    = 0xFF;
uint8_t    g_wifi_ch     = 11;
String     g_ble_name    = "";        // master/standalone runtime BLE name
uint8_t    g_master_mac[6] = {0};     // slaves only

uint8_t    g_self_mac[6] = {0};
String     g_self_mac_str = "";

// Master state.
uint8_t  g_slave_mac[OLIVER_MAX_SLAVES][6];
bool     g_slave_found[OLIVER_MAX_SLAVES] = {false};
bool     g_slave_responded[OLIVER_MAX_SLAVES] = {false};
OliverPayload g_slave_data[OLIVER_MAX_SLAVES];
uint32_t g_slave_last_seen_ms[OLIVER_MAX_SLAVES] = {0};
uint32_t g_master_packet_idx = 0;
OliverPayload g_master_data = {255, 0, 0, 0, 0, 0, 0, 0};

// Master command handling.
volatile bool     g_read_requested = false;
volatile uint32_t g_read_mask      = 0xFFFFFFFF;  // default: read everything
volatile bool     g_rediscover_requested = false;
volatile bool     g_factory_reset_requested = false;
volatile uint8_t  g_factory_reset_target = 0xFF;  // 0xFF = self
volatile bool     g_identify_requested = false;
volatile uint16_t g_identify_ms = 5000;
volatile uint8_t  g_identify_target = 0xFF;       // 0xFF = self

// LED blink state (non-blocking).
uint32_t g_led_off_at_ms     = 0;
uint32_t g_led_toggle_at_ms  = 0;
bool     g_led_state         = false;
uint16_t g_led_period_ms     = 100;  // fast blink

// BLE pointers.
BLEServer         *g_ble_server  = nullptr;
BLECharacteristic *g_ble_notify  = nullptr;
BLECharacteristic *g_ble_cmd     = nullptr;
BLECharacteristic *g_ble_setup_info     = nullptr;
BLECharacteristic *g_ble_setup_identify = nullptr;
BLECharacteristic *g_ble_setup_config   = nullptr;
bool g_ble_client_connected = false;

volatile bool g_setup_config_pending = false;
String g_setup_config_payload = "";

// =============================================================================
// Helpers
// =============================================================================

static String macToString(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

static bool parseMac(const String &s, uint8_t out[6]) {
  unsigned int v[6];
  if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x",
             &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
    return false;
  }
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[i];
  return true;
}

static String macSuffix3(const uint8_t mac[6]) {
  char buf[7];
  snprintf(buf, sizeof(buf), "%02X%02X%02X", mac[3], mac[4], mac[5]);
  return String(buf);
}

// Trim leading/trailing whitespace + quotes.
static String trimToken(String s) {
  s.trim();
  if (s.length() >= 2 && s.charAt(0) == '"' && s.charAt(s.length() - 1) == '"') {
    s = s.substring(1, s.length() - 1);
  }
  return s;
}

// =============================================================================
// NVS persistence
// =============================================================================

static void nvsLoadConfig() {
  nvs.begin(OLIVER_NVS_NAMESPACE, true);
  g_role = (OliverRole)nvs.getUChar("role", (uint8_t)OliverRole::Unset);
  g_slave_id = nvs.getUChar("slave_id", 0xFF);
  g_wifi_ch  = nvs.getUChar("wifi_ch", 11);
  g_ble_name = nvs.getString("ble_name", "");
  size_t mlen = nvs.getBytesLength("master_mac");
  if (mlen == 6) {
    nvs.getBytes("master_mac", g_master_mac, 6);
  } else {
    memset(g_master_mac, 0, 6);
  }
  nvs.end();
}

static void nvsClear() {
  nvs.begin(OLIVER_NVS_NAMESPACE, false);
  nvs.clear();
  nvs.end();
}

static void nvsWriteRoleMaster(const String &ble_name, uint8_t wifi_ch) {
  nvs.begin(OLIVER_NVS_NAMESPACE, false);
  nvs.putUChar("role", (uint8_t)OliverRole::Master);
  nvs.putUChar("wifi_ch", wifi_ch);
  nvs.putString("ble_name", ble_name);
  nvs.end();
}

static void nvsWriteRoleSlave(uint8_t slave_id, uint8_t wifi_ch, const uint8_t master_mac[6]) {
  nvs.begin(OLIVER_NVS_NAMESPACE, false);
  nvs.putUChar("role", (uint8_t)OliverRole::Slave);
  nvs.putUChar("slave_id", slave_id);
  nvs.putUChar("wifi_ch", wifi_ch);
  nvs.putBytes("master_mac", master_mac, 6);
  nvs.end();
}

static void nvsWriteRoleStandalone(const String &ble_name) {
  nvs.begin(OLIVER_NVS_NAMESPACE, false);
  nvs.putUChar("role", (uint8_t)OliverRole::Standalone);
  nvs.putString("ble_name", ble_name);
  nvs.end();
}

// =============================================================================
// LED / identify (non-blocking)
// =============================================================================

static void ledInit() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // off (active-LOW on XIAO)
  g_led_state = false;
}

static void ledStartBlink(uint16_t total_ms, uint16_t period_ms = 100) {
  uint32_t now = millis();
  g_led_off_at_ms     = now + total_ms;
  g_led_toggle_at_ms  = now;
  g_led_period_ms     = period_ms == 0 ? 100 : period_ms;
}

static void ledStep() {
  uint32_t now = millis();
  if (g_led_off_at_ms == 0) return;
  if ((int32_t)(now - g_led_off_at_ms) >= 0) {
    g_led_off_at_ms = 0;
    g_led_state = false;
    digitalWrite(LED_BUILTIN, HIGH);  // off
    return;
  }
  if ((int32_t)(now - g_led_toggle_at_ms) >= 0) {
    g_led_state = !g_led_state;
    digitalWrite(LED_BUILTIN, g_led_state ? LOW : HIGH);  // active-LOW
    g_led_toggle_at_ms = now + g_led_period_ms;
  }
}

// =============================================================================
// AS5047D encoder (algorithm identical to existing master/slave)
// =============================================================================

static uint16_t evenParityBit(uint16_t x) {
  x &= 0x7FFF;
  return __builtin_parity(x);
}

static uint16_t makeReadCmd(uint16_t addr) {
  uint16_t cmd = (1 << 14) | (addr & 0x3FFF);
  cmd |= (evenParityBit(cmd) << 15);
  return cmd;
}

static uint16_t as5047_read() {
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

static uint16_t as5047_read_register(uint16_t addr) {
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

static double getRobustMean(uint16_t *samples, int size) {
  double sum = 0;
  for (int i = 0; i < size; i++) sum += (samples[i] & 0x3FFF);
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

static double getMedian(double *values, int size) {
  std::sort(values, values + size);
  return values[size / 2];
}

static double getUltraPrecisionReading() {
  double blockMeans[NUM_BLOCKS];
  uint16_t blockSamples[SAMPLES_PER_BLOCK];

  for (int b = 0; b < NUM_BLOCKS; b++) {
    for (int s = 0; s < SAMPLES_PER_BLOCK; s++) {
      uint16_t raw = as5047_read();
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

static void fillEncoderPayload(OliverPayload &out, uint8_t id, uint32_t pkt_idx) {
  double angle = getUltraPrecisionReading();
  uint16_t diaagc = as5047_read_register(DIAAGC_REG);
  uint16_t mag    = as5047_read_register(MAG_REG);
  out.id        = id;
  out.value     = (int32_t)(angle * 10000.0);
  out.packetIdx = pkt_idx;
  out.agc       = diaagc & 0xFF;
  out.mag       = mag & 0x3FFF;
  out.magl      = (diaagc >> 8) & 0x01;
  out.magh      = (diaagc >> 10) & 0x01;
  out.cof       = (diaagc >> 9) & 0x01;
}

// =============================================================================
// ESP-NOW: master + slave
// =============================================================================

static void espNowAddPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = g_wifi_ch;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

static void espNowAddBroadcastPeer() {
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  espNowAddPeer(bcast);
}

static void onEspNowRecvMaster(const esp_now_recv_info_t *info,
                               const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t tag = data[0];
  if (tag == OLI_TAG_DISC_REPLY && len >= 2) {
    uint8_t id = data[1];
    if (id < OLIVER_MAX_SLAVES) {
      if (!g_slave_found[id]) {
        memcpy(g_slave_mac[id], info->src_addr, 6);
        g_slave_found[id] = true;
        espNowAddPeer(info->src_addr);
        Serial.printf("[DISC] slave_id=%u mac=%s\n", id,
                      macToString(info->src_addr).c_str());
      }
      g_slave_last_seen_ms[id] = millis();
    }
  } else if (tag == OLI_TAG_DATA_REPLY && len >= (int)(1 + sizeof(OliverPayload))) {
    OliverPayload p;
    memcpy(&p, data + 1, sizeof(p));
    if (p.id < OLIVER_MAX_SLAVES) {
      g_slave_data[p.id] = p;
      g_slave_responded[p.id] = true;
      g_slave_last_seen_ms[p.id] = millis();
    }
  }
}

static void onEspNowRecvSlave(const esp_now_recv_info_t *info,
                              const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t tag = data[0];

  if (tag == OLI_TAG_DISC_PING && len == 1) {
    delayMicroseconds(random(10000, 30000));  // stagger replies
    espNowAddPeer(info->src_addr);
    uint8_t reply[2] = { OLI_TAG_DISC_REPLY, g_slave_id };
    esp_now_send(info->src_addr, reply, 2);
    return;
  }
  if (tag == OLI_TAG_READ_REQ && len >= 2 && data[1] == g_slave_id) {
    OliverPayload pkt;
    static uint32_t s_pkt_counter = 0;
    fillEncoderPayload(pkt, g_slave_id, s_pkt_counter++);
    uint8_t out[1 + sizeof(OliverPayload)];
    out[0] = OLI_TAG_DATA_REPLY;
    memcpy(out + 1, &pkt, sizeof(pkt));
    esp_now_send(info->src_addr, out, sizeof(out));
    return;
  }
  if (tag == OLI_TAG_IDENTIFY && len >= 4 && data[1] == g_slave_id) {
    uint16_t ms = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    ledStartBlink(ms, 80);
    return;
  }
  if (tag == OLI_TAG_FACTORY_RESET && len >= 2 && data[1] == g_slave_id) {
    Serial.println("[CMD] factory reset over ESP-NOW");
    nvsClear();
    delay(100);
    ESP.restart();
  }
}

// =============================================================================
// Master: discovery + read
// =============================================================================

static void masterDiscoverSlaves(uint32_t timeoutMs) {
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint32_t startTime = millis();
  Serial.printf("[DISC] sweep for %u ms\n", (unsigned)timeoutMs);
  while (millis() - startTime < timeoutMs) {
    uint8_t ping = OLI_TAG_DISC_PING;
    esp_now_send(bcast, &ping, 1);
    delay(200);
  }
  int found = 0;
  for (int i = 0; i < OLIVER_MAX_SLAVES; i++) if (g_slave_found[i]) found++;
  Serial.printf("[DISC] done, %d slave(s) known\n", found);
}

static void masterRequestReads(uint32_t mask) {
  // Reset response flags for slaves we are about to ask.
  for (int i = 0; i < OLIVER_MAX_SLAVES; i++) {
    g_slave_responded[i] = false;
  }

  // Read master encoder if bit 0 set.
  if (mask & 0x1u) {
    g_master_packet_idx++;
    fillEncoderPayload(g_master_data, 255, g_master_packet_idx);
  }

  // Request slaves whose bit (slave_id + 1) is set and which we have discovered.
  for (int i = 0; i < OLIVER_MAX_SLAVES; i++) {
    bool wanted = (mask >> (i + 1)) & 0x1u;
    if (wanted && g_slave_found[i]) {
      uint8_t req[2] = { OLI_TAG_READ_REQ, (uint8_t)i };
      esp_now_send(g_slave_mac[i], req, 2);
    }
  }

  // Wait up to 200 ms for all asked slaves to respond.
  uint32_t waitStart = millis();
  while (millis() - waitStart < 200) {
    bool allDone = true;
    for (int i = 0; i < OLIVER_MAX_SLAVES; i++) {
      bool wanted = (mask >> (i + 1)) & 0x1u;
      if (wanted && g_slave_found[i] && !g_slave_responded[i]) {
        allDone = false;
        break;
      }
    }
    if (allDone) break;
    delay(1);
  }
}

static String formatEncoderField(const char *prefix, const OliverPayload &p) {
  String s = String(prefix) + ":" + String((int)p.value) + "," +
             String((unsigned)p.packetIdx) + "," + String((unsigned)p.agc) +
             "," + String((unsigned)p.mag) + "," + String((unsigned)p.magl) +
             "," + String((unsigned)p.magh) + "," + String((unsigned)p.cof);
  return s;
}

static String buildNotifyPayload(uint32_t mask) {
  String msg = String((unsigned)g_master_packet_idx);
  if (mask & 0x1u) {
    msg += "|";
    msg += formatEncoderField("M0", g_master_data);
  }
  for (int i = 0; i < OLIVER_MAX_SLAVES; i++) {
    bool wanted = (mask >> (i + 1)) & 0x1u;
    if (!wanted) continue;
    String label = "S" + String(i);
    if (g_slave_found[i] && g_slave_responded[i]) {
      msg += "|";
      msg += formatEncoderField(label.c_str(), g_slave_data[i]);
    } else if (g_slave_found[i]) {
      msg += "|" + label + ":OFFLINE,0";
    } else {
      msg += "|" + label + ":OFFLINE,0";
    }
  }
  return msg;
}

static void masterFactoryResetSlave(uint8_t slave_id) {
  if (slave_id >= OLIVER_MAX_SLAVES) return;
  if (!g_slave_found[slave_id]) {
    Serial.printf("[CMD] factory reset slave %u: unknown, skipped\n", slave_id);
    return;
  }
  uint8_t pkt[2] = { OLI_TAG_FACTORY_RESET, slave_id };
  esp_now_send(g_slave_mac[slave_id], pkt, 2);
  Serial.printf("[CMD] factory reset sent to slave %u\n", slave_id);
}

static void masterIdentifySlave(uint8_t slave_id, uint16_t ms) {
  if (slave_id >= OLIVER_MAX_SLAVES) return;
  if (!g_slave_found[slave_id]) return;
  uint8_t pkt[4] = {
    OLI_TAG_IDENTIFY,
    slave_id,
    (uint8_t)(ms & 0xFF),
    (uint8_t)((ms >> 8) & 0xFF),
  };
  esp_now_send(g_slave_mac[slave_id], pkt, 4);
}

// =============================================================================
// Runtime BLE (master + standalone)
// =============================================================================

class RuntimeServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    g_ble_client_connected = true;
    Serial.println("[BLE] runtime client connected");
  }
  void onDisconnect(BLEServer *) override {
    g_ble_client_connected = false;
    Serial.println("[BLE] runtime client disconnected");
    BLEDevice::startAdvertising();
  }
};

class RuntimeCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String value = pChar->getValue().c_str();
    value.trim();
    Serial.printf("[CMD] %s\n", value.c_str());

    if (value == "READ") {
      g_read_mask = 0xFFFFFFFFu;
      g_read_requested = true;
    } else if (value.startsWith("READ_MASK:")) {
      g_read_mask = (uint32_t)strtoul(value.c_str() + 10, nullptr, 0);
      g_read_requested = true;
    } else if (value == "REDISCOVER") {
      g_rediscover_requested = true;
    } else if (value.startsWith("IDENTIFY_SLAVE:")) {
      // IDENTIFY_SLAVE:<id>:<ms>
      int colon = value.indexOf(':', 15);
      if (colon > 15) {
        uint8_t id = (uint8_t)value.substring(15, colon).toInt();
        uint16_t ms = (uint16_t)value.substring(colon + 1).toInt();
        g_identify_target = id;
        g_identify_ms = ms;
        g_identify_requested = true;
      }
    } else if (value.startsWith("IDENTIFY:")) {
      g_identify_target = 0xFF;
      g_identify_ms = (uint16_t)value.substring(9).toInt();
      g_identify_requested = true;
    } else if (value == "FACTORY_RESET") {
      g_factory_reset_target = 0xFF;
      g_factory_reset_requested = true;
    } else if (value.startsWith("FACTORY_RESET_SLAVE:")) {
      g_factory_reset_target = (uint8_t)value.substring(20).toInt();
      g_factory_reset_requested = true;
    }
  }
};

static void startRuntimeBle(const String &ble_name, bool advertiseSetupSvc = false) {
  BLEDevice::init(ble_name.c_str());
  g_ble_server = BLEDevice::createServer();
  g_ble_server->setCallbacks(new RuntimeServerCallbacks());

  BLEService *svc = g_ble_server->createService(OLIVER_RUNTIME_SERVICE_UUID);
  g_ble_notify = svc->createCharacteristic(
      OLIVER_RUNTIME_NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  g_ble_notify->addDescriptor(new BLE2902());
  g_ble_cmd = svc->createCharacteristic(
      OLIVER_RUNTIME_COMMAND_UUID, BLECharacteristic::PROPERTY_WRITE);
  g_ble_cmd->setCallbacks(new RuntimeCommandCallbacks());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(OLIVER_RUNTIME_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("[BLE] runtime advertising as '%s'\n", ble_name.c_str());
  (void)advertiseSetupSvc;
}

// =============================================================================
// Setup-mode BLE
// =============================================================================

class SetupServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    g_ble_client_connected = true;
    Serial.println("[SETUP] BLE client connected");
  }
  void onDisconnect(BLEServer *) override {
    g_ble_client_connected = false;
    Serial.println("[SETUP] BLE client disconnected");
    BLEDevice::startAdvertising();
  }
};

class SetupIdentifyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String v = pChar->getValue().c_str();
    v.trim();
    if (v.startsWith("BLINK:")) {
      uint16_t ms = (uint16_t)v.substring(6).toInt();
      ledStartBlink(ms ? ms : 5000, 80);
      Serial.printf("[SETUP] identify blink %u ms\n", ms);
    }
  }
};

class SetupConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    g_setup_config_payload = String(pChar->getValue().c_str());
    g_setup_config_pending = true;
    Serial.printf("[SETUP] config payload received (%d bytes)\n",
                  g_setup_config_payload.length());
  }
};

// Tiny ad-hoc parser for setup payloads. Accepts either:
//   "role=master;ble_name=oliver_gateway;wifi_ch=11"
//   "role=slave;slave_id=2;wifi_ch=11;master_mac=AA:BB:CC:DD:EE:FF"
//   "role=standalone;ble_name=oliver_bench"
// Choosing key=value;... (over JSON) keeps the firmware free of a JSON
// dependency. The Python provisioning script formats this string.
static String setupGetField(const String &payload, const String &key) {
  String prefix = key + "=";
  int idx = 0;
  while (idx < (int)payload.length()) {
    int sc = payload.indexOf(';', idx);
    String token = (sc < 0) ? payload.substring(idx) : payload.substring(idx, sc);
    token.trim();
    if (token.startsWith(prefix)) {
      return trimToken(token.substring(prefix.length()));
    }
    if (sc < 0) break;
    idx = sc + 1;
  }
  return "";
}

static void applySetupConfig(const String &payload) {
  String role = setupGetField(payload, "role");
  role.toLowerCase();
  if (role == "master") {
    String ble_name = setupGetField(payload, "ble_name");
    if (ble_name.length() == 0) ble_name = "oliver_gateway";
    String chs = setupGetField(payload, "wifi_ch");
    uint8_t ch = chs.length() ? (uint8_t)chs.toInt() : 11;
    nvsWriteRoleMaster(ble_name, ch);
    Serial.printf("[SETUP] -> master ble_name='%s' ch=%u\n", ble_name.c_str(), ch);
  } else if (role == "slave") {
    String sid_s = setupGetField(payload, "slave_id");
    String chs   = setupGetField(payload, "wifi_ch");
    String mmac  = setupGetField(payload, "master_mac");
    uint8_t sid  = sid_s.length() ? (uint8_t)sid_s.toInt() : 0;
    uint8_t ch   = chs.length() ? (uint8_t)chs.toInt() : 11;
    uint8_t mac[6] = {0};
    parseMac(mmac, mac);
    nvsWriteRoleSlave(sid, ch, mac);
    Serial.printf("[SETUP] -> slave id=%u ch=%u master=%s\n",
                  sid, ch, macToString(mac).c_str());
  } else if (role == "standalone") {
    String ble_name = setupGetField(payload, "ble_name");
    if (ble_name.length() == 0) ble_name = "oliver_standalone_" + macSuffix3(g_self_mac);
    nvsWriteRoleStandalone(ble_name);
    Serial.printf("[SETUP] -> standalone ble_name='%s'\n", ble_name.c_str());
  } else {
    Serial.printf("[SETUP] unknown role '%s', ignoring\n", role.c_str());
    return;
  }
  delay(200);
  ESP.restart();
}

static void startSetupBle() {
  String name = "Oliver_SETUP_" + macSuffix3(g_self_mac);
  BLEDevice::init(name.c_str());
  g_ble_server = BLEDevice::createServer();
  g_ble_server->setCallbacks(new SetupServerCallbacks());

  BLEService *svc = g_ble_server->createService(OLIVER_SETUP_SERVICE_UUID);

  g_ble_setup_info = svc->createCharacteristic(
      OLIVER_SETUP_INFO_UUID, BLECharacteristic::PROPERTY_READ);
  String info = String("{\"mac\":\"") + g_self_mac_str +
                "\",\"role\":\"unset\",\"fw\":\"" + OLIVER_FW_VERSION + "\"}";
  g_ble_setup_info->setValue(info.c_str());

  g_ble_setup_identify = svc->createCharacteristic(
      OLIVER_SETUP_IDENTIFY_UUID, BLECharacteristic::PROPERTY_WRITE);
  g_ble_setup_identify->setCallbacks(new SetupIdentifyCallbacks());

  g_ble_setup_config = svc->createCharacteristic(
      OLIVER_SETUP_CONFIG_UUID, BLECharacteristic::PROPERTY_WRITE);
  g_ble_setup_config->setCallbacks(new SetupConfigCallbacks());

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(OLIVER_SETUP_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("[SETUP] advertising as '%s'\n", name.c_str());
}

// =============================================================================
// Boot button (factory-reset fallback)
// =============================================================================

static void checkBootButtonForFactoryReset() {
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  delay(20);
  if (digitalRead(BOOT_BUTTON_PIN) != LOW) return;
  uint32_t held = 0;
  while (digitalRead(BOOT_BUTTON_PIN) == LOW && held < BOOT_HOLD_MS) {
    delay(10);
    held += 10;
    if ((held % 200) == 0) {
      digitalWrite(LED_BUILTIN, (held / 200) % 2 ? LOW : HIGH);
    }
  }
  digitalWrite(LED_BUILTIN, HIGH);
  if (held >= BOOT_HOLD_MS) {
    Serial.println("[BOOT] factory reset (BOOT button held)");
    nvsClear();
    delay(200);
    ESP.restart();
  }
}

// =============================================================================
// Common WiFi / SPI bring-up
// =============================================================================

static void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  String mac_str = WiFi.macAddress();
  uint8_t mac[6];
  unsigned int v[6];
  if (sscanf(mac_str.c_str(), "%x:%x:%x:%x:%x:%x",
             &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6) {
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];
  } else {
    memset(mac, 0, 6);
  }
  memcpy(g_self_mac, mac, 6);
  g_self_mac_str = macToString(g_self_mac);
  Serial.printf("[WIFI] STA MAC %s\n", g_self_mac_str.c_str());
}

static void wifiSetChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.printf("[WIFI] channel %u locked\n", ch);
}

static void spiInit() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
}

// =============================================================================
// Setup / loop dispatch by role
// =============================================================================

static void setupRoleSetupMode() {
  Serial.println("[ROLE] SETUP (NVS empty)");
  startSetupBle();
}

static void loopRoleSetupMode() {
  if (g_setup_config_pending) {
    g_setup_config_pending = false;
    applySetupConfig(g_setup_config_payload);
    g_setup_config_payload = "";
  }
}

static void setupRoleMaster() {
  Serial.printf("[ROLE] MASTER ble='%s' ch=%u\n", g_ble_name.c_str(), g_wifi_ch);
  spiInit();
  wifiSetChannel(g_wifi_ch);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecvMaster);
  espNowAddBroadcastPeer();
  masterDiscoverSlaves(15000);
  startRuntimeBle(g_ble_name);
}

static void loopRoleMaster() {
  // Non-blocking mini-ping state: spread 5 pings × 200 ms across loop iterations
  // instead of blocking with delay(200) which stresses the shared BLE/WiFi radio.
  static uint32_t lastRediscover    = 0;
  static uint32_t nextMiniPingAt    = 0;
  static uint8_t  miniPingsLeft     = 0;
  static const uint8_t BCAST[6]     = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  if (g_rediscover_requested) {
    g_rediscover_requested = false;
    // Full sweep: takes up to 15 s. Only do it while no BLE client is active to
    // avoid starving the BLE radio supervision window.
    bool had_client = g_ble_client_connected;
    masterDiscoverSlaves(15000);
    if (had_client) {
      // Client very likely dropped due to radio contention; restart advertising
      // so it can reconnect automatically.
      BLEDevice::startAdvertising();
    }
    lastRediscover = millis();
  }

  // Periodic mini-ping: only when NO BLE client is connected so we don't
  // interrupt the shared radio and cause a supervision timeout / disconnect.
  if (!g_ble_client_connected && millis() - lastRediscover > 10000) {
    lastRediscover = millis();
    int missing = 0;
    for (int i = 0; i < OLIVER_MAX_SLAVES; i++) if (!g_slave_found[i]) missing++;
    if (missing > 0) {
      miniPingsLeft  = 5;
      nextMiniPingAt = millis();
    }
  }
  if (!g_ble_client_connected && miniPingsLeft > 0 && millis() >= nextMiniPingAt) {
    uint8_t ping = OLI_TAG_DISC_PING;
    esp_now_send(BCAST, &ping, 1);
    miniPingsLeft--;
    nextMiniPingAt = millis() + 200;
  }
  if (g_identify_requested) {
    g_identify_requested = false;
    if (g_identify_target == 0xFF) {
      ledStartBlink(g_identify_ms, 80);
    } else {
      masterIdentifySlave(g_identify_target, g_identify_ms);
    }
  }
  if (g_factory_reset_requested) {
    g_factory_reset_requested = false;
    if (g_factory_reset_target == 0xFF) {
      Serial.println("[CMD] factory reset (master)");
      nvsClear();
      delay(200);
      ESP.restart();
    } else {
      masterFactoryResetSlave(g_factory_reset_target);
    }
  }
  if (g_read_requested) {
    g_read_requested = false;
    uint32_t mask = g_read_mask;
    masterRequestReads(mask);
    String msg = buildNotifyPayload(mask);
    if (g_ble_client_connected && g_ble_notify) {
      g_ble_notify->setValue(msg.c_str());
      g_ble_notify->notify();
    }
    Serial.printf("[READ #%u] mask=0x%08X len=%u\n",
                  (unsigned)g_master_packet_idx, (unsigned)mask, msg.length());
  }
}

static void setupRoleSlave() {
  Serial.printf("[ROLE] SLAVE id=%u ch=%u master=%s\n",
                g_slave_id, g_wifi_ch, macToString(g_master_mac).c_str());
  spiInit();
  wifiSetChannel(g_wifi_ch);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecvSlave);
  // Optimistically peer the known master so we can reply without a handshake.
  uint8_t empty[6] = {0};
  if (memcmp(g_master_mac, empty, 6) != 0) {
    espNowAddPeer(g_master_mac);
  }
}

static void loopRoleSlave() {
  delay(20);
}

static void setupRoleStandalone() {
  Serial.printf("[ROLE] STANDALONE ble='%s'\n", g_ble_name.c_str());
  spiInit();
  startRuntimeBle(g_ble_name);
}

static void loopRoleStandalone() {
  if (g_identify_requested) {
    g_identify_requested = false;
    ledStartBlink(g_identify_ms, 80);
  }
  if (g_factory_reset_requested) {
    g_factory_reset_requested = false;
    Serial.println("[CMD] factory reset (standalone)");
    nvsClear();
    delay(200);
    ESP.restart();
  }
  if (g_read_requested) {
    g_read_requested = false;
    uint32_t mask = g_read_mask & 0x1u;  // only M0 available
    g_master_packet_idx++;
    if (mask & 0x1u) {
      fillEncoderPayload(g_master_data, 255, g_master_packet_idx);
    }
    String msg = buildNotifyPayload(mask);
    if (g_ble_client_connected && g_ble_notify) {
      g_ble_notify->setValue(msg.c_str());
      g_ble_notify->notify();
    }
  }
}

// =============================================================================
// Arduino entry points
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("=== OLIVER UNIFIED FIRMWARE ===");
  Serial.printf("fw=%s\n", OLIVER_FW_VERSION);

  ledInit();
  checkBootButtonForFactoryReset();
  wifiInit();
  nvsLoadConfig();

  switch (g_role) {
    case OliverRole::Master:     setupRoleMaster();     break;
    case OliverRole::Slave:      setupRoleSlave();      break;
    case OliverRole::Standalone: setupRoleStandalone(); break;
    case OliverRole::Unset:
    default:                     setupRoleSetupMode();  break;
  }
  Serial.println("[BOOT] setup() complete");
}

void loop() {
  ledStep();
  switch (g_role) {
    case OliverRole::Master:     loopRoleMaster();     break;
    case OliverRole::Slave:      loopRoleSlave();      break;
    case OliverRole::Standalone: loopRoleStandalone(); break;
    case OliverRole::Unset:
    default:                     loopRoleSetupMode();  break;
  }
  delay(1);
}
