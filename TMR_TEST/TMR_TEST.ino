#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

// ============================================================
// PIN CONFIGURATION
// ============================================================
static const int PIN_SCK  = D1;
static const int PIN_MISO = D2;
static const int PIN_MOSI = D3;
static const int PIN_CS   = D4;
static const int PIN_PWR  = D0;
static const int ENABLE_SENSOR_PWR = 1;

// ============================================================
// SPI CONFIGURATION
// ============================================================
SPIClass spi(FSPI);
static const uint32_t SPI_HZ = 10000000;
static const uint8_t STREAM_CMD = 0x05;
static const uint8_t SPI_MODE_USED = SPI_MODE0;

// ============================================================
// TIMING
// ============================================================
static const uint32_t SAMPLE_US = 100;   // 10 kHz sample pacing during a burst
static const int BURST_N = 1000;         // 1 raw sample + 999 more, averaged fresh every time
static const uint32_t RECOVER_MS = 1500;
static const uint32_t PWR_OFF_MS     = 50;
static const uint32_t PWR_ON_WAIT_MS = 30;

static bool gWasMagOk = false;
static uint32_t tRecover = 0;

// ============================================================
// EXPLICIT FORWARD DECLARATIONS
// Written by hand so the Arduino IDE's automatic prototype
// generator (which inserts its own declarations at the very
// top of the file, before any of the code below) does not try
// to generate its own -- doing so is what caused the
// "does not name a type" error. None of these functions return
// a custom struct by value, which removes the ordering problem
// entirely.
// ============================================================
static inline float rawToDeg(uint16_t raw);
static uint8_t crc8_j1850(const uint8_t *msg, int nbytes);
static void csLow();
static void csHigh();
static void sensorPower(bool on);
static void spiBeginTxn();
static void spiEndTxn();
static void enterSpiMode();
static bool tadStream(uint16_t &outAngle, uint8_t &outStatus, bool &outCrcOk);
static void softPowerCycleSensor();
static bool captureBurst(float &outRaw, float &outFiltered, uint8_t &outStatus);
static void backgroundHealthCheck();
static void handleSerialRequest();

// ============================================================
// HELPERS
// ============================================================
static inline float rawToDeg(uint16_t raw) {
  return (float)raw * 360.0f / 65536.0f;
}

static uint8_t crc8_j1850(const uint8_t *msg, int nbytes) {
  uint8_t crc_reg = 0xFF;
  for (int byte_count = 0; byte_count < nbytes; ++byte_count) {
    uint8_t bit_point = 0x80;
    for (int bit_count = 0; bit_count < 8; ++bit_count, bit_point >>= 1) {
      uint8_t poly;
      if (bit_point & msg[byte_count]) {
        poly = (crc_reg & 0x80) ? 1 : 0x1C;
        crc_reg = (uint8_t)(((crc_reg << 1) | 1) ^ poly);
      } else {
        poly = (crc_reg & 0x80) ? 0x1D : 0;
        crc_reg = (uint8_t)((crc_reg << 1) ^ poly);
      }
    }
  }
  return (uint8_t)(~crc_reg);
}

static void csLow()  { digitalWrite(PIN_CS, LOW); }
static void csHigh() { digitalWrite(PIN_CS, HIGH); }

static void sensorPower(bool on) {
  if (!ENABLE_SENSOR_PWR) return;
  digitalWrite(PIN_PWR, on ? HIGH : LOW);
}

static void spiBeginTxn() {
  spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE_USED));
}
static void spiEndTxn() {
  spi.endTransaction();
  delayMicroseconds(5);
}

static void enterSpiMode() {
  csLow();
  delay(2);
  csHigh();
  delay(5);
}

// ============================================================
// SINGLE SENSOR TRANSFER
// Out-params instead of a returned struct: outAngle = 16-bit
// raw angle code, outStatus = status byte, outCrcOk = whether
// the received CRC matched the computed one.
// ============================================================
static bool tadStream(uint16_t &outAngle, uint8_t &outStatus, bool &outCrcOk) {
  uint8_t buf[6] = {STREAM_CMD, 0, 0, 0, 0, 0};

  spiBeginTxn();
  csLow();
  spi.transfer(buf, 6);
  csHigh();
  spiEndTxn();

  outAngle  = (uint16_t)((buf[1] << 8) | buf[2]);
  outStatus = buf[3];
  uint8_t crcRx = buf[4];

  uint8_t payload[3] = {buf[1], buf[2], buf[3]};
  uint8_t crcCalc = crc8_j1850(payload, 3);
  outCrcOk = (crcCalc == crcRx);

  return true;
}

static void softPowerCycleSensor() {
  csLow();
  spi.end();

  if (ENABLE_SENSOR_PWR) {
    sensorPower(false);
    delay(PWR_OFF_MS);
    sensorPower(true);
    delay(PWR_ON_WAIT_MS);
  } else {
    delay(PWR_OFF_MS);
  }

  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  csHigh();
  delay(5);
  enterSpiMode();

  gWasMagOk = false;
}

// ============================================================
// BURST CAPTURE
//
// Triggered on demand by the host. Takes exactly BURST_N (1000)
// FRESH, back-to-back samples starting at the moment of the
// trigger -- nothing from before this call is reused, and
// nothing carries over from a previous target angle. Sample 0
// is written to outRaw. The vector (sin/cos) mean of all
// BURST_N samples is written to outFiltered. Returns false if
// the magnet/CRC went bad mid-burst, in which case outFiltered
// is not valid.
// ============================================================
static bool captureBurst(float &outRaw, float &outFiltered, uint8_t &outStatus) {
  double sumSin = 0.0;
  double sumCos = 0.0;

  uint32_t nextSampleUs = micros();

  for (int i = 0; i < BURST_N; ++i) {

    while ((int32_t)(micros() - nextSampleUs) < 0) {
      // short busy-wait; the whole burst is only ~100 ms total
    }
    nextSampleUs += SAMPLE_US;

    uint16_t angle;
    uint8_t status;
    bool crcOk;
    tadStream(angle, status, crcOk);

    bool magOk = crcOk && ((status & 0x80) == 0);

    if (!magOk) {
      outStatus = status;
      return false;
    }

    float deg = rawToDeg(angle);

    if (i == 0) {
      outRaw = deg;
    }

    float rad = deg * (float)M_PI / 180.0f;
    sumSin += sinf(rad);
    sumCos += cosf(rad);

    outStatus = status;
  }

  float mean = atan2f((float)sumSin, (float)sumCos) * 180.0f / (float)M_PI;
  if (mean < 0.0f) mean += 360.0f;
  outFiltered = mean;

  return true;
}

// ============================================================
// BACKGROUND HEALTH MONITOR
// Runs between bursts. Watches for a dead magnet/link and
// power-cycles the sensor if it stays unhealthy.
// ============================================================
static void backgroundHealthCheck() {
  uint16_t angle;
  uint8_t status;
  bool crcOk;
  tadStream(angle, status, crcOk);

  bool magOk = crcOk && ((status & 0x80) == 0);

  if (magOk) {
    gWasMagOk = true;
  } else {
    gWasMagOk = false;
    uint32_t now = millis();
    if (now - tRecover >= RECOVER_MS) {
      tRecover = now;
      softPowerCycleSensor();
    }
  }
}

// ============================================================
// SERIAL COMMAND HANDLING
// '?' triggers exactly one burst capture and one response line.
// ============================================================
static void handleSerialRequest() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '?') {
      float raw = 0.0f;
      float filt = 0.0f;
      uint8_t status = 0;

      bool ok = captureBurst(raw, filt, status);

      if (ok) {
        Serial.printf("%.4f,%.4f,%u\n", raw, filt, status);
      } else {
        Serial.printf("NAN,NAN,%u\n", status);
      }
      Serial.flush();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_CS, OUTPUT);
  csHigh();

  if (ENABLE_SENSOR_PWR) {
    pinMode(PIN_PWR, OUTPUT);
    sensorPower(true);
  }

  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  delay(PWR_ON_WAIT_MS);
  enterSpiMode();

  gWasMagOk = false;
  tRecover = millis();
}

void loop() {
  backgroundHealthCheck();
  handleSerialRequest();
}
