#include <Arduino.h>
#include <SPI.h>

// ============================================================
// PIN CONFIGURATION
// ============================================================

static const int PIN_SCK = D1;
static const int PIN_MISO = D2;
static const int PIN_MOSI = D3;
static const int PIN_CS = D4;

// D0 = 3V3 CONTROL
//
// HIGH = normal
// LOW  = reset / disable 3V3 control
//
static const int PIN_3V3_CONTROL = D0;


// ============================================================
// RESET / ERROR TIMING
// ============================================================

// Sensor must remain in error continuously for 3 seconds
static const uint32_t ERROR_TIMEOUT_MS = 3000;

// Keep D0 LOW for 200 ms during reset
static const uint32_t RESET_LOW_MS = 200;


// ============================================================
// SPI
// ============================================================

SPIClass spi(FSPI);

static const uint32_t SPI_HZ = 10000000;

// TAD2141 streaming command
static const uint8_t STREAM_CMD = 0x05;


// ============================================================
// ERROR / RESET STATE
// ============================================================

bool errorTimerRunning = false;
bool resetInProgress = false;

uint32_t errorStartTime = 0;
uint32_t resetStartTime = 0;


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  delay(1000);

  // --------------------------------------------------------
  // D0 = 3V3 CONTROL
  // Normal state = HIGH
  // --------------------------------------------------------
  pinMode(PIN_3V3_CONTROL, OUTPUT);
  digitalWrite(PIN_3V3_CONTROL, HIGH);

  // --------------------------------------------------------
  // CS
  // --------------------------------------------------------
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // --------------------------------------------------------
  // SPI
  // --------------------------------------------------------
  spi.begin(
    PIN_SCK,
    PIN_MISO,
    PIN_MOSI,
    -1);

  delay(100);

  Serial.println();
  Serial.println("TAD2141 RAW TEST");
  Serial.println("================");
  Serial.println("SPI: 10 MHz, MODE0");
  Serial.println("D0: 3V3 CONTROL");
  Serial.println();
}


// ============================================================
// CHECK WHETHER SENSOR VALUE IS VALID
// ============================================================

bool sensorValueIsValid(uint16_t raw) {
  // Treat these as invalid / error values.
  if (raw == 0x0000 || raw == 0xFFFF) {
    return false;
  }

  return true;
}


// ============================================================
// HANDLE ERROR + D0 RESET
// ============================================================

void handleSensorHealth(bool sensorOK) {
  // ========================================================
  // SENSOR IS HEALTHY
  // ========================================================

  if (sensorOK) {
    // Cancel any pending error timer
    errorTimerRunning = false;

    // Normal D0 state
    if (!resetInProgress) {
      digitalWrite(PIN_3V3_CONTROL, HIGH);
    }

    return;
  }


  // ========================================================
  // SENSOR IS IN ERROR
  // ========================================================

  // Start timer when error is first detected
  if (!errorTimerRunning) {
    errorStartTime = millis();
    errorTimerRunning = true;
  }


  // ========================================================
  // ERROR HAS LASTED 3 SECONDS
  // ========================================================

  if (!resetInProgress && (millis() - errorStartTime >= ERROR_TIMEOUT_MS)) {
    Serial.println();
    Serial.println("ERROR > 3s");
    Serial.println("D0 -> LOW (RESET)");

    // Assert reset / disable
    digitalWrite(PIN_3V3_CONTROL, LOW);

    resetStartTime = millis();
    resetInProgress = true;
  }


  // ========================================================
  // HOLD D0 LOW FOR RESET_LOW_MS
  // ========================================================

  if (resetInProgress && (millis() - resetStartTime >= RESET_LOW_MS)) {
    // Restore normal 3V3 control
    digitalWrite(PIN_3V3_CONTROL, HIGH);

    resetInProgress = false;

    // Start a NEW 3-second error window.
    // If sensor is still bad, another reset occurs
    // after another 3 seconds.
    errorStartTime = millis();
    errorTimerRunning = true;

    Serial.println("D0 -> HIGH (3V3 RESTORED)");
    Serial.println();
  }
}


// ============================================================
// READ SENSOR
// ============================================================

void readSensor() {
  uint8_t txrx[6] = {
    STREAM_CMD,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
  };


  spi.beginTransaction(
    SPISettings(
      SPI_HZ,
      MSBFIRST,
      SPI_MODE0));

  digitalWrite(PIN_CS, LOW);

  spi.transfer(txrx, 6);

  digitalWrite(PIN_CS, HIGH);

  spi.endTransaction();


  // --------------------------------------------------------
  // Raw angle
  // --------------------------------------------------------

  uint16_t raw =
    ((uint16_t)txrx[1] << 8) | txrx[2];


  // --------------------------------------------------------
  // Convert to degrees
  // --------------------------------------------------------

  float angle =
    (float)raw * 360.0f / 65536.0f;


  // --------------------------------------------------------
  // Determine sensor health
  // --------------------------------------------------------

  bool sensorOK =
    sensorValueIsValid(raw);


  // --------------------------------------------------------
  // Error / reset handling
  // --------------------------------------------------------

  handleSensorHealth(sensorOK);


  // --------------------------------------------------------
  // Print
  // --------------------------------------------------------

  Serial.printf(
    "RAW = %5u    ANGLE = %10.4f deg    STATUS = 0x%02X    RX = %02X %02X %02X %02X %02X %02X    D0 = %s\n",
    raw,
    angle,
    txrx[3],
    txrx[0],
    txrx[1],
    txrx[2],
    txrx[3],
    txrx[4],
    txrx[5],
    digitalRead(PIN_3V3_CONTROL) ? "HIGH" : "LOW");
}


// ============================================================
// LOOP
// ============================================================

void loop() {
  readSensor();

  delay(100);
}