#include <Arduino.h>
#include <SPI.h>

// ============================================================
// PIN CONFIGURATION
// ============================================================
static const int PIN_SCK  = D1;
static const int PIN_MISO = D2;
static const int PIN_MOSI = D3;
static const int PIN_CS   = D4;

// ============================================================
// SPI
// ============================================================
SPIClass spi(FSPI);

static const uint32_t SPI_HZ = 10000000;

// TAD2141 streaming command
static const uint8_t STREAM_CMD = 0x05;

// ============================================================
// SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);

    delay(1000);

    // CS
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);

    // SPI
    spi.begin(
        PIN_SCK,
        PIN_MISO,
        PIN_MOSI,
        -1
    );

    delay(100);

    Serial.println();
    Serial.println("TAD2141 RAW TEST");
    Serial.println("================");
    Serial.println("SPI: 10 MHz, MODE0");
    Serial.println();
}

// ============================================================
// READ SENSOR
// ============================================================
void readSensor()
{
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
            SPI_MODE0
        )
    );

    digitalWrite(PIN_CS, LOW);

    spi.transfer(txrx, 6);

    digitalWrite(PIN_CS, HIGH);

    spi.endTransaction();

    // --------------------------------------------------------
    // Raw angle
    // --------------------------------------------------------
    uint16_t raw =
        ((uint16_t)txrx[1] << 8) |
        txrx[2];

    // --------------------------------------------------------
    // Convert to degrees
    // --------------------------------------------------------
    float angle =
        (float)raw * 360.0f / 65536.0f;

    // --------------------------------------------------------
    // Print
    // --------------------------------------------------------
    Serial.printf(
        "RAW = %5u    ANGLE = %10.4f deg    STATUS = 0x%02X    RX = %02X %02X %02X %02X %02X %02X\n",
        raw,
        angle,
        txrx[3],
        txrx[0],
        txrx[1],
        txrx[2],
        txrx[3],
        txrx[4],
        txrx[5]
    );
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
    readSensor();

    delay(100);
}