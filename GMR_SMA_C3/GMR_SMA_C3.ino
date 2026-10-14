#include <SPI.h>

// ============================================================
// XIAO ESP32-S3
// SAME XIAO PIN LABELS AS YOUR C3 VERSION
// ============================================================
#define CS_PIN   D2    // GPIO3
#define MOSI_PIN D4    // GPIO5
#define MISO_PIN D10   // GPIO9
#define SCK_PIN  D5    // GPIO6

// ============================================================
// TLE5012B Read Commands
// ============================================================
const uint16_t CMD_READ_ANGLE = 0x8020;  // AVAL (0x02)
const uint16_t CMD_READ_STAT  = 0x8000;  // STAT (0x00)

// TLE5012B SSC: Mode 1, 1 MHz
SPISettings tleSPI(1000000, MSBFIRST, SPI_MODE1);

// ============================================================
// SMA Filter
// ============================================================
const int FILTER_WINDOW = 1000;

float readings[FILTER_WINDOW] = {0};

int readIndex = 0;
float runningSum = 0;

bool bufferFull = false;

// ============================================================
// SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);

    while (!Serial)
        delay(10);

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    SPI.begin(
        SCK_PIN,
        MISO_PIN,
        MOSI_PIN,
        CS_PIN
    );

    Serial.println("\n--- TLE5012B Read with SMA & Field Tracking ---");
}

// ============================================================
// READ REGISTER
// ============================================================
uint16_t readRegister(uint16_t command)
{
    uint16_t response = 0;

    SPI.beginTransaction(tleSPI);

    digitalWrite(CS_PIN, LOW);

    // Send command
    SPI.transfer16(command);

    // Read response
    response = SPI.transfer16(0x0000);

    digitalWrite(CS_PIN, HIGH);

    SPI.endTransaction();

    return response;
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
    // --------------------------------------------------------
    // 1. READ ANGLE
    // --------------------------------------------------------
    uint16_t rawValue = readRegister(CMD_READ_ANGLE);

    uint16_t angleData = rawValue & 0x7FFF;

    float currentAngle =
        ((float)angleData * 360.0f) / 32768.0f;

    // --------------------------------------------------------
    // 2. READ STATUS
    // --------------------------------------------------------
    uint16_t statusReg = readRegister(CMD_READ_STAT);

    uint8_t fieldStatus =
        (statusReg >> 10) & 0x01;

    // --------------------------------------------------------
    // 3. SMA FILTER
    // --------------------------------------------------------
    if (bufferFull)
    {
        runningSum -= readings[readIndex];
    }

    readings[readIndex] = currentAngle;

    runningSum += currentAngle;

    readIndex++;

    if (readIndex >= FILTER_WINDOW)
    {
        readIndex = 0;
        bufferFull = true;
    }

    float smaAngle;

    if (bufferFull)
    {
        smaAngle =
            runningSum / (float)FILTER_WINDOW;
    }
    else
    {
        smaAngle =
            runningSum / (float)readIndex;
    }

    // --------------------------------------------------------
    // 4. SEND ONLY WHEN PYTHON SENDS '?'
    // --------------------------------------------------------
    if (Serial.available() > 0)
    {
        char trigger = Serial.read();

        if (trigger == '?')
        {
            Serial.print(currentAngle, 3);
            Serial.print(",");

            Serial.print(smaAngle, 3);
            Serial.print(",");

            Serial.println(fieldStatus);
        }
    }
}