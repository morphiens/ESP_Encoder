#include <SPI.h>

// ============================================================
// XIAO ESP32-C3 PINOUT
// No strapping pins
// ============================================================
#define CS_PIN   D2    // GPIO4
#define MOSI_PIN D4    // GPIO6  -> TLE DATA via resistor
#define MISO_PIN D10   // GPIO10 -> TLE DATA via resistor
#define SCK_PIN  D5    // GPIO7

// ============================================================
// TLE5012B READ COMMANDS
// ============================================================

// AVAL = register 0x02
// RW = 1
// ND = 1 -> one data word + safety word
const uint16_t CMD_READ_ANGLE = 0x8021;

// STAT = register 0x00
// RW = 1
// ND = 1 -> one data word + safety word
const uint16_t CMD_READ_STAT = 0x8001;

SPISettings tleSPI(1000000, MSBFIRST, SPI_MODE1);

// ============================================================
// 2-STAGE FILTER
// ============================================================

#define MEDIAN_WINDOW 10
#define MEAN_WINDOW   10

float rawHistory[MEDIAN_WINDOW] = {0};
int medWriteIdx = 0;
bool medBufferFilled = false;

float medianOutputHistory[MEAN_WINDOW] = {0};
int meanWriteIdx = 0;
float meanRunningSum = 0;
bool meanBufferFilled = false;


// ============================================================
// MEDIAN
// ============================================================

float getMedian(float *arr, int len)
{
    float temp[len];

    for (int i = 0; i < len; i++)
    {
        temp[i] = arr[i];
    }

    for (int i = 1; i < len; i++)
    {
        float key = temp[i];
        int j = i - 1;

        while (j >= 0 && temp[j] > key)
        {
            temp[j + 1] = temp[j];
            j--;
        }

        temp[j + 1] = key;
    }

    return temp[len / 2];
}


// ============================================================
// 2-STAGE MEDIAN + MEAN FILTER
// ============================================================

float processTwoStageFilter(float newSample)
{
    // -----------------------------
    // Stage 1: Median
    // -----------------------------
    rawHistory[medWriteIdx] = newSample;

    medWriteIdx++;

    if (medWriteIdx >= MEDIAN_WINDOW)
    {
        medWriteIdx = 0;
        medBufferFilled = true;
    }

    int activeMedCount =
        medBufferFilled ? MEDIAN_WINDOW : medWriteIdx;

    float medianValue =
        getMedian(rawHistory, activeMedCount);


    // -----------------------------
    // Stage 2: Moving Mean
    // -----------------------------
    if (meanBufferFilled)
    {
        meanRunningSum -=
            medianOutputHistory[meanWriteIdx];
    }

    medianOutputHistory[meanWriteIdx] = medianValue;

    meanRunningSum += medianValue;

    meanWriteIdx++;

    if (meanWriteIdx >= MEAN_WINDOW)
    {
        meanWriteIdx = 0;
        meanBufferFilled = true;
    }

    int activeMeanCount =
        meanBufferFilled ? MEAN_WINDOW : meanWriteIdx;

    return meanRunningSum / (float)activeMeanCount;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        delay(10);
    }

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    pinMode(MOSI_PIN, OUTPUT);
    digitalWrite(MOSI_PIN, HIGH);

    SPI.begin(
        SCK_PIN,
        MISO_PIN,
        MOSI_PIN,
        CS_PIN
    );

    Serial.println(
        "\n--- TLE5012B ESP32-C3 ---"
    );

    Serial.println(
        "--- 10x10 Median-Mean Filter ---"
    );
}


// ============================================================
// TLE5012B REGISTER READ
// ============================================================

uint16_t readRegister(uint16_t command)
{
    uint16_t response = 0;
    uint16_t safety = 0;

    SPI.beginTransaction(tleSPI);

    digitalWrite(CS_PIN, LOW);

    // --------------------------------------------------------
    // MASTER -> SENSOR
    // Send command
    // --------------------------------------------------------
    SPI.transfer16(command);

    // --------------------------------------------------------
    // RELEASE DATA LINE
    //
    // TLE5012B now needs to drive the shared DATA line.
    // MOSI GPIO6 must therefore become Hi-Z.
    // --------------------------------------------------------
    pinMode(MOSI_PIN, INPUT);

    // TLE5012B specifies 130 ns minimum write/read delay.
    // 2 us gives plenty of margin at 1 MHz.
    delayMicroseconds(2);

    // --------------------------------------------------------
    // SENSOR -> MASTER
    // Read data word
    // --------------------------------------------------------
    response = SPI.transfer16(0x0000);

    // Read Safety Word
    safety = SPI.transfer16(0x0000);

    // --------------------------------------------------------
    // End transaction
    // --------------------------------------------------------
    digitalWrite(CS_PIN, HIGH);

    // Restore MOSI
    pinMode(MOSI_PIN, OUTPUT);
    digitalWrite(MOSI_PIN, HIGH);

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

    uint16_t rawValue =
        readRegister(CMD_READ_ANGLE);

    uint16_t angleData =
        rawValue & 0x7FFF;

    float currentAngle =
        ((float)angleData * 360.0f) / 32768.0f;


    // --------------------------------------------------------
    // 2. READ STATUS
    // --------------------------------------------------------

    uint16_t statusReg =
        readRegister(CMD_READ_STAT);

    // S_MAGOL = bit 7
    // S_XYOL  = bit 6

    bool magTooFar =
        (statusReg >> 7) & 0x01;

    bool magTooClose =
        (statusReg >> 6) & 0x01;

    uint8_t fieldStatus =
        (magTooFar || magTooClose) ? 0 : 1;


    // --------------------------------------------------------
    // 3. FILTER
    // --------------------------------------------------------

    float filteredAngle =
        processTwoStageFilter(currentAngle);


    // --------------------------------------------------------
    // 4. RESPOND TO PYTHON '?'
    // --------------------------------------------------------

    if (Serial.available() > 0)
    {
        char trigger = Serial.read();

        if (trigger == '?')
        {
            Serial.print(currentAngle, 3);
            Serial.print(",");

            Serial.print(filteredAngle, 3);
            Serial.print(",");

            Serial.println(fieldStatus);
        }
    }
}