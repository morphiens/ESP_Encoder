#include <SPI.h>

#define CS_PIN   21  // D3 (GPIO21)
#define MOSI_PIN 18  // D10 (GPIO18)
#define MISO_PIN 20  // D9 (GPIO20)
#define SCK_PIN  19  // D8 (GPIO19)

// TLE5012B Read Commands
const uint16_t CMD_READ_ANGLE = 0x8020; // AVAL (0x02) — angle value
const uint16_t CMD_READ_STAT  = 0x8000; // STAT (0x00) — S_MAGOL / S_XYOL magnet diagnostics 

SPISettings tleSPI(1000000, MSBFIRST, SPI_MODE1); 

// =========================================================================
// --- 2-Stage Filter Configuration ---
// Change these values to set your custom combinations (e.g., 3x3, 5x5, 5x10)
// =========================================================================
#define MEDIAN_WINDOW 10  // Stage 1: Strips spikes. Keep odd (e.g., 3, 5, 7) for clean medians
#define MEAN_WINDOW   10  // Stage 2: Smooths residual noise. Any positive integer.

// --- Stage 1 (Median Filter) State Variables ---
float rawHistory[MEDIAN_WINDOW] = {0};
int medWriteIdx = 0;
bool medBufferFilled = false;

// --- Stage 2 (Mean Filter) State Variables ---
float medianOutputHistory[MEAN_WINDOW] = {0};
int meanWriteIdx = 0;
float meanRunningSum = 0;
bool meanBufferFilled = false;

// Fast insertion sort to find the median of a small array
float getMedian(float *arr, int len) {
  // Copy to a temp array to preserve historical order in the main buffer
  float temp[len];
  for (int i = 0; i < len; i++) {
    temp[i] = arr[i];
  }

  // Insertion Sort (Highly efficient for small array sizes)
  for (int i = 1; i < len; i++) {
    float key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j = j - 1;
    }
    temp[j + 1] = key;
  }
  
  // Return center value
  return temp[len / 2];
}

float processTwoStageFilter(float newSample) {
  // === STAGE 1: Median Filter ===
  rawHistory[medWriteIdx] = newSample;
  medWriteIdx++;
  if (medWriteIdx >= MEDIAN_WINDOW) {
    medWriteIdx = 0;
    medBufferFilled = true;
  }

  int activeMedCount = medBufferFilled ? MEDIAN_WINDOW : medWriteIdx;
  float medianValue = getMedian(rawHistory, activeMedCount);

  // === STAGE 2: Mean Moving Average Filter ===
  if (meanBufferFilled) {
    meanRunningSum -= medianOutputHistory[meanWriteIdx];
  }
  
  medianOutputHistory[meanWriteIdx] = medianValue;
  meanRunningSum += medianValue;
  meanWriteIdx++;
  
  if (meanWriteIdx >= MEAN_WINDOW) {
    meanWriteIdx = 0;
    meanBufferFilled = true;
  }

  int activeMeanCount = meanBufferFilled ? MEAN_WINDOW : meanWriteIdx;
  return meanRunningSum / (float)activeMeanCount;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  
  Serial.print("\n--- TLE5012B Active with 2-Stage Filter [");
  Serial.print(MEDIAN_WINDOW);
  Serial.print("x");
  Serial.print(MEAN_WINDOW);
  Serial.println(" Median-Mean] ---");
}

uint16_t readRegister(uint16_t command) {
  uint16_t response = 0;
  SPI.beginTransaction(tleSPI);
  digitalWrite(CS_PIN, LOW);
  
  SPI.transfer16(command);        
  response = SPI.transfer16(0x0000); 
  
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return response;
}

void loop() {
  // 1. Read Raw Angle and Field Status from Sensor
  uint16_t rawValue = readRegister(CMD_READ_ANGLE);
  uint16_t angleData = rawValue & 0x7FFF; 
  float currentAngle = ((float)angleData * 360.0f) / 32768.0f;

  // STAT: S_MAGOL (bit7) = field too weak / magnet too far
  //       S_XYOL  (bit6) = XY amp out of limit / magnet too close
  // fieldStatus: 1 = magnet OK, 0 = out of range (matches existing CSV convention)
  uint16_t statusReg = readRegister(CMD_READ_STAT);
  bool magTooFar   = (statusReg >> 7) & 0x01; // S_MAGOL
  bool magTooClose = (statusReg >> 6) & 0x01; // S_XYOL
  uint8_t fieldStatus = (magTooFar || magTooClose) ? 0 : 1;

  // 2. Feed current angle through the 2-Stage Cascade Filter
  float filteredAngle = processTwoStageFilter(currentAngle);

  // 3. Keep listening for the request poll character ('?') from Python
  if (Serial.available() > 0) {
    char trigger = Serial.read();
    if (trigger == '?') {
      // Print clean CSV telemetry data
      Serial.print(currentAngle, 3);
      Serial.print(",");
      Serial.print(filteredAngle, 3);
      Serial.print(",");
      Serial.println(fieldStatus);
    }
  }
}