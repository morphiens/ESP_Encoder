#include <SPI.h>

#define CS_PIN   21  // D3 (GPIO21)
#define MOSI_PIN 18  // D10 (GPIO18)
#define MISO_PIN 20  // D9 (GPIO20)
#define SCK_PIN  19  // D8 (GPIO19)

// TLE5012B Read Commands (MSB is 1 for Read)
const uint16_t CMD_READ_ANGLE = 0x8020; // AVAL  (0x02) — angle value
const uint16_t CMD_READ_STAT  = 0x8000; // STAT  (0x00) — S_MAGOL / S_XYOL magnet diagnostics

// TLE5012B supports up to 8MHz, Mode 1 (CPOL=0, CPHA=1)
SPISettings tleSPI(1000000, MSBFIRST, SPI_MODE1); 

// --- SMA Filter Variables ---
const int FILTER_WINDOW = 1000;         
float readings[FILTER_WINDOW] = {0};  
int readIndex = 0;                    
float runningSum = 0;                 
bool bufferFull = false;              

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Initialize SPI with your ESP32-C6 pins
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  Serial.println("\n--- TLE5012B Read with SMA & Field Tracking ---");
}

// Reads a 16-bit register from TLE5012B
uint16_t readRegister(uint16_t command) {
  uint16_t response = 0;
  
  SPI.beginTransaction(tleSPI);
  digitalWrite(CS_PIN, LOW);
  
  SPI.transfer16(command);        // Send read command
  response = SPI.transfer16(0x0000); // Read 16-bit response (via shared data line resistor setup)
  
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  return response;
}

// void loop() {
//   // Read and calculate angles continuously so your SMA filter stays fresh and running!
//   uint16_t rawValue = readRegister(CMD_READ_ANGLE);
//   uint16_t angleData = rawValue & 0x7FFF; 
//   float currentAngle = ((float)angleData * 360.0f) / 32768.0f;

//   uint16_t statusReg = readRegister(CMD_READ_STATUS);
//   uint8_t fieldStatus = (statusReg >> 10) & 0x01; 

//   if (bufferFull) {
//     runningSum -= readings[readIndex]; 
//   }
//   readings[readIndex] = currentAngle;  
//   runningSum += currentAngle;          
//   readIndex++;                         
//   if (readIndex >= FILTER_WINDOW) { readIndex = 0; bufferFull = true; }
//   float smaAngle = bufferFull ? (runningSum / (float)FILTER_WINDOW) : (runningSum / (float)readIndex);

//   // ONLY SEND SERIAL DATA WHEN REQUESTED BY PYTHON
//   if (Serial.available() > 0) {
//     char trigger = Serial.read();
//     if (trigger == '?') {
//       // Print a single snapshot frame
//       Serial.print(currentAngle, 3);
//       Serial.print(",");
//       Serial.print(smaAngle, 3);
//       Serial.print(",");
//       Serial.println(fieldStatus);
//     }
//   }
// }

void loop() {
  // 1. Read Angle
  uint16_t rawValue = readRegister(CMD_READ_ANGLE);
  
  // TLE5012B angle is 15-bit (bit 0 to 14). Bit 15 is the value-valid flag.
  uint16_t angleData = rawValue & 0x7FFF; 
  float currentAngle = ((float)angleData * 360.0f) / 32768.0f;

  uint16_t statusReg = readRegister(CMD_READ_STATUS);
  uint8_t fieldStatus = (statusReg >> 10) & 0x01; 

  // 3. Apply Simple Moving Average (SMA) via Circular Buffer
  if (bufferFull) {
    runningSum -= readings[readIndex]; 
  }
  
  readings[readIndex] = currentAngle;  
  runningSum += currentAngle;          
  readIndex++;                         
  
  if (readIndex >= FILTER_WINDOW) {
    readIndex = 0;
    bufferFull = true;                 
  }

  float smaAngle = bufferFull ? (runningSum / (float)FILTER_WINDOW) : (runningSum / (float)readIndex);

  // ONLY SEND SERIAL DATA WHEN REQUESTED BY PYTHON
  if (Serial.available() > 0) {
    char trigger = Serial.read();
    if (trigger == '?') {
      // Print a single snapshot frame
      Serial.print(currentAngle, 3);
      Serial.print(",");
      Serial.print(smaAngle, 3);
      Serial.print(",");
      Serial.println(fieldStatus);
    }
  }
}

// void loop() {
//   // 1. Read Angle
//   uint16_t rawValue = readRegister(CMD_READ_ANGLE);
  
//   // TLE5012B angle is 15-bit (bit 0 to 14). Bit 15 is the value-valid flag.
//   uint16_t angleData = rawValue & 0x7FFF; 
//   float currentAngle = ((float)angleData * 360.0f) / 32768.0f;

//   // 2. Read Status/AGC equivalent
//   uint16_t statusReg = readRegister(CMD_READ_STATUS);
//   // Bit 10 of MOD_STAT is AVL (Amplitude Vector Limit error) 
//   // 0 = Error (Field too weak or too strong), 1 = OK/Normal
//   uint8_t fieldStatus = (statusReg >> 10) & 0x01; 

//   // 3. Apply Simple Moving Average (SMA) via Circular Buffer
//   if (bufferFull) {
//     runningSum -= readings[readIndex]; 
//   }
  
//   readings[readIndex] = currentAngle;  
//   runningSum += currentAngle;          
//   readIndex++;                         
  
//   if (readIndex >= FILTER_WINDOW) {
//     readIndex = 0;
//     bufferFull = true;                 
//   }

//   float smaAngle = bufferFull ? (runningSum / (float)FILTER_WINDOW) : (runningSum / (float)readIndex);

//   // 4. Print values cleanly separated by commas for Python
//   // Format: RawAngle,FilteredAngle,FieldStatus (1 = Magnet OK, 0 = Magnet Error/Out of bounds)
//   Serial.print(currentAngle, 3);
//   Serial.print(",");
//   Serial.print(smaAngle, 3);
//   Serial.print(",");
//   Serial.println(fieldStatus);

//   // Small delay to prevent serial buffer overflow if running Python script
//   delay(1); 
// }