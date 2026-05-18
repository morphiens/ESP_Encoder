/**
 * ============================================================
 *  AEAT-8800-Q24  —  16-bit Absolute Magnetic Rotary Encoder
 *  Board  : Seeed Studio XIAO ESP32-C6
 *
 *  ── DUAL PROTOCOL ────────────────────────────────────────────
 *  • SSI  (SSI_SPI_SEL = HIGH) — reads real-time absolute position
 *  • SPI  (SSI_SPI_SEL = LOW)  — reads / writes configuration registers
 *    NOTE: Position data is ONLY available via SSI. The SPI interface
 *    is exclusively for OTP / configuration registers (no angle register).
 *
 *  ── PIN MAPPING ──────────────────────────────────────────────
 *  Encoder Pin │ Pin Name      │ GPIO  │ Role
 *  ────────────┼───────────────┼───────┼──────────────────────────────────
 *  Pin 21      │ SSI_SPI_SEL   │ GPIO21│ HIGH = SSI mode  /  LOW = SPI mode
 *  Pin 10      │ SCL / CLK     │ GPIO19│ Shared clock  (SCK)
 *  Pin 11      │ NSL / DIN     │ GPIO18│ SSI enable (NSL) / SPI data-in (MOSI)
 *  Pin 12      │ DO / DOUT     │ GPIO20│ Data output from encoder (MISO)
 *  ────────────────────────────────────────────────────────────
 *
 *  ── SSI READ SEQUENCE (16-bit mode — 20 clocks total) ────────
 *  CLK idles HIGH, NSL idles HIGH, SSI_SPI_SEL HIGH.
 *  1. Pull NSL LOW  → encoder latches current angle into shift-reg
 *  2. Wait ≥ 300 ns (tREQ)
 *  3. Toggle CLK (HIGH→LOW→HIGH) × 20  — read DO on falling edge:
 *       Bits [19:4] = 16-bit absolute position D[15]..D[0]  (MSB first)
 *       Bit  [3]   = Ready  (1 = encoder output valid)
 *       Bit  [2]   = MHi   (magnet too close)
 *       Bit  [1]   = MLo   (magnet too far)
 *       Bit  [0]   = Parity (even parity over all 20 bits)
 *  4. Pull NSL HIGH → end of frame
 *  5. Wait ≥ 200 ns (tNSLH) before next read
 *
 *  ── SPI PROTOCOL ─────────────────────────────────────────────
 *  SPI_MODE3  (CPOL=1, CPHA=1), ≤ 1 MHz
 *  Read  cmd : 0b10_AAAAAA  (8-bit) → dummy byte → 8-bit reply
 *  Write cmd : 0b01_AAAAAA  (8-bit) → data byte
 *
 *  ── REGISTER MAP (corrected from datasheet) ──────────────────
 *  Addr  Register         Notes
 *  0x00  CustReserve0     User programmable
 *  0x01  CustReserve1     User programmable
 *  0x02  ZeroPos_L        Zero Reset Position [7:0]
 *  0x03  ZeroPos_H        Zero Reset Position [15:8]
 *  0x04  CustConfig0      UVW/PWM/I-width/UVW-pole config
 *  0x05  CPR_Set1         CPR setting [7:4] / Hysteresis [3:0]
 *  0x06  CPR_Set2         bit[7]=Dir, bit[6]=ZeroLatency,
 *                         bits[5:4]=AbsResolution (00=16-bit, DEFAULT),
 *                         bits[3:0]=CPR_Set2
 *  0x10  Lock             Write 0xAB to unlock shadow regs
 *  0x11  ProgCustRsv      Write 0xA1 → OTP regs 0x00-0x01
 *  0x12  ProgZero         Write 0xA2 → OTP regs 0x02-0x03
 *  0x13  ProgConfig       Write 0xA3 → OTP regs 0x04-0x06
 *
 *  ── SERIAL COMMANDS ──────────────────────────────────────────
 *  p / P  — Read position once (single fast SSI read)
 *  m / M  — Precision measurement (4096 SSI samples, filtered)
 *  c / C  — Toggle continuous precision stream (~3-4 readings/sec)
 *  s / S  — Dump all SPI config registers
 *  r / R  — Read register:   r <hex-addr>          e.g. "r 06"
 *  w / W  — Write register:  w <hex-addr> <hex-val> e.g. "w 10 AB"
 *  u / U  — Unlock config registers  (write 0xAB → Lock reg)
 *  z / Z  — Set hardware zero (writes 0x0000 → ZeroPos shadow)
 *  b / B  — Burn shadow regs to OTP  ⚠ ONE-TIME, IRREVERSIBLE!
 *  h / H  — Print this help
 *
 *  Author : Swaraj Dangare
 * ============================================================
 */

 #include <SPI.h>
 #include <algorithm>
 #include "soc/gpio_reg.h"    // GPIO_OUT_W1TS_REG, GPIO_OUT_W1TC_REG, GPIO_IN_REG
 
 // ─── Pin Definitions ──────────────────────────────────────────────────────────
 #define PIN_SEL   D7    // SSI_SPI_SEL : HIGH = SSI, LOW = SPI   (GPIO21)
 #define PIN_CLK   D10   // SCL / CLK   : shared clock             (GPIO19)
 #define PIN_NSL   D8    // NSL / DIN   : SSI enable / SPI MOSI   (GPIO18)
 #define PIN_DO    D9    // DO  / DOUT  : data from encoder        (GPIO20)
 
 // ─── Fast GPIO Bit Masks ──────────────────────────────────────────────────────
 // These GPIO numbers must match the GPIO column in the pin table above.
 // Using direct register writes avoids digitalWrite() function call overhead
 // in the inner SSI bit-bang loop.
 #define GPIO_CLK_BIT   (1UL << 19)   // D10 = GPIO19
 #define GPIO_NSL_BIT   (1UL << 18)   // D8  = GPIO18
 #define GPIO_DO_BIT    (1UL << 20)   // D9  = GPIO20
 
 #define FAST_CLK_HIGH()   REG_WRITE(GPIO_OUT_W1TS_REG, GPIO_CLK_BIT)
 #define FAST_CLK_LOW()    REG_WRITE(GPIO_OUT_W1TC_REG, GPIO_CLK_BIT)
 #define FAST_NSL_HIGH()   REG_WRITE(GPIO_OUT_W1TS_REG, GPIO_NSL_BIT)
 #define FAST_NSL_LOW()    REG_WRITE(GPIO_OUT_W1TC_REG, GPIO_NSL_BIT)
 #define FAST_READ_DO()    ((REG_READ(GPIO_IN_REG) >> 20) & 1u)
 
 // ─── SSI Settings ─────────────────────────────────────────────────────────────
 // Half-period of 1 us → ~500 kHz SSI clock. Datasheet max is 10 MHz.
 // 20 bits for 16-bit mode: 16 position + Ready + MHi + MLo + Parity.
 #define SSI_HALF_PERIOD_US   5
 #define SSI_TOTAL_BITS       20
 
 // ─── SPI Settings ─────────────────────────────────────────────────────────────
 // SPI_MODE3: CPOL=1 (idle HIGH), CPHA=1 (sample on rising edge)
 SPISettings spiCfg(500000UL, MSBFIRST, SPI_MODE3);
 
 // ─── Register Addresses ───────────────────────────────────────────────────────
 #define REG_CUST_RESERVE_0   0x00
 #define REG_CUST_RESERVE_1   0x01
 #define REG_ZERO_POS_L       0x02
 #define REG_ZERO_POS_H       0x03
 #define REG_CUST_CONFIG_0    0x04
 #define REG_CPR_SET1         0x05
 #define REG_CPR_SET2         0x06   // bits[5:4] = absolute resolution
 #define REG_LOCK             0x10
 #define REG_PROG_CUST_RSV    0x11
 #define REG_PROG_ZERO        0x12
 #define REG_PROG_CONFIG      0x13
 
 // ─── Command / Key Bytes ──────────────────────────────────────────────────────
 #define CMD_READ    0x80   // 0b10xxxxxx
 #define CMD_WRITE   0x40   // 0b01xxxxxx
 #define UNLOCK_KEY  0xAB
 
 // ─── Absolute Resolution (REG_CPR_SET2 bits [5:4]) ───────────────────────────
 // 00 = 16-bit (65536 cpr) — factory default; what we want
 // 01 = 14-bit (16384 cpr)
 // 10 = 12-bit  (4096 cpr)
 // 11 = 10-bit  (1024 cpr)
 #define RES_SHIFT   4
 #define RES_MASK    0x03
 
 // ─── Filtering Constants ──────────────────────────────────────────────────────
 #define NUM_BLOCKS        16
 #define SAMPLES_PER_BLOCK 256
 // TOTAL_SAMPLES = 4096
 
 // ─── SSI Frame ───────────────────────────────────────────────────────────────
 struct SSIFrame {
   uint16_t position;   // absolute angle 0–65535 (16-bit)
   bool     ready;      // 1 = encoder output is valid
   bool     mhi;        // magnet too close
   bool     mlo;        // magnet too far
   bool     parityOk;   // even parity check passed
 };
 
 // ─── State ────────────────────────────────────────────────────────────────────
 static bool g_streaming = false;
 
 // ─── Mode Switching ───────────────────────────────────────────────────────────
 
 static void enterSSI() {
   SPI.endTransaction();
   pinMode(PIN_CLK, OUTPUT);
   pinMode(PIN_NSL, OUTPUT);
   pinMode(PIN_DO,  INPUT);
   digitalWrite(PIN_CLK, HIGH);
   digitalWrite(PIN_NSL, HIGH);
   digitalWrite(PIN_SEL, HIGH);
   delayMicroseconds(2);
 }
 
 static void enterSPI() {
   digitalWrite(PIN_SEL, LOW);
   delayMicroseconds(2);
   SPI.beginTransaction(spiCfg);
 }
 
 static void exitSPI() {
   SPI.endTransaction();
   digitalWrite(PIN_SEL, HIGH);
   delayMicroseconds(2);
 }
 
 // ─── SSI Raw Read ─────────────────────────────────────────────────────────────
 // Reads 20 bits via fast bit-banged SSI using direct GPIO register access.
 // Assumes SSI mode is already active (enterSSI() called before this).
 // Data is read on the falling edge of CLK per datasheet recommendation.
 // Returns raw 20-bit frame:
 //   bits [19:4] = position D[15:0]
 //   bit  [3]    = Ready
 //   bit  [2]    = MHi
 //   bit  [1]    = MLo
 //   bit  [0]    = Parity
 
 static uint32_t ssiReadRaw() {
   uint32_t raw = 0;
 
   FAST_NSL_LOW();
   delayMicroseconds(SSI_HALF_PERIOD_US);    // tREQ >= 300 ns
 
   for (int i = 0; i < SSI_TOTAL_BITS; i++) {
     FAST_CLK_LOW();
     delayMicroseconds(SSI_HALF_PERIOD_US);  // data valid on falling edge
     raw = (raw << 1) | FAST_READ_DO();
     FAST_CLK_HIGH();
     delayMicroseconds(SSI_HALF_PERIOD_US);
   }
 
   FAST_NSL_HIGH();
   delayMicroseconds(SSI_HALF_PERIOD_US);    // tNSLH >= 200 ns
   return raw;
 }
 
 // ─── Parity & Frame Parsing ───────────────────────────────────────────────────
 
 // Even parity: XOR of all 20 bits must be 0.
 static inline bool checkParity20(uint32_t raw) {
   return __builtin_parity(raw & 0xFFFFF) == 0;
 }
 
 static SSIFrame parseSSIFrame(uint32_t raw) {
   SSIFrame f;
   f.position = (uint16_t)((raw >> 4) & 0xFFFF);
   f.ready    = (raw >> 3) & 1u;
   f.mhi      = (raw >> 2) & 1u;
   f.mlo      = (raw >> 1) & 1u;
   f.parityOk = checkParity20(raw);
   return f;
 }
 
 // ─── Single-Shot SSI Read ─────────────────────────────────────────────────────
 
 SSIFrame ssiReadPosition() {
   enterSSI();
   return parseSSIFrame(ssiReadRaw());
 }
 
 // ─── Filtering ────────────────────────────────────────────────────────────────
 
 // Robust mean: average of samples within 1.5 LSB of the initial mean.
 // Outliers (noise spikes > 1.5 counts away) are discarded before re-averaging.
 static double getRobustMean(const uint16_t* samples, int size) {
   double sum = 0;
   for (int i = 0; i < size; i++) sum += samples[i];
   double initMean = sum / size;
 
   double rSum  = 0;
   int    count = 0;
   for (int i = 0; i < size; i++) {
     if (abs((double)samples[i] - initMean) < 1.5) {
       rSum += samples[i];
       count++;
     }
   }
   return (count > 0) ? rSum / count : initMean;
 }
 
 static double getMedian(double* values, int size) {
   std::sort(values, values + size);
   return values[size / 2];
 }
 
 // 4096-sample precision read:
 //   16 blocks x 256 samples -> robust mean per block -> median of block means.
 // Any sample that fails parity or has Ready=0 is silently discarded and
 // retried so every block always accumulates exactly 256 valid counts.
 // Returns angle in degrees (0.0 to 360.0).
 double getUltraPrecisionReading() {
   double   blockMeans[NUM_BLOCKS];
   uint16_t blockSamples[SAMPLES_PER_BLOCK];
 
   enterSSI();
 
   for (int b = 0; b < NUM_BLOCKS; b++) {
     for (int s = 0; s < SAMPLES_PER_BLOCK; ) {
       uint32_t raw = ssiReadRaw();
       if (!checkParity20(raw) || !((raw >> 3) & 1u)) {
         continue;   // bad sample — retry this slot
       }
       blockSamples[s] = (uint16_t)((raw >> 4) & 0xFFFF);
       s++;
     }
     blockMeans[b] = getRobustMean(blockSamples, SAMPLES_PER_BLOCK);
   }
 
   return (getMedian(blockMeans, NUM_BLOCKS) * 360.0) / 65536.0;
 }
 
 // ─── SPI Register Helpers ─────────────────────────────────────────────────────
 
 uint8_t spiReadReg(uint8_t addr) {
   enterSPI();
   SPI.transfer(CMD_READ | (addr & 0x3F));
   uint8_t val = SPI.transfer(0xFF);
   exitSPI();
   delayMicroseconds(5);
   return val;
 }
 
 void spiWriteReg(uint8_t addr, uint8_t data) {
   enterSPI();
   SPI.transfer(CMD_WRITE | (addr & 0x3F));
   SPI.transfer(data);
   exitSPI();
   delayMicroseconds(5);
 }
 
 void unlockRegisters() {
   spiWriteReg(REG_LOCK, UNLOCK_KEY);
   delayMicroseconds(10);
 }
 
 uint16_t readHardwareZero() {
   uint8_t lo = spiReadReg(REG_ZERO_POS_L);
   uint8_t hi = spiReadReg(REG_ZERO_POS_H);
   return ((uint16_t)hi << 8) | lo;
 }
 
 void writeHardwareZero(uint16_t zp) {
   unlockRegisters();
   spiWriteReg(REG_ZERO_POS_L,  zp       & 0xFF);
   spiWriteReg(REG_ZERO_POS_H, (zp >> 8) & 0xFF);
 }
 
 // ─── Resolution Helpers (REG_CPR_SET2 0x06 bits [5:4]) ───────────────────────
 
 static uint8_t getResolutionField() {
   return (spiReadReg(REG_CPR_SET2) >> RES_SHIFT) & RES_MASK;
 }
 
 static const char* resolutionStr(uint8_t field) {
   switch (field & RES_MASK) {
     case 0x00: return "16-bit (65536 cpr)";
     case 0x01: return "14-bit (16384 cpr)";
     case 0x02: return "12-bit  (4096 cpr)";
     case 0x03: return "10-bit  (1024 cpr)";
     default:   return "unknown";
   }
 }
 
 // ─── Helpers ──────────────────────────────────────────────────────────────────
 
 static const char* magnetStatus(bool mhi, bool mlo) {
   if (mhi) return "TOO CLOSE";
   if (mlo) return "TOO FAR";
   return "OK";
 }
 
 // ─── Serial Output Helpers ────────────────────────────────────────────────────
 
 void printPosition(const SSIFrame& f) {
   double deg = (f.position / 65536.0) * 360.0;
   Serial.printf("[SSI]  Pos=%5u  Angle=%8.4f deg  Magnet=%-9s  Ready=%u  Parity=%s\n",
                 f.position, deg,
                 magnetStatus(f.mhi, f.mlo),
                 (uint8_t)f.ready,
                 f.parityOk ? "OK" : "FAIL");
 }
 
 void printAllRegisters() {
   Serial.println(F("\n╔══════════════════════════════════════════════╗"));
   Serial.println(F("║    AEAT-8800-Q24 — SPI Register Dump         ║"));
   Serial.println(F("╠══════════════════════════════════════════════╣"));
 
   struct { uint8_t addr; const char* name; } regs[] = {
     { REG_CUST_RESERVE_0, "CustReserve0 (0x00)" },
     { REG_CUST_RESERVE_1, "CustReserve1 (0x01)" },
     { REG_ZERO_POS_L,     "ZeroPos_L    (0x02)" },
     { REG_ZERO_POS_H,     "ZeroPos_H    (0x03)" },
     { REG_CUST_CONFIG_0,  "CustConfig0  (0x04)" },
     { REG_CPR_SET1,       "CPR_Set1     (0x05)" },
     { REG_CPR_SET2,       "CPR_Set2     (0x06)" },
     { REG_LOCK,           "Lock         (0x10)" },
   };
 
   for (auto& r : regs) {
     uint8_t val = spiReadReg(r.addr);
     Serial.printf("║  %-22s  0x%02X  (%3u)\n", r.name, val, val);
   }
 
   uint16_t zp  = readHardwareZero();
   uint8_t  res = getResolutionField();
   Serial.println(F("╠══════════════════════════════════════════════╣"));
   Serial.printf("║  HW Zero Pos : %5u  (%.4f deg)\n", zp, (zp / 65536.0) * 360.0);
   Serial.printf("║  Resolution  : %s\n", resolutionStr(res));
   Serial.printf("║  [0x06][5:4] : 0b%u%u  (raw field)\n",
                 (res >> 1) & 1u, res & 1u);
   Serial.println(F("╚══════════════════════════════════════════════╝\n"));
 }
 
 void printHelp() {
   Serial.println(F("\n╔══════════════════════════════════════════════════════╗"));
   Serial.println(F("║         AEAT-8800-Q24  Serial Command Reference      ║"));
   Serial.println(F("╠══════════════════════════════════════════════════════╣"));
   Serial.println(F("║  p       — Read position once (single SSI read)      ║"));
   Serial.println(F("║  m       — Precision reading  (4096 SSI samples)     ║"));
   Serial.println(F("║  c       — Toggle continuous precision stream         ║"));
   Serial.println(F("║  s       — Dump all SPI config registers              ║"));
   Serial.println(F("║  r <hh>  — Read register at hex addr  (e.g. r 06)    ║"));
   Serial.println(F("║  w <hh> <vv> — Write val to reg  (e.g. w 10 AB)     ║"));
   Serial.println(F("║  u       — Unlock config registers  (0xAB -> Lock)   ║"));
   Serial.println(F("║  z       — Set zero position = 0x0000 (shadow only)  ║"));
   Serial.println(F("║  burn    — Burn shadow regs to OTP  WARNING:         ║"));
   Serial.println(F("║            Type exactly 'burn' + Enter               ║"));
   Serial.println(F("║  h       — Show this help                             ║"));
   Serial.println(F("╚══════════════════════════════════════════════════════╝\n"));
 }
 
 // ─── Command Parser ───────────────────────────────────────────────────────────
 
 void handleSerial() {
   if (!Serial.available()) return;
 
   String line = Serial.readStringUntil('\n');
   line.trim();
   if (line.length() == 0) return;
 
   char cmd = (char)tolower((unsigned char)line[0]);
 
   switch (cmd) {
 
     // ── Single fast SSI read ───────────────────────────────────────────────
     case 'p': {
       SSIFrame f = ssiReadPosition();
       printPosition(f);
       break;
     }
 
     // ── Precision measurement (4096 SSI samples, filtered) ────────────────
     case 'm': {
       Serial.println(F("[PREC] Measuring (4096 SSI samples)..."));
       uint32_t t0  = millis();
       double   deg = getUltraPrecisionReading();
       Serial.printf("[PREC] Angle=%10.5f deg  (%u ms)\n", deg, millis() - t0);
       break;
     }
 
     // ── Continuous precision stream toggle ────────────────────────────────
     case 'c':
       g_streaming = !g_streaming;
       Serial.printf("[INFO] Continuous precision stream %s\n",
                     g_streaming ? "ON (each sample = 4096 reads)" : "OFF");
       break;
 
     // ── Register dump ─────────────────────────────────────────────────────
     case 's':
       printAllRegisters();
       break;
 
     // ── Read register  "r <hex>" ──────────────────────────────────────────
     case 'r': {
       if (line.length() < 3) {
         Serial.println(F("[ERR] Usage: r <hex-addr>  e.g. 'r 06'"));
         break;
       }
       uint8_t addr = (uint8_t)strtoul(line.c_str() + 2, nullptr, 16);
       uint8_t val  = spiReadReg(addr);
       Serial.printf("[SPI-RD] Reg 0x%02X = 0x%02X  (%u)\n", addr, val, val);
       break;
     }
 
     // ── Write register  "w <hex> <hex>" ───────────────────────────────────
     case 'w': {
       if (line.length() < 5) {
         Serial.println(F("[ERR] Usage: w <hex-addr> <hex-val>  e.g. 'w 10 AB'"));
         break;
       }
       char*   ptr  = nullptr;
       uint8_t addr = (uint8_t)strtoul(line.c_str() + 2, &ptr,    16);
       uint8_t val  = (uint8_t)strtoul(ptr,               nullptr, 16);
       spiWriteReg(addr, val);
       uint8_t rb = spiReadReg(addr);
       Serial.printf("[SPI-WR] Reg 0x%02X <- 0x%02X  |  Readback: 0x%02X  %s\n",
                     addr, val, rb, (rb == val) ? "OK" : "MISMATCH!");
       break;
     }
 
     // ── Unlock registers ──────────────────────────────────────────────────
     case 'u':
       unlockRegisters();
       Serial.println(F("[SPI] Registers unlocked (0xAB written to Lock reg 0x10)"));
       break;
 
     // ── Set zero position ─────────────────────────────────────────────────
     case 'z': {
       Serial.println(F("[ZERO] Writing 0x0000 to ZeroPos shadow registers..."));
       writeHardwareZero(0x0000);
       uint16_t rb = readHardwareZero();
       Serial.printf("[ZERO] Readback: 0x%04X — %s\n", rb,
                     rb == 0x0000 ? "OK" : "MISMATCH!");
       break;
     }
 
     // ── OTP burn  — requires typing "burn" in full ─────────────────────────
     case 'b': {
       if (!line.equalsIgnoreCase("burn")) {
         Serial.println(F("[WARN] OTP BURN is IRREVERSIBLE!"));
         Serial.println(F("[WARN] Type exactly 'burn' + Enter to confirm."));
         break;
       }
       Serial.println(F("[OTP] Unlocking registers..."));
       unlockRegisters();
       Serial.println(F("[OTP] Burning Customer Config OTP (regs 0x04-0x06)..."));
       spiWriteReg(REG_PROG_CONFIG, 0xA3);
       delayMicroseconds(200);
       Serial.println(F("[OTP] Burning Zero Position OTP (regs 0x02-0x03)..."));
       spiWriteReg(REG_PROG_ZERO, 0xA2);
       delayMicroseconds(200);
       Serial.println(F("[OTP] Done. Power-cycle the encoder to verify."));
       break;
     }
 
     // ── Help ──────────────────────────────────────────────────────────────
     case 'h':
     default:
       printHelp();
       break;
   }
 }
 
 // ─── Setup ────────────────────────────────────────────────────────────────────
 
 void setup() {
   Serial.begin(115200);
   delay(300);
 
   // GPIO init — idle state: CLK HIGH, NSL HIGH, SEL HIGH (SSI mode)
   pinMode(PIN_SEL, OUTPUT);
   pinMode(PIN_CLK, OUTPUT);
   pinMode(PIN_NSL, OUTPUT);
   pinMode(PIN_DO,  INPUT);
   digitalWrite(PIN_SEL, HIGH);
   digitalWrite(PIN_CLK, HIGH);
   digitalWrite(PIN_NSL, HIGH);
 
   // Hardware SPI init (SCK=PIN_CLK, MISO=PIN_DO, MOSI=PIN_NSL, no CS)
   SPI.begin(PIN_CLK, PIN_DO, PIN_NSL, -1);
 
   delay(50);   // t_POR encoder power-on stabilisation
 
   // ── Boot banner ──────────────────────────────────────────────────────────
   Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
   Serial.println(F("║  AEAT-8800-Q24  |  SSI + SPI  |  XIAO ESP32-C6   ║"));
   Serial.println(F("╠════════════════════════════════════════════════════╣"));
   Serial.println(F("║  SEL=GPIO21  CLK=GPIO19  NSL=GPIO18  DO=GPIO20    ║"));
   Serial.println(F("╠════════════════════════════════════════════════════╣"));
 
   // Boot SSI read — single fast sample for startup status
   SSIFrame f    = ssiReadPosition();
   double bootDeg = (f.position / 65536.0) * 360.0;
   Serial.printf("║  SSI Position  : %5u counts  (%.4f deg)\n",
                 f.position, bootDeg);
   Serial.printf("║  Magnet Status : %s\n", magnetStatus(f.mhi, f.mlo));
   Serial.printf("║  Encoder Ready : %s\n",  f.ready    ? "YES"  : "NO");
   Serial.printf("║  Frame Parity  : %s\n",  f.parityOk ? "OK"   : "FAIL");
   Serial.println(F("╠════════════════════════════════════════════════════╣"));
 
   // ── Auto-verify 16-bit resolution (REG_CPR_SET2 bits [5:4]) ─────────────
   uint8_t reg06    = spiReadReg(REG_CPR_SET2);
   uint8_t resField = (reg06 >> RES_SHIFT) & RES_MASK;
   Serial.printf("║  Resolution    : %s\n", resolutionStr(resField));
 
   if (resField != 0x00) {
     Serial.println(F("╠════════════════════════════════════════════════════╣"));
     Serial.println(F("║  [WARN] Not 16-bit! Correcting shadow register...  ║"));
     unlockRegisters();
     spiWriteReg(REG_CPR_SET2, reg06 & (uint8_t)~(RES_MASK << RES_SHIFT));
     uint8_t verify = (spiReadReg(REG_CPR_SET2) >> RES_SHIFT) & RES_MASK;
     Serial.printf("║  Resolution now: %s\n", resolutionStr(verify));
     Serial.println(F("║  Shadow updated. Type 'burn' to make permanent.    ║"));
   } else {
     Serial.println(F("║  16-bit confirmed (factory default).               ║"));
   }
 
   uint16_t zp = readHardwareZero();
   Serial.printf("║  HW Zero Pos   : %u counts  (%.4f deg)\n",
                 zp, (zp / 65536.0) * 360.0);
   Serial.println(F("╠════════════════════════════════════════════════════╣"));
   Serial.println(F("║  Type 'h' for command help                         ║"));
   Serial.println(F("╚════════════════════════════════════════════════════╝\n"));
 }
 
 // ─── Main Loop ────────────────────────────────────────────────────────────────
 
 void loop() {
   handleSerial();
 
   // Precision stream: the ~250 ms measurement time is the natural rate limiter.
   if (g_streaming) {
     uint32_t t0  = millis();
     double   deg = getUltraPrecisionReading();
     Serial.printf("[STREAM] Angle=%10.5f deg  (%u ms)\n", deg, millis() - t0);
   }
 }
 