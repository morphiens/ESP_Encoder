/**
 * ============================================================
 *  AEAT-8800-Q24  —  16-bit Absolute Magnetic Rotary Encoder
 *  Board  : Seeed Studio XIAO ESP32-C6 hi
 *
 *  ── DUAL PROTOCOL ────────────────────────────────────────────
 *  • SSI  (SSI_SPI_SEL = HIGH) — reads real-time absolute position
 *  • SPI  (SSI_SPI_SEL = LOW)  — reads / writes configuration registers
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
 *  ── SSI READ SEQUENCE ────────────────────────────────────────
 *  CLK idles HIGH, NSL idles HIGH.
 *  1. Pull NSL LOW  → encoder latches current angle into shift-reg
 *  2. Toggle CLK (HIGH→LOW→HIGH) × 18:
 *       Bits [17:2] = 16-bit absolute position (MSB first)
 *       Bit  [1]   = MHi  (magnet too close)
 *       Bit  [0]   = MLo  (magnet too far)
 *     Data is valid on the rising edge of each CLK cycle.
 *  3. Pull NSL HIGH → end of frame
 *  4. Wait ≥ monoflop time before next read (~20 µs typical)
 *
 *  ── SPI PROTOCOL ─────────────────────────────────────────────
 *  SPI_MODE3  (CPOL=1, CPHA=1), ≤ 1 MHz
 *  Read  cmd : 0b10_AAAAAA  (8-bit) → dummy byte → 8-bit reply
 *  Write cmd : 0b01_AAAAAA  (8-bit) → data byte
 *
 *  ── REGISTER MAP ─────────────────────────────────────────────
 *  0x00 CustReserve0   0x04 CustConfig0
 *  0x01 CustReserve1   0x05 CPR_Set1
 *  0x02 ZeroPos_L      0x06 CPR_Set2
 *  0x03 ZeroPos_H      0x07 Resolution (bits[1:0]: 00=10b…11=16b)
 *  0x10 Lock           0x11 ProgCust
 *
 *  ── SERIAL COMMANDS ──────────────────────────────────────────
 *  p / P  — Print current absolute position (SSI, one-shot)
 *  c / C  — Continuous SSI position stream (toggle on/off)
 *  s / S  — Dump all SPI config registers
 *  r / R  — Read single register:  r <hex-addr>   e.g. "r 07"
 *  w / W  — Write register:        w <hex-addr> <hex-val>  e.g. "w 10 AB"
 *  u / U  — Unlock config registers  (write 0xAB → REG_LOCK)
 *  z / Z  — Set hardware zero (writes 0x0000 → ZeroPos shadow regs)
 *  b / B  — Burn shadow regs to OTP  ⚠ ONE-TIME, IRREVERSIBLE!
 *  h / H  — Print this help
 *
 *  Author : Swaraj Dangare
 * ============================================================
 */

#include <SPI.h>

// ─── Pin Definitions ──────────────────────────────────────────────────────────
// GPIO numbers match the user's pin table exactly.
#define PIN_SEL   D2   // SSI_SPI_SEL : HIGH = SSI, LOW = SPI
#define PIN_SEL1   D8
#define PIN_CLK   D1   // SCL / CLK   : shared clock
#define PIN_NSL   D10   // NSL / DIN   : SSI enable / SPI MOSI
#define PIN_DO    D0   // DO  / DOUT  : data from encoder / SPI MISO

// ─── SSI Settings ─────────────────────────────────────────────────────────────
// Bit-banged; half-period determines SSI clock speed.
// 5 µs half-period → ~100 kHz SSI clock  (datasheet max ~1 MHz)
#define SSI_HALF_PERIOD_US   5
#define SSI_TOTAL_BITS       18   // 16 position + MHi + MLo

// ─── SPI Settings ─────────────────────────────────────────────────────────────
// SPI_MODE3: CPOL=1 (idle high), CPHA=1 (sample on 2nd / rising edge)
SPISettings spiCfg(500000UL, MSBFIRST, SPI_MODE3);

// ─── Register Addresses ───────────────────────────────────────────────────────
#define REG_CUST_RESERVE_0   0x00
#define REG_CUST_RESERVE_1   0x01
#define REG_ZERO_POS_L       0x02
#define REG_ZERO_POS_H       0x03
#define REG_CUST_CONFIG_0    0x04
#define REG_CPR_SET1         0x05
#define REG_CPR_SET2         0x06
#define REG_RESOLUTION       0x07
#define REG_LOCK             0x10
#define REG_PROG_CUST        0x11

// ─── Command / Key Bytes ──────────────────────────────────────────────────────
#define CMD_READ             0x80   // 0b10xxxxxx
#define CMD_WRITE            0x40   // 0b01xxxxxx
#define UNLOCK_KEY           0xAB   // Unlock config writes
#define PROG_KEY             0xA1   // Burn OTP

// Resolution field (REG_RESOLUTION bits[1:0])
#define RES_10BIT  0x00
#define RES_12BIT  0x01
#define RES_14BIT  0x02
#define RES_16BIT  0x03

// ─── SSI Frame ───────────────────────────────────────────────────────────────
// Declared here (global scope) so Arduino's auto-prototype generator
// can see the type before it emits prototypes for ssiReadPosition() etc.
struct SSIFrame {
  uint16_t position;   // absolute angle, 0–65535  (16-bit)
  bool     mhi;        // magnet too close
  bool     mlo;        // magnet too far
};

// ─── State ────────────────────────────────────────────────────────────────────
static bool     g_streaming   = false;   // continuous SSI stream active
static uint32_t g_lastStream  = 0;       // millis of last stream per print

// ─── Mode Switching ───────────────────────────────────────────────────────────

/** Enter SSI mode: SEL HIGH, let CLK and NSL settle HIGH. */
static void enterSSI() {
  SPI.endTransaction();            // release hardware SPI bus
  // Re-assert GPIO control of the shared pins
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_NSL, OUTPUT);
  pinMode(PIN_DO,  INPUT);
  digitalWrite(PIN_CLK, HIGH);
  digitalWrite(PIN_NSL, HIGH);
  digitalWrite(PIN_SEL, HIGH);
  delayMicroseconds(2);
}

/** Enter SPI mode: SEL LOW, hardware SPI takes over CLK and MOSI. */
static void enterSPI() {
  digitalWrite(PIN_SEL, LOW);
  delayMicroseconds(2);
  SPI.beginTransaction(spiCfg);
}

static void exitSPI() {
  SPI.endTransaction();
  digitalWrite(PIN_SEL, HIGH);    // back to SSI idle
  delayMicroseconds(2);
}

// ─── SSI Read ─────────────────────────────────────────────────────────────────

/** Read 18 bits via bit-banged SSI (16-bit position + MHi + MLo). */

SSIFrame ssiReadPosition() {
  enterSSI();

  uint32_t raw = 0;

  // Pull NSL LOW → encoder latches position into shift register
  digitalWrite(PIN_NSL, LOW);
  delayMicroseconds(SSI_HALF_PERIOD_US);

  for (int i = 0; i < SSI_TOTAL_BITS; i++) {
    // Falling edge — encoder shifts next bit onto DO
    digitalWrite(PIN_CLK, LOW);
    delayMicroseconds(SSI_HALF_PERIOD_US);

    // Rising edge — read the bit
    digitalWrite(PIN_CLK, HIGH);
    raw = (raw << 1) | (digitalRead(PIN_DO) ? 1u : 0u);
    delayMicroseconds(SSI_HALF_PERIOD_US);
  }

  // End of frame: NSL HIGH
  digitalWrite(PIN_NSL, HIGH);
  delayMicroseconds(20);   // monoflop recovery time

  SSIFrame f;
  f.mlo      = (raw >> 0) & 0x01;
  f.mhi      = (raw >> 1) & 0x01;
  f.position = (uint16_t)((raw >> 2) & 0xFFFF);
  return f;
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

// ─── Helpers ─────────────────────────────────────────────────────────────────

const char* resolutionStr(uint8_t regVal) {
  switch (regVal & 0x03) {
    case RES_10BIT: return "10-bit (1024 cpr)";
    case RES_12BIT: return "12-bit (4096 cpr)";
    case RES_14BIT: return "14-bit (16384 cpr)";
    case RES_16BIT: return "16-bit (65536 cpr)";
    default:        return "unknown";
  }
}

const char* magnetStatus(bool mhi, bool mlo) {
  if (mhi) return "TOO CLOSE";
  if (mlo) return "TOO FAR";
  return "OK";
}

// ─── Serial Output Helpers ────────────────────────────────────────────────────

void printPosition(const SSIFrame& f) {
  double deg = (f.position / 65536.0) * 360.0;
  Serial.printf("[SSI] Pos=%5u  Angle=%8.4f deg  Magnet=%s%s%s\n",
                f.position, deg,
                magnetStatus(f.mhi, f.mlo),
                f.mhi ? " (MHi)" : "",
                f.mlo ? " (MLo)" : "");
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
    { REG_RESOLUTION,     "Resolution   (0x07)" },
    { REG_LOCK,           "Lock         (0x10)" },
  };

  for (auto& r : regs) {
    uint8_t val = spiReadReg(r.addr);
    Serial.printf("║  %-22s  0x%02X  (%3u)\n", r.name, val, val);
  }

  uint16_t zp     = readHardwareZero();
  uint8_t  resReg = spiReadReg(REG_RESOLUTION);
  Serial.println(F("╠══════════════════════════════════════════════╣"));
  Serial.printf("║  HW Zero Pos : %5u  (%.4f deg)\n", zp, (zp / 65536.0) * 360.0);
  Serial.printf("║  Resolution  : %s\n", resolutionStr(resReg));
  Serial.println(F("╚══════════════════════════════════════════════╝\n"));
}

void printHelp() {
  Serial.println(F("\n╔══════════════════════════════════════════════════════╗"));
  Serial.println(F("║         AEAT-8800-Q24  Serial Command Reference      ║"));
  Serial.println(F("╠══════════════════════════════════════════════════════╣"));
  Serial.println(F("║  p       — Read position once (SSI)                  ║"));
  Serial.println(F("║  c       — Toggle continuous position stream          ║"));
  Serial.println(F("║  s       — Dump all SPI config registers              ║"));
  Serial.println(F("║  r <hh>  — Read register at hex addr  (e.g. r 07)    ║"));
  Serial.println(F("║  w <hh> <vv> — Write val to reg  (e.g. w 10 AB)     ║"));
  Serial.println(F("║  u       — Unlock config registers  (0xAB→Lock)      ║"));
  Serial.println(F("║  z       — Set zero position = 0x0000 (shadow only)  ║"));
  Serial.println(F("║  b       — Burn shadow regs to OTP  ⚠ IRREVERSIBLE   ║"));
  Serial.println(F("║  h       — Show this help                             ║"));
  Serial.println(F("╚══════════════════════════════════════════════════════╝\n"));
}

// ─── Command Parser ───────────────────────────────────────────────────────────

void handleSerial() {
  if (!Serial.available()) return;

  // Read full line
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = (char)tolower((unsigned char)line[0]);

  switch (cmd) {

    // ── Position read (SSI) ────────────────────────────────────────────────
    case 'p': {
      SSIFrame f = ssiReadPosition();
      printPosition(f);
      break;
    }

    // ── Continuous stream toggle ───────────────────────────────────────────
    case 'c':
      g_streaming = !g_streaming;
      Serial.printf("[INFO] Continuous stream %s\n", g_streaming ? "ON" : "OFF");
      break;

    // ── Register dump (SPI) ───────────────────────────────────────────────
    case 's':
      printAllRegisters();
      break;

    // ── Read single register  "r <hex-addr>" ──────────────────────────────
    case 'r': {
      if (line.length() < 3) {
        Serial.println(F("[ERR] Usage: r <hex-addr>  e.g. 'r 07'"));
        break;
      }
      uint8_t addr = (uint8_t)strtoul(line.c_str() + 2, nullptr, 16);
      uint8_t val  = spiReadReg(addr);
      Serial.printf("[SPI-RD] Reg 0x%02X = 0x%02X  (%u)\n", addr, val, val);
      break;
    }

    // ── Write single register  "w <hex-addr> <hex-val>" ──────────────────
    case 'w': {
      if (line.length() < 5) {
        Serial.println(F("[ERR] Usage: w <hex-addr> <hex-val>  e.g. 'w 10 AB'"));
        break;
      }
      char* ptr  = nullptr;
      uint8_t addr = (uint8_t)strtoul(line.c_str() + 2, &ptr, 16);
      uint8_t val  = (uint8_t)strtoul(ptr,               nullptr, 16);
      spiWriteReg(addr, val);
      uint8_t rb = spiReadReg(addr);
      Serial.printf("[SPI-WR] Reg 0x%02X ← 0x%02X  |  Readback: 0x%02X  %s\n",
                    addr, val, rb, (rb == val) ? "OK" : "MISMATCH!");
      break;
    }

    // ── Unlock registers ──────────────────────────────────────────────────
    case 'u':
      unlockRegisters();
      Serial.println(F("[SPI] Config registers unlocked (0xAB written to Lock reg)"));
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

    // ── Burn OTP  ⚠ IRREVERSIBLE ─────────────────────────────────────────
    case 'b': {
      // Require explicit confirmation by sending "burn" as the line
      if (!line.equalsIgnoreCase("burn")) {
        Serial.println(F("[WARN] ⚠  OTP BURN is IRREVERSIBLE!"));
        Serial.println(F("[WARN]    Type exactly 'burn' and press Enter to confirm."));
        break;
      }
      Serial.println(F("[OTP] Unlocking and burning shadow regs to OTP..."));
      unlockRegisters();
      spiWriteReg(REG_PROG_CUST, PROG_KEY);
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

  // GPIO init — SSI pins, all idle HIGH
  pinMode(PIN_SEL, OUTPUT);
  // pinMode(PIN_SEL1, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_NSL, OUTPUT);
  pinMode(PIN_DO,  INPUT);
  digitalWrite(PIN_SEL, HIGH);   // SSI mode at boot
  // digitalWrite(PIN_SEL1, LOW);
  digitalWrite(PIN_CLK, HIGH);
  digitalWrite(PIN_NSL, HIGH);

  // Hardware SPI init (pins will be re-claimed by SPI.beginTransaction when needed)
  SPI.begin(PIN_CLK, PIN_DO, PIN_NSL, -1);  // SCK, MISO, MOSI, no CS

  delay(50);   // t_POR encoder stabilisation

  // ── Boot banner ──────────────────────────────────────────────────────────
  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║  AEAT-8800-Q24  |  SSI + SPI  |  XIAO ESP32-C6   ║"));
  Serial.println(F("╠════════════════════════════════════════════════════╣"));
  Serial.println(F("║  SSI_SPI_SEL=21  CLK=19  NSL/DIN=18  DO=20       ║"));
  Serial.println(F("╠════════════════════════════════════════════════════╣"));

  // Quick SSI position read
  SSIFrame f = ssiReadPosition();
  double deg  = (f.position / 65536.0) * 360.0;
  Serial.printf("║  SSI Position  : %5u counts  (%.4f deg)\n", f.position, deg);
  Serial.printf("║  Magnet Status : %s\n", magnetStatus(f.mhi, f.mlo));

  // Quick SPI config read
  uint8_t resReg = spiReadReg(REG_RESOLUTION);
  uint16_t zp    = readHardwareZero();
  Serial.printf("║  Resolution    : %s\n", resolutionStr(resReg));
  Serial.printf("║  HW Zero Pos   : %u counts  (%.4f deg)\n",
                zp, (zp / 65536.0) * 360.0);
  Serial.println(F("╠════════════════════════════════════════════════════╣"));
  Serial.println(F("║  Type 'h' for command help                         ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝\n"));
}

// ─── Main Loop ────────────────────────────────────────────────────────────────

void loop() {
  // Handle serial commands (full-line parser)
  handleSerial();

  // Continuous SSI position stream (100 ms interval)
  if (g_streaming && (millis() - g_lastStream >= 100)) {
    g_lastStream = millis();
    SSIFrame f = ssiReadPosition();
    printPosition(f);
  }
}
