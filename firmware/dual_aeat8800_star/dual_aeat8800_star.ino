/**
 * ============================================================
 *  AEAT-8800-Q24  —  Dual 16-bit Absolute Magnetic Encoder
 *  Star-Topology Clock Wiring (each encoder has dedicated CLK)
 *  Board  : Seeed Studio XIAO ESP32-C6 
 *
 *  ── WHY STAR TOPOLOGY ────────────────────────────────────────
 *  SSI protocol does not support multiple slaves on one shared
 *  clock line. An unclocked slave holds DOUT in high-impedance
 *  only when its own CLK is idle. Giving each encoder a dedicated
 *  CLK line means the inactive slave is never clocked, so its
 *  DOUT stays tri-state and bus contention is eliminated.
 *
 *  ── PIN MAPPING ──────────────────────────────────────────────
 *  Signal          │ Arduino │ GPIO  │ Notes
 *  ────────────────┼─────────┼───────┼────────────────────────
 *  CLK1            │ D8      │ 19    │ Encoder 1 dedicated clock
 *  CLK2            │ D5      │ 23    │ Encoder 2 dedicated clock
 *  DIN / NSL       │ D10     │ 18    │ Shared: SSI enable / SPI MOSI
 *  DOUT            │ D9      │ 20    │ Shared: data from encoders
 *  SEL1            │ D3      │ 21    │ Encoder 1 SSI_SPI_SEL
 *  SEL2            │ D4      │ 22    │ Encoder 2 SSI_SPI_SEL
 *  WIFI_ENABLE     │ —       │  3    │ RF switch control (active LOW)
 *  WIFI_ANT_CONFIG │ —       │ 14    │ Antenna select (HIGH = external)
 *
 *  ── DUAL-ENCODER BUS ARBITRATION ─────────────────────────────
 *  SSI read ENC1  : SEL1=HIGH, SEL2=LOW,  clock CLK1 × 20 bits.
 *    → ENC2 unclocked + SPI/idle → DOUT high-impedance.
 *  SSI read ENC2  : SEL2=HIGH, SEL1=LOW,  clock CLK2 × 20 bits.
 *    → ENC1 unclocked + SPI/idle → DOUT high-impedance.
 *  SPI access ENC1: SEL1=LOW (SPI mode), SEL2=HIGH (SSI idle),
 *    bit-bang on CLK1. CLK2 stays idle — ENC2 DOUT stays quiet.
 *  SPI access ENC2: SEL2=LOW (SPI mode), SEL1=HIGH (SSI idle),
 *    bit-bang on CLK2. CLK1 stays idle — ENC1 DOUT stays quiet.
 *
 *  ── SSI FRAME (20 bits, 16-bit resolution) ───────────────────
 *  Bits [19:4] = 16-bit absolute position, MSB first
 *  Bit  [3]    = Ready  (must be 1; retry if 0)
 *  Bit  [2]    = MHi   (magnet too strong / too close)
 *  Bit  [1]    = MLo   (magnet too weak   / too far)
 *  Bit  [0]    = Even parity over all 20 bits
 *  Data sampled on the FALLING edge of CLK (datasheet p.15).
 *
 *  ── SPI PROTOCOL ─────────────────────────────────────────────
 *  Write : 0b01_aaaaaa_dddddddd  (16 bits, MSB first)
 *  Read  : 0b10_aaaaaa           (8-bit cmd) then 8-bit reply
 *  CLK idles HIGH (CPOL=1); encoder captures DIN on rising CLK (CPHA=1).
 *
 *  ── KEY TIMING (datasheet Fig. 8 & timing table) ────────────────
 *  Symbol    │ Min  │ Unit │ Datasheet description
 *  ──────────┼──────┼──────┼──────────────────────────────────────────────
 *  tsw(SEL)  │   1  │  µs  │ SSI_SPI_SEL switch time
 *  tREQ      │ 300  │  ns  │ SCL high time between NSL falling edge and
 *            │      │      │   first SCL falling edge
 *  tREQ2     │ 200  │  ns  │ NSL low time after rising edge of last clock
 *            │      │      │   period for an SSI read
 *  tNSLH     │ 200  │  ns  │ NSL high time between 2 successive SSI reads
 *  ──────────┼──────┼──────┼──────────────────────────────────────────────
 *  Notes (datasheet p.15):
 *    • CLK = 1 when inactive; DIN = 1 when inactive.
 *    • CLK must be HIGH when switching between SSI and SPI modes.
 *    • NSL must be held HIGH for at least 3 ms after power-up before
 *      the first SSI read.
 *    • The user is advised to read from the SSI falling edge.
 *    • SSI data length depends on resolution setting:
 *        16-bit → 20 total bits  (16 pos + Ready + MHi + MLo + Parity)
 *  All timing margins use SSI_HALF_US = 5 µs (>> all minimum requirements).
 *
 *  ── SERIAL COMMANDS ──────────────────────────────────────────
 *  p           — Read position from both encoders (SSI, one-shot)
 *  c           — Toggle continuous stream (both encoders, 100 ms)
 *  1 / 2       — Select active encoder for SPI commands
 *  s           — Dump all SPI config registers (active encoder)
 *  r <hh>      — Read register at hex address  e.g. "r 07"
 *  w <hh> <vv> — Write register                e.g. "w 10 AB"
 *  u           — Unlock config registers (0xAB → Lock reg)
 *  z           — Set zero position = 0x0000 (shadow only)
 *  burn        — Burn shadow regs to OTP  ⚠ IRREVERSIBLE
 *  h           — Show this help
 *
 *  Author : Swaraj Dangare
 * ============================================================
 */

// ─── Pin Definitions ──────────────────────────────────────────────────────────
#define PIN_CLK1         D8    // GPIO19 — Encoder 1 dedicated clock
#define PIN_CLK2         D8    // GPIO23 — Encoder 2 dedicated clock
#define PIN_DIN          D10   // GPIO18 — Shared DIN / NSL (SSI enable / SPI MOSI)
#define PIN_DOUT         D9    // GPIO20 — Shared DOUT (data output from encoders)
#define PIN_SEL1         D3    // GPIO21 — Encoder 1 SSI_SPI_SEL
#define PIN_SEL2         D4    // GPIO22 — Encoder 2 SSI_SPI_SEL
#define WIFI_ENABLE      3     // GPIO3  — RF switch control (LOW = active)
#define WIFI_ANT_CONFIG  14    // GPIO14 — Antenna select (HIGH = external)

// ─── Register Addresses ───────────────────────────────────────────────────────
#define REG_CUST_RESERVE_0   0x00
#define REG_CUST_RESERVE_1   0x01
#define REG_ZERO_POS_L       0x02
#define REG_ZERO_POS_H       0x03
#define REG_CUST_CONFIG_0    0x04
#define REG_CPR_SET1         0x05
#define REG_CPR_SET2         0x06
#define REG_RESOLUTION       0x07
#define REG_VCC              0x0A
#define REG_LOCK             0x10
#define REG_PROG_CUST        0x11

// ─── Register Values ──────────────────────────────────────────────────────────
#define UNLOCK_KEY  0xAB
#define PROG_KEY    0xA1
#define VAL_VCC     0x00   // 0x0A [1]=0 → 3.3 V
#define VAL_CFG0    0x00   // 0x04 PWM mode, 1 pole-pair
#define VAL_CPR1    0x40   // 0x05 CPR1=0b0100, no hysteresis
#define VAL_CPR2    0x04   // 0x06 16-bit abs, CW, zero-latency OFF

// ─── SSI Timing ───────────────────────────────────────────────────────────────
// Timing constants derived from AEAT-8800-Q24 datasheet timing table (Fig. 8):
//   tREQ     >= 300 ns   SCL high time between NSL falling edge and first SCL falling edge
//   tREQ2    >= 200 ns   NSL low time after rising edge of last clock period for an SSI read
//   tNSLH    >= 200 ns   NSL high time between 2 successive SSI reads
//   tsw(SEL) >=   1 µs   SSI_SPI_SEL switch time
// SSI_HALF_US = 5 µs is used for every CLK half-period and for all
// tREQ / tREQ2 / tNSLH margins, giving 7–25× margin over the minimums
// and matching the clock rate of the proven single-encoder firmware.
// SSI_MONOFLOP_US: encoder internal load-cycle recovery after NSL goes HIGH.
#define SSI_TOTAL_BITS    20   // 16-bit position + Ready + MHi + MLo + Parity
#define SSI_HALF_US        5   // 5 µs half-period → ~100 kHz SSI clock
#define SSI_MONOFLOP_US   20   // 20 µs monoflop recovery after NSL HIGH

// ─── AEAT8800 Class ───────────────────────────────────────────────────────────
// Each instance owns its CLK pin so SSI and SPI operations on enc1 and enc2
// use completely independent clock lines — no shared-bus arbitration needed.

class AEAT8800 {
public:
  AEAT8800(uint8_t clk, uint8_t din, uint8_t dout, uint8_t sel)
    : _clk(clk), _din(din), _dout(dout), _sel(sel) {}

  /** Assert SEL HIGH → SSI mode. Call only after peer is deselected. */
  void select();

  /** Assert SEL LOW → SPI/idle mode. Safe to call at any time. */
  void deselect();

  /** Bit-bang one SPI register read using this encoder's CLK. */
  uint8_t spiRead(uint8_t addr);

  /** Bit-bang one SPI register write using this encoder's CLK. */
  void spiWrite(uint8_t addr, uint8_t data);

  /** Unlock and write all required OTP shadow registers with readback verify.
   *  Precondition: peer encoder's SEL must be HIGH (SSI idle) before calling
   *  so its DOUT does not contend the shared bus during SPI readback phases.
   *  Returns true on success, false if any register readback mismatches. */
  bool configure();

  /** Perform one 20-bit SSI read. Stores position in `pos` (0–65535).
   *  Precondition: select() must already have been called.
   *  Returns false and prints the reason if Ready=0 or parity fails. */
  bool readAngleRaw(uint16_t &pos);

  /** Read angle in degrees [0.0, 360.0). Returns -1.0f on read error. */
  float readAngleDegrees();

private:
  uint8_t _clk, _din, _dout, _sel;
};

// ─── select / deselect ────────────────────────────────────────────────────────

void AEAT8800::select() {
  // CLK must be HIGH before the SEL transition (datasheet p.15 and p.21:
  // "Make sure CLK is high when switching between SSI and SPI modes.")
  digitalWrite(_clk, HIGH);
  digitalWrite(_sel, HIGH);
  delayMicroseconds(1);   // tsw(SEL) >= 1 µs
}

void AEAT8800::deselect() {
  digitalWrite(_sel, LOW);
  delayMicroseconds(1);   // brief settle before next bus activity
}

// ─── SPI Write ────────────────────────────────────────────────────────────────

void AEAT8800::spiWrite(uint8_t addr, uint8_t data) {
  int prevSel = digitalRead(_sel);

  // CLK HIGH before SEL edge; drive SEL LOW for SPI mode
  digitalWrite(_clk, HIGH);
  digitalWrite(_sel, LOW);
  delayMicroseconds(1);   // tsw(SEL) >= 1 µs

  // 16-bit frame: opcode 0b01 (2b) + addr (6b) + data (8b), MSB first
  uint16_t word = (0b01u << 14) | ((addr & 0x3F) << 8) | (data & 0xFF);

  for (int i = 15; i >= 0; i--) {
    digitalWrite(_din, (word >> i) & 1u);   // DIN valid before CLK falling edge
    digitalWrite(_clk, LOW);                // falling edge
    delayMicroseconds(SSI_HALF_US);
    digitalWrite(_clk, HIGH);               // rising edge — encoder captures DIN
    delayMicroseconds(SSI_HALF_US);         // last iteration: thi(CLK) >= 300 ns
  }

  digitalWrite(_din, HIGH);      // release DIN to idle HIGH
  digitalWrite(_clk, HIGH);      // ensure CLK HIGH before SEL change
  digitalWrite(_sel, prevSel);   // restore caller's SEL state
}

// ─── SPI Read ─────────────────────────────────────────────────────────────────

uint8_t AEAT8800::spiRead(uint8_t addr) {
  int prevSel = digitalRead(_sel);

  digitalWrite(_clk, HIGH);
  digitalWrite(_sel, LOW);
  delayMicroseconds(1);   // tsw(SEL) >= 1 µs

  // 8-bit command: opcode 0b10 (2b) + addr (6b)
  uint8_t cmd = (0b10u << 6) | (addr & 0x3F);

  for (int i = 7; i >= 0; i--) {
    digitalWrite(_din, (cmd >> i) & 1u);
    digitalWrite(_clk, LOW);
    delayMicroseconds(SSI_HALF_US);
    digitalWrite(_clk, HIGH);              // rising edge — encoder captures DIN
    delayMicroseconds(SSI_HALF_US);
  }

  digitalWrite(_din, HIGH);   // release DIN; encoder now drives DOUT for reply

  uint8_t result = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(_clk, LOW);                                     // falling edge — encoder shifts DOUT
    delayMicroseconds(SSI_HALF_US);                              // DOUT valid <= 200 ns after CLK fall
    digitalWrite(_clk, HIGH);                                     // rising edge — master captures DOUT
    result = (result << 1) | (uint8_t)digitalRead(_dout);        // sample on rising edge (datasheet p.20)
    delayMicroseconds(SSI_HALF_US);                              // last iteration: thi(CLK) >= 300 ns
  }

  digitalWrite(_clk, HIGH);
  digitalWrite(_din,  HIGH);
  digitalWrite(_sel,  prevSel);
  return result;
}

// ─── Configure ────────────────────────────────────────────────────────────────

bool AEAT8800::configure() {
  // Unlock must be the very first SPI write (datasheet p.10 note 3)
  spiWrite(REG_LOCK, UNLOCK_KEY);
  delayMicroseconds(10);   // allow lock state to propagate internally

  struct RegTarget { uint8_t addr, val; const char* name; };
  static const RegTarget targets[] = {
    { REG_VCC,           VAL_VCC,  "0x0A (VCC)"  },
    { REG_CUST_CONFIG_0, VAL_CFG0, "0x04 (CFG0)" },
    { REG_CPR_SET1,      VAL_CPR1, "0x05 (CPR1)" },
    { REG_CPR_SET2,      VAL_CPR2, "0x06 (CPR2)" },
  };

  for (const auto& r : targets) {
    spiWrite(r.addr, r.val);
    delayMicroseconds(5);   // propagation margin before readback
    uint8_t rb = spiRead(r.addr);
    if (rb != r.val) {
      Serial.printf("[ERR] Reg %s verify: wrote 0x%02X  readback 0x%02X\n",
                    r.name, r.val, rb);
      return false;
    }
  }

  deselect();   // explicit deselect on exit
  return true;
}

// ─── SSI Read ─────────────────────────────────────────────────────────────────

bool AEAT8800::readAngleRaw(uint16_t &pos) {
  // Precondition: select() already called — SEL is HIGH, encoder in SSI mode

  // CLK = 1 when inactive; DIN = 1 when inactive (datasheet note)
  digitalWrite(_clk, HIGH);
  digitalWrite(_din,  HIGH);
  delayMicroseconds(SSI_HALF_US);   // tNSLH >= 200 ns: NSL high time between 2 successive SSI reads

  // NSL LOW → shift mode: encoder freezes position into shift register (Fig. 8)
  digitalWrite(_din, LOW);
  delayMicroseconds(SSI_HALF_US);   // tREQ >= 300 ns: SCL high time between NSL falling edge and first SCL falling edge

  uint32_t raw = 0;
  for (int i = 0; i < SSI_TOTAL_BITS; i++) {
    digitalWrite(_clk, LOW);                                   // SCL falling edge — encoder shifts next bit onto DO
    delayMicroseconds(SSI_HALF_US);                            // read from SSI falling edge per datasheet (data stable after CLK fall)
    raw = (raw << 1) | (uint32_t)digitalRead(_dout);           // sample DO after falling edge (datasheet Fig. 8, p.15)
    digitalWrite(_clk, HIGH);                                   // SCL rising edge
    delayMicroseconds(SSI_HALF_US);                            // CLK high hold (CLK = 1 when inactive)
  }

  delayMicroseconds(SSI_HALF_US);   // tREQ2 >= 200 ns: NSL low time after rising edge of last clock period for an SSI read

  // End of frame: NSL HIGH → load mode (encoder resumes tracking position)
  digitalWrite(_din, HIGH);
  delayMicroseconds(SSI_MONOFLOP_US);   // monoflop recovery: encoder completes internal load cycle

  // Unpack 20-bit frame (datasheet Fig. 8–9)
  uint8_t  parity   = (raw >> 0) & 0x1u;
  uint8_t  mlo      = (raw >> 1) & 0x1u;
  uint8_t  mhi      = (raw >> 2) & 0x1u;
  uint8_t  ready    = (raw >> 3) & 0x1u;
  uint16_t position = (uint16_t)((raw >> 4) & 0xFFFFu);

  (void)parity;   // value is implicitly checked via __builtin_popcount below

  if (!ready) {
    Serial.printf("[ERR] Ready=0 (raw=0x%05X) — data not valid\n", (unsigned)raw);
    return false;
  }

  if (mhi) Serial.println(F("[WARN] MHi=1 — magnet too strong / too close"));
  if (mlo) Serial.println(F("[WARN] MLo=1 — magnet too weak  / too far"));

  // Even parity: total 1-bit count across all 20 raw bits must be even
  if (__builtin_popcount((unsigned)raw) % 2 != 0) {
    Serial.printf("[ERR] Parity error (raw=0x%05X)\n", (unsigned)raw);
    return false;
  }

  pos = position;
  return true;
}

float AEAT8800::readAngleDegrees() {
  uint16_t raw = 0;
  if (!readAngleRaw(raw)) return -1.0f;
  return (raw / 65536.0f) * 360.0f;
}

// ─── Global Encoder Instances ─────────────────────────────────────────────────
AEAT8800 enc1(PIN_CLK1, PIN_DIN, PIN_DOUT, PIN_SEL1);
AEAT8800 enc2(PIN_CLK2, PIN_DIN, PIN_DOUT, PIN_SEL2);

// ─── Global State ─────────────────────────────────────────────────────────────
static bool     g_streaming  = false;
static uint32_t g_lastStream = 0;
static uint8_t  g_activeEnc  = 1;   // 1 or 2 — target for SPI serial commands

// ─── Helpers ─────────────────────────────────────────────────────────────────

/** Return a pointer to the currently selected encoder. */
static inline AEAT8800* activeEncoder() {
  return (g_activeEnc == 1) ? &enc1 : &enc2;
}

/** Raise the peer's SEL to SSI/idle, isolating its DOUT from the bus. */
static inline void isolatePeer() {
  if (g_activeEnc == 1) { enc2.select(); } else { enc1.select(); }
}

/** Lower the peer's SEL back to SPI/idle after the transaction. */
static inline void releasePeer() {
  if (g_activeEnc == 1) { enc2.deselect(); } else { enc1.deselect(); }
}

const char* resolutionStr(uint8_t regVal) {
  switch (regVal & 0x03) {
    case 0x00: return "10-bit (1024 cpr)";
    case 0x01: return "12-bit (4096 cpr)";
    case 0x02: return "14-bit (16384 cpr)";
    case 0x03: return "16-bit (65536 cpr)";
    default:   return "unknown";
  }
}

// ─── Serial Output Helpers ────────────────────────────────────────────────────

void printSinglePosition(uint8_t id, uint16_t pos, bool ok) {
  if (ok) {
    double deg = (pos / 65536.0) * 360.0;
    Serial.printf("[ENC%u] Pos=%5u  Angle=%8.4f deg\n", id, pos, deg);
  } else {
    Serial.printf("[ENC%u] READ ERROR\n", id);
  }
}

void printAllRegisters() {
  AEAT8800* enc = activeEncoder();

  Serial.printf("\n╔══════════════════════════════════════════════╗\n");
  Serial.printf(  "║  AEAT-8800-Q24 — ENC%u SPI Register Dump     ║\n", g_activeEnc);
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

  isolatePeer();   // peer SEL HIGH → its DOUT tri-state during SPI reads

  for (auto& r : regs) {
    uint8_t val = enc->spiRead(r.addr);
    Serial.printf("║  %-22s  0x%02X  (%3u) ║\n", r.name, val, val);
  }

  uint8_t lo     = enc->spiRead(REG_ZERO_POS_L);
  uint8_t hi     = enc->spiRead(REG_ZERO_POS_H);
  uint16_t zp    = ((uint16_t)hi << 8) | lo;
  uint8_t resReg = enc->spiRead(REG_RESOLUTION);

  releasePeer();

  Serial.println(F("╠══════════════════════════════════════════════╣"));
  Serial.printf(  "║  HW Zero Pos : %5u  (%.4f deg)          ║\n",
                  zp, (zp / 65536.0) * 360.0);
  Serial.printf(  "║  Resolution  : %-30s ║\n", resolutionStr(resReg));
  Serial.println(F("╚══════════════════════════════════════════════╝\n"));
}

void printHelp() {
  Serial.println(F("\n╔══════════════════════════════════════════════════════╗"));
  Serial.println(F("║  AEAT-8800-Q24 DUAL  Star-Topology  Command Menu     ║"));
  Serial.println(F("╠══════════════════════════════════════════════════════╣"));
  Serial.println(F("║  p           — Read both encoders (SSI, one-shot)    ║"));
  Serial.println(F("║  c           — Toggle continuous stream (100 ms)     ║"));
  Serial.println(F("║  1 / 2       — Select active encoder for SPI cmds    ║"));
  Serial.println(F("║  s           — Dump SPI registers (active encoder)   ║"));
  Serial.println(F("║  r <hh>      — Read register  e.g. 'r 07'            ║"));
  Serial.println(F("║  w <hh> <vv> — Write register e.g. 'w 10 AB'         ║"));
  Serial.println(F("║  u           — Unlock config registers               ║"));
  Serial.println(F("║  z           — Set zero position = 0x0000            ║"));
  Serial.println(F("║  burn        — Burn OTP  ⚠ IRREVERSIBLE              ║"));
  Serial.println(F("║  h           — Show this help                         ║"));
  Serial.println(F("╚══════════════════════════════════════════════════════╝\n"));
}

// ─── Dual SSI Read (used by 'p' command and continuous stream) ────────────────

static void readBothEncoders(uint16_t &pos1, uint16_t &pos2,
                              bool &ok1, bool &ok2) {
  // Read ENC1: SEL2 LOW (already), raise SEL1, clock CLK1
  enc2.deselect();
  enc1.select();
  ok1 = enc1.readAngleRaw(pos1);
  enc1.deselect();

  delay(1);   // inter-encoder gap; NSL HIGH > tNSLH before next read

  // Read ENC2: SEL1 LOW (already), raise SEL2, clock CLK2
  enc1.deselect();
  enc2.select();
  ok2 = enc2.readAngleRaw(pos2);
  enc2.deselect();
}

// ─── Serial Command Parser ────────────────────────────────────────────────────

void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // OTP burn requires the exact string "burn" to prevent accidental triggers
  if (line.equalsIgnoreCase("burn")) {
    AEAT8800* enc = activeEncoder();
    Serial.printf("[OTP] Unlocking ENC%u and burning shadow regs to OTP...\n", g_activeEnc);
    enc->spiWrite(REG_LOCK, UNLOCK_KEY);
    delayMicroseconds(10);
    enc->spiWrite(REG_PROG_CUST, PROG_KEY);
    Serial.println(F("[OTP] Done. Power-cycle the encoder to verify."));
    return;
  }

  char cmd = (char)tolower((unsigned char)line[0]);

  switch (cmd) {

    // ── Read both positions (SSI) ─────────────────────────────────────────
    case 'p': {
      uint16_t pos1 = 0, pos2 = 0;
      bool ok1 = false, ok2 = false;
      readBothEncoders(pos1, pos2, ok1, ok2);
      printSinglePosition(1, pos1, ok1);
      printSinglePosition(2, pos2, ok2);
      break;
    }

    // ── Continuous stream toggle ──────────────────────────────────────────
    case 'c':
      g_streaming = !g_streaming;
      Serial.printf("[INFO] Continuous stream %s\n", g_streaming ? "ON" : "OFF");
      break;

    // ── Select active encoder for SPI operations ──────────────────────────
    case '1':
      g_activeEnc = 1;
      Serial.println(F("[INFO] Active encoder: ENC1"));
      break;
    case '2':
      g_activeEnc = 2;
      Serial.println(F("[INFO] Active encoder: ENC2"));
      break;

    // ── Register dump (SPI) ───────────────────────────────────────────────
    case 's':
      printAllRegisters();
      break;

    // ── Read single register  "r <hex-addr>" ─────────────────────────────
    case 'r': {
      if (line.length() < 3) {
        Serial.println(F("[ERR] Usage: r <hex-addr>  e.g. 'r 07'"));
        break;
      }
      uint8_t addr = (uint8_t)strtoul(line.c_str() + 2, nullptr, 16);
      isolatePeer();
      uint8_t val = activeEncoder()->spiRead(addr);
      releasePeer();
      Serial.printf("[ENC%u SPI-RD] Reg 0x%02X = 0x%02X  (%u)\n",
                    g_activeEnc, addr, val, val);
      break;
    }

    // ── Write single register  "w <hex-addr> <hex-val>" ──────────────────
    case 'w': {
      if (line.length() < 5) {
        Serial.println(F("[ERR] Usage: w <hex-addr> <hex-val>  e.g. 'w 10 AB'"));
        break;
      }
      char*   ptr  = nullptr;
      uint8_t addr = (uint8_t)strtoul(line.c_str() + 2, &ptr, 16);
      uint8_t val  = (uint8_t)strtoul(ptr,               nullptr, 16);
      activeEncoder()->spiWrite(addr, val);
      isolatePeer();
      uint8_t rb = activeEncoder()->spiRead(addr);
      releasePeer();
      Serial.printf("[ENC%u SPI-WR] Reg 0x%02X <- 0x%02X  |  Readback: 0x%02X  %s\n",
                    g_activeEnc, addr, val, rb, (rb == val) ? "OK" : "MISMATCH!");
      break;
    }

    // ── Unlock registers ──────────────────────────────────────────────────
    case 'u':
      activeEncoder()->spiWrite(REG_LOCK, UNLOCK_KEY);
      Serial.printf("[ENC%u SPI] Registers unlocked (0xAB written to Lock reg)\n",
                    g_activeEnc);
      break;

    // ── Set zero position ─────────────────────────────────────────────────
    case 'z': {
      AEAT8800* enc = activeEncoder();
      Serial.printf("[ENC%u ZERO] Writing 0x0000 to ZeroPos shadow registers...\n",
                    g_activeEnc);
      enc->spiWrite(REG_LOCK, UNLOCK_KEY);
      delayMicroseconds(10);
      enc->spiWrite(REG_ZERO_POS_L, 0x00);
      enc->spiWrite(REG_ZERO_POS_H, 0x00);
      isolatePeer();
      uint8_t lo = enc->spiRead(REG_ZERO_POS_L);
      uint8_t hi = enc->spiRead(REG_ZERO_POS_H);
      releasePeer();
      uint16_t zp = ((uint16_t)hi << 8) | lo;
      Serial.printf("[ENC%u ZERO] Readback: 0x%04X — %s\n", g_activeEnc, zp,
                    zp == 0x0000 ? "OK" : "MISMATCH!");
      break;
    }

    // ── Burn guard (user must type "burn" exactly) ────────────────────────
    case 'b':
      Serial.println(F("[WARN] OTP BURN is IRREVERSIBLE!"));
      Serial.println(F("[WARN] Type exactly 'burn' and press Enter to confirm."));
      break;

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

  // ── External antenna ─────────────────────────────────────────────────────
  // GPIO3 LOW activates the RF switch control circuit; GPIO14 HIGH selects
  // the external antenna over the built-in ceramic one.
  pinMode(WIFI_ENABLE,     OUTPUT);
  digitalWrite(WIFI_ENABLE, LOW);    // activate RF switch control
  delay(100);
  pinMode(WIFI_ANT_CONFIG, OUTPUT);
  digitalWrite(WIFI_ANT_CONFIG, HIGH);  // select external antenna

  // ── GPIO init — all bus pins to safe idle states ──────────────────────────
  pinMode(PIN_CLK1, OUTPUT); digitalWrite(PIN_CLK1, HIGH);  // CLK1 idles HIGH
  pinMode(PIN_CLK2, OUTPUT); digitalWrite(PIN_CLK2, HIGH);  // CLK2 idles HIGH
  pinMode(PIN_DIN,  OUTPUT); digitalWrite(PIN_DIN,  HIGH);  // NSL/DIN idles HIGH
  pinMode(PIN_DOUT, INPUT);
  pinMode(PIN_SEL1, OUTPUT); digitalWrite(PIN_SEL1, LOW);   // SPI/idle mode
  pinMode(PIN_SEL2, OUTPUT); digitalWrite(PIN_SEL2, LOW);   // SPI/idle mode

  // tPwrUp ~4 ms (datasheet p.6) + NSL must be HIGH >= 3 ms before first SSI read
  delay(5);

  // ── Boot banner ───────────────────────────────────────────────────────────
  Serial.println(F("\n╔══════════════════════════════════════════════════════════╗"));
  Serial.println(F(  "║  AEAT-8800-Q24 DUAL  |  Star-CLK  |  XIAO ESP32-C6     ║"));
  Serial.println(F(  "╠══════════════════════════════════════════════════════════╣"));
  Serial.println(F(  "║  CLK1=GPIO19(D8)  CLK2=GPIO23(D5)  DIN=GPIO18(D10)      ║"));
  Serial.println(F(  "║  DOUT=GPIO20(D9)  SEL1=GPIO21(D3)  SEL2=GPIO22(D4)      ║"));
  Serial.println(F(  "║  Antenna : External  [GPIO3=LOW, GPIO14=HIGH]            ║"));
  Serial.println(F(  "╠══════════════════════════════════════════════════════════╣"));

  // ── Configure ENC1 ────────────────────────────────────────────────────────
  // Raise SEL2 (ENC2 → SSI/idle) so ENC2's DOUT does not contend the shared
  // bus during ENC1's SPI readback transactions.
  Serial.print(F(    "║  Configuring ENC1 ... "));
  enc2.select();
  delayMicroseconds(1);
  bool ok1 = enc1.configure();   // leaves ENC1 SEL LOW on exit
  enc2.deselect();
  Serial.println(ok1 ? F("OK                                       ║")
                     : F("FAILED                                   ║"));

  // ── Configure ENC2 ────────────────────────────────────────────────────────
  Serial.print(F(    "║  Configuring ENC2 ... "));
  enc1.select();
  delayMicroseconds(1);
  bool ok2 = enc2.configure();   // leaves ENC2 SEL LOW on exit
  enc1.deselect();
  Serial.println(ok2 ? F("OK                                       ║")
                     : F("FAILED                                   ║"));

  Serial.println(F(  "╠══════════════════════════════════════════════════════════╣"));

  // ── Initial position read ─────────────────────────────────────────────────
  uint16_t pos1 = 0, pos2 = 0;
  bool r1 = false, r2 = false;
  readBothEncoders(pos1, pos2, r1, r2);

  if (r1)
    Serial.printf(   "║  ENC1 Position : %5u counts  (%.4f deg)           ║\n",
                     pos1, (pos1 / 65536.0) * 360.0);
  else
    Serial.println(F("║  ENC1 Position : READ ERROR                              ║"));

  if (r2)
    Serial.printf(   "║  ENC2 Position : %5u counts  (%.4f deg)           ║\n",
                     pos2, (pos2 / 65536.0) * 360.0);
  else
    Serial.println(F("║  ENC2 Position : READ ERROR                              ║"));

  Serial.println(F(  "╠══════════════════════════════════════════════════════════╣"));
  Serial.println(F(  "║  Type 'h' for command help                               ║"));
  Serial.println(F(  "╚══════════════════════════════════════════════════════════╝\n"));
}

// ─── Main Loop ────────────────────────────────────────────────────────────────

void loop() {
  handleSerial();

  if (g_streaming && (millis() - g_lastStream >= 100)) {
    g_lastStream = millis();

    uint16_t pos1 = 0, pos2 = 0;
    bool ok1 = false, ok2 = false;
    readBothEncoders(pos1, pos2, ok1, ok2);

    float deg1 = ok1 ? (pos1 / 65536.0f) * 360.0f : -1.0f;
    float deg2 = ok2 ? (pos2 / 65536.0f) * 360.0f : -1.0f;

    if (ok1 && ok2) {
      Serial.printf("ENC1: %.4f deg | ENC2: %.4f deg\n", deg1, deg2);
    } else if (!ok1 && !ok2) {
      Serial.println(F("ENC1: ERR | ENC2: ERR"));
    } else if (!ok1) {
      Serial.printf("ENC1: ERR | ENC2: %.4f deg\n", deg2);
    } else {
      Serial.printf("ENC1: %.4f deg | ENC2: ERR\n", deg1);
    }
  }
}
