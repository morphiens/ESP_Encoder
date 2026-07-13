/** jgkrsjgklrslgrs
 * ============================================================
 *  AEAT-8800-Q24  —  Fully Independent Dual Encoder
 *  Board  : Seeed Studio XIAO ESP32-C6
 *
 *  ── FULLY INDEPENDENT WIRING ─────────────────────────────────
 *  Every encoder has its own dedicated CLK, DIN/NSL, DO, and SEL.
 *  No pins are shared between encoders — eliminates all bus
 *  conflicts and DOUT tri-state contention.
 *
 *  ── DEFAULT PIN ASSIGNMENT ───────────────────────────────────
 *  Signal      │ Encoder 1       │ Encoder 2
 *  ────────────┼─────────────────┼─────────────────
 *  CLK         │ D8  / GPIO19    │ D5  / GPIO23
 *  DIN / NSL   │ D10 / GPIO18    │ D2  / GPIO2
 *  DO  / DOUT  │ D9  / GPIO20    │ D1  / GPIO1
 *  SEL         │ D3  / GPIO21    │ D4  / GPIO22
 *
 *  ── ANTENNA PINS ─────────────────────────────────────────────
 *  WIFI_ENABLE     GPIO3  — LOW  = activate RF switch control
 *  WIFI_ANT_CONFIG GPIO14 — HIGH = select external antenna
 *
 *  ── KEY TIMING (datasheet Fig. 8 & timing table) ─────────────
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
 *    • 16-bit resolution → 20 total bits (16 pos + Ready + MHi + MLo + Parity)
 *  All margins use SSI_HALF_US = 5 µs (>> all minimum requirements).
 *
 *  ── BOOT WIZARD ──────────────────────────────────────────────
 *  Set RUN_SETUP_WIZARD = true to prompt over Serial (115200 baud):
 *    1. How many encoders are connected [1/2]
 *    2. Which slot (only if 1 encoder)
 *    3. Use default pins, or enter custom GPIO numbers
 *  Set RUN_SETUP_WIZARD = false to skip the wizard: both encoders on
 *  default pins, continuous stream (c) starts automatically.
 *
 *  ── SERIAL COMMANDS ──────────────────────────────────────────
 *  p           — Read position (SSI, one-shot, enabled encoders)
 *  c           — Toggle continuous stream (both enabled, 100 ms)
 *  o <deg>     — Software offset: current pos reports as <deg> (both encoders)
 *  o clear     — Remove software offset (revert to raw hardware angles)
 *  1 / 2       — Select active encoder for SPI commands
 *  s           — Dump SPI config registers (active encoder)
 *  r <hh>      — Read register at hex address  e.g. "r 07"
 *  w <hh> <vv> — Write register               e.g. "w 10 AB"
 *  u           — Unlock config registers
 *  z           — HW zero position = 0x0000 (SPI shadow only)
 *  burn        — Burn OTP  ⚠ IRREVERSIBLE (type "burn" exactly)
 *  h           — Show this help
 *
 *  Author : Swaraj Dangare
 * ============================================================
 */

#include <algorithm> // For std::sort
#include <math.h>    // For sin, cos, atan2, PI

// ─── Hybrid Robust Filtering Settings ────────────────────────────────────────
#define NUM_BLOCKS 16
#define SAMPLES_PER_BLOCK 16
#define OUTLIER_THRESHOLD                                                      \
  20.0 // Allow some natural jitter, reject massive spikes

// true  = interactive boot wizard (encoder count, slot, custom pins)
// false = skip wizard; both encoders on default pins; start stream (c) immediately
static constexpr bool RUN_SETUP_WIZARD = false;

// 16-bit encoder: 65536 counts/rev → ~0.0055° per count
#define ENCODER_CPR 65536
static constexpr double ENCODER_CPR_D = 65536.0;
static constexpr double DEG_PER_COUNT = 360.0 / ENCODER_CPR_D;

static uint16_t snapToCounts(double counts);
static double countsToDeg(uint16_t counts);

// ─── Antenna Pins
// ─────────────────────────────────────────────────────────────
#define WIFI_ENABLE 3      // GPIO3  — RF switch control (LOW = active)
#define WIFI_ANT_CONFIG 14 // GPIO14 — Antenna select (HIGH = external)

// ─── Default Encoder Pins
// ─────────────────────────────────────────────────────
#define DEFAULT_ENC1_CLK D8  // GPIO19
#define DEFAULT_ENC1_DIN D10 // GPIO18
#define DEFAULT_ENC1_DOUT D9 // GPIO20
#define DEFAULT_ENC1_SEL D7  // GPIO21

#define DEFAULT_ENC2_CLK D5  // GPIO23
#define DEFAULT_ENC2_DIN D3  // GPIO2
#define DEFAULT_ENC2_DOUT D4 // GPIO1
#define DEFAULT_ENC2_SEL D6  // GPIO22

// ─── Register Addresses
// ───────────────────────────────────────────────────────
#define REG_CUST_RESERVE_0 0x00
#define REG_CUST_RESERVE_1 0x01
#define REG_ZERO_POS_L 0x02
#define REG_ZERO_POS_H 0x03
#define REG_CUST_CONFIG_0 0x04
#define REG_CPR_SET1 0x05
#define REG_CPR_SET2 0x06
#define REG_RESOLUTION 0x07
#define REG_VCC 0x0A
#define REG_LOCK 0x10
#define REG_PROG_CUST 0x11

// ─── Register Values
// ──────────────────────────────────────────────────────────
#define UNLOCK_KEY 0xAB
#define PROG_KEY 0xA1
#define VAL_VCC 0x00  // 0x0A [1]=0 → 3.3 V
#define VAL_CFG0 0x00 // 0x04 PWM mode, 1 pole-pair
// 0x05 [7:4]=CPR1, [3:0]=Hysteresis (datasheet Table 3)
//   CPR1     0b0100 → 512 CPR (ABI; absolute-only uses CPR_Set2 in 0x06)
//   Hyst     0b0010 → 0.01 mechanical degree (mdeg)
#define VAL_CPR1 0x42
#define VAL_CPR2 0x04 // 0x06 16-bit abs, CW, zero-latency OFF

// ─── SSI Timing
// ─────────────────────────────────────────────────────────────── Timing
// constants from AEAT-8800-Q24 datasheet timing table (Fig. 8):
//   tREQ     >= 300 ns   SCL high time between NSL falling edge and first SCL
//   falling edge tREQ2    >= 200 ns   NSL low time after rising edge of last
//   clock period for an SSI read tNSLH    >= 200 ns   NSL high time between 2
//   successive SSI reads tsw(SEL) >=   1 µs   SSI_SPI_SEL switch time
// SSI_HALF_US = 5 µs gives 7–25× margin over minimums.
// SSI_MONOFLOP_US: encoder internal load-cycle recovery after NSL goes HIGH.
#define SSI_TOTAL_BITS 20 // 16-bit position + Ready + MHi + MLo + Parity
#define SSI_HALF_US 1 // 1 us half-period -> ~500 kHz SSI clock (Fast for 30Hz)
#define SSI_MONOFLOP_US 20 // monoflop recovery after NSL HIGH

// ─── Encoder Config
// ───────────────────────────────────────────────────────────

struct EncoderConfig {
  uint8_t pinCLK;
  uint8_t pinDIN;
  uint8_t pinDOUT;
  uint8_t pinSEL;
};

// ─── AEAT8800 Class
// ─────────────────────────────────────────────────────────── Each instance is
// completely self-contained: CLK, DIN, DOUT, and SEL are all per-instance. No
// pins are shared with any other instance.

class AEAT8800 {
public:
  AEAT8800(uint8_t clk, uint8_t din, uint8_t dout, uint8_t sel)
      : _clk(clk), _din(din), _dout(dout), _sel(sel) {}

  /** Assert SEL HIGH → SSI mode. */
  void select();

  /** Assert SEL LOW → SPI/idle mode. */
  void deselect();

  /** Bit-bang one SPI register read using this encoder's CLK and DIN. */
  uint8_t spiRead(uint8_t addr);

  /** Bit-bang one SPI register write using this encoder's CLK and DIN. */
  void spiWrite(uint8_t addr, uint8_t data);

  /** Unlock and write all required OTP shadow registers with readback verify.
   *  Returns true on success. Failure is non-fatal; SSI reads still work. */
  bool configure();

  /** Perform one 20-bit SSI read. Stores position in pos (0–65535).
   *  Precondition: select() must have been called.
   *  Returns false if Ready=0 or parity fails. */
  bool readAngleRaw(uint16_t &pos, bool silent = false);

  /** Read angle in degrees [0, 360). Returns -1.0f on error. */
  float readAngleDegrees();

private:
  uint8_t _clk, _din, _dout, _sel;
};

// ─── select / deselect
// ────────────────────────────────────────────────────────

void AEAT8800::select() {
  // CLK must be HIGH before SEL transition (datasheet p.15, p.21)
  digitalWrite(_clk, HIGH);
  digitalWrite(_sel, HIGH);
  delayMicroseconds(1); // tsw(SEL) >= 1 µs
}

void AEAT8800::deselect() {
  digitalWrite(_sel, LOW);
  delayMicroseconds(1);
}

// ─── SPI Write
// ────────────────────────────────────────────────────────────────

void AEAT8800::spiWrite(uint8_t addr, uint8_t data) {
  int prevSel = digitalRead(_sel);

  digitalWrite(_clk, HIGH);
  digitalWrite(_sel, LOW);
  delayMicroseconds(1); // tsw(SEL) >= 1 µs

  // 16-bit frame: opcode 0b01 (2b) + addr (6b) + data (8b), MSB first
  uint16_t word = (0b01u << 14) | ((addr & 0x3F) << 8) | (data & 0xFF);
  for (int i = 15; i >= 0; i--) {
    digitalWrite(_din, (word >> i) & 1u); // DIN valid before CLK fall
    digitalWrite(_clk, LOW);              // falling edge
    delayMicroseconds(SSI_HALF_US);
    digitalWrite(_clk, HIGH); // rising edge — encoder captures DIN
    delayMicroseconds(SSI_HALF_US);
  }

  digitalWrite(_din, HIGH); // DIN back to idle HIGH
  digitalWrite(_clk, HIGH); // CLK must be HIGH before SEL change
  digitalWrite(_sel, prevSel);
}

// ─── SPI Read
// ─────────────────────────────────────────────────────────────────

uint8_t AEAT8800::spiRead(uint8_t addr) {
  int prevSel = digitalRead(_sel);

  digitalWrite(_clk, HIGH);
  digitalWrite(_sel, LOW);
  delayMicroseconds(1); // tsw(SEL) >= 1 µs

  // 8-bit command: opcode 0b10 (2b) + addr (6b)
  uint8_t cmd = (0b10u << 6) | (addr & 0x3F);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(_din, (cmd >> i) & 1u);
    digitalWrite(_clk, LOW);
    delayMicroseconds(SSI_HALF_US);
    digitalWrite(_clk, HIGH); // rising edge — encoder captures DIN
    delayMicroseconds(SSI_HALF_US);
  }
  digitalWrite(_din, HIGH); // release DIN; encoder drives DOUT for reply

  uint8_t result = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(_clk, LOW);        // falling edge
    delayMicroseconds(SSI_HALF_US); // DOUT valid <= 200 ns after CLK fall
    digitalWrite(_clk, HIGH);       // rising edge — master captures DOUT
    result =
        (result << 1) |
        (uint8_t)digitalRead(_dout); // sample on rising edge (datasheet p.20)
    delayMicroseconds(SSI_HALF_US);
  }

  digitalWrite(_clk, HIGH);
  digitalWrite(_din, HIGH);
  digitalWrite(_sel, prevSel);
  return result;
}

// ─── Configure
// ────────────────────────────────────────────────────────────────

bool AEAT8800::configure() {
  // Unlock must be the very first SPI write (datasheet p.10 note 3)
  spiWrite(REG_LOCK, UNLOCK_KEY);
  delayMicroseconds(10);

  struct RegTarget {
    uint8_t addr, val;
    const char *name;
  };
  static const RegTarget targets[] = {
      {REG_VCC, VAL_VCC, "0x0A (VCC)"},
      {REG_CUST_CONFIG_0, VAL_CFG0, "0x04 (CFG0)"},
      {REG_CPR_SET1, VAL_CPR1, "0x05 (CPR1)"},
      {REG_CPR_SET2, VAL_CPR2, "0x06 (CPR2)"},
  };

  for (const auto &r : targets) {
    spiWrite(r.addr, r.val);
    delayMicroseconds(5);
    uint8_t rb = spiRead(r.addr);
    if (rb != r.val) {
      Serial.printf("[ERR] Reg %s verify: wrote 0x%02X  readback 0x%02X\n",
                    r.name, r.val, rb);
      return false;
    }
  }

  deselect();
  return true;
}

// ─── SSI Read
// ─────────────────────────────────────────────────────────────────

bool AEAT8800::readAngleRaw(uint16_t &pos, bool silent) {
  // Precondition: select() already called — SEL is HIGH, encoder in SSI mode

  // CLK = 1 when inactive; DIN = 1 when inactive (datasheet note)
  digitalWrite(_clk, HIGH);
  digitalWrite(_din, HIGH);
  delayMicroseconds(SSI_HALF_US); // tNSLH >= 200 ns: NSL high time between 2
                                  // successive SSI reads

  // NSL LOW → shift mode: encoder freezes position into shift register (Fig. 8)
  digitalWrite(_din, LOW);
  delayMicroseconds(SSI_HALF_US); // tREQ >= 300 ns: SCL high time between NSL
                                  // falling edge and first SCL falling edge

  uint32_t raw = 0;
  for (int i = 0; i < SSI_TOTAL_BITS; i++) {
    digitalWrite(_clk,
                 LOW); // SCL falling edge — encoder shifts next bit onto DO
    delayMicroseconds(SSI_HALF_US); // read from SSI falling edge per datasheet
                                    // (data stable after CLK fall)
    raw = (raw << 1) |
          (uint32_t)digitalRead(
              _dout); // sample DO after falling edge (datasheet Fig. 8, p.15)
    digitalWrite(_clk, HIGH);       // SCL rising edge
    delayMicroseconds(SSI_HALF_US); // CLK high hold (CLK = 1 when inactive)
  }

  delayMicroseconds(SSI_HALF_US); // tREQ2 >= 200 ns: NSL low time after rising
                                  // edge of last clock period for an SSI read

  // End of frame: NSL HIGH → load mode (encoder resumes tracking position)
  digitalWrite(_din, HIGH);
  delayMicroseconds(SSI_MONOFLOP_US); // monoflop recovery: encoder completes
                                      // internal load cycle

  // Unpack 20-bit frame (datasheet Fig. 8–9)
  uint8_t parity = (raw >> 0) & 0x1u;
  uint8_t mlo = (raw >> 1) & 0x1u;
  uint8_t mhi = (raw >> 2) & 0x1u;
  uint8_t ready = (raw >> 3) & 0x1u;
  uint16_t position = (uint16_t)((raw >> 4) & 0xFFFFu);

  (void)parity; // checked implicitly via __builtin_popcount below

  if (!ready) {
    if (!silent)
      Serial.printf("[ERR] Ready=0 (raw=0x%05X) — data not valid\n",
                    (unsigned)raw);
    return false;
  }
  if (!silent) {
    if (mhi)
      Serial.println(F("[WARN] MHi=1 — magnet too strong / too close"));
    if (mlo)
      Serial.println(F("[WARN] MLo=1 — magnet too weak  / too far"));
  }

  // Even parity: total 1-bit count across all 20 raw bits must be even
  if (__builtin_popcount((unsigned)raw) % 2 != 0) {
    if (!silent)
      Serial.printf("[ERR] Parity error (raw=0x%05X)\n", (unsigned)raw);
    return false;
  }

  pos = position;
  return true;
}

float AEAT8800::readAngleDegrees() {
  uint16_t raw = 0;
  if (!readAngleRaw(raw))
    return -1.0f;
  return (float)countsToDeg(raw);
}

// ─── Global State
// ─────────────────────────────────────────────────────────────

static AEAT8800 *g_enc1 = nullptr;
static AEAT8800 *g_enc2 = nullptr;
static bool g_enc1Enabled = false;
static bool g_enc2Enabled = false;
static bool g_streaming = false;
static uint32_t g_lastStream = 0;
static uint8_t g_activeEnc = 1; // target encoder for SPI serial commands

// Software angle offsets (degrees). Applied at display time only.
// Offset = rawDeg - targetDeg at the moment `o <deg>` is sent.
static double g_offsetDeg1 = 0.0;
static double g_offsetDeg2 = 0.0;

// ─── Setup Wizard Helpers
// ─────────────────────────────────────────────────────

/** Block until a complete line is received on Serial. Returns trimmed string.
 */
static String readLine() {
  while (!Serial.available()) { /* busy-wait */
  }
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

/** Print a pin prompt, read a GPIO integer. Returns the chosen GPIO number. */
static uint8_t promptPin(const char *label, uint8_t def) {
  Serial.printf("    %-8s (default GPIO%-2u): ", label, def);
  String s = readLine();
  if (s.length() == 0) {
    Serial.printf("GPIO%u  (default)\n", def);
    return def;
  }
  uint8_t v = (uint8_t)s.toInt();
  Serial.printf("GPIO%u\n", v);
  return v;
}

/** Prompt for custom pin override for one encoder. Modifies cfg in-place. */
static void promptCustomPins(const char *label, EncoderConfig &cfg,
                             const EncoderConfig &def) {
  Serial.printf("\n  Encoder %s — defaults:"
                " CLK=GPIO%u  DIN=GPIO%u  DO=GPIO%u  SEL=GPIO%u\n",
                label, def.pinCLK, def.pinDIN, def.pinDOUT, def.pinSEL);
  Serial.print(F("  Use default pins? [Y/n]: "));
  String ans = readLine();
  ans.toLowerCase();

  if (ans == "n") {
    cfg.pinCLK = promptPin("CLK", def.pinCLK);
    cfg.pinDIN = promptPin("DIN", def.pinDIN);
    cfg.pinDOUT = promptPin("DO", def.pinDOUT);
    cfg.pinSEL = promptPin("SEL", def.pinSEL);
  } else {
    cfg = def;
    Serial.println(F("  Using defaults."));
  }
}

// ─── Setup Wizard
// ─────────────────────────────────────────────────────────────

static void runSetupWizard(EncoderConfig &cfg1, EncoderConfig &cfg2) {
  const EncoderConfig def1 = {DEFAULT_ENC1_CLK, DEFAULT_ENC1_DIN,
                              DEFAULT_ENC1_DOUT, DEFAULT_ENC1_SEL};
  const EncoderConfig def2 = {DEFAULT_ENC2_CLK, DEFAULT_ENC2_DIN,
                              DEFAULT_ENC2_DOUT, DEFAULT_ENC2_SEL};
  cfg1 = def1;
  cfg2 = def2;

  Serial.println(
      F("\n╔══════════════════════════════════════════════════════════╗"));
  Serial.println(
      F("║         AEAT-8800-Q24  —  Encoder Setup Wizard          ║"));
  Serial.println(
      F("╠══════════════════════════════════════════════════════════╣"));
  Serial.println(
      F("║  GPIO reference:                                         ║"));
  Serial.println(
      F("║    D0=0  D1=1  D2=2  D3=21  D4=22  D5=23               ║"));
  Serial.println(
      F("║    D6=16 D7=17 D8=19 D9=20  D10=18                     ║"));
  Serial.println(
      F("╠══════════════════════════════════════════════════════════╣"));
  Serial.println(
      F("║  Default pin layout (fully independent, no sharing):    ║"));
  Serial.println(
      F("║    ENC1: CLK=19(D8)  DIN=18(D10) DO=20(D9) SEL=21(D3) ║"));
  Serial.println(
      F("║    ENC2: CLK=23(D5)  DIN=2(D2)   DO=1(D1)  SEL=22(D4) ║"));
  Serial.println(
      F("╚══════════════════════════════════════════════════════════╝\n"));

  // ── Step 1: number of encoders
  // ────────────────────────────────────────────────
  uint8_t numEnc = 0;
  while (numEnc != 1 && numEnc != 2) {
    Serial.print(F("How many encoders are connected? [1/2]: "));
    String s = readLine();
    numEnc = (uint8_t)s.toInt();
    if (numEnc != 1 && numEnc != 2)
      Serial.println(F("  Please enter 1 or 2."));
  }

  if (numEnc == 1) {
    // ── Step 2 (single encoder): which slot
    // ────────────────────────────────────
    uint8_t slot = 0;
    while (slot != 1 && slot != 2) {
      Serial.print(F("Which encoder slot is connected? [1/2]: "));
      String s = readLine();
      slot = (uint8_t)s.toInt();
      if (slot != 1 && slot != 2)
        Serial.println(F("  Please enter 1 or 2."));
    }
    g_enc1Enabled = (slot == 1);
    g_enc2Enabled = (slot == 2);
    g_activeEnc = slot;
  } else {
    g_enc1Enabled = true;
    g_enc2Enabled = true;
    g_activeEnc = 1;
  }

  // ── Step 3: optional custom pins per enabled encoder
  // ──────────────────────────
  if (g_enc1Enabled)
    promptCustomPins("1", cfg1, def1);
  if (g_enc2Enabled)
    promptCustomPins("2", cfg2, def2);

  // ── Configuration summary
  // ─────────────────────────────────────────────────────
  Serial.println(
      F("\n╔══════════════════════════════════════════════════════════╗"));
  Serial.println(
      F("║  Configuration confirmed:                                ║"));
  if (g_enc1Enabled)
    Serial.printf(
        "║  ENC1  CLK=GPIO%-2u  DIN=GPIO%-2u  DO=GPIO%-2u  SEL=GPIO%-2u  ║\n",
        cfg1.pinCLK, cfg1.pinDIN, cfg1.pinDOUT, cfg1.pinSEL);
  else
    Serial.println(
        F("║  ENC1  disabled                                          ║"));
  if (g_enc2Enabled)
    Serial.printf(
        "║  ENC2  CLK=GPIO%-2u  DIN=GPIO%-2u  DO=GPIO%-2u  SEL=GPIO%-2u  ║\n",
        cfg2.pinCLK, cfg2.pinDIN, cfg2.pinDOUT, cfg2.pinSEL);
  else
    Serial.println(
        F("║  ENC2  disabled                                          ║"));
  Serial.println(
      F("╚══════════════════════════════════════════════════════════╝\n"));
}

/** Apply built-in dual-encoder pin config and enable continuous stream. */
static void applyDefaultDualConfig(EncoderConfig &cfg1, EncoderConfig &cfg2) {
  cfg1 = {DEFAULT_ENC1_CLK, DEFAULT_ENC1_DIN, DEFAULT_ENC1_DOUT,
          DEFAULT_ENC1_SEL};
  cfg2 = {DEFAULT_ENC2_CLK, DEFAULT_ENC2_DIN, DEFAULT_ENC2_DOUT,
          DEFAULT_ENC2_SEL};
  g_enc1Enabled = true;
  g_enc2Enabled = true;
  g_activeEnc = 1;
  g_streaming = true;
}

// ─── Pin Init
// ─────────────────────────────────────────────────────────────────

static void initEncoderPins(const EncoderConfig &cfg) {
  pinMode(cfg.pinCLK, OUTPUT);
  digitalWrite(cfg.pinCLK, HIGH); // CLK idles HIGH
  pinMode(cfg.pinDIN, OUTPUT);
  digitalWrite(cfg.pinDIN, HIGH); // DIN/NSL idles HIGH
  pinMode(cfg.pinDOUT, INPUT);
  pinMode(cfg.pinSEL, OUTPUT);
  digitalWrite(cfg.pinSEL, LOW); // SPI/idle mode
}

// ─── Runtime Helpers ─────────────────────────────────────────────────────────

/** Snap a fractional count to the nearest integer encoder step [0, CPR). */
static uint16_t snapToCounts(double counts) {
  long c = (long)(counts + 0.5);
  c %= ENCODER_CPR;
  if (c < 0)
    c += ENCODER_CPR;
  return (uint16_t)c;
}

/** Convert encoder counts to degrees, quantized to 3 decimal places. */
static double countsToDeg(uint16_t counts) {
  return round(counts * DEG_PER_COUNT * 1000.0) / 1000.0;
}

/** Wrap deg into [0, 360) and round to 3 decimal places. */
static double normalizeDeg3(double deg) {
  deg = fmod(deg, 360.0);
  if (deg < 0.0)
    deg += 360.0;
  return round(deg * 1000.0) / 1000.0;
}

/** Snap filtered counts → integer counts → degrees (no offset). */
static double rawDisplayDeg(double filteredCounts) {
  return countsToDeg(snapToCounts(filteredCounts));
}

/** Return the active encoder pointer, or nullptr if that encoder is disabled.
 */
static AEAT8800 *activeEncoder() {
  if (g_activeEnc == 1 && g_enc1Enabled)
    return g_enc1;
  if (g_activeEnc == 2 && g_enc2Enabled)
    return g_enc2;
  return nullptr;
}

static const char *resolutionStr(uint8_t regVal) {
  switch (regVal & 0x03) {
  case 0x00:
    return "10-bit (1024 cpr)";
  case 0x01:
    return "12-bit (4096 cpr)";
  case 0x02:
    return "14-bit (16384 cpr)";
  case 0x03:
    return "16-bit (65536 cpr)";
  default:
    return "unknown";
  }
}

static void printSinglePosition(uint8_t id, double pos, bool ok) {
  if (ok) {
    uint16_t snapped = snapToCounts(pos);
    double deg = displayDeg(id, pos);
    Serial.printf("[ENC%u] Pos=%5u  Angle=%8.3f deg\n", id, snapped, deg);
  } else {
    Serial.printf("[ENC%u] READ ERROR\n", id);
  }
}

// ─── Hybrid Circular Filtering Algorithms ───────────────────────────────────

static double getCircularRobustMean(uint16_t *samples, int size) {
  if (size == 0)
    return 0;

  double sum_x = 0;
  double sum_y = 0;
  for (int i = 0; i < size; i++) {
    double theta = (samples[i] / ENCODER_CPR_D) * 2.0 * PI;
    sum_x += cos(theta);
    sum_y += sin(theta);
  }
  double initialMeanTheta = atan2(sum_y, sum_x);
  if (initialMeanTheta < 0)
    initialMeanTheta += 2.0 * PI;
  uint16_t initialMeanCnt =
      (uint16_t)((initialMeanTheta / (2.0 * PI)) * ENCODER_CPR_D);

  double robustSum_x = 0;
  double robustSum_y = 0;
  int count = 0;
  for (int i = 0; i < size; i++) {
    // Calculate circular distance
    uint16_t diff = samples[i] - initialMeanCnt;
    uint16_t dist = (diff > 32768) ? (ENCODER_CPR - diff) : diff;

    if (dist < OUTLIER_THRESHOLD) {
      double theta = (samples[i] / ENCODER_CPR_D) * 2.0 * PI;
      robustSum_x += cos(theta);
      robustSum_y += sin(theta);
      count++;
    }
  }

  // If everything was an outlier (e.g. extremely noisy), fallback to standard
  // circular mean
  if (count == 0) {
    return (double)initialMeanCnt;
  }

  double robustMeanTheta = atan2(robustSum_y, robustSum_x);
  if (robustMeanTheta < 0)
    robustMeanTheta += 2.0 * PI;
  return (robustMeanTheta / (2.0 * PI)) * ENCODER_CPR_D;
}

static double getCircularMedian(double *values, int size) {
  if (size == 0)
    return 0;

  // Unwrap all block means relative to the first block mean to handle
  // wrap-around
  double unwrapped[NUM_BLOCKS];
  unwrapped[0] = values[0];

  for (int i = 1; i < size; i++) {
    double diff = values[i] - values[0];
    if (diff > 32768.0)
      diff -= ENCODER_CPR_D;
    else if (diff < -32768.0)
      diff += ENCODER_CPR_D;
    unwrapped[i] = values[0] + diff;
  }

  std::sort(unwrapped, unwrapped + size);
  double med = unwrapped[size / 2];

  // Wrap back to [0, CPR)
  if (med < 0)
    med += ENCODER_CPR_D;
  else if (med >= ENCODER_CPR_D)
    med -= ENCODER_CPR_D;

  return med;
}

static bool readAngleFiltered(AEAT8800 *enc, double &finalPos) {
  if (!enc)
    return false;

  double blockMeans[NUM_BLOCKS];
  uint16_t blockSamples[SAMPLES_PER_BLOCK];

  enc->select();
  int validBlocks = 0;

  for (int b = 0; b < NUM_BLOCKS; b++) {
    int validSamples = 0;
    for (int s = 0; s < SAMPLES_PER_BLOCK; s++) {
      uint16_t raw;
      if (enc->readAngleRaw(raw, true)) { // silent = true
        blockSamples[validSamples++] = raw;
      }
    }
    if (validSamples > 0) {
      blockMeans[validBlocks++] =
          getCircularRobustMean(blockSamples, validSamples);
    }
  }
  enc->deselect();

  if (validBlocks == 0)
    return false;

  finalPos = getCircularMedian(blockMeans, validBlocks);
  return true;
}

// ─── Read Both Enabled Encoders
// ───────────────────────────────────────────────

static void readBothEnabled(double &pos1, double &pos2, bool &ok1, bool &ok2) {
  pos1 = 0;
  pos2 = 0;
  ok1 = false;
  ok2 = false;

  if (g_enc1Enabled) {
    ok1 = readAngleFiltered(g_enc1, pos1);
    if (g_enc2Enabled)
      delay(1); // brief gap before reading enc2
  }
  if (g_enc2Enabled) {
    ok2 = readAngleFiltered(g_enc2, pos2);
  }
}

/** Snap filtered counts, apply software offset, normalize to [0,360), 3 dp. */
static double displayDeg(uint8_t encId, double filteredCounts) {
  double raw = rawDisplayDeg(filteredCounts);
  double off = (encId == 1) ? g_offsetDeg1 : g_offsetDeg2;

  
  return normalizeDeg3(raw - off);
}

// ─── Register Dump
// ────────────────────────────────────────────────────────────

static void printAllRegisters() {
  AEAT8800 *enc = activeEncoder();
  if (!enc) {
    Serial.printf("[ERR] ENC%u is not enabled.\n", g_activeEnc);
    return;
  }

  Serial.printf("\n╔══════════════════════════════════════════════╗\n");
  Serial.printf("║  AEAT-8800-Q24 — ENC%u SPI Register Dump     ║\n",
                g_activeEnc);
  Serial.println(F("╠══════════════════════════════════════════════╣"));

  struct {
    uint8_t addr;
    const char *name;
  } regs[] = {
      {REG_CUST_RESERVE_0, "CustReserve0 (0x00)"},
      {REG_CUST_RESERVE_1, "CustReserve1 (0x01)"},
      {REG_ZERO_POS_L, "ZeroPos_L    (0x02)"},
      {REG_ZERO_POS_H, "ZeroPos_H    (0x03)"},
      {REG_CUST_CONFIG_0, "CustConfig0  (0x04)"},
      {REG_CPR_SET1, "CPR_Set1     (0x05)"},
      {REG_CPR_SET2, "CPR_Set2     (0x06)"},
      {REG_RESOLUTION, "Resolution   (0x07)"},
      {REG_LOCK, "Lock         (0x10)"},
  };

  for (auto &r : regs) {
    uint8_t val = enc->spiRead(r.addr);
    Serial.printf("║  %-22s  0x%02X  (%3u) ║\n", r.name, val, val);
  }

  uint8_t lo = enc->spiRead(REG_ZERO_POS_L);
  uint8_t hi = enc->spiRead(REG_ZERO_POS_H);
  uint16_t zp = ((uint16_t)hi << 8) | lo;
  uint8_t resReg = enc->spiRead(REG_RESOLUTION);

  Serial.println(F("╠══════════════════════════════════════════════╣"));
  Serial.printf("║  HW Zero Pos : %5u  (%.3f deg)          ║\n", zp,
                countsToDeg(zp));
  Serial.printf("║  Resolution  : %-30s ║\n", resolutionStr(resReg));
  Serial.println(F("╚══════════════════════════════════════════════╝\n"));
}

// ─── Help
// ─────────────────────────────────────────────────────────────────────

static void printHelp() {
  Serial.println(
      F("\n╔══════════════════════════════════════════════════════╗"));
  Serial.println(F("║  AEAT-8800-Q24  Independent Dual  —  Commands        ║"));
  Serial.println(F("╠══════════════════════════════════════════════════════╣"));
  Serial.println(F("║  p           — Read enabled encoders (SSI, one-shot) ║"));
  Serial.println(F("║  c           — Toggle continuous stream (100 ms)     ║"));
  Serial.println(F("║  o <deg>     — Offset: current pos reports as <deg>  ║"));
  Serial.println(F("║  o clear     — Remove software offset (back to raw)  ║"));
  Serial.println(F("║  1 / 2       — Select active encoder for SPI cmds    ║"));
  Serial.println(F("║  s           — Dump SPI registers (active encoder)   ║"));
  Serial.println(F("║  r <hh>      — Read register  e.g. 'r 07'            ║"));
  Serial.println(F("║  w <hh> <vv> — Write register e.g. 'w 10 AB'         ║"));
  Serial.println(F("║  u           — Unlock config registers               ║"));
  Serial.println(F("║  z           — HW zero position = 0x0000 (SPI)       ║"));
  Serial.println(F("║  burn        — Burn OTP  ⚠ IRREVERSIBLE              ║"));
  Serial.println(
      F("║  h           — Show this help                         ║"));
  Serial.println(
      F("╚══════════════════════════════════════════════════════╝\n"));
}

// ─── Serial Command Parser
// ────────────────────────────────────────────────────

static void handleSerial() {
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0)
    return;

  // ── Software angle offset  "o [deg]"  or  "o clear" ─────────────────────
  if (line.length() >= 1 &&
      tolower((unsigned char)line[0]) == 'o' &&
      (line.length() == 1 || line[1] == ' ')) {
    String arg = (line.length() > 2) ? line.substring(2) : String("");
    arg.trim();
    arg.toLowerCase();

    if (arg == "clear") {
      g_offsetDeg1 = 0.0;
      g_offsetDeg2 = 0.0;
      Serial.println(F("[OFFSET] Cleared — both encoders report raw hardware angles"));
    } else {
      double targetDeg = (arg.length() > 0) ? arg.toDouble() : 0.0;
      double pos1 = 0, pos2 = 0;
      bool ok1 = false, ok2 = false;
      readBothEnabled(pos1, pos2, ok1, ok2);

      if (g_enc1Enabled) {
        if (ok1) {
          double raw1 = rawDisplayDeg(pos1);
          g_offsetDeg1 = raw1 - targetDeg;
          Serial.printf("[OFFSET] ENC1 raw=%.3f → reports as %.3f  (offset=%.3f)\n",
                        raw1, targetDeg, g_offsetDeg1);
        } else {
          Serial.println(F("[OFFSET] ENC1 read failed — offset unchanged"));
        }
      }
      if (g_enc2Enabled) {
        if (ok2) {
          double raw2 = rawDisplayDeg(pos2);
          g_offsetDeg2 = raw2 - targetDeg;
          Serial.printf("[OFFSET] ENC2 raw=%.3f → reports as %.3f  (offset=%.3f)\n",
                        raw2, targetDeg, g_offsetDeg2);
        } else {
          Serial.println(F("[OFFSET] ENC2 read failed — offset unchanged"));
        }
      }
    }
    return;
  }

  // OTP burn — requires the exact string "burn" to prevent accidents
  if (line.equalsIgnoreCase("burn")) {
    AEAT8800 *enc = activeEncoder();
    if (!enc) {
      Serial.printf("[ERR] ENC%u is not enabled.\n", g_activeEnc);
      return;
    }
    Serial.printf("[OTP] Unlocking ENC%u and burning shadow regs to OTP...\n",
                  g_activeEnc);
    enc->spiWrite(REG_LOCK, UNLOCK_KEY);
    delayMicroseconds(10);
    enc->spiWrite(REG_PROG_CUST, PROG_KEY);
    Serial.println(F("[OTP] Done. Power-cycle the encoder to verify."));
    return;
  }

  char cmd = (char)tolower((unsigned char)line[0]);

  switch (cmd) {

  // ── Read both enabled encoders (SSI) ─────────────────────────────────────
  case 'p': {
    double pos1 = 0, pos2 = 0;
    bool ok1 = false, ok2 = false;
    readBothEnabled(pos1, pos2, ok1, ok2);
    if (g_enc1Enabled)
      printSinglePosition(1, pos1, ok1);
    if (g_enc2Enabled)
      printSinglePosition(2, pos2, ok2);
    break;
  }

  // ── Continuous stream toggle ──────────────────────────────────────────────
  case 'c':
    g_streaming = !g_streaming;
    Serial.printf("[INFO] Continuous stream %s\n", g_streaming ? "ON" : "OFF");
    break;

  // ── Select active encoder for SPI commands ────────────────────────────────
  case '1':
    if (!g_enc1Enabled) {
      Serial.println(F("[ERR] ENC1 is not enabled."));
      break;
    }
    g_activeEnc = 1;
    Serial.println(F("[INFO] Active encoder: ENC1"));
    break;
  case '2':
    if (!g_enc2Enabled) {
      Serial.println(F("[ERR] ENC2 is not enabled."));
      break;
    }
    g_activeEnc = 2;
    Serial.println(F("[INFO] Active encoder: ENC2"));
    break;

  // ── Register dump ─────────────────────────────────────────────────────────
  case 's':
    printAllRegisters();
    break;

  // ── Read single register  "r <hex-addr>" ─────────────────────────────────
  case 'r': {
    if (line.length() < 3) {
      Serial.println(F("[ERR] Usage: r <hex-addr>  e.g. 'r 07'"));
      break;
    }
    AEAT8800 *enc = activeEncoder();
    if (!enc) {
      Serial.printf("[ERR] ENC%u is not enabled.\n", g_activeEnc);
      break;
    }
    uint8_t addr = (uint8_t)strtoul(line.c_str() + 2, nullptr, 16);
    uint8_t val = enc->spiRead(addr);
    Serial.printf("[ENC%u SPI-RD] Reg 0x%02X = 0x%02X  (%u)\n", g_activeEnc,
                  addr, val, val);
    break;
  }

  // ── Write single register  "w <hex-addr> <hex-val>" ──────────────────────
  case 'w': {
    if (line.length() < 5) {
      Serial.println(F("[ERR] Usage: w <hex-addr> <hex-val>  e.g. 'w 10 AB'"));
      break;
    }
    AEAT8800 *enc = activeEncoder();
    if (!enc) {
      Serial.printf("[ERR] ENC%u is not enabled.\n", g_activeEnc);
      break;
    }
    char *ptr = nullptr;
    uint8_t addr = (uint8_t)strtoul(line.c_str() + 2, &ptr, 16);
    uint8_t val = (uint8_t)strtoul(ptr, nullptr, 16);
    enc->spiWrite(addr, val);
    uint8_t rb = enc->spiRead(addr);
    Serial.printf(
        "[ENC%u SPI-WR] Reg 0x%02X <- 0x%02X  |  Readback: 0x%02X  %s\n",
        g_activeEnc, addr, val, rb, (rb == val) ? "OK" : "MISMATCH!");
    break;
  }

  // ── Unlock registers ──────────────────────────────────────────────────────
  case 'u': {
    AEAT8800 *enc = activeEncoder();
    if (!enc) {
      Serial.printf("[ERR] ENC%u is not enabled.\n", g_activeEnc);
      break;
    }
    enc->spiWrite(REG_LOCK, UNLOCK_KEY);
    Serial.printf("[ENC%u SPI] Registers unlocked (0xAB → Lock reg)\n",
                  g_activeEnc);
    break;
  }

  // ── Set zero position ─────────────────────────────────────────────────────
  case 'z': {
    AEAT8800 *enc = activeEncoder();
    if (!enc) {
      Serial.printf("[ERR] ENC%u is not enabled.\n", g_activeEnc);
      break;
    }
    Serial.printf("[ENC%u ZERO] Writing 0x0000 to ZeroPos shadow regs...\n",
                  g_activeEnc);
    enc->spiWrite(REG_LOCK, UNLOCK_KEY);
    delayMicroseconds(10);
    enc->spiWrite(REG_ZERO_POS_L, 0x00);
    enc->spiWrite(REG_ZERO_POS_H, 0x00);
    uint8_t lo = enc->spiRead(REG_ZERO_POS_L);
    uint8_t hi = enc->spiRead(REG_ZERO_POS_H);
    uint16_t zp = ((uint16_t)hi << 8) | lo;
    Serial.printf("[ENC%u ZERO] Readback: 0x%04X — %s\n", g_activeEnc, zp,
                  zp == 0x0000 ? "OK" : "MISMATCH!");
    break;
  }

  // ── Burn guard (must type "burn" exactly) ─────────────────────────────────
  case 'b':
    Serial.println(F("[WARN] OTP BURN is IRREVERSIBLE!"));
    Serial.println(F("[WARN] Type exactly 'burn' and press Enter to confirm."));
    break;

  // ── Help ──────────────────────────────────────────────────────────────────
  case 'h':
  default:
    printHelp();
    break;
  }
}

// ─── Setup
// ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(300);

  // ── External antenna
  // ───────────────────────────────────────────────────────── GPIO3 LOW
  // activates the RF switch; GPIO14 HIGH selects the external antenna.
  pinMode(WIFI_ENABLE, OUTPUT);
  digitalWrite(WIFI_ENABLE, LOW);
  delay(100);
  pinMode(WIFI_ANT_CONFIG, OUTPUT);
  digitalWrite(WIFI_ANT_CONFIG, HIGH);

  // ── Encoder configuration (wizard or defaults)
  // ─────────────────────────────────────────────
  EncoderConfig cfg1, cfg2;
  if (RUN_SETUP_WIZARD) {
    runSetupWizard(cfg1, cfg2);
  } else {
    applyDefaultDualConfig(cfg1, cfg2);
    Serial.println(F(
        "[INFO] Setup wizard skipped — ENC1+ENC2 on default pins, stream ON"));
  }

  // ── GPIO init for enabled encoders
  // ───────────────────────────────────────────
  if (g_enc1Enabled)
    initEncoderPins(cfg1);
  if (g_enc2Enabled)
    initEncoderPins(cfg2);

  // tPwrUp ~4 ms (datasheet p.6) + NSL must be HIGH >= 3 ms before first SSI
  // read
  delay(5);

  // ── Instantiate encoder objects
  // ───────────────────────────────────────────────
  if (g_enc1Enabled)
    g_enc1 = new AEAT8800(cfg1.pinCLK, cfg1.pinDIN, cfg1.pinDOUT, cfg1.pinSEL);
  if (g_enc2Enabled)
    g_enc2 = new AEAT8800(cfg2.pinCLK, cfg2.pinDIN, cfg2.pinDOUT, cfg2.pinSEL);

  // ── SPI configuration
  // ────────────────────────────────────────────────────────
  Serial.println(
      F("╔══════════════════════════════════════════════════════════╗"));
  Serial.println(
      F("║  AEAT-8800-Q24  Independent  |  XIAO ESP32-C6           ║"));
  Serial.println(
      F("╠══════════════════════════════════════════════════════════╣"));

  if (g_enc1Enabled) {
    Serial.print(F("║  Configuring ENC1 ... "));
    bool ok = g_enc1->configure();
    Serial.println(ok ? F("OK                                       ║")
                      : F("FAILED — SSI reads will still be tried   ║"));
  }
  if (g_enc2Enabled) {
    Serial.print(F("║  Configuring ENC2 ... "));
    bool ok = g_enc2->configure();
    Serial.println(ok ? F("OK                                       ║")
                      : F("FAILED — SSI reads will still be tried   ║"));
  }

  Serial.println(
      F("╠══════════════════════════════════════════════════════════╣"));

  // ── Initial SSI position read
  // ─────────────────────────────────────────────────
  double filt1 = 0, filt2 = 0;
  bool r1 = false, r2 = false;
  readBothEnabled(filt1, filt2, r1, r2);

  if (g_enc1Enabled) {
    if (r1)
      Serial.printf("║  ENC1 Position : %5u counts  (%.3f deg)           ║\n",
                    (unsigned)(filt1 + 0.5), countsToDeg((uint16_t)(filt1 + 0.5)));
    else
      Serial.println(
          F("║  ENC1 Position : READ ERROR                              ║"));
  }
  if (g_enc2Enabled) {
    if (r2)
      Serial.printf("║  ENC2 Position : %5u counts  (%.3f deg)           ║\n",
                    (unsigned)(filt2 + 0.5), countsToDeg((uint16_t)(filt2 + 0.5)));
    else
      Serial.println(
          F("║  ENC2 Position : READ ERROR                              ║"));
  }

  Serial.println(
      F("╠══════════════════════════════════════════════════════════╣"));
  if (!RUN_SETUP_WIZARD) {
    Serial.println(
        F("║  Continuous stream ON  (send 'c' to toggle)            ║"));
  }
  Serial.println(
      F("║  Type 'h' for command help                               ║"));
  Serial.println(
      F("╚══════════════════════════════════════════════════════════╝\n"));
}

// ─── Main Loop
// ────────────────────────────────────────────────────────────────

void loop() {
  handleSerial();

  if (g_streaming && (millis() - g_lastStream >= 33)) {
    g_lastStream = millis();

    double pos1 = 0, pos2 = 0;
    bool ok1 = false, ok2 = false;
    readBothEnabled(pos1, pos2, ok1, ok2);

    if (g_enc1Enabled && g_enc2Enabled) {
      float d1 = ok1 ? (float)displayDeg(1, pos1) : -1.0f;
      float d2 = ok2 ? (float)displayDeg(2, pos2) : -1.0f;
      if (ok1 && ok2)
        Serial.printf("ENC1: %.3f deg | ENC2: %.3f deg\n", d1, d2);
      else if (!ok1 && !ok2)
        Serial.println(F("ENC1: ERR | ENC2: ERR"));
      else if (!ok1)
        Serial.printf("ENC1: ERR | ENC2: %.3f deg\n", d2);
      else
        Serial.printf("ENC1: %.3f deg | ENC2: ERR\n", d1);
    } else if (g_enc1Enabled) {
      if (ok1)
        Serial.printf("ENC1: %.3f deg\n", displayDeg(1, pos1));
      else
        Serial.println(F("ENC1: ERR"));
    } else if (g_enc2Enabled) {
      if (ok2)
        Serial.printf("ENC2: %.3f deg\n", displayDeg(2, pos2));
      else
        Serial.println(F("ENC2: ERR"));
    }
  }
}
