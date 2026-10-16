/*
 * TLE5012B_ErrorCodes_C3.ino
 * ============================================================================
 * TLE5012B GMR angle sensor: continuous angle readout with live error codes.
 * Board: Seeed Studio XIAO ESP32C3.
 *
 * WHAT IT DOES
 *   - Reads the angle (AVAL) as fast as possible and checks every transfer
 *     (CRC, RESP nibble, Safety Word).
 *   - Every 10 ms it checks the magnet amplitude (ADC_X/ADC_Y).
 *   - Every 200 ms it reads STAT, temperature, D_MAG, T_RAW and MOD_1/2/3
 *     and runs the remaining checks (temperature, configuration).
 *   - Prints raw and filtered angle (1000-sample average, same method as
 *     GMR_SMA_C3) plus every error event seen since the previous print
 *     and every condition still active
 *     (magnet, temperature, configuration). Three print modes, see SERIAL
 *     COMMANDS. Full guide: GMR_Error_Codes/TLE5012B_Error_Codes_Guide.md
 *   - Runs a decoder self-test at boot (no sensor needed) and on 't'.
 *   - Only reads from the sensor. It never writes to a sensor register.
 *
 * WIRING (same pins as GMR_SMA_C3.ino)
 *   TLE5012B         XIAO ESP32C3
 *   VDD   --------   3V3
 *   GND   --------   GND
 *   CSQ   --------   D2  (GPIO4)
 *   SCK   --------   D5  (GPIO7)
 *   DATA  --470R---  D4  (GPIO6, MOSI)
 *   DATA  --470R---  D10 (GPIO10, MISO)
 *   The series resistors limit current while the sensor and MCU share the
 *   bidirectional DATA line [DS 3.7].
 *
 * ARDUINO IDE SETTINGS
 *   Board:           XIAO_ESP32C3 (esp32 by Espressif Systems)
 *   USB CDC On Boot: Enabled
 *   Serial Monitor:  115200 baud, any line ending
 *
 * SERIAL COMMANDS (type the character, press Enter)
 *   1  line mode       10 per second, one line per print
 *   2  dashboard mode   1 per second, full status block
 *   3  table mode       5 per second, aligned columns
 *   d  register dump with decoded fields
 *   e  list of all error codes with meaning, action and source
 *   t  rerun the decoder self-test
 *   a  interface-access test (reads MOD_2 with a wrong lock)
 *   r  reset error counters
 *   h  help
 *
 * SOURCES (tags used in all comments)
 *   [DS]   Infineon TLE5012B Data Sheet Rev 2.1
 *   [LIB]  Infineon TLE5012B Arduino library v3.1.0
 *   [PLAN] GMR_Error_Codes/GMR_TLE5012B_Error_Diagnostics_and_Test_Plan.md
 *   [HEC]  GMR_Error_Codes/(Description) Hallever Error Codes.pdf
 *   [XIAO] Seeed Studio XIAO ESP32C3 pinout
 *   ESTIMATE = no documented value exists; chosen as a reasonable default.
 *   Full references are in tle5012b_errors.h.
 * ============================================================================
 */

#include <math.h>
#include "tle5012b_errors.h"
#include "tle5012b_ssc.h"

// ============================================================================
// USER SETTINGS
// ============================================================================

// Pins [XIAO], copied from GMR_SMA_C3.ino
#define CS_PIN    D2    // GPIO4
#define MOSI_PIN  D4    // GPIO6
#define MISO_PIN  D10   // GPIO10
#define SCK_PIN   D5    // GPIO7

const uint32_t SSC_CLOCK_HZ = 1000000;   // 1 MHz, well below the 8 Mbit/s max [DS Table 15]
const uint8_t  FRAME_RETRIES = 2;        // retries on CRC/RESP/no-response [PLAN Phase D]

// 1000-sample average of the angle, same window as GMR_SMA_C3.ino.
// Each sample is unwrapped (shortest step from the previous one) before it
// is averaged, so a window that crosses 0/360 is not pulled toward 180.
const uint16_t SMA_WINDOW = 1000;

// Magnet amplitude sqrt(ADC_X^2 + ADC_Y^2), digits at ambient [DS Table 10] SOURCED
const float MAG_MIN_DIGITS = 6000.0f;
const float MAG_MAX_DIGITS = 15781.0f;

// Hysteresis: MAG_WEAK clears only above MIN + HYST, MAG_STRONG only below
// MAX - HYST, so a noisy amplitude near a limit does not flicker.
// ESTIMATE (~2 % of the 9500-digit typical amplitude [DS Table 10]).
const float MAG_HYST_DIGITS = 200.0f;

// Magnet check period. Runs separately from the 200 ms slow poll so the state
// is fresh on every print. One ADC_X/ADC_Y read takes ~60 us at 1 MHz.
// ESTIMATE.
const uint32_t MAG_POLL_MS = 10;

// Temperature limits, deg C
const float TEMP_MIN_C = -40.0f;         // lower junction limit [DS Table 4] SOURCED
const float TEMP_MAX_C = 125.0f;         // ESTIMATE (DS Table 4 junction max is 150)

// Stale angle: RD_AV not set for this long. Angle updates every
// 42.7-170.6 us [DS Table 12]; 10 ms is ESTIMATE (~60x the slowest update).
const uint32_t STALE_ANGLE_MS = 10;

// Angle jump: more than JUMP_MAX_DEG between valid samples taken within
// JUMP_WINDOW_MS. ESTIMATE, set for a hand-turned or slow motor shaft.
const float    JUMP_MAX_DEG   = 20.0f;
const uint32_t JUMP_WINDOW_MS = 10;

// Timing
const uint32_t POWER_ON_WAIT_MS   = 10;    // power-on time tPon max 7 ms [DS Table 4]
const uint32_t SLOW_POLL_MS       = 200;
const uint32_t LINE_PERIOD_MS     = 100;   // 10 per second
const uint32_t DASH_PERIOD_MS     = 1000;  // 1 per second
const uint32_t TABLE_PERIOD_MS    = 200;   // 5 per second
const uint16_t TABLE_HEADER_EVERY = 20;    // rows between repeated headers

// Register compare masks for the configuration baseline [LIB tle5012b_reg.cpp]
const uint16_t MOD1_COMPARE_MASK = 0xC017;   // FIR_MD, CLK_SEL, DSPU_HOLD, IIF_MOD
const uint16_t MOD2_COMPARE_MASK = 0x7FFF;   // ANG_RANGE, ANG_DIR, PREDICT, AUTOCAL
const uint16_t MOD3_COMPARE_MASK = 0xFFFF;   // ANG_BASE, SPIKEF, SSC_OD, PAD_DRV

// ============================================================================
// STATE
// ============================================================================

const uint8_t MODE_LINE  = 1;
const uint8_t MODE_DASH  = 2;
const uint8_t MODE_TABLE = 3;

uint8_t  g_mode = MODE_LINE;
uint32_t g_lastPrintMs = 0;
uint32_t g_lastSlowMs  = 0;
uint32_t g_lastMagMs   = 0;
uint16_t g_tableRows   = 0;

// Fast path (angle)
uint16_t g_rawCount      = 0;
float    g_rawDeg        = 0.0f;
bool     g_rawValid      = false;   // sensor reports a valid angle
bool     g_rawTrusted    = false;   // valid and magnet inside its limits
uint16_t g_lastSafety    = 0;
uint32_t g_lastNewAngleMs = 0;
float    g_prevValidDeg  = 0.0f;
uint32_t g_prevValidMs   = 0;
bool     g_havePrevValid = false;

// Filter. The buffer holds unwrapped degrees; the sum is the same straight
// average GMR_SMA_C3.ino uses.
double   g_smaBuf[SMA_WINDOW];
double   g_smaSum = 0.0;
uint16_t g_smaCount = 0;
uint16_t g_smaIndex = 0;
float    g_smaPrevDeg = 0.0f;
double   g_smaUnwrapped = 0.0;
bool     g_smaHavePrev = false;

// Slow path (diagnostics)
uint16_t g_stat     = 0;
bool     g_statRead = false;
float    g_tempC    = NAN;
int16_t  g_adcX     = 0;
int16_t  g_adcY     = 0;
float    g_magDigits = NAN;
uint16_t g_dmag     = 0;
uint16_t g_traw     = 0;
uint16_t g_mod[3]     = {0, 0, 0};
uint16_t g_modBase[3] = {0, 0, 0};
bool     g_modBaseSet = false;

// Error bookkeeping
uint32_t g_errSincePrint = 0;   // events seen since the last print
uint32_t g_conditions    = 0;   // active states (ERR_CONDITION_MASK)
uint32_t g_errCount[ERR_CODE_COUNT];
uint32_t g_samples = 0;
uint32_t g_validSamples = 0;
uint32_t g_retries = 0;
uint32_t g_samplesAtLastPrint = 0;
uint32_t g_msAtLastRate = 0;
float    g_sampleRate = 0.0f;

// ============================================================================
// ERROR BOOKKEEPING
// ============================================================================

void recordErrors(uint32_t mask)
{
    g_errSincePrint |= mask;
    for (uint8_t bit = 0; bit < ERR_CODE_COUNT; bit++) {
        if (mask & errBit(bit)) g_errCount[bit]++;
    }
}

// Updates the condition bits in `which` to the state in `active`. Only a
// condition becoming active is counted, not every check while it stays active.
void setConditions(uint32_t which, uint32_t active)
{
    uint32_t next = (g_conditions & ~which) | (active & which);
    recordErrors(next & ~g_conditions);
    g_conditions = next;
}

// Everything to show in a print: events since the last print plus every
// condition that is still active.
uint32_t shownErrors()
{
    return g_errSincePrint | g_conditions;
}

void resetCounters()
{
    for (uint8_t bit = 0; bit < ERR_CODE_COUNT; bit++) g_errCount[bit] = 0;
    g_errSincePrint = 0;
    g_samples = 0;
    g_validSamples = 0;
    g_retries = 0;
    g_samplesAtLastPrint = 0;
    g_msAtLastRate = millis();
}

// Read with retry on frame errors [PLAN Phase D]. Errors of retried attempts
// are recorded here; the caller records the errors of the returned frame.
SscFrame readWithRetry(uint16_t command)
{
    SscFrame f;
    for (uint8_t attempt = 0; attempt <= FRAME_RETRIES; attempt++) {
        if (attempt > 0) g_retries++;
        f = sscRead(command);
        if (f.frameOk() || attempt == FRAME_RETRIES) break;
        recordErrors(f.errors & ERR_FRAME_MASK);
    }
    return f;
}

// Reads STAT (this also clears its latched bits [DS Table 18]) and returns
// frame, Safety Word and decoded STAT errors.
uint32_t readStatus()
{
    SscFrame f = readWithRetry(CMD_READ_STAT);
    if (!f.frameOk()) return f.errors;
    g_stat     = f.data[0];
    g_statRead = true;
    return f.errors | decodeStat(g_stat);
}

// ============================================================================
// FILTER
// ============================================================================

void smaAdd(float deg)
{
    // Shortest step from the previous sample, so 359 -> 1 counts as +2
    // and not as a jump across the circle.
    if (!g_smaHavePrev) {
        g_smaUnwrapped = deg;
        g_smaHavePrev = true;
    } else {
        g_smaUnwrapped += angleDelta(deg, g_smaPrevDeg);
    }
    g_smaPrevDeg = deg;

    if (g_smaCount == SMA_WINDOW) g_smaSum -= g_smaBuf[g_smaIndex];
    else g_smaCount++;

    g_smaBuf[g_smaIndex] = g_smaUnwrapped;
    g_smaSum += g_smaUnwrapped;
    g_smaIndex = (g_smaIndex + 1) % SMA_WINDOW;
}

void smaClear()
{
    g_smaSum = 0.0;
    g_smaCount = 0;
    g_smaIndex = 0;
    g_smaHavePrev = false;
    g_smaUnwrapped = 0.0;
}

float smaDegrees()
{
    if (g_smaCount == 0) return NAN;
    float deg = (float)fmod(g_smaSum / g_smaCount, 360.0);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

// ============================================================================
// FAST PATH: angle, every loop
// ============================================================================

void sampleAngle()
{
    SscFrame f = readWithRetry(CMD_READ_AVAL);
    uint32_t errors = f.errors;
    uint32_t now = millis();
    g_samples++;
    g_lastSafety = f.safety;

    if (!f.frameOk()) {
        g_rawValid = false;
        g_rawTrusted = false;
        recordErrors(errors);
        return;
    }

    // Any Safety Word flag: read STAT for the detailed cause [PLAN s3]
    if (errors & ERR_SAFETY_MASK) errors |= readStatus();

    uint16_t aval = f.data[0];
    g_rawCount = aval & AVAL_ANGLE_MASK;
    g_rawDeg   = avalToDegrees(aval);

    // RD_AV = new angle since last read [LIB reg]
    if (aval & AVAL_RD_AV) {
        g_lastNewAngleMs = now;
    } else if (now - g_lastNewAngleMs > STALE_ANGLE_MS) {
        errors |= ERR_STALE_ANGLE;
    }

    g_rawValid = !(errors & (ERR_SW_ANGLE_INVALID | ERR_NO_GMR_A | ERR_NO_GMR_XY));
    g_rawTrusted = g_rawValid && !(g_conditions & ERR_MAG_MASK);
    // Every valid angle goes into the average, same as GMR_SMA_C3.ino.
    // A failed frame is left out. Magnet-out still flags the line with '*'.
    if (g_rawValid) {
        if (g_havePrevValid && (now - g_prevValidMs) <= JUMP_WINDOW_MS &&
            fabsf(angleDelta(g_rawDeg, g_prevValidDeg)) > JUMP_MAX_DEG) {
            errors |= ERR_ANGLE_JUMP;
        }
        g_prevValidDeg  = g_rawDeg;
        g_prevValidMs   = now;
        g_havePrevValid = true;
        smaAdd(g_rawDeg);
        g_validSamples++;
    }

    recordErrors(errors);
}

// ============================================================================
// MAGNET CHECK: every MAG_POLL_MS
// Amplitude of the raw X/Y vector against [DS Table 10], with hysteresis.
// A failed read keeps the previous state.
// ============================================================================

void checkMagnet()
{
    SscFrame f = readWithRetry(CMD_READ_ADC_XY);
    recordErrors(f.errors);
    if (!f.frameOk()) return;

    g_adcX = (int16_t)f.data[0];
    g_adcY = (int16_t)f.data[1];
    g_magDigits = sqrtf((float)g_adcX * g_adcX + (float)g_adcY * g_adcY);

    uint32_t active = g_conditions & ERR_MAG_MASK;
    if (g_magDigits < MAG_MIN_DIGITS)                         active |= ERR_MAG_WEAK;
    else if (g_magDigits > MAG_MIN_DIGITS + MAG_HYST_DIGITS)  active &= ~ERR_MAG_WEAK;
    if (g_magDigits > MAG_MAX_DIGITS)                         active |= ERR_MAG_STRONG;
    else if (g_magDigits < MAG_MAX_DIGITS - MAG_HYST_DIGITS)  active &= ~ERR_MAG_STRONG;
    setConditions(ERR_MAG_MASK, active);
}

// ============================================================================
// SLOW PATH: diagnostics, every SLOW_POLL_MS
// ============================================================================

void pollDiagnostics()
{
    uint32_t errors = readStatus();

    // Temperature [LIB getTemperature]
    SscFrame f = readWithRetry(CMD_READ_FSYNC);
    errors |= f.errors;
    if (f.frameOk()) {
        g_tempC = fsyncToCelsius(f.data[0]);
        bool out = (g_tempC < TEMP_MIN_C || g_tempC > TEMP_MAX_C);
        setConditions(ERR_TEMP_LIMIT, out ? ERR_TEMP_LIMIT : 0);
    }

    f = readWithRetry(CMD_READ_DMAG);
    errors |= f.errors;
    if (f.frameOk()) g_dmag = f.data[0] & DMAG_MASK;

    f = readWithRetry(CMD_READ_TRAW);
    errors |= f.errors;
    if (f.frameOk()) g_traw = f.data[0] & TRAW_MASK;

    // Configuration baseline: first good read is the reference [PLAN s5]
    const uint16_t modCmd[3]  = {CMD_READ_MOD1, CMD_READ_MOD2, CMD_READ_MOD3};
    const uint16_t modMask[3] = {MOD1_COMPARE_MASK, MOD2_COMPARE_MASK, MOD3_COMPARE_MASK};
    bool allOk = true;
    for (uint8_t i = 0; i < 3; i++) {
        f = readWithRetry(modCmd[i]);
        errors |= f.errors;
        if (f.frameOk()) g_mod[i] = f.data[0];
        else allOk = false;
    }
    if (allOk) {
        if (!g_modBaseSet) {
            for (uint8_t i = 0; i < 3; i++) g_modBase[i] = g_mod[i];
            g_modBaseSet = true;
        } else {
            bool changed = false;
            for (uint8_t i = 0; i < 3; i++) {
                if ((g_mod[i] ^ g_modBase[i]) & modMask[i]) changed = true;
            }
            setConditions(ERR_CONFIG_CHANGED, changed ? ERR_CONFIG_CHANGED : 0);
        }
    }

    recordErrors(errors);
}

// ============================================================================
// SELF-TEST: decoder logic only, no sensor needed [PLAN Phase B]
// ============================================================================

uint16_t g_testPass = 0;
uint16_t g_testFail = 0;

void testResult(const char* label, bool pass)
{
    if (pass) g_testPass++; else g_testFail++;
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

void testMask(const char* label, uint32_t got, uint32_t expected)
{
    bool pass = (got == expected);
    if (pass) g_testPass++; else g_testFail++;
    Serial.printf("  [%s] %-34s expect ", pass ? "PASS" : "FAIL", label);
    printErrorNames(Serial, expected);
    if (!pass) {
        Serial.print("  got ");
        printErrorNames(Serial, got);
    }
    Serial.println();
}

void runSelfTest()
{
    g_testPass = 0;
    g_testFail = 0;
    Serial.println();
    Serial.println("==================== DECODER SELF-TEST ====================");

    // CRC golden vector [PLAN Phase B]: cmd 0x5081, data 0x0804 -> 0x89
    const uint16_t goldenData[1] = {0x0804};
    uint8_t crc = sscCrc(0x5081, goldenData, 1);
    char label[64];
    snprintf(label, sizeof(label), "CRC 0x5081+0x0804 = 0x89 (got 0x%02X)", crc);
    testResult(label, crc == 0x89);

    // Safety Word vectors, built on the golden frame [DS Table 18]
    struct SwVector { uint16_t data; uint16_t safety; uint32_t expected; const char* label; };
    const SwVector vectors[] = {
        { 0x0804, 0xFE89, ERR_NONE,             "SW 0xFE89 good frame" },
        { 0x0000, 0x0000, ERR_NO_RESPONSE,      "SW 0x0000 data 0x0000" },
        { 0xFFFF, 0xFFFF, ERR_NO_RESPONSE,      "SW 0xFFFF data 0xFFFF" },
        { 0x0804, 0xFE88, ERR_CRC,              "SW 0xFE88 wrong CRC" },
        { 0x0804, 0xFF89, ERR_RESP,             "SW 0xFF89 RESP 1111" },
        { 0x0804, 0x7E89, ERR_SW_RESET,         "SW 0x7E89 bit15 low" },
        { 0x0804, 0xBE89, ERR_SW_SYSTEM,        "SW 0xBE89 bit14 low" },
        { 0x0804, 0xDE89, ERR_SW_ACCESS,        "SW 0xDE89 bit13 low" },
        { 0x0804, 0xEE89, ERR_SW_ANGLE_INVALID, "SW 0xEE89 bit12 low" },
    };
    for (const SwVector& v : vectors) {
        const uint16_t data[1] = {v.data};
        testMask(v.label, checkFrame(0x5081, data, 1, v.safety), v.expected);
    }

    // Each STAT bit alone, against an independent table [LIB reg] [PLAN s4]
    const uint32_t expectedStat[16] = {
        ERR_S_RST, ERR_S_WD, ERR_S_VR, ERR_S_FUSE, ERR_S_DSPU, ERR_S_OV,
        ERR_S_XYOL, ERR_S_MAGOL, ERR_NONE, ERR_S_ADCT, ERR_S_ROM,
        ERR_NO_GMR_XY, ERR_NO_GMR_A, ERR_NONE, ERR_NONE, ERR_NONE
    };
    for (uint8_t bit = 0; bit < 16; bit++) {
        snprintf(label, sizeof(label), "STAT bit %2u (%s)", bit, STAT_BIT_NAMES[bit]);
        testMask(label, decodeStat((uint16_t)(1u << bit)), expectedStat[bit]);
    }

    // Conversions
    float d = angleDelta(0.1f, 359.9f);
    snprintf(label, sizeof(label), "angle wrap 0.1-359.9 = +0.2 (got %+.3f)", d);
    testResult(label, fabsf(d - 0.2f) < 0.001f);
    d = angleDelta(359.9f, 0.1f);
    snprintf(label, sizeof(label), "angle wrap 359.9-0.1 = -0.2 (got %+.3f)", d);
    testResult(label, fabsf(d + 0.2f) < 0.001f);
    float a = avalToDegrees(0x4000);
    snprintf(label, sizeof(label), "AVAL 0x4000 = 180.000 deg (got %.3f)", a);
    testResult(label, fabsf(a - 180.0f) < 0.001f);

    Serial.println("-----------------------------------------------------------");
    Serial.printf("  RESULT: %u passed, %u failed  ->  %s\n",
                  g_testPass, g_testFail, g_testFail == 0 ? "ALL PASS" : "FAILURES PRESENT");
    Serial.println("===========================================================");
    Serial.println();
}

// ============================================================================
// OUTPUT
// ============================================================================

void updateSampleRate()
{
    uint32_t now = millis();
    uint32_t dt = now - g_msAtLastRate;
    if (dt == 0) return;
    g_sampleRate = (float)(g_samples - g_samplesAtLastPrint) * 1000.0f / (float)dt;
    g_samplesAtLastPrint = g_samples;
    g_msAtLastRate = now;
}

const char* okText(uint32_t mask, uint32_t bit)
{
    return (mask & bit) ? "FAIL" : "OK";
}

void printFrameSummary(uint32_t errs)
{
    Serial.printf("CRC %-4s | RESP %-4s | SW 0x%04X",
                  okText(errs, ERR_CRC | ERR_NO_RESPONSE),
                  okText(errs, ERR_RESP | ERR_NO_RESPONSE), g_lastSafety);
}

// '*' after an angle = not trusted (magnet out of range)
char trustMark()
{
    return g_rawTrusted ? ' ' : '*';
}

void printLine()
{
    uint32_t errs = shownErrors();
    float filt = smaDegrees();

    Serial.printf("t=%9.3f s | RAW ", millis() / 1000.0f);
    if (g_rawValid) Serial.printf("%7.3f deg%c(%5u)", g_rawDeg, trustMark(), g_rawCount);
    else            Serial.print("    --- deg (-----)");
    Serial.print(" | FILT ");
    if (isnan(filt)) Serial.print("    --- deg ");
    else             Serial.printf("%7.3f deg%c", filt, trustMark());
    Serial.print("| MAG ");
    if (isnan(g_magDigits)) Serial.print("  ---");
    else                    Serial.printf("%5.0f", g_magDigits);
    Serial.print(" | ");
    printFrameSummary(errs);
    Serial.print(" | ERR ");
    printErrorNames(Serial, errs);
    Serial.println();
}

void printTableHeader()
{
    Serial.println();
    Serial.println("  TIME s  |  RAW deg  | FILT deg  |   SW   |  STAT  | CRC  | RESP | TEMP C |  MAG  | ERRORS");
    Serial.println("----------+-----------+-----------+--------+--------+------+------+--------+-------+------------------");
}

void printTableRow()
{
    if (g_tableRows % TABLE_HEADER_EVERY == 0) printTableHeader();
    g_tableRows++;

    uint32_t errs = shownErrors();
    float filt = smaDegrees();

    Serial.printf("%9.2f | ", millis() / 1000.0f);
    if (g_rawValid) Serial.printf("%8.3f%c | ", g_rawDeg, trustMark()); else Serial.print("    ---   | ");
    if (isnan(filt)) Serial.print("    ---   | "); else Serial.printf("%8.3f%c | ", filt, trustMark());
    Serial.printf("0x%04X | ", g_lastSafety);
    if (g_statRead) Serial.printf("0x%04X | ", g_stat); else Serial.print("  ---  | ");
    Serial.printf("%-4s | %-4s | ",
                  okText(errs, ERR_CRC | ERR_NO_RESPONSE),
                  okText(errs, ERR_RESP | ERR_NO_RESPONSE));
    if (isnan(g_tempC)) Serial.print("  ---  | "); else Serial.printf("%6.1f | ", g_tempC);
    if (isnan(g_magDigits)) Serial.print("  --- | "); else Serial.printf("%5.0f | ", g_magDigits);
    printErrorNames(Serial, errs);
    Serial.println();
}

void printSafetyBits(uint16_t sw)
{
    Serial.printf("    RESET %s   SYSTEM %s   ACCESS %s   ANGLE %s   RESP 0x%X   CRC 0x%02X\n",
                  (sw & SW_BIT_RESET)  ? "ok " : "ERR",
                  (sw & SW_BIT_SYSTEM) ? "ok " : "ERR",
                  (sw & SW_BIT_ACCESS) ? "ok " : "ERR",
                  (sw & SW_BIT_ANGLE)  ? "ok " : "ERR",
                  safetyResp(sw), safetyCrc(sw));
}

void printStatBits(uint16_t stat)
{
    Serial.print("    set: ");
    bool any = false;
    for (uint8_t bit = 0; bit < 16; bit++) {
        if (!(stat & (1u << bit))) continue;
        if (any) Serial.print(", ");
        Serial.print(STAT_BIT_NAMES[bit]);
        any = true;
    }
    if (!any) Serial.print("none");
    Serial.println("   (S_NR = slave number, RD_ST = info only)");
}

void printDashboard()
{
    uint32_t errs = shownErrors();
    float filt = smaDegrees();

    Serial.println();
    Serial.printf("==================== TLE5012B  t = %.1f s ====================\n", millis() / 1000.0f);
    Serial.println("ANGLE");
    if (g_rawValid) Serial.printf("    raw       %8.3f deg   (count %5u of 32768)\n", g_rawDeg, g_rawCount);
    else            Serial.println("    raw       invalid");
    if (isnan(filt)) Serial.println("    filtered  no valid samples yet");
    else             Serial.printf("    filtered  %8.3f deg   (SMA %u/%u samples)\n", filt, g_smaCount, SMA_WINDOW);
    if (g_rawValid && !g_rawTrusted)
        Serial.println("    NOT TRUSTED: magnet out of range");

    Serial.println("FRAME");
    Serial.print("    ");
    printFrameSummary(errs);
    Serial.println();

    Serial.println("SAFETY WORD (last angle read)");
    printSafetyBits(g_lastSafety);

    Serial.print("STAT");
    if (g_statRead) {
        Serial.printf(" 0x%04X\n", g_stat);
        printStatBits(g_stat);
    } else {
        Serial.println("\n    not read yet");
    }

    Serial.println("SENSOR");
    if (isnan(g_tempC)) Serial.println("    temperature  ---");
    else                Serial.printf("    temperature  %6.1f C      (limits %.0f..%.0f C)\n", g_tempC, TEMP_MIN_C, TEMP_MAX_C);
    if (isnan(g_magDigits)) Serial.println("    magnet       ---");
    else Serial.printf("    magnet       %6.0f digits (limits %.0f..%.0f, hyst %.0f)   X %6d   Y %6d\n",
                       g_magDigits, MAG_MIN_DIGITS, MAG_MAX_DIGITS, MAG_HYST_DIGITS, g_adcX, g_adcY);
    Serial.printf("    D_MAG        %6u        T_RAW %u\n", g_dmag, g_traw);

    Serial.println("CONFIG");
    Serial.printf("    MOD_1 0x%04X   MOD_2 0x%04X   MOD_3 0x%04X   -> %s\n",
                  g_mod[0], g_mod[1], g_mod[2],
                  !g_modBaseSet ? "no baseline yet"
                  : (errs & ERR_CONFIG_CHANGED) ? "CHANGED since boot" : "unchanged since boot");

    Serial.println("ACTIVE ERRORS (events since last print + active conditions)");
    printErrorDetails(Serial, errs);

    Serial.println("COUNTS (since boot or 'r'; conditions count onsets)");
    Serial.printf("    samples %lu   valid %lu   retries %lu   rate %.0f samples/s\n",
                  (unsigned long)g_samples, (unsigned long)g_validSamples,
                  (unsigned long)g_retries, g_sampleRate);
    bool anyCount = false;
    for (uint8_t bit = 0; bit < ERR_CODE_COUNT; bit++) {
        if (g_errCount[bit] == 0) continue;
        Serial.printf("    %-16s %lu\n", ERROR_TABLE[bit].name, (unsigned long)g_errCount[bit]);
        anyCount = true;
    }
    if (!anyCount) Serial.println("    no errors recorded");
    Serial.println("==============================================================");
}

void printOutput()
{
    updateSampleRate();
    if (g_mode == MODE_LINE)       printLine();
    else if (g_mode == MODE_DASH)  printDashboard();
    else                           printTableRow();
    g_errSincePrint = 0;
}

uint32_t printPeriodMs()
{
    if (g_mode == MODE_DASH)  return DASH_PERIOD_MS;
    if (g_mode == MODE_TABLE) return TABLE_PERIOD_MS;
    return LINE_PERIOD_MS;
}

// ============================================================================
// REPORTS AND COMMANDS
// ============================================================================

// Reads one register and prints the raw frame. Returns the data word.
uint16_t reportRegister(const char* name, uint16_t command, uint32_t* errors)
{
    SscFrame f = readWithRetry(command);
    *errors |= f.errors;
    Serial.printf("  %-7s cmd 0x%04X  data 0x%04X", name, command, f.data[0]);
    if (f.dataWords > 1) Serial.printf(" 0x%04X", f.data[1]);
    else                 Serial.print("       ");
    Serial.printf("  SW 0x%04X  ", f.safety);
    if (f.errors == 0) Serial.print("ok");
    else printErrorNames(Serial, f.errors);
    Serial.println();
    return f.data[0];
}

void printRegisterReport()
{
    uint32_t errors = 0;
    Serial.println();
    Serial.println("==================== REGISTER REPORT ====================");

    uint16_t stat = reportRegister("STAT", CMD_READ_STAT, &errors);
    errors |= decodeStat(stat);
    g_stat = stat;
    g_statRead = true;
    printStatBits(stat);

    uint16_t acstat = reportRegister("ACSTAT", CMD_READ_ACSTAT, &errors);
    Serial.print("    self-tests enabled: ");
    bool any = false;
    for (uint8_t bit = 0; bit < 11; bit++) {
        if (bit == 8 || !(acstat & (1u << bit))) continue;
        if (any) Serial.print(", ");
        Serial.print(ACSTAT_BIT_NAMES[bit]);
        any = true;
    }
    Serial.println(any ? "" : "none");

    uint16_t aval = reportRegister("AVAL", CMD_READ_AVAL, &errors);
    Serial.printf("    angle %.3f deg (count %u), RD_AV %u\n",
                  avalToDegrees(aval), aval & AVAL_ANGLE_MASK, (aval & AVAL_RD_AV) ? 1 : 0);

    uint16_t fsync = reportRegister("FSYNC", CMD_READ_FSYNC, &errors);
    Serial.printf("    temperature %.1f C (TEMPR raw 0x%03X)\n", fsyncToCelsius(fsync), fsync & FSYNC_TEMPR_MASK);

    uint16_t m1 = reportRegister("MOD_1", CMD_READ_MOD1, &errors);
    Serial.printf("    IIF_MOD %u  DSPU_HOLD %u  CLK_SEL %u  FIR_MD %u\n",
                  m1 & 0x3, (m1 >> 2) & 0x1, (m1 >> 4) & 0x1, (m1 >> 14) & 0x3);

    uint16_t m2 = reportRegister("MOD_2", CMD_READ_MOD2, &errors);
    Serial.printf("    AUTOCAL %u  PREDICT %u  ANG_DIR %u  ANG_RANGE 0x%03X\n",
                  m2 & 0x3, (m2 >> 2) & 0x1, (m2 >> 3) & 0x1, (m2 >> 4) & 0x7FF);

    uint16_t m3 = reportRegister("MOD_3", CMD_READ_MOD3, &errors);
    Serial.printf("    PAD_DRV %u  SSC_OD %u  SPIKEF %u  ANG_BASE 0x%03X\n",
                  m3 & 0x3, (m3 >> 2) & 0x1, (m3 >> 3) & 0x1, (m3 >> 4) & 0xFFF);

    SscFrame xy = readWithRetry(CMD_READ_ADC_XY);
    errors |= xy.errors;
    Serial.printf("  ADC_XY  cmd 0x%04X  data 0x%04X 0x%04X  SW 0x%04X  ",
                  CMD_READ_ADC_XY, xy.data[0], xy.data[1], xy.safety);
    if (xy.errors == 0) Serial.print("ok"); else printErrorNames(Serial, xy.errors);
    Serial.println();
    int16_t x = (int16_t)xy.data[0];
    int16_t y = (int16_t)xy.data[1];
    Serial.printf("    X %d  Y %d  amplitude %.0f digits (limits %.0f..%.0f)\n",
                  x, y, sqrtf((float)x * x + (float)y * y), MAG_MIN_DIGITS, MAG_MAX_DIGITS);

    uint16_t dmag = reportRegister("D_MAG", CMD_READ_DMAG, &errors);
    Serial.printf("    magnitude %u\n", dmag & DMAG_MASK);

    uint16_t traw = reportRegister("T_RAW", CMD_READ_TRAW, &errors);
    Serial.printf("    raw temperature %u  TTGL %u\n", traw & TRAW_MASK, (traw >> 15) & 0x1);

    Serial.print("  Errors in this report: ");
    printErrorNames(Serial, errors);
    Serial.println();
    Serial.println("=========================================================");
    recordErrors(errors);
}

void printErrorTable()
{
    Serial.println();
    Serial.println("==================== ERROR CODES ====================");
    for (uint8_t bit = 0; bit < ERR_CODE_COUNT; bit++) {
        const ErrorInfo& e = ERROR_TABLE[bit];
        Serial.printf("bit %2u  %-16s [%s]\n", bit, e.name, e.layer);
        Serial.printf("        meaning: %s\n", e.meaning);
        Serial.printf("        action:  %s\n", e.action);
        Serial.printf("        source:  %s\n", e.source);
    }
    Serial.println("=====================================================");
}

// Reads MOD_2 with lock 0000 instead of 1010 [DS Table 17] and reports
// whether the sensor flags an interface access error. The result is not
// added to the live counters.
void runAccessTest()
{
    Serial.println();
    Serial.println("==================== ACCESS TEST ====================");
    SscFrame f = sscRead(CMD_TEST_WRONG_LOCK);
    Serial.printf("  cmd 0x%04X (MOD_2, lock 0000)  data 0x%04X  SW 0x%04X  -> ",
                  f.command, f.data[0], f.safety);
    printErrorNames(Serial, f.errors);
    Serial.println();
    if (f.errors & ERR_SW_ACCESS) {
        Serial.println("  SW_ACCESS raised: the sensor rejects a wrong lock on reads.");
    } else if (!f.frameOk()) {
        Serial.println("  Frame error: result inconclusive, check wiring and retry.");
    } else {
        Serial.println("  SW_ACCESS not raised: this sensor accepts reads with lock 0000.");
    }

    // Clear the latched access flag by reading STAT [DS Table 18]
    uint32_t clearErrors = readStatus();
    recordErrors(clearErrors & ~ERR_SW_ACCESS);
    Serial.println("  Latched flag cleared with a STAT read.");
    Serial.println("=====================================================");
}

void printHelp()
{
    Serial.println();
    Serial.println("COMMANDS  (type a character, then Enter)");
    Serial.println("  1  line mode       10 per second");
    Serial.println("  2  dashboard mode   1 per second");
    Serial.println("  3  table mode       5 per second");
    Serial.println("  d  register report");
    Serial.println("  e  error code list");
    Serial.println("  t  decoder self-test");
    Serial.println("  a  interface-access test");
    Serial.println("  r  reset error counters");
    Serial.println("  h  this help");
    Serial.println("Error lists show every event seen since the previous print, plus");
    Serial.println("conditions (MAG_*, TEMP_LIMIT, CONFIG_CHANGED) for as long as they last.");
    Serial.println("'*' after an angle: magnet out of range, do not use this reading.");
    Serial.println();
}

void setMode(uint8_t mode)
{
    g_mode = mode;
    g_tableRows = 0;
    g_errSincePrint = 0;
    const char* names[] = {"", "LINE (10/s)", "DASHBOARD (1/s)", "TABLE (5/s)"};
    Serial.printf("\n>> Mode: %s\n", names[mode]);
}

void handleCommands()
{
    while (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case '1': setMode(MODE_LINE);  break;
            case '2': setMode(MODE_DASH);  break;
            case '3': setMode(MODE_TABLE); break;
            case 'd': case 'D': printRegisterReport(); break;
            case 'e': case 'E': printErrorTable();     break;
            case 't': case 'T': runSelfTest();         break;
            case 'a': case 'A': runAccessTest();       break;
            case 'r': case 'R': resetCounters(); Serial.println("\n>> Counters reset"); break;
            case 'h': case 'H': case '?': printHelp(); break;
            case '\r': case '\n': case ' ': break;
            default:
                Serial.printf("\n>> Unknown command '%c', type h for help\n", c);
                break;
        }
        // Reprint the table header after any command output
        if (c != '\r' && c != '\n' && c != ' ') g_tableRows = 0;
    }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup()
{
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 3000) delay(10);

    delay(POWER_ON_WAIT_MS);
    sscBegin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN, SSC_CLOCK_HZ);

    Serial.println();
    Serial.println("=========================================================");
    Serial.println(" TLE5012B error-code diagnostics  |  XIAO ESP32C3");
    Serial.printf (" SSC %lu Hz, SPI mode 1  |  MOSI release during read: %s\n",
                   (unsigned long)SSC_CLOCK_HZ, TLE_RELEASE_MOSI_DURING_READ ? "on" : "off");
    Serial.println("=========================================================");

    runSelfTest();

    // First STAT read after power-up: S_RST (and SW_RESET) are expected here
    // and are cleared by this read [DS Table 18]. Not counted.
    SscFrame f = sscRead(CMD_READ_STAT);
    Serial.printf("Startup STAT 0x%04X  SW 0x%04X  frame: ", f.data[0], f.safety);
    if (f.frameOk()) Serial.println("ok"); else { printErrorNames(Serial, f.errors); Serial.println(); }
    if (f.frameOk()) {
        Serial.print("  startup flags: ");
        printErrorNames(Serial, decodeStat(f.data[0]) | (f.errors & ERR_SAFETY_MASK));
        Serial.println();
        Serial.println("  (S_RST / SW_RESET right after power-up is normal and now cleared)");
    } else {
        Serial.println("  No valid response: check wiring and power. Continuing anyway.");
    }

    pollDiagnostics();   // sets the MOD_1/2/3 baseline
    checkMagnet();
    printRegisterReport();
    printHelp();

    smaClear();
    resetCounters();
    g_lastNewAngleMs = millis();
    g_lastSlowMs = millis();
    g_lastMagMs = millis();
    g_lastPrintMs = millis();
    setMode(MODE_LINE);
}

void loop()
{
    handleCommands();
    sampleAngle();

    uint32_t now = millis();
    if (now - g_lastMagMs >= MAG_POLL_MS) {
        g_lastMagMs = now;
        checkMagnet();
    }
    if (now - g_lastSlowMs >= SLOW_POLL_MS) {
        g_lastSlowMs = now;
        pollDiagnostics();
    }
    if (now - g_lastPrintMs >= printPeriodMs()) {
        g_lastPrintMs = now;
        printOutput();
    }
}
