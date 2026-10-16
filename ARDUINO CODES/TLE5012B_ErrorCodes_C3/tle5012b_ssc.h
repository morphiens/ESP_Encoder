/*
 * tle5012b_ssc.h — TLE5012B SSC bus access, CRC, Safety Word and STAT decoding
 *
 * Source tags used in this file (full list in tle5012b_errors.h):
 *   [DS]   Infineon TLE5012B Data Sheet Rev 2.1
 *   [LIB]  Infineon TLE5012B Arduino library v3.1.0 (src/corelib)
 *   [PLAN] GMR_Error_Codes/GMR_TLE5012B_Error_Diagnostics_and_Test_Plan.md
 *
 * Only READ commands are ever sent. This sketch never writes to the sensor.
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "tle5012b_errors.h"

// 1 = release MOSI while the sensor drives DATA (method used in
//     TLE5012B_dual_S3_hwSPI.ino). 0 = keep MOSI attached and clock out 0xFFFF;
//     the 470 ohm series resistors limit the current [DS 3.7].
#ifndef TLE_RELEASE_MOSI_DURING_READ
#define TLE_RELEASE_MOSI_DURING_READ 1
#endif

// ============================================================================
// COMMAND WORDS
// Format [DS Table 17]: RW[15] LOCK[14:11] UPD[10] ADDR[9:4] ND[3:0]
//   RW   = 1 (read)
//   LOCK = 0000 for addresses 0x00-0x04, 1010 for 0x05-0x11 [DS Table 17]
//          0000 for 0x14-0x15 [PLAN section 2]
//   ND   = number of data words; ND >= 1 appends the Safety Word [DS 4.4.1.2]
// ============================================================================
constexpr uint16_t CMD_READ_STAT   = 0x8001;  // STAT    0x00  status flags
constexpr uint16_t CMD_READ_ACSTAT = 0x8011;  // ACSTAT  0x01  enabled self-tests
constexpr uint16_t CMD_READ_AVAL   = 0x8021;  // AVAL    0x02  angle value
constexpr uint16_t CMD_READ_FSYNC  = 0xD051;  // FSYNC   0x05  temperature (TEMPR)
constexpr uint16_t CMD_READ_MOD1   = 0xD061;  // MOD_1   0x06  interface mode 1
constexpr uint16_t CMD_READ_MOD2   = 0xD081;  // MOD_2   0x08  angle range/direction
constexpr uint16_t CMD_READ_MOD3   = 0xD091;  // MOD_3   0x09  pad driver/angle base
constexpr uint16_t CMD_READ_ADC_XY = 0xD102;  // ADC_X 0x10 + ADC_Y 0x11 (ND = 2)
constexpr uint16_t CMD_READ_DMAG   = 0x8141;  // D_MAG   0x14  vector magnitude
constexpr uint16_t CMD_READ_TRAW   = 0x8151;  // T_RAW   0x15  raw temperature

// Deliberately wrong: MOD_2 (0x08) with lock 0000 instead of 1010.
// Used only by the 'a' interface-access test [PLAN Phase C].
constexpr uint16_t CMD_TEST_WRONG_LOCK = 0x8081;

// ============================================================================
// REGISTER FIELDS [LIB tle5012b_reg.cpp]
// ============================================================================
constexpr uint16_t AVAL_RD_AV       = 0x8000;  // bit 15: new angle value since last read
constexpr uint16_t AVAL_ANGLE_MASK  = 0x7FFF;  // bits 14:0: 15-bit angle
constexpr uint16_t FSYNC_TEMPR_MASK = 0x01FF;  // bits 8:0: signed 9-bit temperature
constexpr uint16_t DMAG_MASK        = 0x03FF;  // bits 9:0: unsigned magnitude
constexpr uint16_t TRAW_MASK        = 0x03FF;  // bits 9:0: raw temperature ADC value

// Temperature conversion [LIB tle5012b_util.hpp TEMP_OFFSET / TEMP_DIV]
constexpr float TEMP_OFFSET = 152.0f;
constexpr float TEMP_DIV    = 2.776f;

// ============================================================================
// SAFETY WORD [DS Table 18]
//   bit 15  reset / watchdog      0 = occurred   (active-low)
//   bit 14  system error          0 = occurred   (active-low)
//   bit 13  interface access err  0 = occurred   (active-low)
//   bit 12  angle valid           0 = invalid    (active-low)
//   11:8    RESP: own slave bit pulled low, others high
//   7:0     CRC
// Status bits stay low until STAT is read [DS Table 18, footnote 1].
// ============================================================================
constexpr uint16_t SW_BIT_RESET  = 0x8000;
constexpr uint16_t SW_BIT_SYSTEM = 0x4000;
constexpr uint16_t SW_BIT_ACCESS = 0x2000;
constexpr uint16_t SW_BIT_ANGLE  = 0x1000;
constexpr uint8_t  RESP_SLAVE_0  = 0xE;  // 1110: slave 0 pulls bit 8 low [DS Table 18]

static inline uint8_t safetyResp(uint16_t safety) { return (safety >> 8) & 0x0F; }
static inline uint8_t safetyCrc(uint16_t safety)  { return safety & 0xFF; }

// ============================================================================
// CRC — J1850: polynomial x^8+x^4+x^3+x^2+1 (0x1D), seed 0xFF, result inverted.
// Covers the command word and every data word [DS 4.4.1.2], [LIB crc8()/checkSafety()].
// Golden vector: command 0x5081, data 0x0804 -> CRC 0x89 [PLAN Phase B].
// ============================================================================
static inline uint8_t crc8J1850(const uint8_t* bytes, uint8_t length)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= bytes[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
        }
    }
    return (uint8_t)~crc;
}

static inline uint8_t sscCrc(uint16_t command, const uint16_t* data, uint8_t dataWords)
{
    uint8_t bytes[2 + 2 * 2];
    uint8_t n = 0;
    bytes[n++] = command >> 8;
    bytes[n++] = command & 0xFF;
    for (uint8_t i = 0; i < dataWords && i < 2; i++) {
        bytes[n++] = data[i] >> 8;
        bytes[n++] = data[i] & 0xFF;
    }
    return crc8J1850(bytes, n);
}

// ============================================================================
// FRAME CHECK [PLAN sections 3 and 6]
// Frame errors (no response, CRC, RESP) mean the Safety Word status bits are
// not trustworthy, so they are only decoded when the frame itself is good.
// ============================================================================
static inline uint32_t checkFrame(uint16_t command, const uint16_t* data,
                                  uint8_t dataWords, uint16_t safety)
{
    bool allZero = (safety == 0x0000);
    bool allOnes = (safety == 0xFFFF);
    for (uint8_t i = 0; i < dataWords; i++) {
        allZero = allZero && (data[i] == 0x0000);
        allOnes = allOnes && (data[i] == 0xFFFF);
    }
    if (allZero || allOnes) return ERR_NO_RESPONSE;

    uint32_t errors = 0;
    if (sscCrc(command, data, dataWords) != safetyCrc(safety)) errors |= ERR_CRC;
    if (safetyResp(safety) != RESP_SLAVE_0)                    errors |= ERR_RESP;
    if (errors) return errors;

    if (!(safety & SW_BIT_RESET))  errors |= ERR_SW_RESET;
    if (!(safety & SW_BIT_SYSTEM)) errors |= ERR_SW_SYSTEM;
    if (!(safety & SW_BIT_ACCESS)) errors |= ERR_SW_ACCESS;
    if (!(safety & SW_BIT_ANGLE))  errors |= ERR_SW_ANGLE_INVALID;
    return errors;
}

// ============================================================================
// STAT DECODER [LIB tle5012b_reg.cpp], [PLAN section 4], [HEC]
// Bit 8 is reserved, bits 14:13 are the slave number, bit 15 (RD_ST) is
// information only. None of those are faults.
// ============================================================================
static inline uint32_t decodeStat(uint16_t stat)
{
    uint32_t errors = 0;
    if (stat & (1u << 0))  errors |= ERR_S_RST;
    if (stat & (1u << 1))  errors |= ERR_S_WD;
    if (stat & (1u << 2))  errors |= ERR_S_VR;
    if (stat & (1u << 3))  errors |= ERR_S_FUSE;
    if (stat & (1u << 4))  errors |= ERR_S_DSPU;
    if (stat & (1u << 5))  errors |= ERR_S_OV;
    if (stat & (1u << 6))  errors |= ERR_S_XYOL;
    if (stat & (1u << 7))  errors |= ERR_S_MAGOL;
    if (stat & (1u << 9))  errors |= ERR_S_ADCT;
    if (stat & (1u << 10)) errors |= ERR_S_ROM;
    if (stat & (1u << 11)) errors |= ERR_NO_GMR_XY;
    if (stat & (1u << 12)) errors |= ERR_NO_GMR_A;
    return errors;
}

// Names of all 16 STAT bits, for printing [LIB tle5012b_reg.cpp]
static const char* const STAT_BIT_NAMES[16] = {
    "S_RST", "S_WD", "S_VR", "S_FUSE", "S_DSPU", "S_OV", "S_XYOL", "S_MAGOL",
    "(res)", "S_ADCT", "S_ROM", "NO_GMR_XY", "NO_GMR_A", "S_NR0", "S_NR1", "RD_ST"
};

// Names of the ACSTAT enable bits [LIB tle5012b_reg.cpp]
static const char* const ACSTAT_BIT_NAMES[11] = {
    "AS_RST", "AS_WD", "AS_VR", "AS_FUSE", "AS_DSPU", "AS_OV",
    "AS_VEC_XY", "AS_VEC_MAG", "(res)", "AS_ADCT", "AS_FRST"
};

// ============================================================================
// CONVERSIONS
// ============================================================================

// AVAL 15-bit count -> degrees in [0, 360) [DS Features: 15-bit angle]
static inline float avalToDegrees(uint16_t count)
{
    return (float)(count & AVAL_ANGLE_MASK) * 360.0f / 32768.0f;
}

// FSYNC TEMPR (signed 9-bit) -> degrees Celsius [LIB getTemperature()]
static inline float fsyncToCelsius(uint16_t fsync)
{
    int16_t raw = fsync & FSYNC_TEMPR_MASK;
    if (raw & 0x0100) raw -= 0x0200;
    return ((float)raw + TEMP_OFFSET) / TEMP_DIV;
}

// Shortest signed difference a - b, in [-180, 180)
static inline float angleDelta(float a, float b)
{
    float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
    return d;
}

// ============================================================================
// BUS ACCESS
// Timing [DS Table 15]: tCSs >= 105 ns, twr_delay >= 130 ns, tCSoff >= 600 ns.
// SPI mode 1, MSB first [DS 4.4.1.2: data put on rising edge, read on falling].
// ============================================================================
struct SscFrame {
    uint16_t command   = 0;
    uint16_t data[2]   = {0, 0};
    uint8_t  dataWords = 0;
    uint16_t safety    = 0;
    uint32_t errors    = ERR_NO_RESPONSE;

    bool frameOk() const { return (errors & ERR_FRAME_MASK) == 0; }
};

static int         g_sscCs   = -1;
static int         g_sscMosi = -1;
static SPISettings g_sscSettings(1000000, MSBFIRST, SPI_MODE1);

static inline void sscBegin(int sck, int miso, int mosi, int cs, uint32_t clockHz)
{
    g_sscCs   = cs;
    g_sscMosi = mosi;
    g_sscSettings = SPISettings(clockHz, MSBFIRST, SPI_MODE1);

    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    SPI.begin(sck, miso, mosi, -1);
}

static inline void sscReleaseMosi()
{
#if TLE_RELEASE_MOSI_DURING_READ
    pinMode(g_sscMosi, INPUT);
#endif
}

static inline void sscAttachMosi()
{
#if TLE_RELEASE_MOSI_DURING_READ
    pinMode(g_sscMosi, OUTPUT);
    esp_rom_gpio_connect_out_signal(g_sscMosi, FSPID_OUT_IDX, false, false);
#endif
}

// Sends one read command, receives ND data words plus the Safety Word, and
// validates the frame.
static inline SscFrame sscRead(uint16_t command)
{
    SscFrame frame;
    frame.command   = command;
    frame.dataWords = command & 0x000F;
    if (frame.dataWords > 2) frame.dataWords = 2;

    uint16_t rx[3] = {0, 0, 0};
    const uint8_t words = frame.dataWords + 1;

    SPI.beginTransaction(g_sscSettings);
    digitalWrite(g_sscCs, LOW);
    delayMicroseconds(1);                       // tCSs

    SPI.transfer16(command);
    sscReleaseMosi();
    delayMicroseconds(1);                       // twr_delay

    for (uint8_t i = 0; i < words; i++) rx[i] = SPI.transfer16(0xFFFF);

    digitalWrite(g_sscCs, HIGH);
    sscAttachMosi();
    SPI.endTransaction();
    delayMicroseconds(1);                       // tCSoff

    for (uint8_t i = 0; i < frame.dataWords; i++) frame.data[i] = rx[i];
    frame.safety = rx[frame.dataWords];
    frame.errors = checkFrame(command, frame.data, frame.dataWords, frame.safety);
    return frame;
}
