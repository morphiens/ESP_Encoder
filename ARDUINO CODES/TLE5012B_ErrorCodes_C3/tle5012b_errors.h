/*
 * tle5012b_errors.h — TLE5012B error codes
 *
 * Every error is one bit in a uint32_t mask, so several can be active at once.
 * Codes are grouped in four layers [PLAN section 1]:
 *
 *   FRAME   checked by the ESP32: the transfer itself is bad, the sample is
 *           rejected and the Safety Word status bits are not trusted.
 *   SAFETY  summary bits the sensor sends with every transfer (Safety Word).
 *   STAT    detailed sensor flags from the STAT register.
 *   APP     inferred by this firmware from other registers (not sensor flags).
 *
 * SOURCES
 *   [DS]   Infineon TLE5012B Data Sheet Rev 2.1, 2018-06-20
 *          (DOCS, LOGS and PICS/Infineon_TLE5012B_Datasheet.pdf)
 *   [LIB]  Infineon TLE5012B Arduino library v3.1.0, src/corelib
 *          https://github.com/Infineon/TLE5012-Magnetic-Angle-Sensor
 *   [PLAN] GMR_Error_Codes/GMR_TLE5012B_Error_Diagnostics_and_Test_Plan.md
 *   [HEC]  GMR_Error_Codes/(Description) Hallever Error Codes.pdf
 *   [XIAO] Seeed Studio XIAO ESP32C3 pinout
 *          https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/
 */

#pragma once

#include <Arduino.h>

enum ErrorBit : uint8_t {
    // FRAME
    EB_NO_RESPONSE = 0,
    EB_CRC,
    EB_RESP,
    // SAFETY
    EB_SW_RESET,
    EB_SW_SYSTEM,
    EB_SW_ACCESS,
    EB_SW_ANGLE_INVALID,
    // STAT
    EB_S_RST,
    EB_S_WD,
    EB_S_VR,
    EB_S_FUSE,
    EB_S_DSPU,
    EB_S_OV,
    EB_S_XYOL,
    EB_S_MAGOL,
    EB_S_ADCT,
    EB_S_ROM,
    EB_NO_GMR_XY,
    EB_NO_GMR_A,
    // APP
    EB_MAG_WEAK,
    EB_MAG_STRONG,
    EB_TEMP_LIMIT,
    EB_CONFIG_CHANGED,
    EB_STALE_ANGLE,
    EB_ANGLE_JUMP,

    ERR_CODE_COUNT
};

constexpr uint32_t errBit(uint8_t bit) { return 1UL << bit; }

constexpr uint32_t ERR_NONE             = 0;
constexpr uint32_t ERR_NO_RESPONSE      = errBit(EB_NO_RESPONSE);
constexpr uint32_t ERR_CRC              = errBit(EB_CRC);
constexpr uint32_t ERR_RESP             = errBit(EB_RESP);
constexpr uint32_t ERR_SW_RESET         = errBit(EB_SW_RESET);
constexpr uint32_t ERR_SW_SYSTEM        = errBit(EB_SW_SYSTEM);
constexpr uint32_t ERR_SW_ACCESS        = errBit(EB_SW_ACCESS);
constexpr uint32_t ERR_SW_ANGLE_INVALID = errBit(EB_SW_ANGLE_INVALID);
constexpr uint32_t ERR_S_RST            = errBit(EB_S_RST);
constexpr uint32_t ERR_S_WD             = errBit(EB_S_WD);
constexpr uint32_t ERR_S_VR             = errBit(EB_S_VR);
constexpr uint32_t ERR_S_FUSE           = errBit(EB_S_FUSE);
constexpr uint32_t ERR_S_DSPU           = errBit(EB_S_DSPU);
constexpr uint32_t ERR_S_OV             = errBit(EB_S_OV);
constexpr uint32_t ERR_S_XYOL           = errBit(EB_S_XYOL);
constexpr uint32_t ERR_S_MAGOL          = errBit(EB_S_MAGOL);
constexpr uint32_t ERR_S_ADCT           = errBit(EB_S_ADCT);
constexpr uint32_t ERR_S_ROM            = errBit(EB_S_ROM);
constexpr uint32_t ERR_NO_GMR_XY        = errBit(EB_NO_GMR_XY);
constexpr uint32_t ERR_NO_GMR_A         = errBit(EB_NO_GMR_A);
constexpr uint32_t ERR_MAG_WEAK         = errBit(EB_MAG_WEAK);
constexpr uint32_t ERR_MAG_STRONG       = errBit(EB_MAG_STRONG);
constexpr uint32_t ERR_TEMP_LIMIT       = errBit(EB_TEMP_LIMIT);
constexpr uint32_t ERR_CONFIG_CHANGED   = errBit(EB_CONFIG_CHANGED);
constexpr uint32_t ERR_STALE_ANGLE      = errBit(EB_STALE_ANGLE);
constexpr uint32_t ERR_ANGLE_JUMP       = errBit(EB_ANGLE_JUMP);

constexpr uint32_t ERR_FRAME_MASK       = ERR_NO_RESPONSE | ERR_CRC | ERR_RESP;
constexpr uint32_t ERR_SAFETY_MASK      = ERR_SW_RESET | ERR_SW_SYSTEM |
                                          ERR_SW_ACCESS | ERR_SW_ANGLE_INVALID;

// Conditions are states, not events: they stay active (and are shown on every
// print) until the measured value is back inside its limit.
constexpr uint32_t ERR_MAG_MASK         = ERR_MAG_WEAK | ERR_MAG_STRONG;
constexpr uint32_t ERR_CONDITION_MASK   = ERR_MAG_MASK | ERR_TEMP_LIMIT | ERR_CONFIG_CHANGED;

struct ErrorInfo {
    const char* name;
    const char* layer;
    const char* meaning;
    const char* action;   // suggested response [PLAN Phase D]
    const char* source;
};

// Indexed by ErrorBit.
static const ErrorInfo ERROR_TABLE[ERR_CODE_COUNT] = {
    // ---------------------------------------------------------------- FRAME
    { "NO_RESPONSE", "FRAME",
      "Data and Safety Word all 0x0000 or all 0xFFFF (heuristic)",
      "Check wiring, sensor power and CS; retry",
      "[PLAN s3, s6.7]" },
    { "CRC", "FRAME",
      "Safety Word CRC does not match command + data",
      "Reject sample, retry; persistent = noise, wiring or SPI timing",
      "[DS 4.4.1.2] [LIB crc8]" },
    { "RESP", "FRAME",
      "RESP nibble is not 1110 (slave 0)",
      "Reject sample, retry; check CS and DATA line",
      "[DS Table 18]" },

    // --------------------------------------------------------------- SAFETY
    { "SW_RESET", "SAFETY",
      "Chip reset or watchdog overflow since last STAT read",
      "Expected once after power-up; otherwise read STAT and reinitialise",
      "[DS Table 18 bit 15]" },
    { "SW_SYSTEM", "SAFETY",
      "System error summary (S_VR/DSPU/OV/XYOL/MAGOL/FUSE/ROM/ADCT)",
      "Read STAT for the detailed cause",
      "[DS Table 18 bit 14]" },
    { "SW_ACCESS", "SAFETY",
      "Interface access error: wrong address or wrong lock",
      "Check the command word and lock bits",
      "[DS Table 18 bit 13] [DS Table 17]" },
    { "SW_ANGLE_INVALID", "SAFETY",
      "Angle not valid (NO_GMR_A or NO_GMR_XY set)",
      "Reject the angle; check the magnet",
      "[DS Table 18 bit 12]" },

    // ----------------------------------------------------------------- STAT
    { "S_RST", "STAT",
      "Reset event recorded",
      "Expected after power-up; otherwise check supply and reinitialise",
      "[LIB reg] [HEC]" },
    { "S_WD", "STAT",
      "DSP watchdog expired",
      "Repeated = persistent sensor fault, escalate",
      "[LIB reg] [HEC]" },
    { "S_VR", "STAT",
      "Voltage regulator / VDD / GND fault (over-, undervoltage, VDD-off, GND-off)",
      "Check supply (3.0-5.5 V, DS Table 4) and ground connection",
      "[DS 4.6] [HEC]" },
    { "S_FUSE", "STAT",
      "Fuse / configuration integrity CRC failed",
      "Persistent = sensor fault, escalate",
      "[LIB reg] [HEC]" },
    { "S_DSPU", "STAT",
      "DSPU self-test (CORDIC/CCU) failed or still running",
      "If it persists after startup = sensor fault",
      "[LIB reg] [HEC]" },
    { "S_OV", "STAT",
      "DSP overflow",
      "Check magnet; persistent = sensor fault",
      "[LIB reg] [HEC]" },
    { "S_XYOL", "STAT",
      "Raw X/Y values out of allowed range",
      "Check magnet strength and air gap",
      "[LIB reg] [HEC]" },
    { "S_MAGOL", "STAT",
      "Magnetic vector magnitude out of limits",
      "Check magnet distance (field 30-50 mT, DS Table 4)",
      "[LIB reg] [HEC]" },
    { "S_ADCT", "STAT",
      "ADC test-vector check failed (held until chip reset)",
      "Power-cycle the sensor; persistent = sensor fault",
      "[DS 4.4.2] [HEC]" },
    { "S_ROM", "STAT",
      "ROM CRC / self-test failed or still running",
      "If it persists after startup = sensor fault",
      "[LIB reg] [HEC]" },
    { "NO_GMR_XY", "STAT",
      "No valid X/Y GMR values",
      "Reject sample; check magnet",
      "[LIB reg] [HEC]" },
    { "NO_GMR_A", "STAT",
      "No valid GMR angle value",
      "Reject angle; check magnet",
      "[LIB reg] [HEC]" },

    // ------------------------------------------------------------------ APP
    { "MAG_WEAK", "APP",
      "|ADC_X,ADC_Y| below 6000 digits (min amplitude at ambient); active state",
      "Move magnet closer or use a stronger magnet; angle is not trusted",
      "[DS Table 10] SOURCED, hysteresis ESTIMATE" },
    { "MAG_STRONG", "APP",
      "|ADC_X,ADC_Y| above 15781 digits (max amplitude at ambient); active state",
      "Move magnet further away; angle is not trusted",
      "[DS Table 10] SOURCED, hysteresis ESTIMATE" },
    { "TEMP_LIMIT", "APP",
      "Sensor temperature outside -40..125 C; active state",
      "Check the thermal environment",
      "[LIB getTemperature] [DS Table 4] ESTIMATE (upper limit)" },
    { "CONFIG_CHANGED", "APP",
      "MOD_1/MOD_2/MOD_3 differ from the values read at boot; active state",
      "Unexpected reset or write; reinitialise configuration",
      "[PLAN s5]" },
    { "STALE_ANGLE", "APP",
      "AVAL RD_AV reported no new angle for > 10 ms",
      "Check DSPU (STAT) and SSC; reinitialise",
      "[LIB reg RD_AV] [DS Table 12] ESTIMATE (timeout)" },
    { "ANGLE_JUMP", "APP",
      "Angle changed > 20 deg within 10 ms",
      "Check magnet and mechanics; reject sample",
      "[PLAN s5] ESTIMATE (limits)" },
};

// "none" or "NAME, NAME, ..." for every set bit.
static inline void printErrorNames(Print& out, uint32_t mask)
{
    if (mask == 0) {
        out.print("none");
        return;
    }
    bool first = true;
    for (uint8_t bit = 0; bit < ERR_CODE_COUNT; bit++) {
        if (!(mask & errBit(bit))) continue;
        if (!first) out.print(", ");
        out.print(ERROR_TABLE[bit].name);
        first = false;
    }
}

// One indented line per set bit: name, meaning and suggested action.
static inline void printErrorDetails(Print& out, uint32_t mask)
{
    if (mask == 0) {
        out.println("    none");
        return;
    }
    for (uint8_t bit = 0; bit < ERR_CODE_COUNT; bit++) {
        if (!(mask & errBit(bit))) continue;
        const ErrorInfo& e = ERROR_TABLE[bit];
        out.printf("    %-16s %-6s  %s\n", e.name, e.layer, e.meaning);
        out.printf("    %-16s %-6s  -> %s\n", "", "", e.action);
    }
}
