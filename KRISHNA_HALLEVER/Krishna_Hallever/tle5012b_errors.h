/*
 * TLE5012B error definitions and user-facing diagnostics.
 *
 * Sources:
 *   [DS] TLE5012B Data Sheet, Rev. 2.1
 *   [UM] TLE5012B User Manual, Rev. 1.2
 *
 * Every error is one bit in a 32-bit mask, so any number of errors can be
 * active at the same time. Bit positions are part of the serial format and
 * must not be reordered; add new codes at the end only.
 *
 * Layers:
 *   FRAME   detected by this firmware: the transfer itself is corrupt, so the
 *           Safety Word and data of that transfer are not trusted.
 *   SAFETY  active-low summary bits of the Safety Word [DS Table 18].
 *   STAT    active-high detailed sensor flags [UM STAT register].
 *   APP     derived by this firmware from documented sensor values.
 */

#pragma once

#include <Arduino.h>

enum TleErrorBit : uint8_t {
    EB_NO_RESPONSE = 0,
    EB_CRC,
    EB_RESP,

    EB_SW_RESET,
    EB_SW_SYSTEM,
    EB_SW_ACCESS,
    EB_SW_ANGLE_INVALID,

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

    EB_TEMP_LIMIT,
    EB_NO_MAGNET,
    EB_MAG_WEAK,
    EB_MAG_STRONG,
    EB_STALE_ANGLE,

    TLE_ERROR_COUNT
};

constexpr uint32_t tleErrorBit(uint8_t bit)
{
    return 1UL << bit;
}

constexpr uint32_t ERR_NONE             = 0;
constexpr uint32_t ERR_NO_RESPONSE      = tleErrorBit(EB_NO_RESPONSE);
constexpr uint32_t ERR_CRC              = tleErrorBit(EB_CRC);
constexpr uint32_t ERR_RESP             = tleErrorBit(EB_RESP);
constexpr uint32_t ERR_SW_RESET         = tleErrorBit(EB_SW_RESET);
constexpr uint32_t ERR_SW_SYSTEM        = tleErrorBit(EB_SW_SYSTEM);
constexpr uint32_t ERR_SW_ACCESS        = tleErrorBit(EB_SW_ACCESS);
constexpr uint32_t ERR_SW_ANGLE_INVALID = tleErrorBit(EB_SW_ANGLE_INVALID);
constexpr uint32_t ERR_S_RST            = tleErrorBit(EB_S_RST);
constexpr uint32_t ERR_S_WD             = tleErrorBit(EB_S_WD);
constexpr uint32_t ERR_S_VR             = tleErrorBit(EB_S_VR);
constexpr uint32_t ERR_S_FUSE           = tleErrorBit(EB_S_FUSE);
constexpr uint32_t ERR_S_DSPU           = tleErrorBit(EB_S_DSPU);
constexpr uint32_t ERR_S_OV             = tleErrorBit(EB_S_OV);
constexpr uint32_t ERR_S_XYOL           = tleErrorBit(EB_S_XYOL);
constexpr uint32_t ERR_S_MAGOL          = tleErrorBit(EB_S_MAGOL);
constexpr uint32_t ERR_S_ADCT           = tleErrorBit(EB_S_ADCT);
constexpr uint32_t ERR_S_ROM            = tleErrorBit(EB_S_ROM);
constexpr uint32_t ERR_NO_GMR_XY        = tleErrorBit(EB_NO_GMR_XY);
constexpr uint32_t ERR_NO_GMR_A         = tleErrorBit(EB_NO_GMR_A);
constexpr uint32_t ERR_TEMP_LIMIT       = tleErrorBit(EB_TEMP_LIMIT);
constexpr uint32_t ERR_NO_MAGNET        = tleErrorBit(EB_NO_MAGNET);
constexpr uint32_t ERR_MAG_WEAK         = tleErrorBit(EB_MAG_WEAK);
constexpr uint32_t ERR_MAG_STRONG       = tleErrorBit(EB_MAG_STRONG);
constexpr uint32_t ERR_STALE_ANGLE      = tleErrorBit(EB_STALE_ANGLE);

constexpr uint32_t ERR_FRAME_MASK =
    ERR_NO_RESPONSE | ERR_CRC | ERR_RESP;

constexpr uint32_t ERR_SAFETY_MASK =
    ERR_SW_RESET | ERR_SW_SYSTEM | ERR_SW_ACCESS | ERR_SW_ANGLE_INVALID;

constexpr uint32_t ERR_STAT_MASK =
    ERR_S_RST | ERR_S_WD | ERR_S_VR | ERR_S_FUSE | ERR_S_DSPU |
    ERR_S_OV | ERR_S_XYOL | ERR_S_MAGOL | ERR_S_ADCT | ERR_S_ROM |
    ERR_NO_GMR_XY | ERR_NO_GMR_A;

constexpr uint32_t ERR_FIELD_MASK =
    ERR_NO_MAGNET | ERR_MAG_WEAK | ERR_MAG_STRONG;

// Any of these means the angle of the current sample must not be used.
constexpr uint32_t ERR_ANGLE_UNUSABLE_MASK =
    ERR_SW_ANGLE_INVALID | ERR_NO_GMR_XY | ERR_NO_GMR_A |
    ERR_NO_MAGNET | ERR_STALE_ANGLE;

struct TleErrorInfo {
    const char *name;
    const char *layer;
    const char *meaning;
    const char *fix;
};

// Indexed by TleErrorBit.
static const TleErrorInfo TLE_ERROR_TABLE[TLE_ERROR_COUNT] = {
    { "NO_RESPONSE", "FRAME",
      "Sensor returned all 0x0000 or all 0xFFFF: nothing is answering on this bus.",
      "Check sensor VDD/GND, CS, SCK and DATA wiring and connectors." },
    { "CRC", "FRAME",
      "Safety Word CRC does not match the command and data: the transfer was corrupted.",
      "Check DATA/SCK wiring, ground return and cable noise; shorten or shield the cable." },
    { "RESP", "FRAME",
      "Safety Word RESP nibble is not 1110 (slave 0): wrong or no device responded.",
      "Check CS and DATA wiring; make sure only one sensor is on this bus." },

    { "SW_RESET", "SAFETY",
      "Sensor reports a reset or watchdog event since the last STAT read.",
      "See S_RST/S_WD in this report; check supply stability if not at power-up." },
    { "SW_SYSTEM", "SAFETY",
      "Sensor reports a system error summary.",
      "See the STAT code(s) reported with it for the exact cause." },
    { "SW_ACCESS", "SAFETY",
      "Sensor rejected the command address or lock bits.",
      "Firmware command word is wrong; reflash known-good firmware." },
    { "SW_ANGLE_INVALID", "SAFETY",
      "Sensor reports that its angle output is not valid.",
      "Angle discarded; check magnet presence, centring and air gap." },

    { "S_RST", "STAT",
      "Sensor went through a reset (power-up, short power break or internal reset).",
      "If not at power-up: check VDD for dips and connector contact." },
    { "S_WD", "STAT",
      "Sensor DSP watchdog expired; the sensor stopped processing.",
      "Power-cycle the sensor; replace it if this repeats." },
    { "S_VR", "STAT",
      "Supply fault: VDD over-voltage, VDD-off, GND-off or internal regulator fault.",
      "Measure VDD (3.0-5.5 V) at the sensor, check GND and the 100 nF capacitor." },
    { "S_FUSE", "STAT",
      "Configuration fuse / register CRC check failed.",
      "Power-cycle the sensor; replace it if this persists." },
    { "S_DSPU", "STAT",
      "DSPU / CORDIC self-test failed (set after a watchdog stop).",
      "Power-cycle the sensor; replace it if this persists." },
    { "S_OV", "STAT",
      "Internal DSP overflow while calculating the angle.",
      "Check magnet field and alignment; replace the sensor if this persists." },
    { "S_XYOL", "STAT",
      "Raw X/Y GMR values beyond +/-23230 digits: field far too strong or sensor saturated.",
      "Increase the air gap or use a weaker magnet." },
    { "S_MAGOL", "STAT",
      "Sensor's internal magnetic vector magnitude check failed.",
      "Check magnet distance, centring and strength (30-50 mT)." },
    { "S_ADCT", "STAT",
      "ADC test-vector self-test failed.",
      "Power-cycle the sensor; replace it if this persists." },
    { "S_ROM", "STAT",
      "Startup ROM CRC failed or is still running; the DSP does not start.",
      "Power-cycle the sensor; replace it if this persists." },
    { "NO_GMR_XY", "STAT",
      "Sensor has no valid X/Y GMR values.",
      "Angle discarded; check magnet and sensor; power-cycle if persistent." },
    { "NO_GMR_A", "STAT",
      "Sensor has no valid angle value.",
      "Angle discarded; check magnet and sensor; power-cycle if persistent." },

    { "TEMP_LIMIT", "APP",
      "Sensor temperature outside the -40..150 C operating range.",
      "Remove the heat source or cooling; let the sensor return to its range." },
    { "NO_MAGNET", "APP",
      "Field amplitude <= 4096 digits, the datasheet no-magnet level: no magnet detected.",
      "Angle discarded; fit the magnet and centre it over the sensor." },
    { "MAG_WEAK", "APP",
      "Field amplitude below 6000 digits (spec minimum at 30 mT, 25 C): magnet too weak or too far.",
      "Reduce the air gap or use a stronger magnet; angle accuracy is not guaranteed." },
    { "MAG_STRONG", "APP",
      "Field amplitude above 15781 digits (spec maximum at 25 C): magnet too strong or too close.",
      "Increase the air gap or use a weaker magnet; angle accuracy is not guaranteed." },
    { "STALE_ANGLE", "APP",
      "AVAL RD_AV = 0: no new angle in 20 ms although the sensor updates every <=171 us.",
      "Angle discarded; sensor processing stopped; power-cycle the sensor." }
};

// "OK" or every active name separated by commas, without spaces, so the field
// is always one whitespace-free token.
static inline void printTleErrorNames(Print &out, uint32_t mask)
{
    if (mask == ERR_NONE) {
        out.print("OK");
        return;
    }

    bool first = true;
    for (uint8_t bit = 0; bit < TLE_ERROR_COUNT; ++bit) {
        if ((mask & tleErrorBit(bit)) == 0) {
            continue;
        }
        if (!first) {
            out.print(',');
        }
        out.print(TLE_ERROR_TABLE[bit].name);
        first = false;
    }
}

// Fixed-width hexadecimal mask, e.g. 0x00100200.
static inline void printTleErrorMask(Print &out, uint32_t mask)
{
    char text[11];
    snprintf(text, sizeof(text), "0x%08lX", (unsigned long)mask);
    out.print(text);
}
