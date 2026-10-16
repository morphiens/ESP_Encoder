/*
 * Read-only TLE5012B SSC transport for an Arduino SPIClass instance.
 *
 * Each Tle5012bBus owns one SPI peripheral and one sensor. This is required
 * by the custom board, where encoder 1 uses FSPI and encoder 2 uses HSPI.
 *
 * Sources:
 *   [DS] TLE5012B Data Sheet Rev. 2.1, Tables 15, 17 and 18
 *   [UM] TLE5012B User Manual Rev. 1.2, CRC example and STAT register
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "tle5012b_errors.h"

// Read commands: RW=1, the register-appropriate LOCK value, and ND>=1.
constexpr uint16_t CMD_READ_STAT  = 0x8001;  // STAT, one data word + Safety Word
constexpr uint16_t CMD_READ_AVAL  = 0x8021;  // AVAL, one data word + Safety Word
constexpr uint16_t CMD_READ_FSYNC = 0xD051;  // FSYNC, one data word + Safety Word
constexpr uint16_t CMD_READ_DMAG  = 0x8141;  // D_MAG, lock 0000 [UM Lock field]

constexpr uint16_t AVAL_RD_AV       = 0x8000;
constexpr uint16_t AVAL_ANGLE_MASK  = 0x7FFF;
constexpr uint16_t FSYNC_TEMPR_MASK = 0x01FF;
constexpr uint16_t DMAG_MASK        = 0x03FF;

// D_MAG = sqrt(X^2 + Y^2) / 64 after offset compensation [UM D_MAG].
constexpr float DMAG_DIGITS_PER_LSB = 64.0f;

// X/Y amplitude limits [DS Table 10].
constexpr float FIELD_NO_MAGNET_MAX_DIGITS = 4096.0f;   // amplitude without magnet
constexpr float FIELD_MIN_DIGITS           = 6000.0f;   // 30 mT, ambient
constexpr float FIELD_MAX_DIGITS           = 15781.0f;  // ambient
// One D_MAG step, so quantisation noise at a limit cannot toggle the state.
constexpr float FIELD_HYSTERESIS_DIGITS    = DMAG_DIGITS_PER_LSB;

// Safety Word bits 15:12 are active-low.
constexpr uint16_t SW_BIT_RESET  = 0x8000;
constexpr uint16_t SW_BIT_SYSTEM = 0x4000;
constexpr uint16_t SW_BIT_ACCESS = 0x2000;
constexpr uint16_t SW_BIT_ANGLE  = 0x1000;
constexpr uint8_t RESP_SLAVE_0   = 0x0E;

constexpr float TLE_TEMP_MIN_C = -40.0f;
constexpr float TLE_TEMP_MAX_C = 150.0f;

struct SscFrame {
    uint16_t command;
    uint16_t data[2];
    uint8_t dataWords;
    uint16_t safety;
    uint32_t errors;

    SscFrame()
        : command(0),
          data{0, 0},
          dataWords(0),
          safety(0),
          errors(ERR_NO_RESPONSE)
    {
    }

    bool frameOk() const
    {
        return (errors & ERR_FRAME_MASK) == 0;
    }
};

static inline uint8_t tleSafetyResp(uint16_t safety)
{
    return (uint8_t)((safety >> 8) & 0x0F);
}

static inline uint8_t tleSafetyCrc(uint16_t safety)
{
    return (uint8_t)(safety & 0x00FF);
}

// SAE J1850 CRC: polynomial 0x1D, seed 0xFF, inverted result.
static inline uint8_t tleCrc8J1850(const uint8_t *bytes, uint8_t length)
{
    uint8_t crc = 0xFF;

    for (uint8_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80)
                ? (uint8_t)((crc << 1) ^ 0x1D)
                : (uint8_t)(crc << 1);
        }
    }

    return (uint8_t)~crc;
}

static inline uint8_t tleSscCrc(
    uint16_t command,
    const uint16_t *data,
    uint8_t dataWords)
{
    uint8_t bytes[6];
    uint8_t count = 0;

    bytes[count++] = (uint8_t)(command >> 8);
    bytes[count++] = (uint8_t)(command & 0x00FF);

    for (uint8_t i = 0; i < dataWords && i < 2; ++i) {
        bytes[count++] = (uint8_t)(data[i] >> 8);
        bytes[count++] = (uint8_t)(data[i] & 0x00FF);
    }

    return tleCrc8J1850(bytes, count);
}

static inline uint32_t tleCheckFrame(
    uint16_t command,
    const uint16_t *data,
    uint8_t dataWords,
    uint16_t safety)
{
    bool allZero = (safety == 0x0000);
    bool allOnes = (safety == 0xFFFF);

    for (uint8_t i = 0; i < dataWords; ++i) {
        allZero = allZero && (data[i] == 0x0000);
        allOnes = allOnes && (data[i] == 0xFFFF);
    }

    if (allZero || allOnes) {
        return ERR_NO_RESPONSE;
    }

    uint32_t errors = ERR_NONE;

    if (tleSscCrc(command, data, dataWords) != tleSafetyCrc(safety)) {
        errors |= ERR_CRC;
    }
    if (tleSafetyResp(safety) != RESP_SLAVE_0) {
        errors |= ERR_RESP;
    }

    // A corrupt frame cannot safely be interpreted as a sensor diagnosis.
    if (errors != ERR_NONE) {
        return errors;
    }

    if ((safety & SW_BIT_RESET) == 0) {
        errors |= ERR_SW_RESET;
    }
    if ((safety & SW_BIT_SYSTEM) == 0) {
        errors |= ERR_SW_SYSTEM;
    }
    if ((safety & SW_BIT_ACCESS) == 0) {
        errors |= ERR_SW_ACCESS;
    }
    if ((safety & SW_BIT_ANGLE) == 0) {
        errors |= ERR_SW_ANGLE_INVALID;
    }

    return errors;
}

static inline uint32_t tleDecodeStat(uint16_t stat)
{
    uint32_t errors = ERR_NONE;

    if (stat & (1U << 0))  errors |= ERR_S_RST;
    if (stat & (1U << 1))  errors |= ERR_S_WD;
    if (stat & (1U << 2))  errors |= ERR_S_VR;
    if (stat & (1U << 3))  errors |= ERR_S_FUSE;
    if (stat & (1U << 4))  errors |= ERR_S_DSPU;
    if (stat & (1U << 5))  errors |= ERR_S_OV;
    if (stat & (1U << 6))  errors |= ERR_S_XYOL;
    if (stat & (1U << 7))  errors |= ERR_S_MAGOL;
    if (stat & (1U << 9))  errors |= ERR_S_ADCT;
    if (stat & (1U << 10)) errors |= ERR_S_ROM;
    if (stat & (1U << 11)) errors |= ERR_NO_GMR_XY;
    if (stat & (1U << 12)) errors |= ERR_NO_GMR_A;

    return errors;
}

static inline float tleFieldAmplitudeDigits(uint16_t dmag)
{
    return (float)(dmag & DMAG_MASK) * DMAG_DIGITS_PER_LSB;
}

// Returns exactly one of NO_MAGNET, MAG_WEAK, MAG_STRONG, or ERR_NONE.
// `previous` is the last field state; a state is left only once the amplitude
// is past its limit by FIELD_HYSTERESIS_DIGITS.
static inline uint32_t tleClassifyField(float amplitude, uint32_t previous)
{
    const float h = FIELD_HYSTERESIS_DIGITS;

    if (amplitude <= FIELD_NO_MAGNET_MAX_DIGITS) {
        return ERR_NO_MAGNET;
    }
    if ((previous & ERR_NO_MAGNET) &&
        amplitude <= FIELD_NO_MAGNET_MAX_DIGITS + h) {
        return ERR_NO_MAGNET;
    }
    if (amplitude < FIELD_MIN_DIGITS) {
        return ERR_MAG_WEAK;
    }
    if ((previous & ERR_MAG_WEAK) && amplitude < FIELD_MIN_DIGITS + h) {
        return ERR_MAG_WEAK;
    }
    if (amplitude > FIELD_MAX_DIGITS) {
        return ERR_MAG_STRONG;
    }
    if ((previous & ERR_MAG_STRONG) && amplitude > FIELD_MAX_DIGITS - h) {
        return ERR_MAG_STRONG;
    }
    return ERR_NONE;
}

static inline float tleAngleDegrees(uint16_t aval)
{
    return (float)(aval & AVAL_ANGLE_MASK) * 360.0f / 32768.0f;
}

// Infineon's conversion for the signed 9-bit FSYNC.TEMPR field.
static inline float tleTemperatureC(uint16_t fsync)
{
    int16_t raw = (int16_t)(fsync & FSYNC_TEMPR_MASK);
    if (raw & 0x0100) {
        raw -= 0x0200;
    }
    return ((float)raw + 152.0f) / 2.776f;
}

class Tle5012bBus {
public:
    Tle5012bBus(
        SPIClass &spi,
        int8_t sckPin,
        int8_t misoPin,
        int8_t mosiPin,
        int8_t csPin,
        uint32_t clockHz = 1000000UL)
        : spi_(spi),
          sckPin_(sckPin),
          misoPin_(misoPin),
          mosiPin_(mosiPin),
          csPin_(csPin),
          settings_(clockHz, MSBFIRST, SPI_MODE1)
    {
    }

    void begin()
    {
        pinMode(csPin_, OUTPUT);
        digitalWrite(csPin_, HIGH);
        spi_.begin(sckPin_, misoPin_, mosiPin_, -1);
    }

    SscFrame read(uint16_t command)
    {
        SscFrame frame;
        frame.command = command;
        frame.dataWords = (uint8_t)(command & 0x000F);

        // This implementation intentionally supports the one- and two-word
        // reads used by this firmware, without risking a buffer overrun.
        if (frame.dataWords == 0 || frame.dataWords > 2) {
            frame.errors = ERR_NO_RESPONSE;
            return frame;
        }

        uint16_t received[3] = {0, 0, 0};
        const uint8_t receiveWords = frame.dataWords + 1;

        spi_.beginTransaction(settings_);
        digitalWrite(csPin_, LOW);
        delayMicroseconds(1);  // tCSs >= 105 ns

        spi_.transfer16(command);
        delayMicroseconds(1);  // twr_delay >= 130 ns

        // The custom PCB splits DATA to MOSI and MISO through series
        // resistors, as shown by the datasheet's SSC application circuit.
        // Clocking idle-high fill words requires no ESP32-specific GPIO
        // remapping; the fitted resistors limit contention on DATA.
        for (uint8_t i = 0; i < receiveWords; ++i) {
            received[i] = spi_.transfer16(0xFFFF);
        }

        digitalWrite(csPin_, HIGH);
        spi_.endTransaction();
        delayMicroseconds(1);  // tCSoff >= 600 ns

        for (uint8_t i = 0; i < frame.dataWords; ++i) {
            frame.data[i] = received[i];
        }
        frame.safety = received[frame.dataWords];
        frame.errors = tleCheckFrame(
            frame.command,
            frame.data,
            frame.dataWords,
            frame.safety);

        return frame;
    }

    // Repeats a read that fails its frame check. `encounteredFrameErrors`
    // collects the frame errors of every attempt and `lastFailed` holds the
    // raw words of the most recent failed attempt, so a transient fault that
    // a retry recovered from is still reported precisely.
    SscFrame readWithRetry(
        uint16_t command,
        uint8_t retryLimit,
        uint32_t &encounteredFrameErrors,
        uint8_t &retriesUsed,
        SscFrame &lastFailed)
    {
        encounteredFrameErrors = ERR_NONE;
        retriesUsed = 0;
        SscFrame frame;

        for (uint8_t attempt = 0; attempt <= retryLimit; ++attempt) {
            frame = read(command);
            encounteredFrameErrors |= frame.errors & ERR_FRAME_MASK;

            if (frame.frameOk()) {
                break;
            }
            lastFailed = frame;
            if (attempt == retryLimit) {
                break;
            }
            ++retriesUsed;
        }

        return frame;
    }

private:
    SPIClass &spi_;
    int8_t sckPin_;
    int8_t misoPin_;
    int8_t mosiPin_;
    int8_t csPin_;
    SPISettings settings_;
};
