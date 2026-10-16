#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "tle5012b_errors.h"
#include "tle5012b_ssc.h"

// ============================================================================
// ESP32-S3 CUSTOM BOARD — TWO INDEPENDENT TLE5012B SSC BUSES
// ============================================================================
//
// Encoder 1: CS GPIO13, MOSI GPIO11, MISO GPIO10, SCK GPIO12 (FSPI)
// Encoder 2: CS GPIO 9, MOSI GPIO 6, MISO GPIO 7, SCK GPIO 8 (HSPI)
//
// Each sensor DATA signal reaches that bus's MOSI and MISO through the series
// resistors already fitted to the custom PCB. GPIO4 / CS1_in is not used.
//
// ============================================================================
// SERIAL OUTPUT
// ============================================================================
//
// At boot, one legend line per error code:
//   HL_INFO BIT=20 CODE=NO_MAGNET LAYER=APP
//
// Every 20 ms, one sample line with a fixed number of fields:
//   HL <n> <micros> ENC1: <deg> deg | ENC2: <deg> deg | ENC1_ERR: <mask> <names> | ENC2_ERR: <mask> <names>
//   <deg>   filtered angle, or nan when this sample's angle must not be used
//   <mask>  0x%08X error bit mask (bit numbers in the legend)
//   <names> OK, or every active code joined by commas without spaces
//
// Detail lines, only when PRINT_ERROR is 1, when a code appears and every
// second while it stays active:
//   HL_ERR ENC1 CODE=NO_MAGNET BIT=20 LAYER=APP STATE=ACTIVE COUNT=1
//          DATA="amplitude=1984 digits D_MAG=31" ERROR="..." FIX="..."
//   STATE=ACTIVE  still active now
//   STATE=SEEN    occurred, but already gone again (transient)
//   COUNT         number of times the code has become active since boot
//
// When codes go away:
//   HL_ERR ENC1 CLEARED=NO_MAGNET STATUS=0x00000000 OK

static constexpr uint8_t ENC1_CS_PIN   = 13;
static constexpr uint8_t ENC1_MOSI_PIN = 11;
static constexpr uint8_t ENC1_MISO_PIN = 10;
static constexpr uint8_t ENC1_SCK_PIN  = 12;

static constexpr uint8_t ENC2_CS_PIN   = 9;
static constexpr uint8_t ENC2_MOSI_PIN = 6;
static constexpr uint8_t ENC2_MISO_PIN = 7;
static constexpr uint8_t ENC2_SCK_PIN  = 8;

static constexpr uint32_t SERIAL_BAUD          = 115200UL;
static constexpr uint32_t SSC_CLOCK_HZ         = 1000000UL;
static constexpr uint32_t SAMPLE_PERIOD_US     = 20000UL;  // 50 Hz
static constexpr uint32_t DIAGNOSTIC_PERIOD_MS = 200UL;    // STAT + temperature
static constexpr uint32_t DETAIL_REPEAT_MS     = 1000UL;
// 0 = do not print the detailed HL_ERR analysis.
// 1 = print it when an error appears, and again every 1 s while it stays active.
static constexpr uint8_t  PRINT_ERROR          = 0;
static constexpr uint8_t  FRAME_RETRIES        = 2;
static constexpr size_t   FILTER_WINDOW        = 8;

// Detail lines are spread over samples so a burst of errors cannot block
// the 50 Hz sampling on a 115200 baud link.
static constexpr uint8_t DETAIL_LINES_PER_SAMPLE = 2;

SPIClass encoder1SPI(FSPI);
SPIClass encoder2SPI(HSPI);

Tle5012bBus encoder1Bus(
    encoder1SPI, ENC1_SCK_PIN, ENC1_MISO_PIN, ENC1_MOSI_PIN, ENC1_CS_PIN,
    SSC_CLOCK_HZ);

Tle5012bBus encoder2Bus(
    encoder2SPI, ENC2_SCK_PIN, ENC2_MISO_PIN, ENC2_MOSI_PIN, ENC2_CS_PIN,
    SSC_CLOCK_HZ);

// ============================================================================
// WRAP-SAFE CIRCULAR MOVING AVERAGE
// ============================================================================

class CircularAngleFilter {
public:
    CircularAngleFilter()
    {
        reset();
    }

    void reset()
    {
        index_ = 0;
        count_ = 0;
        sumSin_ = 0.0f;
        sumCos_ = 0.0f;
        for (size_t i = 0; i < FILTER_WINDOW; ++i) {
            sinBuffer_[i] = 0.0f;
            cosBuffer_[i] = 0.0f;
        }
    }

    float update(float angleDeg)
    {
        const float radians = angleDeg * DEG_TO_RAD;
        const float newSin = sinf(radians);
        const float newCos = cosf(radians);

        if (count_ == FILTER_WINDOW) {
            sumSin_ -= sinBuffer_[index_];
            sumCos_ -= cosBuffer_[index_];
        } else {
            ++count_;
        }

        sinBuffer_[index_] = newSin;
        cosBuffer_[index_] = newCos;
        sumSin_ += newSin;
        sumCos_ += newCos;

        index_ = (index_ + 1) % FILTER_WINDOW;

        const float magnitudeSquared =
            (sumSin_ * sumSin_) + (sumCos_ * sumCos_);
        if (magnitudeSquared < 0.000001f) {
            return angleDeg;
        }

        float averageDeg = atan2f(sumSin_, sumCos_) * RAD_TO_DEG;
        if (averageDeg < 0.0f) {
            averageDeg += 360.0f;
        }
        if (averageDeg >= 360.0f) {
            averageDeg -= 360.0f;
        }
        return averageDeg;
    }

private:
    float sinBuffer_[FILTER_WINDOW];
    float cosBuffer_[FILTER_WINDOW];
    size_t index_;
    size_t count_;
    float sumSin_;
    float sumCos_;
};

// ============================================================================
// PER-ENCODER STATE
// ============================================================================

struct EncoderState {
    const char *name;
    Tle5012bBus &bus;
    CircularAngleFilter filter;

    float filteredAngleDeg;

    // Error state. The mask shown on a line is the union of all four:
    //   sampleErrors  events of this sample only (FRAME, SAFETY, STALE_ANGLE)
    //   statErrors    decoded from the last good STAT read, held until the next
    //   fieldErrors   NO_MAGNET / MAG_WEAK / MAG_STRONG from the last D_MAG read
    //   tempErrors    TEMP_LIMIT from the last temperature read
    uint32_t sampleErrors;
    uint32_t statErrors;
    uint32_t fieldErrors;
    uint32_t tempErrors;
    uint32_t carriedErrors;  // startup events, shown on the first line
    bool fieldKnown;

    // Measured values, printed as DATA="..." to pinpoint each code.
    uint16_t badFrameCommand;
    uint16_t badFrameData;
    uint16_t badFrameSafety;
    uint16_t faultSafety;
    uint16_t lastStat;
    uint16_t lastDmag;
    uint16_t staleAval;
    float temperatureC;

    // Reporting.
    uint32_t lastLineErrors;
    uint32_t pendingDetails;
    uint32_t lastDetailRepeatMs;
    uint32_t onsetCounts[TLE_ERROR_COUNT];
    uint32_t retryCount;

    EncoderState(const char *encoderName, Tle5012bBus &encoderBus)
        : name(encoderName),
          bus(encoderBus),
          filteredAngleDeg(NAN),
          sampleErrors(ERR_NONE),
          statErrors(ERR_NONE),
          fieldErrors(ERR_NONE),
          tempErrors(ERR_NONE),
          carriedErrors(ERR_NONE),
          fieldKnown(false),
          badFrameCommand(0),
          badFrameData(0),
          badFrameSafety(0),
          faultSafety(0),
          lastStat(0),
          lastDmag(0),
          staleAval(0),
          temperatureC(NAN),
          lastLineErrors(ERR_NONE),
          pendingDetails(ERR_NONE),
          lastDetailRepeatMs(0),
          onsetCounts{0},
          retryCount(0)
    {
    }

    uint32_t activeErrors() const
    {
        return sampleErrors | statErrors | fieldErrors | tempErrors;
    }
};

EncoderState encoder1("ENC1", encoder1Bus);
EncoderState encoder2("ENC2", encoder2Bus);

// ============================================================================
// SENSOR ACCESS
// ============================================================================

// Reads with retry and records every error seen on any attempt into
// `errors`. Returns the final frame. `retried` tells the caller whether the
// returned frame came from a repeated read.
SscFrame readChecked(
    EncoderState &encoder,
    uint16_t command,
    uint32_t &errors,
    bool &retried)
{
    uint32_t attemptFrameErrors = ERR_NONE;
    uint8_t retriesUsed = 0;
    SscFrame failed;

    SscFrame frame = encoder.bus.readWithRetry(
        command, FRAME_RETRIES, attemptFrameErrors, retriesUsed, failed);

    if (attemptFrameErrors != ERR_NONE) {
        encoder.badFrameCommand = failed.command;
        encoder.badFrameData = failed.data[0];
        encoder.badFrameSafety = failed.safety;
    }

    if (frame.frameOk() && (frame.errors & ERR_SAFETY_MASK)) {
        encoder.faultSafety = frame.safety;
    }

    encoder.retryCount += retriesUsed;
    retried = (retriesUsed > 0);
    errors |= attemptFrameErrors | frame.errors;
    return frame;
}

SscFrame readChecked(EncoderState &encoder, uint16_t command, uint32_t &errors)
{
    bool retried = false;
    return readChecked(encoder, command, errors, retried);
}

// Reads STAT and replaces the held STAT state. Reading STAT also releases the
// latched Safety Word summary bits [DS Table 18]. A corrupt STAT frame is
// never decoded; the previous STAT state is kept instead.
void readStatus(EncoderState &encoder, uint32_t &errors, uint32_t ignoreMask)
{
    const SscFrame frame = readChecked(encoder, CMD_READ_STAT, errors);
    if (!frame.frameOk()) {
        return;
    }

    encoder.lastStat = frame.data[0];
    encoder.statErrors = tleDecodeStat(encoder.lastStat) & ~ignoreMask;
}

void readField(EncoderState &encoder, uint32_t &errors)
{
    const SscFrame frame = readChecked(encoder, CMD_READ_DMAG, errors);
    if (!frame.frameOk()) {
        return;
    }

    encoder.lastDmag = frame.data[0] & DMAG_MASK;
    encoder.fieldErrors = tleClassifyField(
        tleFieldAmplitudeDigits(encoder.lastDmag), encoder.fieldErrors);
    encoder.fieldKnown = true;
}

void readTemperature(EncoderState &encoder, uint32_t &errors)
{
    const SscFrame frame = readChecked(encoder, CMD_READ_FSYNC, errors);
    if (!frame.frameOk()) {
        return;
    }

    encoder.temperatureC = tleTemperatureC(frame.data[0]);
    const bool outside = encoder.temperatureC < TLE_TEMP_MIN_C ||
                         encoder.temperatureC > TLE_TEMP_MAX_C;
    encoder.tempErrors = outside ? ERR_TEMP_LIMIT : ERR_NONE;
}

// ============================================================================
// SAMPLING AND PERIODIC DIAGNOSTICS
// ============================================================================

void sampleEncoder(EncoderState &encoder)
{
    uint32_t errors = ERR_NONE;
    bool angleRetried = false;

    const SscFrame angleFrame =
        readChecked(encoder, CMD_READ_AVAL, errors, angleRetried);

    // The sensor computes a new angle at least every 170.6 us [DS Table 12],
    // and samples are 20 ms apart, so RD_AV = 0 means processing stopped.
    // A retried read follows the previous one within microseconds and may
    // legitimately see RD_AV = 0, so only a first-attempt read is judged.
    if (angleFrame.frameOk() && !angleRetried &&
        (angleFrame.data[0] & AVAL_RD_AV) == 0) {
        errors |= ERR_STALE_ANGLE;
        encoder.staleAval = angleFrame.data[0];
    }

    readField(encoder, errors);

    // Any Safety Word summary bit: read STAT now for the exact cause.
    if (errors & ERR_SAFETY_MASK) {
        readStatus(encoder, errors, ERR_NONE);
    }

    encoder.sampleErrors = (errors & ~ERR_STAT_MASK) | encoder.carriedErrors;
    encoder.carriedErrors = ERR_NONE;

    const bool angleUsable =
        angleFrame.frameOk() &&
        encoder.fieldKnown &&
        (encoder.activeErrors() & ERR_ANGLE_UNUSABLE_MASK) == 0;

    if (angleUsable) {
        encoder.filteredAngleDeg =
            encoder.filter.update(tleAngleDegrees(angleFrame.data[0]));
    } else {
        // Do not let samples from before the fault blend into the first
        // samples after recovery.
        encoder.filter.reset();
        encoder.filteredAngleDeg = NAN;
    }
}

void pollEncoderDiagnostics(EncoderState &encoder)
{
    uint32_t errors = ERR_NONE;

    readTemperature(encoder, errors);
    readStatus(encoder, errors, ERR_NONE);

    // Frame or Safety Word errors of these reads belong to this sample too.
    encoder.sampleErrors |= errors & ~ERR_STAT_MASK;
}

// ============================================================================
// SERIAL OUTPUT
// ============================================================================

void printAngle(float angleDeg)
{
    if (isnan(angleDeg)) {
        Serial.print("nan");
    } else {
        Serial.print(angleDeg, 4);
    }
}

void printErrorField(uint32_t mask)
{
    printTleErrorMask(Serial, mask);
    Serial.print(' ');
    printTleErrorNames(Serial, mask);
}

void printReadings(
    uint32_t sampleNumber,
    uint32_t timestampUs,
    const EncoderState &enc1,
    const EncoderState &enc2)
{
    Serial.print("HL ");
    Serial.print(sampleNumber);
    Serial.print(' ');
    Serial.print(timestampUs);

    Serial.print(" ENC1: ");
    printAngle(enc1.filteredAngleDeg);
    Serial.print(" deg | ENC2: ");
    printAngle(enc2.filteredAngleDeg);
    Serial.print(" deg | ENC1_ERR: ");
    printErrorField(enc1.activeErrors());
    Serial.print(" | ENC2_ERR: ");
    printErrorField(enc2.activeErrors());
    Serial.println();
}

void printHex16(const char *label, uint16_t value)
{
    char text[24];
    snprintf(text, sizeof(text), "%s=0x%04X", label, value);
    Serial.print(text);
}

// The measured values behind one code.
void printErrorData(const EncoderState &encoder, uint8_t bit)
{
    const uint32_t mask = tleErrorBit(bit);

    if (mask & ERR_FRAME_MASK) {
        printHex16("cmd", encoder.badFrameCommand);
        Serial.print(' ');
        printHex16("data", encoder.badFrameData);
        Serial.print(' ');
        printHex16("sw", encoder.badFrameSafety);
        Serial.print(" retries_total=");
        Serial.print(encoder.retryCount);
    } else if (mask & ERR_SAFETY_MASK) {
        printHex16("sw", encoder.faultSafety);
    } else if (mask & ERR_STAT_MASK) {
        printHex16("stat", encoder.lastStat);
    } else if (mask & ERR_FIELD_MASK) {
        Serial.print("amplitude=");
        Serial.print((uint32_t)tleFieldAmplitudeDigits(encoder.lastDmag));
        Serial.print(" digits D_MAG=");
        Serial.print(encoder.lastDmag);
    } else if (mask & ERR_TEMP_LIMIT) {
        Serial.print("temp=");
        Serial.print(encoder.temperatureC, 1);
        Serial.print(" C");
    } else if (mask & ERR_STALE_ANGLE) {
        printHex16("aval", encoder.staleAval);
    }
}

void printErrorDetail(const EncoderState &encoder, uint8_t bit)
{
    const TleErrorInfo &info = TLE_ERROR_TABLE[bit];
    const bool active = (encoder.activeErrors() & tleErrorBit(bit)) != 0;

    Serial.print("HL_ERR ");
    Serial.print(encoder.name);
    Serial.print(" CODE=");
    Serial.print(info.name);
    Serial.print(" BIT=");
    Serial.print(bit);
    Serial.print(" LAYER=");
    Serial.print(info.layer);
    Serial.print(active ? " STATE=ACTIVE" : " STATE=SEEN");
    Serial.print(" COUNT=");
    Serial.print(encoder.onsetCounts[bit]);
    Serial.print(" DATA=\"");
    printErrorData(encoder, bit);
    Serial.print("\" ERROR=\"");
    Serial.print(info.meaning);
    Serial.print("\" FIX=\"");
    Serial.print(info.fix);
    Serial.println('"');
}

// Compares this line's errors with the previous line: counts and queues new
// codes, reports cleared codes, and re-queues active codes every second.
void trackErrorChanges(EncoderState &encoder, uint32_t nowMs)
{
    const uint32_t current = encoder.activeErrors();
    const uint32_t appeared = current & ~encoder.lastLineErrors;
    const uint32_t cleared = encoder.lastLineErrors & ~current;

    for (uint8_t bit = 0; bit < TLE_ERROR_COUNT; ++bit) {
        if (appeared & tleErrorBit(bit)) {
            ++encoder.onsetCounts[bit];
        }
    }
    if (PRINT_ERROR) {
        encoder.pendingDetails |= appeared;
    }

    if (cleared != ERR_NONE) {
        Serial.print("HL_ERR ");
        Serial.print(encoder.name);
        Serial.print(" CLEARED=");
        printTleErrorNames(Serial, cleared);
        Serial.print(" STATUS=");
        printErrorField(current);
        Serial.println();
    }

    if (PRINT_ERROR) {
        if (appeared != ERR_NONE) {
            encoder.lastDetailRepeatMs = nowMs;
        } else if (current != ERR_NONE &&
                   (uint32_t)(nowMs - encoder.lastDetailRepeatMs) >=
                       DETAIL_REPEAT_MS) {
            encoder.pendingDetails |= current;
            encoder.lastDetailRepeatMs = nowMs;
        }
    }

    encoder.lastLineErrors = current;
}

// Prints up to `budget` queued detail lines for one encoder, lowest bit
// first. Returns the unused budget.
uint8_t printPendingDetails(EncoderState &encoder, uint8_t budget)
{
    for (uint8_t bit = 0; bit < TLE_ERROR_COUNT && budget > 0; ++bit) {
        const uint32_t mask = tleErrorBit(bit);
        if ((encoder.pendingDetails & mask) == 0) {
            continue;
        }
        printErrorDetail(encoder, bit);
        encoder.pendingDetails &= ~mask;
        --budget;
    }
    return budget;
}

void printErrorLegend()
{
    for (uint8_t bit = 0; bit < TLE_ERROR_COUNT; ++bit) {
        Serial.print("HL_INFO BIT=");
        Serial.print(bit);
        Serial.print(" CODE=");
        Serial.print(TLE_ERROR_TABLE[bit].name);
        Serial.print(" LAYER=");
        Serial.println(TLE_ERROR_TABLE[bit].layer);
    }
}

// ============================================================================
// STARTUP VALIDATION
// ============================================================================

bool decoderSelfTest()
{
    const uint16_t goldenData[1] = {0x0804};
    const uint16_t zeroData[1] = {0x0000};
    const uint16_t onesData[1] = {0xFFFF};

    // CRC golden vector from the User Manual SSC example.
    if (tleSscCrc(0x5081, goldenData, 1) != 0x89) return false;

    struct FrameVector {
        const uint16_t *data;
        uint16_t safety;
        uint32_t expected;
    };
    const FrameVector frames[] = {
        { goldenData, 0xFE89, ERR_NONE },
        { zeroData,   0x0000, ERR_NO_RESPONSE },
        { onesData,   0xFFFF, ERR_NO_RESPONSE },
        { goldenData, 0xFE88, ERR_CRC },
        { goldenData, 0xFF89, ERR_RESP },
        { goldenData, 0x7E89, ERR_SW_RESET },
        { goldenData, 0xBE89, ERR_SW_SYSTEM },
        { goldenData, 0xDE89, ERR_SW_ACCESS },
        { goldenData, 0xEE89, ERR_SW_ANGLE_INVALID },
        { goldenData, 0x0E89, ERR_SAFETY_MASK },
    };
    for (const FrameVector &v : frames) {
        if (tleCheckFrame(0x5081, v.data, 1, v.safety) != v.expected) {
            return false;
        }
    }

    static const uint32_t expectedStat[16] = {
        ERR_S_RST, ERR_S_WD, ERR_S_VR, ERR_S_FUSE,
        ERR_S_DSPU, ERR_S_OV, ERR_S_XYOL, ERR_S_MAGOL,
        ERR_NONE, ERR_S_ADCT, ERR_S_ROM, ERR_NO_GMR_XY,
        ERR_NO_GMR_A, ERR_NONE, ERR_NONE, ERR_NONE
    };
    for (uint8_t bit = 0; bit < 16; ++bit) {
        if (tleDecodeStat((uint16_t)(1U << bit)) != expectedStat[bit]) {
            return false;
        }
    }
    if (tleDecodeStat(0xFFFF) != ERR_STAT_MASK) return false;

    // Field classification, including hysteresis at each limit.
    struct FieldVector {
        float amplitude;
        uint32_t previous;
        uint32_t expected;
    };
    const FieldVector fields[] = {
        {     0.0f, ERR_NONE,       ERR_NO_MAGNET },
        {  4096.0f, ERR_NONE,       ERR_NO_MAGNET },
        {  4160.0f, ERR_NO_MAGNET,  ERR_NO_MAGNET },
        {  4224.0f, ERR_NO_MAGNET,  ERR_MAG_WEAK },
        {  5999.0f, ERR_NONE,       ERR_MAG_WEAK },
        {  6016.0f, ERR_MAG_WEAK,   ERR_MAG_WEAK },
        {  6080.0f, ERR_MAG_WEAK,   ERR_NONE },
        {  9500.0f, ERR_NONE,       ERR_NONE },
        { 15781.0f, ERR_NONE,       ERR_NONE },
        { 15808.0f, ERR_NONE,       ERR_MAG_STRONG },
        { 15744.0f, ERR_MAG_STRONG, ERR_MAG_STRONG },
        { 15680.0f, ERR_MAG_STRONG, ERR_NONE },
    };
    for (const FieldVector &v : fields) {
        if (tleClassifyField(v.amplitude, v.previous) != v.expected) {
            return false;
        }
    }

    return true;
}

// S_RST and SW_RESET are expected once after power-up; this read clears them.
// Every other startup fault stays reportable.
void captureStartupStatus(EncoderState &encoder)
{
    uint32_t errors = ERR_NONE;
    readStatus(encoder, errors, ERR_S_RST);
    encoder.carriedErrors = errors & ~(ERR_STAT_MASK | ERR_SW_RESET);
}

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

void setup()
{
    Serial.begin(SERIAL_BAUD);

    encoder1Bus.begin();
    encoder2Bus.begin();

    // Datasheet tPon is at most 7 ms. Ten milliseconds gives startup margin.
    delay(10);

    printErrorLegend();

    if (!decoderSelfTest()) {
        Serial.println(
            "HL_ERR SYSTEM CODE=DECODER_SELF_TEST LAYER=INTERNAL "
            "ERROR=\"CRC/decoder self-test failed\" "
            "FIX=\"Do not trust diagnostics; reflash known-good firmware\"");
    }

    captureStartupStatus(encoder1);
    captureStartupStatus(encoder2);
}

void loop()
{
    static uint32_t sampleNumber = 0;
    static uint32_t nextSampleUs = 0;
    static uint32_t lastDiagnosticMs = 0;

    const uint32_t nowUs = micros();
    if ((int32_t)(nowUs - nextSampleUs) < 0) {
        delay(1);
        return;
    }

    // Schedule from now to avoid catch-up bursts after an unusually slow pass.
    nextSampleUs = nowUs + SAMPLE_PERIOD_US;

    sampleEncoder(encoder1);
    sampleEncoder(encoder2);

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastDiagnosticMs) >= DIAGNOSTIC_PERIOD_MS) {
        lastDiagnosticMs = nowMs;
        pollEncoderDiagnostics(encoder1);
        pollEncoderDiagnostics(encoder2);
    }

    ++sampleNumber;
    printReadings(sampleNumber, micros(), encoder1, encoder2);

    trackErrorChanges(encoder1, nowMs);
    trackErrorChanges(encoder2, nowMs);

    if (PRINT_ERROR) {
        uint8_t budget = DETAIL_LINES_PER_SAMPLE;
        budget = printPendingDetails(encoder1, budget);
        printPendingDetails(encoder2, budget);
    }
}
