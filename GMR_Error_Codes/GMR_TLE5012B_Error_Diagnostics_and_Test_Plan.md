# TLE5012B Error Diagnostics and Test Plan

**Target:** ESP32 communicating with a TLE5012B through SSC (SPI-compatible, bidirectional DATA line)  
**Purpose:** Define what the firmware should read, how it should map errors, and how to test those mappings without confusing injected software faults with genuine sensor faults.

## 1. Diagnostic layers

Use four layers. Keep their errors distinct so a corrupted bus frame is not mistaken for an internal sensor failure.

1. **Frame/communication checks performed by the ESP32** — no response, CRC failure, wrong response identifier, malformed transaction.
2. **Safety Word summary** — reset/watchdog indication, system-error summary, interface-access error, and angle-valid indication.
3. **STAT register details** — individual sensor diagnostic flags.
4. **Application checks** — configuration mismatch, stale measurements, implausible motion, temperature threshold, and magnetic-field margin. These are firmware inferences, not dedicated TLE5012B error flags.

The supplied project note identifies the same layers and lists the principal diagnostic registers: ACSTAT, D_MAG, ADC_X, ADC_Y, T_RAW, MOD_1, MOD_2, and MOD_3.

## 2. Registers and commands to use

| Register / data | Address | Command for one data word plus Safety Word | When to read | Purpose |
|---|---:|---:|---|---|
| AVAL | `0x02` | `0x8021` | Every measurement | Angle measurement and angle/freshness checks. |
| STAT | `0x00` | `0x8001` | Startup, after a Safety Word error, and during periodic diagnostics as appropriate | Detailed sensor status flags. Reading STAT affects the latched status indicated in the Safety Word. Save the raw value first. |
| ACSTAT | `0x01` | `0x8011` | Startup or diagnostics review | Check enabled diagnostic/self-test functions. |
| ADC_X | `0x10` | `0xD101` | When investigating X-channel measurements | Raw X measurement. |
| ADC_Y | `0x11` | `0xD111` | When investigating Y-channel measurements | Raw Y measurement. |
| D_MAG | `0x14` | `0x8141` | When investigating magnetic-field margin | Magnetic vector magnitude. |
| T_RAW | `0x15` | `0x8151` | Periodically if temperature monitoring is required | Raw internal temperature value; convert using the datasheet formula. |
| MOD_1 | `0x06` | `0xD061` | Startup/readback verification | Interface/operating configuration. |
| MOD_2 | `0x08` | `0xD081` | Startup/readback verification | Angular range, direction, and related configuration. |
| MOD_3 | `0x09` | `0xD091` | Startup/readback verification | Additional operating configuration. |

**Command-word reminder:** the low four command bits (`ND`) specify the number of data words. `ND = 0` means no Safety Word is appended. `0x8021` and `0x8001` request one data word followed by a Safety Word.

**Access-lock reminder:** the TLE5012B uses default access for registers `0x00–0x04`, `0x14–0x15`, `0x20`, and `0x30`. Configuration registers `0x05–0x11` require the configuration lock (`1010` in the command word). Confirm each command against the exact datasheet/library revision before using it.

## 3. Error codes already in `tle5012b_errors.h`

The current header defines seven errors. They are a bitmask: multiple bits can be set at the same time.

| Code | Bit | Detection source | Meaning |
|---|---:|---|---|
| `ERR_NO_RESPONSE` | 0 | ESP32 heuristic | Data word(s) and Safety Word are all zero or all one. This is a heuristic, not a definitive diagnosis of disconnection. |
| `ERR_CRC` | 1 | ESP32 CRC validation | Computed CRC does not match the received Safety Word CRC byte. |
| `ERR_RESP` | 2 | ESP32 response validation | Safety Word `RESP[11:8]` does not match the expected one-cold sensor/slave identifier. |
| `ERR_RESET` | 3 | Safety Word bit 15 | Reset/watchdog indication is active (bit is active-low). |
| `ERR_SYSTEM` | 4 | Safety Word bit 14 | Grouped sensor-system fault; read STAT to determine the detailed cause. |
| `ERR_ACCESS` | 5 | Safety Word bit 13 | Interface access error, such as an incorrect address/access lock. |
| `ERR_ANGLE_INVALID` | 6 | Safety Word bit 12 | Sensor says the angle is not valid, typically because `NO_GMR_A` or `NO_GMR_XY` is set. |

The current helper deliberately does not trust Safety Word status flags if CRC or RESP validation fails. Keep that behavior.

### Safety Word status polarity

The following Safety Word status bits are **active-low** (zero indicates the associated condition):

- bit 15: reset/watchdog indication
- bit 14: system error summary
- bit 13: interface-access error
- bit 12: angle-valid indication

For a healthy single sensor using slave number 0, the upper byte is typically `0xFE` (`1111 1110`): the four status flags are high and the one-cold RESP nibble is `1110`. The lower byte is the CRC.

## 4. STAT register fault map

The project's `StatBit` enum maps these bits. A value of 1 in these fault fields means the flag is asserted; `RD_ST` is information, not a fault.

| STAT bit | Flag | Interpretation | Suggested firmware code |
|---:|---|---|---|
| 0 | `S_RST` | Reset event recorded | `ERR_STAT_RESET` |
| 1 | `S_WD` | DSP watchdog expired | `ERR_STAT_WATCHDOG` |
| 2 | `S_VR` | Voltage/ground-related diagnostic fault | `ERR_STAT_VOLTAGE` |
| 3 | `S_FUSE` | Fuse/configuration integrity CRC fault | `ERR_STAT_FUSE_CRC` |
| 4 | `S_DSPU` | DSP/CORDIC/CCU startup self-test failure or still running | `ERR_STAT_DSP_SELFTEST` |
| 5 | `S_OV` | DSP overflow | `ERR_STAT_DSP_OVERFLOW` |
| 6 | `S_XYOL` | Raw X/Y signal out of its allowed range | `ERR_STAT_XY_OUT_OF_RANGE` |
| 7 | `S_MAGOL` | Magnetic vector magnitude outside its allowed range | `ERR_STAT_MAG_OUT_OF_RANGE` |
| 8 | Reserved | Do not map as an error | — |
| 9 | `S_ADCT` | ADC diagnostic test failure / active diagnostic | `ERR_STAT_ADC_TEST` |
| 10 | `S_ROM` | ROM CRC/self-test failure or startup check still running | `ERR_STAT_ROM_CRC` |
| 11 | `NO_GMR_XY` | Valid X/Y measurements unavailable | `ERR_STAT_NO_VALID_XY` |
| 12 | `NO_GMR_A` | Valid angle unavailable | `ERR_STAT_NO_VALID_ANGLE` |
| 13–14 | `S_NR` | Sensor/slave number field, not a fault flag | Compare with configured/expected value if applicable |
| 15 | `RD_ST` | Status-change information; not a fault by itself | Record if useful; do not map to a fault |

Use the raw STAT value as well as the mapped bitmask. Startup self-tests can still be running shortly after power-on, and not every flag has the same clearing/recovery behavior. Do not assume that every flag clears simply because STAT was read.

## 5. Additional application-level errors

These are useful, but the sensor does not directly return these names. Firmware must calculate them.

| Suggested code | How to detect it | Important caveat |
|---|---|---|
| `ERR_CONFIG_MISMATCH` | Read back relevant configuration registers and compare them with expected values. | Only compare defined fields; some registers may contain status or reserved bits. |
| `ERR_STALE_ANGLE` | Use the documented AVAL freshness/new-data indication or a defined sample-age policy. | Equal consecutive angle values are not enough; the shaft might be stationary. |
| `ERR_ANGLE_RATE` | Compute circular angle delta / elapsed time and compare with a configured physical maximum speed. | Choose a limit supported by the mechanics and sample rate. |
| `ERR_ANGLE_JUMP` | Detect a circular angle change beyond the application's allowed per-sample movement. | Handle the 0°/360° wrap correctly. |
| `ERR_TEMPERATURE_LIMIT` | Convert `T_RAW` using the datasheet conversion and compare with an application limit. | `T_RAW` is not degrees Celsius directly. |
| `ERR_MAGNET_MARGIN` | Read `D_MAG` and compare against application-defined calibrated lower/upper margins. | Characterize the intended magnet and air gap first; do not guess thresholds. |
| `ERR_STAT_SLAVE_MISMATCH` | Compare STAT's `S_NR` field with the expected ID, where configuration makes that meaningful. | Use the correct register-field encoding. |

If the application needs more than 16 error bits, widen the error enum and all return values to `uint32_t`; the existing `Err : uint16_t` and `check_frame()` return type are intended for the initial small mask.

## 6. Important implementation checks before hardware testing

1. **Make the validator bitmask-aware.** `err_name(Err e)` with a `switch` only names one exact value. It will return `"?"` when two error bits are combined. Print all set bits, or implement a formatter that iterates through the error mask.
2. **Keep frame failures separate from sensor status.** If CRC or RESP fails, reject the sample and do not trust the upper Safety Word flags.
3. **Store diagnostics before recovery.** Record timestamp, command, received data, Safety Word, CRC result, RESP nibble, STAT raw value, mapped errors, and retry count before a reset or reinitialization.
4. **Read angle and status consistently.** Use `AVAL` with `ND=1` so its Safety Word is available. Read STAT only when necessary or on a controlled periodic schedule; reading it changes the latched status behavior.
5. **Verify the CRC implementation against a golden vector.** The supplied header uses CRC-8 polynomial `0x1D`, seed `0xFF`, final inversion, and the worked vector `command=0x5081`, `data=0x0804`, expected CRC `0x89` (Safety Word example `0xFE89`). There is a documentation inconsistency between the Safety Word table and the worked CRC example in some manual revisions about whether STAT/RESP are included. Preserve the known-good worked vector for now and verify with the exact silicon/library/manual revision and captured frames before changing coverage.
6. **Check the actual bus implementation.** TLE5012B SSC uses one bidirectional DATA line. On a hardware SPI arrangement with separate MOSI/MISO pins joined through resistors, ensure the MCU releases the transmitting side before sampling the sensor response.
7. **Treat all-zero/all-one as a heuristic.** A raw angle of zero can be legitimate. The current check is stronger because it examines all data words plus the Safety Word, but it is still not a physical root-cause diagnosis.

## 7. Staged test plan

The safest way to test all codes is to test **the decoder in software first**, then test sensor behavior on hardware. Several internal faults cannot be safely or reliably forced on a production sensor.

### Phase A — establish a known-good baseline

1. Power the sensor and wait at least the documented startup/self-test interval before judging startup flags.
2. Read `AVAL` with `ND=1` and capture the angle data word and Safety Word.
3. Read `STAT` and capture its raw 16-bit value.
4. Log 100 or more consecutive transactions with the sensor stationary, then rotate the magnet slowly.
5. Confirm stable CRC, expected RESP, sensible angle changes, and no persistent status fault.

Save a known-good raw transaction. Do not begin by changing the sensor configuration or power rails.

### Phase B — host-side frame-validator tests (no physical fault required)

Use synthetic words to validate the current `check_frame()` logic. With the CRC convention currently encoded in the header, this is the supplied worked test vector:

| Test | Command | Data | Safety Word | Expected result |
|---|---:|---:|---:|---|
| Valid reference vector | `0x5081` | `0x0804` | `0xFE89` | `ERR_NONE` for expected slave 0 |
| Dead-bus heuristic | any suitable command | `0x0000` | `0x0000` | `ERR_NO_RESPONSE` |
| CRC corruption | `0x5081` | `0x0804` | `0xFE88` | `ERR_CRC` |
| RESP mismatch | `0x5081` | `0x0804` | `0xFF89` | `ERR_RESP` |
| Reset summary | `0x5081` | `0x0804` | `0x7E89` | `ERR_RESET` |
| System summary | `0x5081` | `0x0804` | `0xBE89` | `ERR_SYSTEM` |
| Interface access summary | `0x5081` | `0x0804` | `0xDE89` | `ERR_ACCESS` |
| Angle invalid summary | `0x5081` | `0x0804` | `0xEE89` | `ERR_ANGLE_INVALID` |

The last four vectors rely on the current implementation's CRC coverage (command + data only) and keep RESP at `1110`. If CRC coverage is changed after the manual/library reconciliation, regenerate the test vectors accordingly.

For `STAT`, test the decoder with synthetic status values one bit at a time: `1<<0`, `1<<1`, `1<<2`, `1<<3`, `1<<4`, `1<<5`, `1<<6`, `1<<7`, `1<<9`, `1<<10`, `1<<11`, and `1<<12`. Confirm each maps to exactly the expected individual error code. Also test `RD_ST` and reserved bit 8 to ensure they do not become fault codes.

### Phase C — safe sensor-on-board tests

| Error or feature | Safe test method | Expected observation | Notes |
|---|---|---|---|
| `ERR_NO_RESPONSE` | With power off, disconnect/hold inactive the sensor DATA connection only if your board makes this safe; otherwise inject a synthetic receive frame in the host test. | Validator detects an invalid all-zero/all-one frame; repeated real failures increment a counter. | Do not short a live GPIO to a rail. |
| `ERR_CRC` | Corrupt one received byte in a test copy of the frame before calling the decoder. | `ERR_CRC`. | This validates software. Real CRC errors can also be observed by logging naturally occurring failures. |
| `ERR_RESP` | Call the validator with an intentionally wrong expected slave number, or use a synthetic response word. | `ERR_RESP`. | No physical fault is required. |
| `ERR_RESET` / `S_RST` | Start logging before a controlled sensor power cycle, then read Safety Word/STAT after the startup interval. | Reset indication should be observable according to latch/readout behavior. | Save status before additional reads. |
| `ERR_ACCESS` | In a dedicated diagnostic build, issue a read to a known configuration register with the wrong access lock; verify the Safety Word. | Interface-access status should indicate an access error. | Use a read-only diagnostic test, then restore normal commands. |
| `ERR_SYSTEM`, `S_MAGOL`, `NO_GMR_XY`, `NO_GMR_A` | Slowly increase the magnet-to-sensor distance or remove the magnet in a controlled manner. | Depending on geometry and configuration, magnitude/valid-angle diagnostics may assert. | Not every sensor/magnet setup triggers every flag. Stop if readings become erratic; do not force the magnet against the IC. |
| `S_XYOL` | Prefer synthetic STAT decoder testing. A real out-of-range X/Y condition may require out-of-spec magnetic input. | Decoder maps the STAT bit when present. | Do not deliberately overdrive the sensor just to force the flag. |
| `S_VR` | Test the decoder synthetically. If physical verification is required, use a current-limited programmable supply and only voltage conditions explicitly permitted by the datasheet/test procedure. | STAT/Safety Word indicates voltage-related diagnostic if the monitored condition is detected. | Do not deliberately overvoltage or reverse-power the sensor. |
| `S_WD`, `S_DSPU`, `S_ROM`, `S_ADCT`, `S_OV`, `S_FUSE` | Test mapping with synthetic STAT words first. For hardware tests, follow an Infineon-approved diagnostic procedure only. | Correct individual mapping when those bits are actually returned. | Avoid deliberate internal test failures or arbitrary register corruption on the production board. |
| `ERR_CONFIG_MISMATCH` | Alter an expected-value constant in a test build while leaving the hardware unchanged. | Readback comparison raises the error. | Separately verify correct register access/lock. |
| `ERR_STALE_ANGLE`, `ERR_ANGLE_RATE`, `ERR_ANGLE_JUMP` | Feed recorded or synthetic angle sequences to the application checker. | Each policy triggers at the designed threshold, including across 359°→0°. | A stationary shaft is valid, not automatically stale. |
| `ERR_TEMPERATURE_LIMIT` | Test the conversion and threshold using synthetic `T_RAW` values. | Converts and trips the configured threshold. | Do not heat the live sensor without a controlled method. |
| `ERR_MAGNET_MARGIN` | Use recorded `D_MAG` values and calibrated limits. | Low/high margin cases flag correctly. | Determine thresholds from the actual magnet/air gap. |

### Phase D — fault recovery tests

For each fault code, define the policy separately:

- **Retry only:** CRC, RESP, transient no-response.
- **Reject the sample and log:** invalid angle, magnetic limit, implausible movement.
- **Reinitialize configuration:** reset detected or configuration mismatch, after preserving the fault record.
- **Escalate as persistent sensor fault:** repeated watchdog, ROM/fuse integrity, or self-test failures.

Do not blindly reset on every `ERR_SYSTEM`; read STAT and retain the underlying reason first. Use retry limits and back-off so a failed sensor does not trap the ESP32 in an infinite fast reset loop.

## 8. Minimum diagnostic log format

Use one line per attempt, for example:

```text
ms,cmd,data,safety,stat,crc_ok,resp_ok,error_mask,retry_count
123456,0x8021,0xABCD,0xFE42,0x0000,1,1,0x00000000,0
```

- `data`: received data word (hex)
- `safety`: full 16-bit Safety Word (hex)
- `stat`: raw STAT value when read; use `----` if not read for that attempt
- `crc_ok` / `resp_ok`: host-side validation outcomes
- `error_mask`: all currently detected errors, in hex

This log makes it possible to tell whether the first failure was a bad frame, a sensor-reported fault, or an application-level plausibility failure.

## 9. References

1. **Project reference supplied by the user:** `(Description) Hallever Error Codes.pdf`. Pages 1–4 describe sensor safety features, the diagnostic layers, STAT fault names, and the extra diagnostic registers.
2. **Infineon TLE5012B User's Manual Rev. 1.2**, SSC command/Safety Word tables and CRC example: <https://www.farnell.com/datasheets/2700287.pdf>
3. **Infineon TLE5012B datasheet**, SSC transfer details and Safety Word: <https://www.infineon.com/assets/row/public/documents/24/49/infineon-tle5012b-exxxx-datasheet-en.pdf>
4. **Infineon TLE5012B library/community reference**, useful for cross-checking implementation conventions: <https://community.infineon.com/t5/Angle-Magnetic-sensor/Solved-TLE5012B-SSC-in-the-TLE9879-evalboard/td-p/391338>

> This is an implementation and verification plan, not a claim that every listed fault can be physically induced in a safe or repeatable way on a production board. Unit-test the decoders first, then perform only controlled hardware tests.
