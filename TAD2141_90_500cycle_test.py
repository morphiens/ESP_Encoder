"""
Encoder calibration capture script.

Per position:
  1. Move the axis.
  2. Settle for SETTLE_S seconds with serial completely untouched
     (no reads, no writes) so no in-flight motion data can leak
     into the measurement.
  3. Flush any stale bytes, send one '?'.
  4. Block for exactly one response line. The firmware does the
     full 1000-sample fresh vector-average burst itself, so there
     is no polling/averaging loop on the host side.
"""

import csv
import time

import serial

SERIAL_PORT = "COM5"          # <-- set to your port
BAUD = 115200
SETTLE_S = 4.0                # was 3.5 -- full settle, zero serial traffic
CYCLES = 500
POSITIONS = [0, 90, 180, 270]
CSV_FILENAME = "TLE_test_burst.csv"
RESPONSE_TIMEOUT_S = 2.0


def move_to(position_deg: float) -> None:
    """
    TODO: replace with your actual motor-move call.
    Must be synchronous/blocking: don't return until the axis is
    physically at `position_deg` (or until your controller's
    "in position" signal fires).
    """
    raise NotImplementedError


def request_burst(ser: serial.Serial):
    """
    Sends '?' and blocks for exactly one response line.
    Returns (raw, filtered, status) or None if the firmware
    aborted the burst (mag loss / CRC fault mid-capture).
    """
    ser.reset_input_buffer()   # drop anything stale before asking
    ser.write(b"?")

    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        raise TimeoutError("No response from encoder within timeout")

    raw_s, filt_s, status_s = line.split(",")
    if raw_s == "NAN":
        return None

    return float(raw_s), float(filt_s), int(status_s)


def main() -> None:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=RESPONSE_TIMEOUT_S)
    time.sleep(2.0)  # let the board finish its reset after the port opens

    with open(CSV_FILENAME, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["Cycle", "Supposed Position", "Raw Angle", "Filtered Angle", "Field Status"]
        )

        for cycle in range(1, CYCLES + 1):
            for pos in POSITIONS:
                move_to(pos)

                # Settle window: serial is completely untouched here.
                # No '?' sent, no bytes drained -- the burst that
                # follows starts clean, with nothing carried over
                # from the move or the previous position.
                time.sleep(SETTLE_S)

                result = request_burst(ser)
                if result is None:
                    print(f"[WARN] cycle {cycle} pos {pos}: burst aborted, retrying once")
                    time.sleep(0.2)
                    result = request_burst(ser)
                    if result is None:
                        print(f"[ERROR] cycle {cycle} pos {pos}: burst failed twice, skipping")
                        continue

                raw, filt, status = result
                writer.writerow([cycle, pos, raw, filt, status])
                print(f"cycle={cycle} pos={pos}\u00b0 raw={raw:.4f} filt={filt:.4f} status={status}")

    ser.close()
    print(f"Done. Wrote {CSV_FILENAME}")


if __name__ == "__main__":
    main()