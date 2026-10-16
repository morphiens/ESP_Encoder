#!/usr/bin/env python3
"""
TLE5012B power-cycle drift monitor.

Logs serial output from ESP32-S3 Plus firmware to CSV and terminal.

Usage:
  Flash: TLE5012B_PowerCycle/TLE5012B_PowerCycle.ino  (XIAO ESP32S3 Plus)
  python3 monitor_tle5012_powercycles.py [/dev/ttyACM0]
  python3 monitor_tle5012_powercycles.py --show-sketch   # same firmware as .ino
"""

import csv
import math
import os
import sys
import time
from datetime import datetime

import serial

# ============================================================
# CONFIG
# ============================================================

BAUD = 115200
HEARTBEAT_S = 15  # print "still listening" if no serial for this long
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, "tle5012_power_cycles.csv")
CSV_HEADER = [
    "Cycle",
    "Timestamp",
    "Angle_deg",
    "Raw",
    "Delta_deg",
    "Delta_raw",
    "Status",
]

ARDUINO_SKETCH = r"""
/*
 * TLE5012B Power-Cycle Drift Test — XIAO ESP32-S3 Plus
 * Paste into Arduino IDE, board: XIAO ESP32S3 Plus
 *
 * D8=SCK  D9=MISO  D10=MOSI  D0=CS  D2=power (HIGH=ON)
 */

#include <Arduino.h>
#include <SPI.h>

static const int PIN_SCK  = D8;
static const int PIN_MISO = D9;
static const int PIN_MOSI = D10;
static const int PIN_CS   = D0;
static const int PIN_PWR  = D2;

static const uint32_t BOOT_WAIT_MS  = 2000;
static const uint32_t POWER_OFF_MS  = 180000;
static const uint32_t COUNTDOWN_S   = 30;

static const uint16_t CMD_READ_ANGLE = 0x8020;

SPIClass spi(FSPI);
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

enum State { ST_POWER_ON, ST_WAIT_BOOT, ST_READ, ST_POWER_OFF, ST_WAIT_OFF };

static State state = ST_POWER_ON;
static uint32_t cycleNum = 0;
static uint32_t stateStartMs = 0;
static uint32_t lastCountdownMs = 0;

static void sensorPower(bool on) {
  digitalWrite(PIN_PWR, on ? HIGH : LOW);
}

static bool readEncoder(uint16_t &rawOut) {
  spi.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);

  spi.transfer16(CMD_READ_ANGLE);
  uint16_t data = spi.transfer16(0x0000);
  spi.transfer16(0x0000);

  digitalWrite(PIN_CS, HIGH);
  spi.endTransaction();
  delayMicroseconds(5);

  if (data == 0x0000 || data == 0xFFFF) {
    return false;
  }

  rawOut = data & 0x7FFF;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_PWR, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  sensorPower(false);

  Serial.println("--- TLE5012B Power-Cycle Test ---");
  Serial.println("PINS SCK=D8 MISO=D9 MOSI=D10 CS=D0 PWR=D2");

  state = ST_POWER_ON;
  stateStartMs = millis();
}

void loop() {
  uint32_t now = millis();

  switch (state) {
    case ST_POWER_ON:
      cycleNum++;
      sensorPower(true);
      spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
      digitalWrite(PIN_CS, HIGH);
      Serial.printf("POWER_ON cycle=%lu\n", cycleNum);
      stateStartMs = now;
      state = ST_WAIT_BOOT;
      break;

    case ST_WAIT_BOOT:
      if (now - stateStartMs >= BOOT_WAIT_MS) {
        state = ST_READ;
      }
      break;

    case ST_READ: {
      uint16_t raw = 0;
      bool ok = readEncoder(raw);
      if (ok) {
        float angle = (float)raw * 360.0f / 32768.0f;
        Serial.printf("CYCLE=%lu ANGLE=%.6f RAW=%u STATUS=OK\n",
                      cycleNum, angle, raw);
      } else {
        Serial.printf("CYCLE=%lu ANGLE=nan RAW=0 STATUS=ERR\n", cycleNum);
      }
      state = ST_POWER_OFF;
      break;
    }

    case ST_POWER_OFF:
      spi.end();
      sensorPower(false);
      Serial.printf("POWER_OFF cycle=%lu off_for=%lus\n",
                    cycleNum, POWER_OFF_MS / 1000UL);
      stateStartMs = now;
      lastCountdownMs = now;
      state = ST_WAIT_OFF;
      break;

    case ST_WAIT_OFF:
      if (now - stateStartMs >= POWER_OFF_MS) {
        state = ST_POWER_ON;
      } else if (COUNTDOWN_S > 0 &&
                 (now - lastCountdownMs) >= COUNTDOWN_S * 1000UL) {
        lastCountdownMs = now;
        uint32_t remaining = (POWER_OFF_MS - (now - stateStartMs)) / 1000UL;
        Serial.printf("WAIT=%lus\n", remaining);
      }
      break;
  }
}
"""


# ============================================================
# HELPERS
# ============================================================

def angle_delta(deg: float, ref: float) -> float:
    d = deg - ref
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def parse_fields(line: str) -> dict[str, str]:
    return dict(part.split("=", 1) for part in line.split() if "=" in part)


def is_data_line(fields: dict[str, str]) -> bool:
    return "CYCLE" in fields and "STATUS" in fields


def ensure_csv() -> None:
    if os.path.exists(CSV_FILE):
        with open(CSV_FILE, "r", newline="") as f:
            first = f.readline().strip()
        if first == ",".join(CSV_HEADER):
            return
    with open(CSV_FILE, "w", newline="") as f:
        csv.DictWriter(f, fieldnames=CSV_HEADER).writeheader()


def print_data_block(
    cycle: int,
    timestamp: str,
    angle: float | None,
    raw: int | None,
    delta_deg: float | None,
    delta_raw: int | None,
    status: str,
    deltas: list[float],
) -> None:
    mn, mx, sigma = drift_stats(deltas)
    sep = "─" * 40

    log(sep)
    log(f" Cycle   {cycle:>5}  |  {timestamp}")

    if status == "OK" and angle is not None and raw is not None:
        log(f" Angle   {angle:>10.6f} deg  (raw {raw})")
        if delta_deg is not None and delta_raw is not None:
            log(
                f" Delta   {fmt_signed(delta_deg)} deg  "
                f"({delta_raw:+d} raw)   vs cycle 1"
            )
    else:
        log(f" Status  {status}  (no valid angle)")

    if deltas:
        log(
            f" Stats   min {fmt_signed(mn, 8, 3)}  "
            f"max {fmt_signed(mx, 8, 3)}  "
            f"σ {sigma:.6f}  (n={len(deltas)})"
        )

    log(f" Saved → {CSV_FILE}")
    log(sep)
    log("")


def load_session_from_csv() -> tuple[int | None, float | None, int | None, list[float]]:
    """Return (last_cycle, ref_angle, ref_raw, ok_deltas) from existing CSV."""
    if not os.path.exists(CSV_FILE):
        return None, None, None, []

    with open(CSV_FILE, "r", newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        return None, None, None, []

    last_cycle = None
    ref_angle = None
    ref_raw = None
    ok_deltas: list[float] = []

    for row in rows:
        try:
            last_cycle = int(row["Cycle"])
        except (KeyError, TypeError, ValueError):
            continue

        if row.get("Status") != "OK":
            continue

        try:
            delta = float(row["Delta_deg"]) if row.get("Delta_deg") else None
        except ValueError:
            delta = None

        if ref_angle is None:
            try:
                ref_angle = float(row["Angle_deg"])
                ref_raw = int(row["Raw"])
            except (KeyError, TypeError, ValueError):
                pass

        if delta is not None:
            ok_deltas.append(delta)

    return last_cycle, ref_angle, ref_raw, ok_deltas


def drift_stats(deltas: list[float]) -> tuple[float, float, float]:
    if not deltas:
        return 0.0, 0.0, 0.0
    mn = min(deltas)
    mx = max(deltas)
    if len(deltas) == 1:
        return mn, mx, 0.0
    mean = sum(deltas) / len(deltas)
    var = sum((d - mean) ** 2 for d in deltas) / len(deltas)
    return mn, mx, math.sqrt(var)


def fmt_signed(value: float, width: int = 10, decimals: int = 6) -> str:
    return f"{value:+{width}.{decimals}f}"


def write_csv_row(row: dict[str, str | int | float]) -> None:
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
        writer.writerow({k: row.get(k, "") for k in CSV_HEADER})
        f.flush()
        os.fsync(f.fileno())

    log_status(
        "CSV >> "
        f"Cycle={row.get('Cycle')} "
        f"Angle={row.get('Angle_deg')} "
        f"Raw={row.get('Raw')} "
        f"Delta={row.get('Delta_deg')} "
        f"Status={row.get('Status')}"
    )


def connect_serial(port: str) -> serial.Serial:
    while True:
        try:
            log_status(f"Connecting to {port}...")
            ser = serial.Serial(port, BAUD, timeout=1)
            log_status(f"Connected to {port}")
            time.sleep(2)
            pending = ser.in_waiting
            if pending:
                log_status(f"Flushing {pending} stale byte(s) from buffer")
                ser.reset_input_buffer()
            return ser
        except Exception as exc:
            log_status(f"Waiting for ESP32: {exc}")
            time.sleep(1)


def ts_short() -> str:
    return datetime.now().strftime("%H:%M:%S")


def log(msg: str) -> None:
    print(msg, flush=True)


def log_rx(line: str) -> None:
    log(f"[{ts_short()}] RX | {line}")


def log_status(msg: str) -> None:
    log(f"[{ts_short()}] ** {msg}")


def print_banner(port: str, ref_angle: float | None, ref_raw: int | None) -> None:
    log("")
    log("=" * 50)
    log("  TLE5012B POWER-CYCLE MONITOR — RUNNING")
    log("=" * 50)
    log(f"  Port:     {port}")
    log(f"  Baud:     {BAUD}")
    log(f"  CSV:      {CSV_FILE}")
    if ref_angle is not None:
        log(f"  Resumed:  ref {ref_angle:.6f} deg (raw {ref_raw})")
    log("  Every serial line is printed below as  [HH:MM:SS] RX | ...")
    log(f"  Heartbeat every {HEARTBEAT_S}s if ESP is quiet (3 min off between cycles)")
    log("  Press Ctrl+C to stop.")
    log("=" * 50)
    log("")


def parse_port_arg() -> str:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    return args[0] if args else "/dev/ttyACM0"


# ============================================================
# MAIN
# ============================================================

def main() -> None:
    if "--show-sketch" in sys.argv:
        print(ARDUINO_SKETCH.strip())
        return

    port = parse_port_arg()
    ensure_csv()

    _, ref_angle, ref_raw, ok_deltas = load_session_from_csv()
    total_logged = len(ok_deltas)
    err_count = 0
    if os.path.exists(CSV_FILE):
        with open(CSV_FILE, "r", newline="") as f:
            for row in csv.DictReader(f):
                if row.get("Status") == "ERR":
                    err_count += 1

    ser = connect_serial(port)
    print_banner(port, ref_angle, ref_raw)

    lines_received = 0
    last_rx_time = time.time()
    last_heartbeat = time.time()

    try:
        while True:
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                log_status("Serial disconnected — reconnecting...")
                time.sleep(1)
                ser = connect_serial(port)
                last_heartbeat = time.time()
                continue

            now = time.time()

            if not line:
                if now - last_heartbeat >= HEARTBEAT_S:
                    ago = int(now - last_rx_time)
                    log_status(
                        f"Listening on {port} | "
                        f"lines received: {lines_received} | "
                        f"logged OK: {total_logged} ERR: {err_count} | "
                        f"last RX: {ago}s ago"
                    )
                    last_heartbeat = now
                continue

            lines_received += 1
            last_rx_time = now
            last_heartbeat = now

            # Always print raw serial line
            log_rx(line)

            fields = parse_fields(line)

            if is_data_line(fields):
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                status = fields.get("STATUS", "ERR")

                try:
                    cycle = int(fields["CYCLE"])
                except (KeyError, ValueError):
                    cycle = total_logged + err_count + 1

                angle = None
                raw = None
                delta_deg = ""
                delta_raw = ""

                if status == "OK" and "ANGLE" in fields and "RAW" in fields:
                    try:
                        angle = float(fields["ANGLE"])
                        raw = int(fields["RAW"])
                    except ValueError:
                        status = "ERR"

                if status == "OK" and angle is not None and raw is not None:
                    if ref_angle is None:
                        ref_angle = angle
                        ref_raw = raw

                    delta_f = angle_delta(angle, ref_angle)
                    delta_i = raw - ref_raw
                    delta_deg = f"{delta_f:.6f}"
                    delta_raw = delta_i
                    ok_deltas.append(delta_f)
                    total_logged += 1

                    write_csv_row({
                        "Cycle": cycle,
                        "Timestamp": timestamp,
                        "Angle_deg": f"{angle:.6f}",
                        "Raw": raw,
                        "Delta_deg": delta_deg,
                        "Delta_raw": delta_raw,
                        "Status": status,
                    })
                else:
                    err_count += 1
                    write_csv_row({
                        "Cycle": cycle,
                        "Timestamp": timestamp,
                        "Angle_deg": "",
                        "Raw": "",
                        "Delta_deg": "",
                        "Delta_raw": "",
                        "Status": "ERR",
                    })

                print_data_block(
                    cycle, timestamp, angle, raw,
                    float(delta_deg) if delta_deg != "" else None,
                    int(delta_raw) if delta_raw != "" else None,
                    status, ok_deltas,
                )

    except KeyboardInterrupt:
        log("")
        if ok_deltas:
            mn, mx, sigma = drift_stats(ok_deltas)
            log_status(
                f"Stopped | {total_logged} OK, {err_count} ERR | "
                f"drift {fmt_signed(mn, 8, 3)} .. {fmt_signed(mx, 8, 3)} deg "
                f"(σ {sigma:.6f}) | {lines_received} serial lines | {CSV_FILE}"
            )
        else:
            log_status(
                f"Stopped | {total_logged} OK, {err_count} ERR | "
                f"{lines_received} serial lines | {CSV_FILE}"
            )


if __name__ == "__main__":
    main()
