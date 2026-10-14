#!/usr/bin/env python3
"""
TLE5012B power-cycle drift monitor — 5 second off interval.

Flash: TLE5012B_PowerCycle_5s/TLE5012B_PowerCycle_5s.ino  (XIAO ESP32S3 Plus)
Run:   python3 monitor_tle5012_powercycles_5s.py [/dev/ttyACM0]

Timing: 2s boot + read, then 5s powered off (~7s per cycle).
CSV:    tle5012_power_cycles_5s.csv  (next to this script)
"""

import csv
import math
import os
import sys
import time
from datetime import datetime

import serial

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BAUD = 115200
HEARTBEAT_S = 10
CSV_FILE = os.path.join(SCRIPT_DIR, "tle5012_power_cycles_5s.csv")
CSV_HEADER = [
    "Cycle",
    "Timestamp",
    "Angle_deg",
    "Raw",
    "Delta_deg",
    "Delta_raw",
    "Status",
]


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


def ts_short() -> str:
    return datetime.now().strftime("%H:%M:%S")


def log(msg: str) -> None:
    print(msg, flush=True)


def log_rx(line: str) -> None:
    log(f"[{ts_short()}] RX | {line}")


def log_status(msg: str) -> None:
    log(f"[{ts_short()}] ** {msg}")


def fmt_signed(value: float, width: int = 10, decimals: int = 6) -> str:
    return f"{value:+{width}.{decimals}f}"


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


def ensure_csv() -> None:
    if os.path.exists(CSV_FILE):
        with open(CSV_FILE, "r", newline="") as f:
            first = f.readline().strip()
        if first == ",".join(CSV_HEADER):
            return
    with open(CSV_FILE, "w", newline="") as f:
        csv.DictWriter(f, fieldnames=CSV_HEADER).writeheader()


def load_session_from_csv() -> tuple[float | None, int | None, list[float]]:
    if not os.path.exists(CSV_FILE):
        return None, None, []

    with open(CSV_FILE, "r", newline="") as f:
        rows = list(csv.DictReader(f))

    ref_angle = None
    ref_raw = None
    ok_deltas: list[float] = []

    for row in rows:
        if row.get("Status") != "OK":
            continue
        try:
            angle = float(row["Angle_deg"])
            raw = int(row["Raw"])
            delta = float(row["Delta_deg"]) if row.get("Delta_deg") else None
        except (KeyError, TypeError, ValueError):
            continue

        if ref_angle is None:
            ref_angle = angle
            ref_raw = raw
        if delta is not None:
            ok_deltas.append(delta)

    return ref_angle, ref_raw, ok_deltas


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


def connect_serial(port: str) -> serial.Serial:
    while True:
        try:
            log_status(f"Connecting to {port}...")
            ser = serial.Serial(port, BAUD, timeout=1)
            log_status(f"Connected to {port}")
            time.sleep(2)
            if ser.in_waiting:
                log_status(f"Flushing {ser.in_waiting} stale byte(s)")
                ser.reset_input_buffer()
            return ser
        except Exception as exc:
            log_status(f"Waiting for ESP32: {exc}")
            time.sleep(1)


def print_banner(port: str, ref_angle: float | None, ref_raw: int | None) -> None:
    log("")
    log("=" * 50)
    log("  TLE5012B POWER-CYCLE MONITOR (5s) — RUNNING")
    log("=" * 50)
    log(f"  Port:     {port}")
    log(f"  Baud:     {BAUD}")
    log(f"  CSV:      {CSV_FILE}")
    log(f"  Timing:   2s boot + 5s off (~7s per cycle)")
    if ref_angle is not None:
        log(f"  Resumed:  ref {ref_angle:.6f} deg (raw {ref_raw})")
    log("  Press Ctrl+C to stop.")
    log("=" * 50)
    log("")


def parse_port_arg() -> str:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    return args[0] if args else "/dev/ttyACM0"


def main() -> None:
    port = parse_port_arg()
    ensure_csv()

    ref_angle, ref_raw, ok_deltas = load_session_from_csv()
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
                        f"Listening | lines={lines_received} "
                        f"OK={total_logged} ERR={err_count} | last RX {ago}s ago"
                    )
                    last_heartbeat = now
                continue

            lines_received += 1
            last_rx_time = now
            last_heartbeat = now
            log_rx(line)

            fields = parse_fields(line)
            if not is_data_line(fields):
                continue

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
                cycle,
                timestamp,
                angle,
                raw if isinstance(raw, int) else None,
                float(delta_deg) if delta_deg != "" else None,
                int(delta_raw) if delta_raw != "" else None,
                status,
                ok_deltas,
            )

    except KeyboardInterrupt:
        log("")
        if ok_deltas:
            mn, mx, sigma = drift_stats(ok_deltas)
            log_status(
                f"Stopped | {total_logged} OK, {err_count} ERR | "
                f"drift {fmt_signed(mn, 8, 3)} .. {fmt_signed(mx, 8, 3)} deg "
                f"(σ {sigma:.6f}) | {CSV_FILE}"
            )
        else:
            log_status(f"Stopped | {total_logged} OK, {err_count} ERR | {CSV_FILE}")


if __name__ == "__main__":
    main()
