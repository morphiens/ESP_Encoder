#!/usr/bin/env python3
"""
TLE5012B power-cycle monitor — 5s off, raw + 1000-sample SMA filtered.

Flash: TLE5012B_PowerCycle_5s_SMA/TLE5012B_PowerCycle_5s_SMA.ino
Run:   python3 monitor_tle5012_powercycles_5s_sma.py [/dev/ttyACM0]
CSV:   tle5012_power_cycles_5s_sma.csv
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
LABEL = "5s + 1000-SMA"
TIMING = "2s boot + 1000-sample burst + 5s off"
CSV_FILE = os.path.join(SCRIPT_DIR, "tle5012_power_cycles_5s_sma.csv")
CSV_HEADER = [
    "Cycle",
    "Timestamp",
    "Angle_deg",
    "Filtered_deg",
    "Raw",
    "Delta_deg",
    "Delta_filtered_deg",
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
            if f.readline().strip() == ",".join(CSV_HEADER):
                return
    with open(CSV_FILE, "w", newline="") as f:
        csv.DictWriter(f, fieldnames=CSV_HEADER).writeheader()


def load_session_from_csv() -> tuple[
    float | None, float | None, int | None, list[float], list[float]
]:
    if not os.path.exists(CSV_FILE):
        return None, None, None, [], []

    with open(CSV_FILE, "r", newline="") as f:
        rows = list(csv.DictReader(f))

    ref_angle = None
    ref_filtered = None
    ref_raw = None
    ok_deltas: list[float] = []
    ok_filtered_deltas: list[float] = []

    for row in rows:
        if row.get("Status") != "OK":
            continue
        try:
            delta = float(row["Delta_deg"]) if row.get("Delta_deg") else None
            delta_f = float(row["Delta_filtered_deg"]) if row.get("Delta_filtered_deg") else None
        except ValueError:
            delta = None
            delta_f = None

        if ref_angle is None:
            try:
                ref_angle = float(row["Angle_deg"])
                ref_filtered = float(row["Filtered_deg"])
                ref_raw = int(row["Raw"])
            except (KeyError, TypeError, ValueError):
                pass

        if delta is not None:
            ok_deltas.append(delta)
        if delta_f is not None:
            ok_filtered_deltas.append(delta_f)

    return ref_angle, ref_filtered, ref_raw, ok_deltas, ok_filtered_deltas


def write_csv_row(row: dict[str, str | int | float]) -> None:
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
        writer.writerow({k: row.get(k, "") for k in CSV_HEADER})
        f.flush()
        os.fsync(f.fileno())

    log_status(
        "CSV >> "
        f"Cycle={row.get('Cycle')} "
        f"Raw={row.get('Angle_deg')} "
        f"Filtered={row.get('Filtered_deg')} "
        f"Delta={row.get('Delta_deg')} "
        f"DeltaF={row.get('Delta_filtered_deg')} "
        f"Status={row.get('Status')}"
    )


def print_data_block(
    cycle: int,
    timestamp: str,
    angle: float | None,
    filtered: float | None,
    raw: int | None,
    delta_deg: float | None,
    delta_filtered: float | None,
    delta_raw: int | None,
    status: str,
    deltas: list[float],
    filtered_deltas: list[float],
) -> None:
    mn, mx, sigma = drift_stats(deltas)
    fmn, fmx, fsigma = drift_stats(filtered_deltas)
    sep = "─" * 40

    log(sep)
    log(f" Cycle   {cycle:>5}  |  {timestamp}")

    if status == "OK" and angle is not None and filtered is not None and raw is not None:
        log(f" Raw     {angle:>10.6f} deg  (raw {raw})")
        log(f" Filter  {filtered:>10.6f} deg  (1000-SMA)")
        if delta_deg is not None and delta_filtered is not None and delta_raw is not None:
            log(
                f" Delta   raw {fmt_signed(delta_deg)} deg  "
                f"filtered {fmt_signed(delta_filtered)} deg  "
                f"({delta_raw:+d} raw counts) vs cycle 1"
            )
    else:
        log(f" Status  {status}  (no valid angle)")

    if deltas:
        log(
            f" Stats   raw min {fmt_signed(mn, 8, 3)} max {fmt_signed(mx, 8, 3)} σ {sigma:.6f} | "
            f"filtered min {fmt_signed(fmn, 8, 3)} max {fmt_signed(fmx, 8, 3)} σ {fsigma:.6f} "
            f"(n={len(deltas)})"
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


def print_banner(
    port: str,
    ref_angle: float | None,
    ref_filtered: float | None,
    ref_raw: int | None,
) -> None:
    log("")
    log("=" * 50)
    log(f"  TLE5012B POWER-CYCLE MONITOR ({LABEL}) — RUNNING")
    log("=" * 50)
    log(f"  Port:     {port}")
    log(f"  Baud:     {BAUD}")
    log(f"  CSV:      {CSV_FILE}")
    log(f"  Timing:   {TIMING}")
    if ref_angle is not None and ref_filtered is not None:
        log(f"  Resumed:  raw {ref_angle:.6f} deg  filtered {ref_filtered:.6f} deg  (raw {ref_raw})")
    log("  Press Ctrl+C to stop.")
    log("=" * 50)
    log("")


def parse_port_arg() -> str:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    return args[0] if args else "/dev/ttyACM0"


def count_err_rows() -> int:
    if not os.path.exists(CSV_FILE):
        return 0
    with open(CSV_FILE, "r", newline="") as f:
        return sum(1 for row in csv.DictReader(f) if row.get("Status") == "ERR")


def main() -> None:
    port = parse_port_arg()
    ensure_csv()

    ref_angle, ref_filtered, ref_raw, ok_deltas, ok_filtered_deltas = load_session_from_csv()
    total_logged = len(ok_deltas)
    err_count = count_err_rows()

    ser = connect_serial(port)
    print_banner(port, ref_angle, ref_filtered, ref_raw)

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
            filtered = None
            raw = None

            if status == "OK" and "ANGLE" in fields and "FILTERED" in fields and "RAW" in fields:
                try:
                    angle = float(fields["ANGLE"])
                    filtered = float(fields["FILTERED"])
                    raw = int(fields["RAW"])
                except ValueError:
                    status = "ERR"

            if status == "OK" and angle is not None and filtered is not None and raw is not None:
                if ref_angle is None:
                    ref_angle = angle
                    ref_filtered = filtered
                    ref_raw = raw

                delta_f = angle_delta(angle, ref_angle)
                delta_ff = angle_delta(filtered, ref_filtered)
                delta_i = raw - ref_raw
                ok_deltas.append(delta_f)
                ok_filtered_deltas.append(delta_ff)
                total_logged += 1

                write_csv_row({
                    "Cycle": cycle,
                    "Timestamp": timestamp,
                    "Angle_deg": f"{angle:.6f}",
                    "Filtered_deg": f"{filtered:.6f}",
                    "Raw": raw,
                    "Delta_deg": f"{delta_f:.6f}",
                    "Delta_filtered_deg": f"{delta_ff:.6f}",
                    "Delta_raw": delta_i,
                    "Status": status,
                })

                print_data_block(
                    cycle, timestamp, angle, filtered, raw,
                    delta_f, delta_ff, delta_i, status,
                    ok_deltas, ok_filtered_deltas,
                )
            else:
                err_count += 1
                write_csv_row({
                    "Cycle": cycle,
                    "Timestamp": timestamp,
                    "Angle_deg": "",
                    "Filtered_deg": "",
                    "Raw": "",
                    "Delta_deg": "",
                    "Delta_filtered_deg": "",
                    "Delta_raw": "",
                    "Status": "ERR",
                })
                print_data_block(
                    cycle, timestamp, None, None, None,
                    None, None, None, "ERR",
                    ok_deltas, ok_filtered_deltas,
                )

    except KeyboardInterrupt:
        log("")
        if ok_deltas:
            mn, mx, sigma = drift_stats(ok_deltas)
            fmn, fmx, fsigma = drift_stats(ok_filtered_deltas)
            log_status(
                f"Stopped | {total_logged} OK, {err_count} ERR | "
                f"raw drift {fmt_signed(mn, 8, 3)}..{fmt_signed(mx, 8, 3)} | "
                f"filtered drift {fmt_signed(fmn, 8, 3)}..{fmt_signed(fmx, 8, 3)} | "
                f"{CSV_FILE}"
            )
        else:
            log_status(f"Stopped | {total_logged} OK, {err_count} ERR | {CSV_FILE}")


if __name__ == "__main__":
    main()
