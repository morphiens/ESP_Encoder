#!/usr/bin/env python3
"""
Oliver Encoder Interactive CSV Logger
=====================================
Connects to the Oliver BLE gateway and logs encoder data (M0, S0..SN) to a CSV file.
Combines background logging with interactive commands from oliver.py.

Interactive Commands:
    Enter - Take a precision measurement (median of samples)
    z     - Set current position as zero reference
    v     - Get instantaneous values display
    c     - Connect
    d     - Disconnect
    r     - Reconnect
    q     - Quit

Usage:
    python oliver_logger.py
"""

import asyncio
import csv
import time
import os
import sys
import select
import logging
from datetime import datetime
from oliver import OliverAPI, ENCODERS

# ── Configuration ────────────────────────────────────────────────────────────
LOG_DIRECTORY        = "logs"
LOG_FILENAME_PREFIX  = "encoder_data"
LOG_INTERVAL_S       = 1.0   # Time between background readings

# ── Slave count — change this value only; everything else auto-adjusts ───────
NUM_SLAVES = 5               # 1 master (M0) + NUM_SLAVES slaves (S0, S1, ...)

# Derived encoder list: M0 first, then S0..S(N-1)
ALL_ENCODERS = ["M0"] + [f"S{i}" for i in range(NUM_SLAVES)]

# Set up basic logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("oliver_logger")


async def get_all_encoder_values(probe: OliverAPI):
    """
    Return a dict  {encoder_name: float_or_None}  for every encoder in
    ALL_ENCODERS.  Pulls from probe._latest_raw which is populated by the
    OliverAPI after each BLE notification.
    """
    values = {}
    for enc in ALL_ENCODERS:
        raw = probe._latest_raw.get(enc)
        values[enc] = raw  # None if not received yet
    return values


async def background_logger(probe: OliverAPI, csv_filename: str):
    """Coro to handle background CSV logging for all encoders."""
    try:
        # Build CSV fieldnames dynamically from ALL_ENCODERS
        angle_fields = [f"{enc}_Angle_Deg" for enc in ALL_ENCODERS]
        fieldnames   = ["Timestamp", "System_Time"] + angle_fields + ["Status"]

        with open(csv_filename, mode="a", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # Write header only if file is empty
            if os.path.getsize(csv_filename) == 0:
                writer.writeheader()

            while True:
                if probe.is_connected:
                    # Trigger a fresh BLE read and wait for response
                    try:
                        await probe.get_instantaneous_encoder_values()
                    except Exception:
                        pass  # best-effort; use whatever is in _latest_raw

                    now        = datetime.now()
                    timestamp  = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    sys_time   = time.time()

                    values     = await get_all_encoder_values(probe)
                    any_online = any(v is not None for v in values.values())
                    all_online = all(v is not None for v in values.values())

                    if all_online:
                        status = "OK"
                    elif any_online:
                        status = "PARTIAL"
                    else:
                        status = "OFFLINE"

                    row = {
                        "Timestamp":   timestamp,
                        "System_Time": f"{sys_time:.3f}",
                        "Status":      status,
                    }
                    for enc in ALL_ENCODERS:
                        v = values[enc]
                        row[f"{enc}_Angle_Deg"] = f"{v:.5f}" if v is not None else "offline"

                    writer.writerow(row)
                    csvfile.flush()

                await asyncio.sleep(LOG_INTERVAL_S)

    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error("Background logger error: %s", e)


def _print_encoder_table(values: dict, title: str = "Measurement"):
    """Pretty-print a {enc: float|None} dict."""
    width = 80
    print(f"\n{'=' * width}")
    print(f"  {title}")
    print(f"{'=' * width}")
    for enc, val in values.items():
        if val is not None:
            print(f"  {enc:<4}  {val:12.5f}°")
        else:
            print(f"  {enc:<4}  offline")
    print(f"{'=' * width}\n")


async def main():
    # Ensure log directory exists
    os.makedirs(LOG_DIRECTORY, exist_ok=True)

    # Generate filename with timestamp
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename  = os.path.join(
        LOG_DIRECTORY, f"{LOG_FILENAME_PREFIX}_{timestamp_str}.csv"
    )

    probe = OliverAPI()

    logger.info("Starting Oliver Interactive CSV Logger...")
    logger.info("Encoders tracked: %s", ", ".join(ALL_ENCODERS))
    logger.info("Background logging to: %s", csv_filename)

    # Start background logging task
    bg_task = asyncio.create_task(background_logger(probe, csv_filename))

    # Initial connect
    await probe.connect()

    print("\n" + "=" * 70)
    print("  Oliver Interactive Logger")
    print(f"  Encoders: {', '.join(ALL_ENCODERS)}")
    print("  Commands:  Enter = measure  |  z = set zero  |  v = instantaneous")
    print("             c = connect  |  d = disconnect  |  r = reconnect  |  q = quit")
    print("=" * 70 + "\n")

    # ── Interactive loop ─────────────────────────────────────────────────────
    try:
        if sys.platform != "win32":
            import tty, termios
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            old_settings = None

        while True:
            # Non-blocking key check
            if sys.platform != "win32" and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
            else:
                key = None

            if key:
                # ── Quit ──────────────────────────────────────────────────
                if key.lower() == "q":
                    break

                # ── Set zero ──────────────────────────────────────────────
                elif key.lower() == "z":
                    try:
                        offsets = probe.set_zero()
                        print(f"✓ Zero set: {offsets}")
                    except RuntimeError as e:
                        print(f"✗ {e}")

                # ── Connect ───────────────────────────────────────────────
                elif key.lower() == "c":
                    ok = await probe.connect()
                    print("✓ Connected." if ok else "✗ Connect failed.")

                # ── Disconnect ────────────────────────────────────────────
                elif key.lower() == "d":
                    await probe.disconnect()
                    print("✓ Disconnected.")

                # ── Reconnect ─────────────────────────────────────────────
                elif key.lower() == "r":
                    ok = await probe.reconnect()
                    print("✓ Reconnected." if ok else "✗ Reconnect failed.")

                # ── Precision measurement (Enter) ─────────────────────────
                elif key == "\n":
                    try:
                        await probe.get_encoder_values()
                        values = {enc: probe._latest_raw.get(enc) for enc in ALL_ENCODERS}
                        _print_encoder_table(values, title="Precision Measurement")
                    except asyncio.TimeoutError as e:
                        print(f"✗ Timeout: {e}")
                    except Exception as e:
                        print(f"✗ Error: {e}")

                # ── Instantaneous display ─────────────────────────────────
                elif key.lower() == "v":
                    try:
                        await probe.get_instantaneous_encoder_values()
                        values = {enc: probe._latest_raw.get(enc) for enc in ALL_ENCODERS}
                        _print_encoder_table(values, title="Instantaneous Values")
                        elapsed = getattr(probe, "last_get_instantaneous_encoder_values_time_s", None)
                        if elapsed is not None:
                            print(f"  (read took {elapsed:.3f} s)\n")
                    except Exception as e:
                        print(f"✗ Error: {e}")

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        bg_task.cancel()
        await bg_task
        if sys.platform != "win32" and old_settings:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        if probe.is_connected:
            await probe.disconnect()
        logger.info("Oliver Interactive CSV Logger shut down.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass