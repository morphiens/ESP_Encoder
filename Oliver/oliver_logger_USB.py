#!/usr/bin/env python3
"""
Oliver Encoder Interactive CSV Logger (USB Serial Version)
=========================================================
Connects to the Oliver Serial gateway and logs encoder data (M0, S0) to a CSV file.
"""

import asyncio
import csv
import time
import os
import sys
import select
import logging
from datetime import datetime
from oliver_serial import OliverSerialAPI

# ── Configuration ────────────────────────────────────────────────────────────
LOG_DIRECTORY = "logs"
LOG_FILENAME_PREFIX = "encoder_data_USB"
LOG_INTERVAL_S = 1.0  
SERIAL_PORT = "/dev/ttyACM0" # May need adjustment

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("oliver_logger_usb")

async def background_logger(probe: OliverSerialAPI, csv_filename: str):
    """Coro to handle background CSV logging."""
    try:
        with open(csv_filename, mode='a', newline='') as csvfile:
            fieldnames = ['Timestamp', 'System_Time', 'M0_Angle_Deg', 'S0_Angle_Deg', 'Status']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            if os.path.getsize(csv_filename) == 0:
                writer.writeheader()
            
            while True:
                if probe.is_connected:
                    m0, s0 = await probe.get_instantaneous_encoder_values()
                    
                    now = datetime.now()
                    timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    sys_time = time.time()
                    
                    m0_val = f"{m0:.5f}" if m0 is not None else "offline"
                    s0_val = f"{s0:.5f}" if s0 is not None else "offline"
                    status = "OK" if m0 is not None and s0 is not None else "PARTIAL/OFFLINE"
                    
                    row = {
                        'Timestamp': timestamp,
                        'System_Time': f"{sys_time:.3f}",
                        'M0_Angle_Deg': m0_val,
                        'S0_Angle_Deg': s0_val,
                        'Status': status
                    }
                    writer.writerow(row)
                    csvfile.flush()
                
                await asyncio.sleep(LOG_INTERVAL_S)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error("Background logger error: %s", e)

async def main():
    if not os.path.exists(LOG_DIRECTORY):
        os.makedirs(LOG_DIRECTORY)

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = os.path.join(LOG_DIRECTORY, f"{LOG_FILENAME_PREFIX}_{timestamp_str}.csv")

    probe = OliverSerialAPI(port=SERIAL_PORT)
    
    logger.info("Starting Oliver Interactive CSV Logger (USB Serial)...")
    logger.info("Serial Port: %s", SERIAL_PORT)
    logger.info("Background logging to: %s", csv_filename)
    
    if not await probe.connect():
        logger.error("Failed to connect to Serial port %s. Make sure ESP is connected and port is correct.", SERIAL_PORT)
        return

    bg_task = asyncio.create_task(background_logger(probe, csv_filename))

    print("\n" + "=" * 70)
    print("  Oliver Interactive Logger (USB Serial)")
    print("  Commands:  Enter = measure  |  z = set zero  |  v = instantaneous")
    print("             r = rediscover slaves  |  q = quit")
    print("=" * 70 + "\n")

    try:
        if sys.platform != "win32":
            import tty, termios
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            old_settings = None

        while True:
            if sys.platform != "win32" and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
            else:
                key = None

            if key:
                if key.lower() == "q":
                    break
                elif key.lower() == "z":
                    try:
                        offsets = probe.set_zero()
                        print(f"✓ Zero set: {offsets}")
                    except RuntimeError as e:
                        print(f"✗ {e}")
                elif key.lower() == "r":
                    await probe.send_command("REDISCOVER")
                    print("✓ Rediscover command sent.")
                elif key == "\n":
                    m0_med, s0_med = await probe.get_encoder_values()
                    print(f"\n{'=' * 80}")
                    print("  Measurement (M0_median, S0_median)")
                    print(f"{'=' * 80}")
                    print(f"  M0   {m0_med:12.5f}°" if m0_med is not None else "  M0   no data")
                    print(f"  S0   {s0_med:12.5f}°" if s0_med is not None else "  S0   no data")
                    print(f"{'=' * 80}\n")
                elif key == "v":
                    m0, s0 = await probe.get_instantaneous_encoder_values()
                    print(f"  M0: {m0:.5f}°" if m0 is not None else "  M0: no data")
                    print(f"  S0: {s0:.5f}°" if s0 is not None else "  S0: no data")

            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        bg_task.cancel()
        await bg_task
        if sys.platform != "win32" and old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        await probe.disconnect()
        logger.info("Oliver Interactive CSV Logger shut down.")

if __name__ == "__main__":
    asyncio.run(main())
