"""
Author: Swaraj Dangare
"""
import asyncio
import time
import sys
import select
import numpy as np
from collections import deque
from bleak import BleakClient, BleakScanner
import os
import csv
from datetime import datetime

# BLE configuration
DEVICE_NAME = "AIRPROBE_MASTER_GATEWAY"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
COMMAND_UUID = "8d53dc1d-1db7-4cd3-868b-8a527460aa84"


class GatewayInterface:
    def __init__(self):
        self.start_time = time.time()
        self.last_g_idx = -1
        self.missed_packets = 0
        self.client = None

        # Manual sampling mode
        self.manual_mode = True
        self.samples_per_measurement = 5
        self.current_samples = {
            'M0': [], 'S0': []
        }
        self.sample_metadata = {
            'M0': [], 'S0': []
        }
        self.collecting = False
        self.measurement_id = 0

        # Legacy variance tracking (kept for reference)
        self.sample_size = 100
        self.encoder_samples = {
            'M0': deque(maxlen=self.sample_size),
            'S0': deque(maxlen=self.sample_size),
        }

        # ---------- CSV LOGGING ----------
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_path = f"logs/airprobe_log_{ts}.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "timestamp",
            "uptime_s",
            "measurement_id",
            "sample_num",
            "gateway_idx",
            "encoder",
            "angle_deg",
            "median",
            "jitter",
            "precision_um",
            "pkt",
            "agc",
            "mag",
            "magl",
            "magh",
            "cof",
            "status"
        ])

        print(f"[LOG] CSV logging to {self.csv_path}")

    def variance(self, key):
        vals = list(self.encoder_samples[key])
        return np.var(vals) if len(vals) > 1 else 0.0

    def start_measurement(self):
        """Initialize a new measurement session"""
        self.measurement_id += 1
        self.collecting = True
        for key in self.current_samples:
            self.current_samples[key] = []
            self.sample_metadata[key] = []
        print("\nCollecting 5 samples", end="", flush=True)

    def calculate_precision_metrics(self, samples):
        """Calculate median, jitter, and precision from samples"""
        if len(samples) < 3:
            return None, None, None
        
        median_angle = np.median(samples)
        jitter = max(samples) - min(samples)
        # Assuming 183mm reach (from reference script)
        precision_um = 183 * (jitter * np.pi / 180.0) * 1000
        return median_angle, jitter, precision_um

    def display_results(self):
        """Display measurement results with precision metrics"""
        print("\n\n" + "="*130)
        print(f"=== AIRPROBE PRECISION MEASUREMENT #{self.measurement_id} | Uptime {int(time.time() - self.start_time)}s ===")
        print("="*130)
        print(
            f"{'ENC':<4} | {'MEDIAN (deg)':>12} | {'JITTER (deg)':>13} | {'PRECISION (μm)':>15} | "
            f"{'AGC':<4} | {'MAG':<5} | {'STATUS':<12}"
        )
        print("-" * 130)

        for enc_name in ['M0', 'S0']:
            samples = self.current_samples[enc_name]
            
            if len(samples) == 0:
                print(f"{enc_name:<4} | {'---':>12} | {'---':>13} | {'---':>15} | {'---':<4} | {'---':<5} | OFFLINE")
                continue
            
            median, jitter, precision = self.calculate_precision_metrics(samples)
            
            if median is None:
                print(f"{enc_name:<4} | {'INSUFFICIENT':>12} | {'DATA':>13} | {'---':>15} | {'---':<4} | {'---':<5} | ERROR")
                continue
            
            # Get metadata from last sample
            meta = self.sample_metadata[enc_name][-1] if self.sample_metadata[enc_name] else {}
            agc = meta.get('agc', '---')
            mag = meta.get('mag', '---')
            status = meta.get('status', 'OK')
            
            print(
                f"{enc_name:<4} | {median:12.5f} | {jitter:13.5f} | {precision:15.1f} | "
                f"{agc:<4} | {mag:<5} | {status:<12}"
            )
        
        print("="*130)
        print("\nPress Enter to measure again (Q to quit)")

    def log_measurement(self):
        """Log all samples from current measurement to CSV"""
        for enc_name in ['M0', 'S0']:
            samples = self.current_samples[enc_name]
            median, jitter, precision = self.calculate_precision_metrics(samples)
            
            for i, angle in enumerate(samples):
                meta = self.sample_metadata[enc_name][i] if i < len(self.sample_metadata[enc_name]) else {}
                
                self.csv_writer.writerow([
                    time.time(),
                    round(time.time() - self.start_time, 3),
                    self.measurement_id,
                    i + 1,
                    meta.get('g_idx', ''),
                    enc_name,
                    angle,
                    median if median is not None else '',
                    jitter if jitter is not None else '',
                    precision if precision is not None else '',
                    meta.get('pkt', ''),
                    meta.get('agc', ''),
                    meta.get('mag', ''),
                    meta.get('magl', ''),
                    meta.get('magh', ''),
                    meta.get('cof', ''),
                    meta.get('status', '')
                ])
        
        self.csv_file.flush()

    def handle_data(self, payload):
        # Handle incoming BLE data - collect samples in manual mode
        parts = payload.split('|')
        try:
            g_idx = int(parts[0])

            if self.last_g_idx != -1 and g_idx - self.last_g_idx > 1:
                self.missed_packets += g_idx - self.last_g_idx - 1
            self.last_g_idx = g_idx

            if not self.collecting:
                return  # Ignore data when not actively collecting

            # Progress indicator
            print(".", end="", flush=True)

            # -------- MASTER --------
            _, d = parts[1].split(":")
            fields = d.split(",")

            ang = float(fields[0]) / 10000.0
            pkt = int(fields[1])
            agc = int(fields[2])
            mag = int(fields[3])
            magl = fields[4]
            magh = fields[5]
            cof = fields[6]

            status = "MASTER"
            if cof == "1":
                status = "CORDIC_ERR"
            elif magl == "1":
                status = "FIELD_HIGH"
            elif magh == "1":
                status = "FIELD_LOW"

            self.current_samples['M0'].append(ang)
            self.sample_metadata['M0'].append({
                'g_idx': g_idx, 'pkt': pkt, 'agc': agc, 'mag': mag,
                'magl': magl, 'magh': magh, 'cof': cof, 'status': status
            })

            # -------- SLAVE (S0 only) --------
            name = "S0"
            idx = 2  # S0 is at index 2 in the parts array

            if idx < len(parts):
                _, d = parts[idx].split(":")
                fields = d.split(",")

                if fields[0] != "OFFLINE":
                    ang = float(fields[0]) / 10000.0
                    pkt = int(fields[1])
                    agc = int(fields[2])
                    mag = int(fields[3])
                    magl = fields[4]
                    magh = fields[5]
                    cof = fields[6]

                    status = "OK"
                    if cof == "1":
                        status = "CORDIC_ERR"
                    elif magl == "1":
                        status = "FIELD_HIGH"
                    elif magh == "1":
                        status = "FIELD_LOW"

                    self.current_samples[name].append(ang)
                    self.sample_metadata[name].append({
                        'g_idx': g_idx, 'pkt': pkt, 'agc': agc, 'mag': mag,
                        'magl': magl, 'magh': magh, 'cof': cof, 'status': status
                    })

            # Check if we've collected enough samples
            # Use M0 as reference (master is always present)
            if len(self.current_samples['M0']) >= self.samples_per_measurement:
                self.collecting = False
                self.display_results()
                self.log_measurement()

        except Exception as e:
            print(f"\nParse error: {e}")
            print(payload)

    async def send_cmd(self, cmd):
        if self.client and self.client.is_connected:
            try:
                await self.client.write_gatt_char(COMMAND_UUID, cmd.encode())
                print(f"\n✓ Sent command: {cmd}\n")
            except Exception:
                print("\n✗ Command characteristic not available on gateway\n")


iface = GatewayInterface()


def notify(_, data):
    iface.handle_data(data.decode())


async def get_key():
    if sys.platform == "win32":
        return None
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


async def main():
    print("Scanning for gateway...")
    dev = await BleakScanner.find_device_by_filter(lambda d, a: d.name == DEVICE_NAME)
    if not dev:
        print("✗ Gateway not found")
        return

    async with BleakClient(dev) as c:
        iface.client = c
        await c.start_notify(CHARACTERISTIC_UUID, notify)
        print("✓ Connected to AIRPROBE_MASTER_GATEWAY\n")
        
        print("=" * 80)
        print(" AIRPROBE PRECISION MEASUREMENT SYSTEM")
        print(" Mode: Manual sampling (5 samples per encoder)")
        print(" Encoders: M0 (Master) + S0 (Slave)")
        print("=" * 80)
        print("\nPress Enter to start a measurement (Q to quit)\n")

        try:
            while c.is_connected:
                k = await get_key()
                if k:
                    if k.lower() == 'q':
                        print("\nDisconnecting...")
                        break
                    elif k == '\n':
                        # Start a new measurement
                        iface.start_measurement()
                        # Data will be collected via notify callback
                        # When 5 samples collected, results will auto-display
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nDisconnecting...")


if __name__ == "__main__":
    if sys.platform != "win32":
        import tty, termios
        old = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            asyncio.run(main())
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            iface.csv_file.close()
    else:
        asyncio.run(main())
        iface.csv_file.close()

