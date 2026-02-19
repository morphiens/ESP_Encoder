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

# BLE configuration
DEVICE_NAME = "Oliver_3"
CHARACTERISTIC_UUID = "1d4cd358-172d-4c33-b0b2-ddce9a071aab"
COMMAND_UUID = "308a0c43-80f0-4b01-81e5-bb2798eb92f9"


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

        # Zero offset: subtract this from raw angles to get relative values
        self.zero_offsets = {'M0': 0.0, 'S0': 0.0}
        # Track the latest raw angle from each encoder (updated every packet)
        self.latest_raw = {'M0': None, 'S0': None}

        # Legacy variance tracking (kept for reference)
        self.sample_size = 100
        self.encoder_samples = {
            'M0': deque(maxlen=self.sample_size),
            'S0': deque(maxlen=self.sample_size),
        }

    def set_zero(self):
        """Set current encoder positions as the zero reference"""
        zeroed = []
        for enc in ['M0', 'S0']:
            if self.latest_raw[enc] is not None:
                self.zero_offsets[enc] = self.latest_raw[enc]
                zeroed.append(f"{enc}={self.latest_raw[enc]:.5f}°")
        if zeroed:
            print(f"\n✓ ZERO set: {', '.join(zeroed)}")
            print("  All future measurements will be relative to this position.")
        else:
            print("\n✗ No encoder data received yet — cannot set zero.")
        print()

    def variance(self, key):
        vals = list(self.encoder_samples[key])
        return np.var(vals) if len(vals) > 1 else 0.0

    def start_measurement(self):
        """Initialize a new measurement session"""
        self.measurement_id += 1
        self.collecting = True
        self.samples_collected = False
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
        print(f"=== OLIVER PRECISION MEASUREMENT #{self.measurement_id} | Uptime {int(time.time() - self.start_time)}s ===")
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
        print("\nPress Enter to measure again (Z to set zero, Q to quit)")

    def handle_data(self, payload):
        # Handle incoming BLE data - collect samples in manual mode
        parts = payload.split('|')
        try:
            g_idx = int(parts[0])

            if self.last_g_idx != -1 and g_idx - self.last_g_idx > 1:
                self.missed_packets += g_idx - self.last_g_idx - 1
            self.last_g_idx = g_idx

            # -------- helper to parse an encoder field --------
            def parse_encoder(part):
                """Parse 'XX:angle,pkt[,agc,mag,magl,magh,cof]' -> (name, dict)
                Supports both full 7-field and short 2-field formats."""
                name_str, d = part.split(":")
                fields = d.split(",")

                if fields[0] == "OFFLINE":
                    return name_str, None

                ang = float(fields[0]) / 10000.0
                pkt = int(fields[1]) if len(fields) > 1 else 0

                # Full format has 7 fields; short format has 2
                if len(fields) >= 7:
                    agc = int(fields[2])
                    mag = int(fields[3])
                    magl = fields[4]
                    magh = fields[5]
                    cof = fields[6]
                else:
                    agc = 0
                    mag = 0
                    magl = "0"
                    magh = "0"
                    cof = "0"

                status = "OK"
                if cof == "1":
                    status = "CORDIC_ERR"
                elif magl == "1":
                    status = "FIELD_LOW"
                elif magh == "1":
                    status = "FIELD_HIGH"

                meta = {
                    'g_idx': g_idx, 'pkt': pkt, 'agc': agc, 'mag': mag,
                    'magl': magl, 'magh': magh, 'cof': cof, 'status': status
                }
                return name_str, (ang, meta)

            # -------- MASTER --------
            m_result = None
            name_str, result = parse_encoder(parts[1])
            if result is not None:
                raw_ang, meta = result
                self.latest_raw['M0'] = raw_ang
                m_result = (raw_ang, meta)

            # -------- SLAVE (S0 only) --------
            s_result = None
            idx = 2  # S0 is at index 2 in the parts array
            if idx < len(parts):
                name_str, result = parse_encoder(parts[idx])
                if result is not None:
                    raw_ang, meta = result
                    self.latest_raw['S0'] = raw_ang
                    s_result = (raw_ang, meta)

            if not self.collecting:
                return  # Ignore data when not actively collecting

            # Progress indicator
            print(".", end="", flush=True)

            # Store zero-adjusted samples for MASTER
            if m_result is not None:
                raw_ang, meta = m_result
                adj_ang = raw_ang - self.zero_offsets['M0']
                meta['status'] = "MASTER" if meta['status'] == "OK" else meta['status']
                self.current_samples['M0'].append(adj_ang)
                self.sample_metadata['M0'].append(meta)

            # Store zero-adjusted samples for SLAVE
            if s_result is not None:
                raw_ang, meta = s_result
                adj_ang = raw_ang - self.zero_offsets['S0']
                self.current_samples['S0'].append(adj_ang)
                self.sample_metadata['S0'].append(meta)

            # Check if we've collected enough samples
            # Use M0 as reference (master is always present)
            if len(self.current_samples['M0']) >= self.samples_per_measurement:
                self.collecting = False
                self.samples_collected = True
                self.display_results()

        except Exception as e:
            print(f"\nParse error: {e}")
            print(payload)

    async def send_read(self):
        """Send a READ command to trigger one on-demand reading."""
        if self.client and self.client.is_connected:
            try:
                await self.client.write_gatt_char(COMMAND_UUID, b"READ")
            except Exception:
                pass

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
        print("✓ Connected to Oliver_3\n")
        
        print("=" * 80)
        print(" OLIVER PRECISION MEASUREMENT SYSTEM")
        print(" Mode: On-demand sampling (5 samples per encoder)")
        print(" Encoders: M0 (Master) + S0 (Slave)")
        print("=" * 80)
        print("\nPress Enter to start a measurement (Z to set zero, Q to quit)\n")

        try:
            while c.is_connected:
                k = await get_key()
                if k:
                    if k.lower() == 'q':
                        print("\nDisconnecting...")
                        break
                    elif k.lower() == 'z':
                        await iface.send_read()
                        await asyncio.sleep(0.3)
                        iface.set_zero()
                    elif k == '\n':
                        iface.start_measurement()
                        while iface.collecting:
                            await iface.send_read()
                            await asyncio.sleep(0.15)
                await asyncio.sleep(0.05)
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
    else:
        asyncio.run(main())

