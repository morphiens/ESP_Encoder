"""
Oliver Encoder API
==================
Clean async API for reading encoder values from the Oliver BLE gateway.

Usage:
    import asyncio
    from oliver import OliverAPI, ENCODERS

    async def main():
        probe = OliverAPI()
        await probe.connect()

        # Request a fresh encoder reading on-demand
        await probe.get_instantaneous_encoder_values()

        # Take a precision measurement (collects 5 samples)
        await probe.get_encoder_values()

        # All encoder values (M0, S0, S1, S2) are always in probe._latest_raw
        for enc in ENCODERS:
            print(enc, probe._latest_raw[enc])

        # Set current position as zero reference
        probe.set_zero()

        # Disconnect cleanly
        await probe.disconnect()

    asyncio.run(main())

Author: Swaraj Dangare
"""

import asyncio
import time
import logging
import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from bleak import BleakClient, BleakScanner

# Set False to disable all prints and debug/info logging for the entire file.
VERBOSE = True

# ── Logging ───────────────────────────────────────────────────────────────────
logger = logging.getLogger("oliver")
logger.setLevel(logging.DEBUG if VERBOSE else logging.WARNING)

# ── BLE identifiers ───────────────────────────────────────────────────────────
SERVICE_UUID        = "6ab88bb9-cf50-4564-b1c4-f53be2abc53f"
CHARACTERISTIC_UUID = "1d4cd358-172d-4c33-b0b2-ddce9a071aab"
COMMAND_UUID        = "308a0c43-80f0-4b01-81e5-bb2798eb92f9"
DEVICE_NAME         = "hallever_gateway"

# ── Encoder names — order must match master BLE message ──────────────────────
ENCODERS = ("M0", "S0", "S1", "S2", "S3", "S4")

SCAN_TIMEOUT_S          = 15.0
MEASURE_TIMEOUT_S       = 30.0
SAMPLES_PER_MEASUREMENT = 5
REACH_MM                = 183.0  # lever arm for µm precision calc


# ── Data classes ──────────────────────────────────────────────────────────────
@dataclass
class EncoderReading:
    """Single encoder snapshot."""
    name: str
    angle_deg: float
    raw_angle_deg: float
    pkt: int = 0
    agc: int = 0
    mag: int = 0
    status: str = "OK"


@dataclass
class MeasurementResult:
    """Result of a multi-sample precision measurement."""
    measurement_id: int
    encoders: dict = field(default_factory=dict)
    timestamp: float = 0.0


# ── Packet parser ─────────────────────────────────────────────────────────────
def _parse_encoder_field(part: str, gateway_idx: int) -> tuple[str, Optional[tuple[float, dict]]]:
    """
    Parse one encoder field from the BLE payload.
    Format:  'XX:angle,pkt,agc,mag,magl,magh,cof'
             'XX:OFFLINE,0,0,0,0,0,0'

    Returns (name, None)                   for OFFLINE encoders.
    Returns (name, (angle_deg, meta_dict)) on success.
    """
    name_str, data = part.split(":", 1)
    fields = data.split(",")

    if fields[0] == "OFFLINE":
        return name_str, None

    angle = float(fields[0]) / 10000.0
    pkt   = int(fields[1]) if len(fields) > 1 else 0

    if len(fields) >= 7:
        agc, mag           = int(fields[2]), int(fields[3])
        magl, magh, cof    = fields[4], fields[5], fields[6]
    else:
        agc, mag, magl, magh, cof = 0, 0, "0", "0", "0"

    status = "OK"
    if cof  == "1": status = "CORDIC_ERR"
    elif magl == "1": status = "FIELD_LOW"
    elif magh == "1": status = "FIELD_HIGH"

    meta = {
        "g_idx": gateway_idx, "pkt": pkt, "agc": agc, "mag": mag,
        "magl": magl, "magh": magh, "cof": cof, "status": status,
    }
    return name_str, (angle, meta)


# ── Main API class ────────────────────────────────────────────────────────────
class OliverAPI:
    """
    Async API for the Oliver BLE encoder gateway.
    Supports M0 (master) + S0, S1, S2 (slaves) simultaneously.
    All public methods are safe to call from any async context.
    """

    def __init__(self):
        # Connection state
        self._client: Optional[BleakClient] = None
        self._device = None
        self._connected = False

        # Live encoder data — updated on every BLE notification for ALL encoders
        self._latest_raw:  dict[str, Optional[float]] = {e: None for e in ENCODERS}
        self._latest_meta: dict[str, dict]            = {e: {}   for e in ENCODERS}
        self._last_gateway_idx  = -1
        self._missed_packets    = 0
        self._last_notify_time: float = 0.0

        # Notification event (set on every _on_notify)
        self._notify_received: asyncio.Event = asyncio.Event()

        # Zero offsets — one per encoder
        self._zero_offsets: dict[str, float] = {e: 0.0 for e in ENCODERS}

        # Measurement collection state
        self._measurement_id = 0
        self._collecting     = False
        self._collect_samples: dict[str, list[float]] = {e: [] for e in ENCODERS}
        self._collect_meta:   dict[str, list[dict]]  = {e: [] for e in ENCODERS}

        # Timing
        self._last_connect_time_s: float = 0.0
        self._last_get_encoder_values_time_s: float = 0.0
        self._last_get_instantaneous_encoder_values_time_s: float = 0.0

    # ── Properties ────────────────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._connected and self._client is not None and self._client.is_connected

    @property
    def missed_packets(self) -> int:
        return self._missed_packets

    @property
    def last_gateway_idx(self) -> int:
        return self._last_gateway_idx

    @property
    def last_connect_time_s(self) -> float:
        return self._last_connect_time_s

    @property
    def last_get_encoder_values_time_s(self) -> float:
        return self._last_get_encoder_values_time_s

    @property
    def last_get_instantaneous_encoder_values_time_s(self) -> float:
        return self._last_get_instantaneous_encoder_values_time_s

    # ── Connect / Disconnect / Reconnect ──────────────────────────────────

    async def connect(self, timeout: float = SCAN_TIMEOUT_S) -> bool:
        if self.is_connected:
            logger.info("Already connected.")
            return True

        t0 = time.perf_counter()
        logger.info("Scanning for %s …", DEVICE_NAME)
        self._device = await BleakScanner.find_device_by_filter(
            lambda d, ad: SERVICE_UUID.lower() in [s.lower() for s in ad.service_uuids],
            timeout=timeout,
        )
        if self._device is None:
            logger.error("Gateway not found (scanned for %.0fs).", timeout)
            return False

        logger.info("Found %s (%s). Connecting …", DEVICE_NAME, self._device.address)
        self._client = BleakClient(self._device, disconnected_callback=self._on_disconnect)
        await self._client.connect()
        await self._client.start_notify(CHARACTERISTIC_UUID, self._on_notify)
        self._connected = True
        self._last_connect_time_s = time.perf_counter() - t0
        logger.info("Connected to %s (%.2f s).", DEVICE_NAME, self._last_connect_time_s)
        return True

    async def disconnect(self):
        if self._client is not None:
            try:
                await self._client.disconnect()
            except Exception:
                pass
        self._connected = False
        logger.info("Disconnected.")

    async def reconnect(self, timeout: float = SCAN_TIMEOUT_S) -> bool:
        logger.info("Reconnecting …")
        await self.disconnect()
        await asyncio.sleep(1.0)
        return await self.connect(timeout=timeout)

    def _on_disconnect(self, _client):
        self._connected = False
        logger.warning("BLE disconnected unexpectedly.")

    # ── Live encoder values ────────────────────────────────────────────────

    async def get_instantaneous_encoder_values(self) -> tuple[Optional[float], Optional[float]]:
        """
        Send READ, wait for the notification, return (M0, S0) for backward
        compatibility. All encoders (M0, S0, S1, S2) are also updated in
        self._latest_raw for direct access.
        """
        t0 = time.perf_counter()
        if not self.is_connected:
            ok = await self.reconnect()
            if not ok:
                self._last_get_instantaneous_encoder_values_time_s = time.perf_counter() - t0
                return (None, None)

        self._notify_received.clear()
        await self.send_command("READ")
        try:
            await asyncio.wait_for(self._notify_received.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            logger.warning("READ response timed out.")
            self._last_get_instantaneous_encoder_values_time_s = time.perf_counter() - t0
            return (None, None)

        m0_raw = self._latest_raw.get("M0")
        s0_raw = self._latest_raw.get("S0")
        m0 = (m0_raw - self._zero_offsets["M0"]) if m0_raw is not None else None
        s0 = (s0_raw - self._zero_offsets["S0"]) if s0_raw is not None else None
        self._last_get_instantaneous_encoder_values_time_s = time.perf_counter() - t0
        return (m0, s0)

    # ── Precision measurement ──────────────────────────────────────────────

    async def get_encoder_values(
        self,
        num_samples: int = SAMPLES_PER_MEASUREMENT,
        timeout: float = MEASURE_TIMEOUT_S,
    ) -> tuple[Optional[float], Optional[float]]:
        """
        Collect *num_samples* readings for every encoder, compute medians,
        store full results in self._latest_raw, and return (M0_median, S0_median)
        for backward compatibility.
        All encoder medians are also available via self._latest_raw after this call.
        """
        t0 = time.perf_counter()
        if not self.is_connected:
            ok = await self.reconnect()
            if not ok:
                self._last_get_encoder_values_time_s = time.perf_counter() - t0
                return (None, None)

        self._measurement_id += 1
        mid = self._measurement_id

        self._collect_samples = {e: [] for e in ENCODERS}
        self._collect_meta    = {e: [] for e in ENCODERS}
        self._collecting      = True
        per_sample_timeout    = timeout / num_samples

        logger.info("Measurement #%d: collecting %d samples …", mid, num_samples)

        try:
            for i in range(num_samples):
                self._notify_received.clear()
                await self.send_command("READ")
                try:
                    await asyncio.wait_for(self._notify_received.wait(), timeout=per_sample_timeout)
                except asyncio.TimeoutError:
                    self._collecting = False
                    self._last_get_encoder_values_time_s = time.perf_counter() - t0
                    raise TimeoutError(
                        f"Measurement #{mid} timed out at sample {i+1}/{num_samples} "
                        f"after {time.perf_counter() - t0:.1f}s."
                    )
        finally:
            self._collecting = False

        # Build result for all encoders and update _latest_raw with medians
        result = MeasurementResult(measurement_id=mid, timestamp=time.time())
        for enc in ENCODERS:
            samples  = self._collect_samples[enc]
            enc_data: dict = {"samples": samples, "count": len(samples)}

            if len(samples) >= 1:
                median       = float(np.median(samples))
                jitter       = (max(samples) - min(samples)) if len(samples) >= 2 else 0.0
                precision_um = REACH_MM * (jitter * np.pi / 180.0) * 1000.0
                enc_data.update(median=median, jitter=jitter, precision_um=precision_um)
                # Update _latest_raw with the median so callers can read it directly
                self._latest_raw[enc] = median
            else:
                enc_data.update(median=None, jitter=None, precision_um=None)
                self._latest_raw[enc] = None

            metas            = self._collect_meta[enc]
            enc_data["status"] = metas[-1].get("status", "OK") if metas else "NO_DATA"
            result.encoders[enc] = enc_data

        self._last_get_encoder_values_time_s = time.perf_counter() - t0
        logger.info("Measurement #%d complete (%.2f s).", mid, self._last_get_encoder_values_time_s)

        m0_median = result.encoders["M0"].get("median")
        s0_median = result.encoders["S0"].get("median")
        return (m0_median, s0_median)

    # ── Zero reference ─────────────────────────────────────────────────────

    def set_zero(self) -> dict[str, float]:
        """
        Capture current raw angles as the zero reference for all encoders.
        Raises RuntimeError if no data has been received yet.
        """
        applied: dict[str, float] = {}
        for enc in ENCODERS:
            raw = self._latest_raw[enc]
            if raw is not None:
                self._zero_offsets[enc] = raw
                applied[enc] = raw

        if not applied:
            raise RuntimeError("No encoder data received yet — cannot set zero.")

        logger.info("Zero set: %s", {k: f"{v:.5f}°" for k, v in applied.items()})
        return applied

    def clear_zero(self):
        """Remove all zero offsets (revert to absolute angles)."""
        self._zero_offsets = {e: 0.0 for e in ENCODERS}
        logger.info("Zero offsets cleared.")

    # ── Send BLE command ───────────────────────────────────────────────────

    async def send_command(self, cmd: str) -> bool:
        if not self.is_connected:
            raise RuntimeError("Not connected.")
        try:
            await self._client.write_gatt_char(COMMAND_UUID, cmd.encode())
            logger.info("Sent command: %s", cmd)
            return True
        except Exception as exc:
            logger.error("Command failed: %s", exc)
            return False

    # ── BLE notification handler ───────────────────────────────────────────

    def _on_notify(self, _sender, data: bytearray):
        """
        Called by bleak on every BLE notification.

        Expected payload format:
            packetIdx|M0:val,pkt,agc,mag,magl,magh,cof|S0:...|S1:...|S2:...

        FIXED: previously only parsed parts[1] (M0) and parts[2] (S0).
        Now iterates over ALL parts[1:] so S1, S2 (and any future slaves)
        are always parsed and stored in _latest_raw regardless of which
        slaves are online or offline.
        """
        try:
            payload = data.decode()
            parts   = payload.split("|")
            g_idx   = int(parts[0])

            # Track missed packets
            if self._last_gateway_idx != -1 and g_idx - self._last_gateway_idx > 1:
                self._missed_packets += g_idx - self._last_gateway_idx - 1
            self._last_gateway_idx = g_idx
            self._last_notify_time = time.time()

            # ── Parse every encoder field in the message ──────────────────
            # parts[1] = M0, parts[2] = S0, parts[3] = S1, parts[4] = S2
            # Each is parsed independently; an OFFLINE slave does not affect others.
            parsed: dict[str, Optional[tuple[float, dict]]] = {}

            for part in parts[1:]:
                try:
                    name, result = _parse_encoder_field(part, g_idx)
                except Exception as field_exc:
                    logger.debug("Field parse error (%s): %s", part, field_exc)
                    continue

                parsed[name] = result

                if result is not None:
                    raw_ang, meta = result
                    self._latest_raw[name]  = raw_ang
                    self._latest_meta[name] = meta
                else:
                    # Slave is OFFLINE — mark as None so callers see no data
                    self._latest_raw[name]  = None
                    self._latest_meta[name] = {}

            # ── Accumulate samples for a precision measurement ─────────────
            if self._collecting:
                for enc in ENCODERS:
                    result = parsed.get(enc)
                    if result is None:
                        continue  # offline or not in this message — skip, don't crash
                    raw_ang, meta = result
                    adj = raw_ang - self._zero_offsets[enc]
                    self._collect_samples[enc].append(adj)
                    self._collect_meta[enc].append(meta)

            # Signal that a notification was received
            self._notify_received.set()

        except Exception as exc:
            logger.debug("Parse error: %s | payload=%s", exc, data)


# ── Interactive demo ───────────────────────────────────────────────────────────
async def _interactive_demo():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")

    probe = OliverAPI()
    ok = await probe.connect()
    if not ok:
        print("✗ Could not find gateway.")
        return

    print("\n" + "=" * 70)
    print("  Oliver API — Interactive Demo")
    print(f"  Encoders: {', '.join(ENCODERS)}")
    print("  Commands:  Enter = measure  |  z = set zero  |  v = instantaneous")
    print("             c = connect  |  d = disconnect  |  r = reconnect  |  q = quit")
    print("=" * 70 + "\n")

    import sys, select

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

                elif key.lower() == "c":
                    ok = await probe.connect()
                    print("✓ Connected." if ok else "✗ Connect failed.")

                elif key.lower() == "d":
                    await probe.disconnect()
                    print("✓ Disconnected.")

                elif key.lower() == "r":
                    ok = await probe.reconnect()
                    print("✓ Reconnected." if ok else "✗ Reconnect failed.")

                elif key == "\n":
                    try:
                        await probe.get_encoder_values()
                        print(f"\n{'=' * 80}")
                        print("  Precision Measurement")
                        print(f"{'=' * 80}")
                        for enc in ENCODERS:
                            val = probe._latest_raw.get(enc)
                            if val is not None:
                                print(f"  {enc:<4} {val:12.5f}°")
                            else:
                                print(f"  {enc:<4} offline")
                        print(f"{'=' * 80}\n")
                    except TimeoutError as e:
                        print(f"✗ {e}")

                elif key.lower() == "v":
                    await probe.get_instantaneous_encoder_values()
                    for enc in ENCODERS:
                        val = probe._latest_raw.get(enc)
                        if val is not None:
                            print(f"  {enc}: {val:.5f}°")
                        else:
                            print(f"  {enc}: offline")
                    print(f"  ({probe.last_get_instantaneous_encoder_values_time_s:.3f} s)")

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        if sys.platform != "win32" and old_settings:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        await probe.disconnect()
        print("\nDone.")


if __name__ == "__main__":
    asyncio.run(_interactive_demo())