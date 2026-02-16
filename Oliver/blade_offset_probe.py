"""
Oliver Encoder API
==================
Clean async API for reading encoder values from the Oliver BLE gateway.

Usage:
    import asyncio
    from blade_offset_probe import OliverAPI

    async def main():
        probe = OliverAPI()
        await probe.connect()

        # Read the latest encoder angles (updates every ~1s); (M0_deg, S0_deg)
        m0, s0 = await probe.get_instantaneous_encoder_values()

        # Take a precision measurement (collects 5 samples); returns (M0_median, S0_median)
        m0_med, s0_med = await probe.get_encoder_values()

        # Set current position as zero reference
        probe.set_zero()

        # Take another measurement (now relative to zero)
        result = await probe.measure()

        # Disconnect cleanly
        await probe.disconnect()

    asyncio.run(main())
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

# ── Logging ──────────────────────────────────────────────────────────────────
logger = logging.getLogger("oliver")
logger.setLevel(logging.DEBUG if VERBOSE else logging.WARNING)

# ── BLE identifiers ─────────────────────────────────────────────────────────
DEVICE_NAME = "Oliver_1"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
COMMAND_UUID = "8d53dc1d-1db7-4cd3-868b-8a527460aa84"

ENCODERS = ("M0", "S0")
SCAN_TIMEOUT_S = 15.0
MEASURE_TIMEOUT_S = 30.0
SAMPLES_PER_MEASUREMENT = 5
REACH_MM = 183.0  # lever arm for µm precision calc


# ── Data classes ─────────────────────────────────────────────────────────────
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

    # Per-encoder nested data
    # encoders = {
    #     "M0": {"samples": [...], "median": float, "jitter": float, "precision_um": float, "status": str},
    #     "S0": { ... },
    # }


# ── Packet parser ────────────────────────────────────────────────────────────
def _parse_encoder_field(part: str, gateway_idx: int) -> tuple[str, Optional[tuple[float, dict]]]:
    """
    Parse a single encoder field from the BLE payload.
    Format: 'XX:angle,pkt[,agc,mag,magl,magh,cof]'
    Returns (name, None) for OFFLINE encoders.
    Returns (name, (angle_deg, metadata_dict)) on success.
    """
    name_str, data = part.split(":")
    fields = data.split(",")

    if fields[0] == "OFFLINE":
        return name_str, None

    angle = float(fields[0]) / 10000.0
    pkt = int(fields[1]) if len(fields) > 1 else 0

    if len(fields) >= 7:
        agc, mag = int(fields[2]), int(fields[3])
        magl, magh, cof = fields[4], fields[5], fields[6]
    else:
        agc, mag, magl, magh, cof = 0, 0, "0", "0", "0"

    status = "OK"
    if cof == "1":
        status = "CORDIC_ERR"
    elif magl == "1":
        status = "FIELD_LOW"
    elif magh == "1":
        status = "FIELD_HIGH"

    meta = {
        "g_idx": gateway_idx, "pkt": pkt, "agc": agc, "mag": mag,
        "magl": magl, "magh": magh, "cof": cof, "status": status,
    }
    return name_str, (angle, meta)


# ── Main API class ───────────────────────────────────────────────────────────
class OliverAPI:
    """
    Async API for the Oliver BLE encoder gateway.

    All public methods are safe to call from any async context.
    The object manages its own BLE connection lifecycle.
    """

    def __init__(self):
        # Connection state
        self._client: Optional[BleakClient] = None
        self._device = None
        self._connected = False

        # Live encoder data (updated on every BLE notification)
        self._latest_raw: dict[str, Optional[float]] = {e: None for e in ENCODERS}
        self._latest_meta: dict[str, dict] = {e: {} for e in ENCODERS}
        self._last_gateway_idx = -1
        self._missed_packets = 0
        self._last_notify_time: float = 0.0

        # Zero offsets
        self._zero_offsets: dict[str, float] = {e: 0.0 for e in ENCODERS}

        # Measurement collection state
        self._measurement_id = 0
        self._collecting = False
        self._collect_samples: dict[str, list[float]] = {e: [] for e in ENCODERS}
        self._collect_meta: dict[str, list[dict]] = {e: [] for e in ENCODERS}
        self._collect_done: Optional[asyncio.Event] = None

        # Timing (seconds) of last connect and last get_encoder_values / get_instantaneous call
        self._last_connect_time_s: float = 0.0
        self._last_get_encoder_values_time_s: float = 0.0
        self._last_get_instantaneous_encoder_values_time_s: float = 0.0

    # ── Properties ───────────────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        """True when BLE link is active."""
        return self._connected and self._client is not None and self._client.is_connected

    @property
    def missed_packets(self) -> int:
        return self._missed_packets

    @property
    def last_gateway_idx(self) -> int:
        return self._last_gateway_idx

    @property
    def last_connect_time_s(self) -> float:
        """Time in seconds taken by the last connect() (or reconnect()) that did a full connect."""
        return self._last_connect_time_s

    @property
    def last_get_encoder_values_time_s(self) -> float:
        """Time in seconds taken by the last get_encoder_values() call from start to return."""
        return self._last_get_encoder_values_time_s

    @property
    def last_get_instantaneous_encoder_values_time_s(self) -> float:
        """Time in seconds taken by the last get_instantaneous_encoder_values() call from start to return."""
        return self._last_get_instantaneous_encoder_values_time_s

    # ── Connect / Disconnect / Reconnect ─────────────────────────────────

    async def connect(self, timeout: float = SCAN_TIMEOUT_S) -> bool:
        """
        Scan for the gateway and connect.
        Returns True on success, False if the device was not found.
        """
        if self.is_connected:
            logger.info("Already connected.")
            return True

        t0 = time.perf_counter()
        logger.info("Scanning for %s …", DEVICE_NAME)
        self._device = await BleakScanner.find_device_by_filter(
            lambda d, _ad: d.name == DEVICE_NAME, timeout=timeout,
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
        """Cleanly close the BLE connection."""
        if self._client is not None:
            try:
                await self._client.disconnect()
            except Exception:
                pass
        self._connected = False
        logger.info("Disconnected.")

    async def reconnect(self, timeout: float = SCAN_TIMEOUT_S) -> bool:
        """
        Drop existing connection (if any) and reconnect from scratch.
        Returns True on success.
        """
        logger.info("Reconnecting …")
        await self.disconnect()
        await asyncio.sleep(1.0)  # give BLE stack time to clean up
        return await self.connect(timeout=timeout)

    def _on_disconnect(self, _client):
        self._connected = False
        logger.warning("BLE disconnected unexpectedly.")

    # ── Live encoder values ──────────────────────────────────────────────

    async def get_instantaneous_encoder_values(self) -> tuple[Optional[float], Optional[float]]:
        """
        Return the latest encoder readings (zero-adjusted) as (M0, S0) angles in degrees.

        If not connected to BLE, reconnects first. Returns (None, None) if reconnect fails.
        Value is None for an encoder that has not reported yet.
        """
        t0 = time.perf_counter()
        if not self.is_connected:
            ok = await self.reconnect()
            if not ok:
                self._last_get_instantaneous_encoder_values_time_s = time.perf_counter() - t0
                return (None, None)
        m0_raw = self._latest_raw["M0"]
        s0_raw = self._latest_raw["S0"]
        m0 = (m0_raw - self._zero_offsets["M0"]) if m0_raw is not None else None
        s0 = (s0_raw - self._zero_offsets["S0"]) if s0_raw is not None else None
        self._last_get_instantaneous_encoder_values_time_s = time.perf_counter() - t0
        return (m0, s0)

    # ── Precision measurement ────────────────────────────────────────────

    async def get_encoder_values(
        self,
        num_samples: int = SAMPLES_PER_MEASUREMENT,
        timeout: float = MEASURE_TIMEOUT_S,
    ) -> tuple[Optional[float], Optional[float]]:
        """
        Collect *num_samples* consecutive readings from every encoder,
        compute medians, and return (M0_median_deg, S0_median_deg).

        If not connected to BLE, reconnects first. Returns (None, None) if reconnect fails.
        Raises TimeoutError if samples are not received in time.
        """
        t0 = time.perf_counter()
        if not self.is_connected:
            ok = await self.reconnect()
            if not ok:
                self._last_get_encoder_values_time_s = time.perf_counter() - t0
                return (None, None)

        self._measurement_id += 1
        mid = self._measurement_id

        # Prepare collection buffers
        self._collect_samples = {e: [] for e in ENCODERS}
        self._collect_meta = {e: [] for e in ENCODERS}
        self._collect_done = asyncio.Event()
        self._samples_needed = num_samples
        self._collecting = True

        logger.info("Measurement #%d: collecting %d samples …", mid, num_samples)

        # Wait for collection to complete (or timeout)
        try:
            await asyncio.wait_for(self._collect_done.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            self._collecting = False
            self._last_get_encoder_values_time_s = time.perf_counter() - t0
            raise TimeoutError(
                f"Measurement #{mid} timed out after {timeout}s "
                f"(got {len(self._collect_samples['M0'])}/{num_samples} M0 samples)."
            )
        finally:
            self._collecting = False

        # Build result
        result = MeasurementResult(measurement_id=mid, timestamp=time.time())
        for enc in ENCODERS:
            samples = self._collect_samples[enc]
            enc_data: dict = {"samples": samples, "count": len(samples)}

            if len(samples) >= 1:
                median = float(np.median(samples))
                jitter = (max(samples) - min(samples)) if len(samples) >= 2 else 0.0
                precision_um = REACH_MM * (jitter * np.pi / 180.0) * 1000.0
                enc_data.update(median=median, jitter=jitter, precision_um=precision_um)
            else:
                enc_data.update(median=None, jitter=None, precision_um=None)

            # Attach last known status
            metas = self._collect_meta[enc]
            enc_data["status"] = metas[-1].get("status", "OK") if metas else "NO_DATA"
            result.encoders[enc] = enc_data

        self._last_get_encoder_values_time_s = time.perf_counter() - t0
        logger.info("Measurement #%d complete (%.2f s).", mid, self._last_get_encoder_values_time_s)
        m0_median = result.encoders["M0"].get("median")
        s0_median = result.encoders["S0"].get("median")
        return (m0_median, s0_median)

    # ── Zero reference ───────────────────────────────────────────────────

    def set_zero(self) -> dict[str, float]:
        """
        Capture the current raw angles as the zero reference.
        All subsequent get_encoder_values() and measure() calls will be
        relative to this position.

        Returns a dict of the offsets that were applied, e.g.:
            {"M0": 166.990, "S0": 125.395}

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
        """Remove any zero offset (revert to absolute angles)."""
        self._zero_offsets = {e: 0.0 for e in ENCODERS}
        logger.info("Zero offsets cleared.")

    # ── Send BLE command to gateway ──────────────────────────────────────

    async def send_command(self, cmd: str) -> bool:
        """
        Send an arbitrary string command to the gateway's command characteristic.
        Returns True on success.
        """
        if not self.is_connected:
            raise RuntimeError("Not connected.")
        try:
            await self._client.write_gatt_char(COMMAND_UUID, cmd.encode())
            logger.info("Sent command: %s", cmd)
            return True
        except Exception as exc:
            logger.error("Command failed: %s", exc)
            return False

    # ── Internal BLE notification handler ────────────────────────────────

    def _on_notify(self, _sender, data: bytearray):
        """Called by bleak on every BLE notification."""
        try:
            payload = data.decode()
            parts = payload.split("|")
            g_idx = int(parts[0])

            # Track missed packets
            if self._last_gateway_idx != -1 and g_idx - self._last_gateway_idx > 1:
                self._missed_packets += g_idx - self._last_gateway_idx - 1
            self._last_gateway_idx = g_idx
            self._last_notify_time = time.time()

            # ── Parse Master (M0) ──
            _, m_result = _parse_encoder_field(parts[1], g_idx)
            if m_result is not None:
                raw_ang, meta = m_result
                self._latest_raw["M0"] = raw_ang
                self._latest_meta["M0"] = meta

            # ── Parse Slave (S0) ──
            if len(parts) > 2:
                _, s_result = _parse_encoder_field(parts[2], g_idx)
                if s_result is not None:
                    raw_ang, meta = s_result
                    self._latest_raw["S0"] = raw_ang
                    self._latest_meta["S0"] = meta

            # ── Collecting samples for a measurement? ──
            if self._collecting and self._collect_done is not None:
                for enc, result in [("M0", m_result), ("S0", s_result if len(parts) > 2 else None)]:
                    if result is None:
                        continue
                    raw_ang, meta = result
                    adj = raw_ang - self._zero_offsets[enc]
                    if enc == "M0" and meta["status"] == "OK":
                        meta["status"] = "MASTER"
                    self._collect_samples[enc].append(adj)
                    self._collect_meta[enc].append(meta)

                # Done when master has enough samples
                if len(self._collect_samples["M0"]) >= self._samples_needed:
                    self._collect_done.set()

        except Exception as exc:
            logger.debug("Parse error: %s | payload=%s", exc, data)


# ── Convenience: run interactively if executed directly ──────────────────────
async def _interactive_demo():
    """Quick interactive session for testing."""
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")

    probe = OliverAPI()
    ok = await probe.connect()
    if not ok:
        if VERBOSE:
            print("✗ Could not find gateway.")
        return

    if VERBOSE:
        print("\n" + "=" * 70)
        print("  Oliver API — Interactive Demo")
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
            # Non-blocking key check
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
                        if VERBOSE:
                            print(f"✓ Zero set: {offsets}")
                    except RuntimeError as e:
                        if VERBOSE:
                            print(f"✗ {e}")

                elif key.lower() == "c":
                    ok = await probe.connect()
                    if VERBOSE:
                        print("✓ Connected." if ok else "✗ Connect failed.")

                elif key.lower() == "d":
                    await probe.disconnect()
                    if VERBOSE:
                        print("✓ Disconnected.")

                elif key.lower() == "r":
                    ok = await probe.reconnect()
                    if VERBOSE:
                        print("✓ Reconnected." if ok else "✗ Reconnect failed.")

                elif key == "\n":
                    try:
                        m0_med, s0_med = await probe.get_encoder_values()
                        if VERBOSE:
                            print(f"\n{'=' * 80}")
                            print("  Measurement (M0_median, S0_median)")
                            print(f"{'=' * 80}")
                            if m0_med is not None:
                                print(f"  M0   {m0_med:12.5f}°")
                            else:
                                print("  M0   no data")
                            if s0_med is not None:
                                print(f"  S0   {s0_med:12.5f}°")
                            else:
                                print("  S0   no data")
                            print(f"{'=' * 80}\n")
                    except TimeoutError as e:
                        if VERBOSE:
                            print(f"✗ {e}")

                elif key == "v":
                    m0, s0 = await probe.get_instantaneous_encoder_values()
                    if VERBOSE:
                        print(f"  M0: {m0:.5f}°" if m0 is not None else "  M0: no data")
                        print(f"  S0: {s0:.5f}°" if s0 is not None else "  S0: no data")
                        print(f"  (%.3f s)" % probe.last_get_instantaneous_encoder_values_time_s)

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        if sys.platform != "win32" and old_settings:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        await probe.disconnect()
        if VERBOSE:
            print("\nDone.")


if __name__ == "__main__":
    asyncio.run(_interactive_demo())
