"""
Oliver Encoder Serial API
=========================
Clean async API for reading encoder values from the Oliver gateway via Serial USB.

Author: Antigravity
"""

import asyncio
import time
import logging
import numpy as np
import serial
from dataclasses import dataclass, field
from typing import Optional

# ── Configuration ────────────────────────────────────────────────────────────
VERBOSE = True
logger = logging.getLogger("oliver_serial")
logger.setLevel(logging.DEBUG if VERBOSE else logging.WARNING)

ENCODERS = ("M0", "S0")
SAMPLES_PER_MEASUREMENT = 5
REACH_MM = 183.0  # lever arm for µm precision calc

@dataclass
class MeasurementResult:
    """Result of a multi-sample precision measurement."""
    measurement_id: int
    encoders: dict = field(default_factory=dict)
    timestamp: float = 0.0

def _parse_encoder_field(part: str, gateway_idx: int) -> tuple[str, Optional[tuple[float, dict]]]:
    """
    Parse a single encoder field from the payload.
    Format: 'XX:angle,pkt[,agc,mag,magl,magh,cof]'
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

class OliverSerialAPI:
    """
    Async API for the Oliver encoder gateway via Serial USB.
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None
        self._connected = False

        # Live encoder data
        self._latest_raw: dict[str, Optional[float]] = {e: None for e in ENCODERS}
        self._latest_meta: dict[str, dict] = {e: {} for e in ENCODERS}
        self._last_gateway_idx = -1
        
        # Zero offsets
        self._zero_offsets: dict[str, float] = {e: 0.0 for e in ENCODERS}
        
        self._measurement_id = 0
        self._collecting = False
        self._collect_samples: dict[str, list[float]] = {e: [] for e in ENCODERS}
        self._collect_meta: dict[str, list[dict]] = {e: [] for e in ENCODERS}
        
        self._read_task: Optional[asyncio.Task] = None
        self._data_event = asyncio.Event()

    @property
    def is_connected(self) -> bool:
        return self._connected and self._ser is not None and self._ser.is_open

    async def connect(self) -> bool:
        if self.is_connected:
            return True
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self._connected = True
            self._read_task = asyncio.create_task(self._read_loop())
            logger.info(f"Connected to Serial port {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Serial: {e}")
            return False

    async def disconnect(self):
        self._connected = False
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
        if self._ser:
            self._ser.close()
        logger.info("Disconnected from Serial.")

    async def _read_loop(self):
        loop = asyncio.get_running_loop()
        while self._connected:
            try:
                # Read line from serial (run_in_executor to avoid blocking)
                line = await loop.run_in_executor(None, self._ser.readline)
                if not line:
                    await asyncio.sleep(0.01)
                    continue
                
                decoded = line.decode(errors='ignore').strip()
                if decoded.startswith("DATA:"):
                    payload = decoded[5:]
                    self._parse_payload(payload)
                    self._data_event.set()
                    self._data_event.clear()
            except Exception as e:
                logger.error(f"Serial read error: {e}")
                await asyncio.sleep(1)

    def _parse_payload(self, payload: str):
        try:
            parts = payload.split("|")
            g_idx = int(parts[0])
            self._last_gateway_idx = g_idx

            # ── Parse Master (M0) ──
            _, m_result = _parse_encoder_field(parts[1], g_idx)
            if m_result is not None:
                raw_ang, meta = m_result
                self._latest_raw["M0"] = raw_ang
                self._latest_meta["M0"] = meta

            # ── Parse Slave (S0) ──
            s_result = None
            if len(parts) > 2:
                _, s_result = _parse_encoder_field(parts[2], g_idx)
                if s_result is not None:
                    raw_ang, meta = s_result
                    self._latest_raw["S0"] = raw_ang
                    self._latest_meta["S0"] = meta

            # ── Collecting samples? ──
            if self._collecting:
                for enc, result in [("M0", m_result), ("S0", s_result)]:
                    if result is None:
                        continue
                    raw_ang, meta = result
                    adj = raw_ang - self._zero_offsets[enc]
                    self._collect_samples[enc].append(adj)
                    self._collect_meta[enc].append(meta)
        except Exception as e:
            logger.debug(f"Parse error: {e} | payload={payload}")

    async def send_command(self, cmd: str):
        if not self.is_connected:
            return
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._ser.write, (cmd + "\n").encode())

    async def get_instantaneous_encoder_values(self) -> tuple[Optional[float], Optional[float]]:
        if not self.is_connected:
            await self.connect()
        
        await self.send_command("READ")
        try:
            await asyncio.wait_for(self._data_event.wait(), timeout=2.0)
        except asyncio.TimeoutError:
            return None, None

        m0_raw = self._latest_raw["M0"]
        s0_raw = self._latest_raw["S0"]
        m0 = (m0_raw - self._zero_offsets["M0"]) if m0_raw is not None else None
        s0 = (s0_raw - self._zero_offsets["S0"]) if s0_raw is not None else None
        return m0, s0

    async def get_encoder_values(self, num_samples: int = SAMPLES_PER_MEASUREMENT) -> tuple[Optional[float], Optional[float]]:
        if not self.is_connected:
            await self.connect()

        self._measurement_id += 1
        self._collect_samples = {e: [] for e in ENCODERS}
        self._collect_meta = {e: [] for e in ENCODERS}
        self._collecting = True

        try:
            for _ in range(num_samples):
                await self.send_command("READ")
                try:
                    await asyncio.wait_for(self._data_event.wait(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                await asyncio.sleep(0.05) # Small delay between samples
        finally:
            self._collecting = False

        m0_samples = self._collect_samples["M0"]
        s0_samples = self._collect_samples["S0"]
        
        m0_med = float(np.median(m0_samples)) if m0_samples else None
        s0_med = float(np.median(s0_samples)) if s0_samples else None
        
        return m0_med, s0_med

    def set_zero(self) -> dict[str, float]:
        applied = {}
        for enc in ENCODERS:
            raw = self._latest_raw[enc]
            if raw is not None:
                self._zero_offsets[enc] = raw
                applied[enc] = raw
        if not applied:
            raise RuntimeError("No data yet.")
        return applied
