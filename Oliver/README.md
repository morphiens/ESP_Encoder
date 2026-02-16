# Oliver

Wireless ultra-precision encoder measurement system using ESP32 devices with ESP-NOW and BLE communication.

## Architecture

```
┌─────────────┐     ESP-NOW      ┌─────────────┐
│   Slave 0   │◄────────────────►│             │
│  (ESP32C3)  │                  │   Master    │     BLE      ┌──────────┐
└─────────────┘                  │  (ESP32C6)  │◄────────────►│   PC     │
                                 │   "Oliver"  │              │ (Python) │
┌─────────────┐     ESP-NOW      │             │              └──────────┘
│   Slave N   │◄────────────────►│             │
│  (ESP32C3)  │                  └─────────────┘
└─────────────┘
```

- **Slaves**: Read AS5047D magnetic encoders via SPI, respond to master requests over ESP-NOW
- **Master**: Coordinates slaves, has its own encoder, broadcasts data to PC via BLE
- **PC**: Python scripts connect via BLE to receive encoder readings

## Hardware

- **Master**: XIAO ESP32C6 with AS5047D encoder
- **Slaves**: XIAO ESP32C3 with AS5047D encoder each
- **Sensor**: AS5047D 14-bit magnetic rotary encoder (SPI interface)

### Pin Configuration

| Function | Pin  |
|----------|------|
| CS       | D7   |
| SCK      | D1   |
| MISO     | D0   |
| MOSI     | D10  |

## BLE Configuration

### Oliver 1

| Parameter          | Value                                      |
|--------------------|--------------------------------------------|
| Device Name        | `Oliver 1`                                  |
| Service UUID       | `4fafc201-1fb5-459e-8fcc-c5c9c331914b`     |
| Characteristic UUID| `beb5483e-36e1-4688-b7f5-ea07361b26a8`     |
| Command UUID       | `8d53dc1d-1db7-4cd3-868b-8a527460aa84`     |

### Oliver 2

| Parameter          | Value                                      |
|--------------------|--------------------------------------------|
| Device Name        | `Oliver 2`                                  |
| Service UUID       | `a1b2c3d4-e5f6-7890-abcd-ef1234567890`     |
| Characteristic UUID| `b2c3d4e5-f6a7-8901-bcde-f12345678901`     |
| Command UUID       | `c3d4e5f6-a7b8-9012-cdef-123456789012`     |

## Files

```
Oliver/
├── master/
│   └── master.ino          # ESP32C6 master firmware
├── slave/
│   └── slave.ino           # ESP32C3 slave firmware
├── blade_offset_probe.py   # Python API for BLE communication
├── BLADE_OFFSET_PROBE.py   # Standalone measurement script
└── README.md
```

## Setup

### 1. Flash Slave Devices

1. Open `slave/slave.ino` in Arduino IDE
2. Set `SLAVE_ID` (0, 1, 2, etc.) for each board:
   ```cpp
   #define SLAVE_ID 0  // Change for each slave
   ```
3. Set `WIFI_CHANNEL` to match the master this slave belongs to:
   ```cpp
   #define WIFI_CHANNEL 1  // Must match master's WIFI_CHANNEL
   ```
4. Select board: XIAO ESP32C3
5. Upload to each slave device

### 2. Flash Master Device

1. Open `master/master.ino` in Arduino IDE
2. Set `NUM_SLAVES` to match your setup:
   ```cpp
   #define NUM_SLAVES 1  // Number of slave devices
   ```
3. Set `WIFI_CHANNEL` (must match the slaves for this set):
   ```cpp
   #define WIFI_CHANNEL 1  // ESP-NOW channel for this Oliver set
   ```
4. Select board: XIAO ESP32C6
5. Upload

### WiFi Channel Assignment

When running multiple Oliver sets simultaneously, each set (master + its slaves) must use a different WiFi channel to avoid cross-talk:

| Set      | WIFI_CHANNEL |
|----------|--------------|
| Oliver 1 | 1            |
| Oliver 2 | 6            |
| Oliver 3 | 11           |

### 3. Python Environment

```bash
# From the repo root
./setup.sh

# Or manually
source .venv/bin/activate
pip install numpy bleak
```

## Usage

### Run the API Demo

```bash
source .venv/bin/activate
cd Oliver
python blade_offset_probe.py
```

**Interactive Commands:**
- `Enter` - Take a measurement
- `z` - Set current position as zero reference
- `v` - Get instantaneous values
- `c` - Connect
- `d` - Disconnect
- `r` - Reconnect
- `q` - Quit

### Run the Measurement Script

```bash
source .venv/bin/activate
cd Oliver
python BLADE_OFFSET_PROBE.py
```

**Interactive Commands:**
- `Enter` - Start measurement (collects 5 samples)
- `z` - Set zero reference
- `q` - Quit

### Using the API in Your Code

```python
import asyncio
from blade_offset_probe import OliverAPI

async def main():
    probe = OliverAPI()
    await probe.connect()

    # Get instantaneous encoder values (M0, S0) in degrees
    m0, s0 = await probe.get_instantaneous_encoder_values()
    print(f"M0: {m0:.5f}°, S0: {s0:.5f}°")

    # Take a precision measurement (median of multiple samples)
    m0_med, s0_med = await probe.get_encoder_values()

    # Set current position as zero
    probe.set_zero()

    # Subsequent readings are relative to zero
    m0, s0 = await probe.get_instantaneous_encoder_values()

    await probe.disconnect()

asyncio.run(main())
```

## Protocol Details

### ESP-NOW Communication

- **Channel**: Configurable via `WIFI_CHANNEL` (default 1; use 1, 6, or 11 to isolate sets)
- **Discovery**: Master broadcasts `0xFF`, slaves respond with their ID
- **Data Request**: Master sends slave ID, slave responds with angle data

### BLE Data Format

```
gateway_idx|M0:angle,pkt|S0:angle,pkt
```

Example: `123|M0:1234567,45|S0:987654,44`

- `gateway_idx`: Packet counter from master
- `angle`: Encoder angle × 10000 (integer)
- `pkt`: Per-encoder packet index

## Troubleshooting

### Device Not Found
- Ensure master is powered and flashed with correct firmware
- Check BLE is enabled on your PC
- Try `bluetoothctl scan on` to verify "Oliver" is advertising

### Connection Drops Immediately
- Power cycle the ESP32 after flashing new firmware
- Verify UUIDs match between master.ino and Python files

### No Slave Data
- Check slave is powered and SLAVE_ID matches expected value
- Verify both devices are on WiFi channel 1
- Check serial monitor for discovery messages

## License

MIT
