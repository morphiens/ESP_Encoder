# ESP Encoder

A collection of ESP32-based encoder measurement projects using the AS5047D 14-bit magnetic rotary encoder.

Each project lives in `projects/` with its own Python scripts. All projects share firmware templates from `firmware/`.

---

## Firmware Templates

| Template | Location | Use When |
|---|---|---|
| **Single Encoder** | [`firmware/single_encoder/`](firmware/single_encoder/) | One ESP reads one encoder, sends directly to PC via BLE |
| **Master** | [`firmware/master/`](firmware/master/) | ESP32C6 — aggregates encoder data from slaves, sends to PC via BLE |
| **Slave** | [`firmware/slave/`](firmware/slave/) | ESP32C3 — reads one encoder, responds to master over ESP-NOW |

### Flashing a Single-Encoder Project

1. Open `firmware/single_encoder/single_encoder.ino` in Arduino IDE
2. Edit the config block at the top:
   ```cpp
   #define ESP_NAME            "YOUR_DEVICE_NAME"
   #define SERVICE_UUID        "4fafc201-..."
   #define CHARACTERISTIC_UUID "beb5483e-..."
   ```
3. Select board (XIAO ESP32C3 or ESP32C6) and upload

### Flashing a Multi-Encoder Project (Master + Slave)

See [`Oliver/README.md`](Oliver/README.md) for full setup instructions.

---

## Projects

| Project | Firmware | Description |
|---|---|---|
| [block_height](projects/block_height/) | Single Encoder | Block height from encoder rotation via trigonometry or polynomial fit |
| [Oliver](Oliver/) | Master + Slave | Wireless multi-encoder system (ESP32C6 master + ESP32C3 slaves) |

---

## Repository Structure

```
Esp_Encoder/
├── firmware/
│   ├── single_encoder/
│   │   └── single_encoder.ino    # Single ESP → BLE to PC
│   ├── master/
│   │   └── master.ino            # ESP32C6 master — aggregates slaves, sends BLE
│   └── slave/
│       └── slave.ino             # ESP32C3 slave — reads encoder, replies via ESP-NOW
│
├── projects/
│   └── block_height/             # Block height measurement Python scripts
│
├── Oliver/                       # Multi-encoder project (firmware + Python)
│   ├── master/master.ino         # (copy of firmware/master)
│   ├── slave/slave.ino           # (copy of firmware/slave)
│   ├── oliver.py                 # Python API for BLE communication
│   ├── oliver_raw.py             # Raw BLE data script
│   └── README.md
│
├── Datasheets/
│   └── AS5047D.pdf
├── requirements.txt
├── setup.sh
└── README.md
```

---

## How It Works

### Ultra-Precision Sampling Algorithm

All projects use the same AS5047D reading algorithm:

1. **4,096 raw SPI reads**, split into 16 blocks of 256 samples
2. **Robust mean per block** — discard samples > 1.5 LSB from mean, re-average the rest
3. **Median of 16 block means** — rejects any block corrupted by radio bursts
4. **Convert to degrees**: `angle = (counts × 360) / 16384`

### Single-Encoder BLE Data Format

The single encoder uses the **same 7-field format as Oliver master** for protocol compatibility:

```
<packetIdx>|M0:<angle>,<pkt>,<agc>,<mag>,<magl>,<magh>,<cof>
```

Example: `5|M0:1669900,5,38,1823,0,0,0`

| Field | Description |
|---|---|
| `packetIdx` | Global packet counter (increments per READ) |
| `angle` | Encoder angle × 10000 as integer (e.g. `1669900` = 166.9900°) |
| `pkt` | Same as packetIdx (per-encoder index) |
| `agc` | Automatic gain control (0–255) |
| `mag` | CORDIC magnitude (field strength) |
| `magl / magh / cof` | Diagnostic flags (0 or 1) |

**BLE Commands:**

| Command | Description |
|---|---|
| `READ` | Request one on-demand encoder reading |
| `ZERO` | Set current position as zero reference |

### Multi-Encoder (Oliver) BLE Data Format

```
<gateway_idx>|M0:<angle>,<pkt>,<agc>,<mag>,<magl>,<magh>,<cof>|S0:...|S1:...
```

See [`Oliver/README.md`](Oliver/README.md) for full protocol details.

---

## Getting Started

```bash
./setup.sh
source .venv/bin/activate
```

## License

MIT

---

**Author:** Swaraj Dangare
