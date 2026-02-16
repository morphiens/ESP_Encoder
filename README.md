# ESP Encoder

A collection of ESP32-based encoder measurement projects.

Each project lives in its own directory with dedicated firmware, scripts, and documentation.

## Projects

| Project | Description | Details |
|---------|-------------|---------|
| [Oliver](Oliver/) | Wireless ultra-precision encoder system using ESP-NOW + BLE (ESP32C6 master, ESP32C3 slaves, AS5047D sensors) | [README](Oliver/README.md) |

## Repository Structure

```
ESP_Encoder/
├── Oliver/                 # Wireless precision encoder system
│   ├── master/             # ESP32C6 master firmware
│   ├── slave/              # ESP32C3 slave firmware
│   ├── *.py                # Python BLE scripts
│   └── README.md
├── requirements.txt        # Shared Python dependencies
├── setup.sh                # Environment setup (all projects)
└── README.md               # This file
```

## How It Works

All projects in this repository use the AS5047D 14-bit magnetic rotary encoder. Each encoder reading uses an **ultra-precision algorithm** that takes **4,096 raw SPI samples**, applies outlier rejection within blocks of 256 samples, then takes the median of all 16 block means. This produces sub-LSB angular precision.

Communication follows an **on-demand** model: the PC sends a request over BLE, the master reads its own encoder and polls slaves via ESP-NOW, and a single response with angle and full diagnostics (AGC, magnetic field strength, error flags) is sent back. No data is transmitted until the PC asks.

See each project's README for detailed protocol documentation.

## Getting Started

### Python Environment

```bash
./setup.sh
```

This creates a `.venv` virtual environment and installs shared dependencies from `requirements.txt`.

To activate manually:

```bash
source .venv/bin/activate
```

### Per-Project Setup

Each project may require flashing firmware to ESP32 boards. See the project's own README for specific instructions.

## License

MIT
