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
