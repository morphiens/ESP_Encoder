# Oliver Unified Firmware

One firmware for every XIAO ESP32-C6 in the Oliver cluster. The same `.ino`
file is flashed to **every** board; role (master / slave / standalone) is
chosen at provisioning time over BLE from the gantry PC and persisted in NVS.
source .morphle_fast/bin/activate
DEVICE_NAME=<your_device> python -m src.master.devices.oliver.provision_oliver_cluster
## Modes

| Mode         | BLE                                                    | ESP-NOW                                  | Encoder              |
|--------------|--------------------------------------------------------|------------------------------------------|----------------------|
| `setup`      | Advertises `Oliver_SETUP_<MAC_LAST3>` (setup service)  | off                                      | idle                 |
| `master`     | Advertises configured `ble_name` (Oliver-1 service)    | coordinates discovery + slave reads      | own AS5047D = M0     |
| `slave`      | off                                                    | responds to discovery + read for own ID  | own AS5047D = S<id>  |
| `standalone` | Advertises configured `ble_name` (Oliver-1 service)    | off                                      | own AS5047D = M0     |

## Flash

Same command for every board (uses the gantry repo's helper):

```
python -m src.master.devices.oliver.flash_oliver_unified
```

Or with the Arduino CLI directly:

```
arduino-cli compile --upload \
  --fqbn esp32:esp32:XIAO_ESP32C6:PartitionScheme=huge_app \
  -p /dev/ttyACM0 \
  firmwares/ESP_Encoder/Oliver/oliver_unified
```

The `huge_app` partition (3 MB APP / 1 MB SPIFFS, no OTA) is required: BLE +
WiFi + ESP-NOW + NVS together exceed the default 1.31 MB app slot. NVS lives
in its own partition so the smaller SPIFFS doesn't affect us.

If you flash from the Arduino IDE, set:
- Tools -> Board -> XIAO_ESP32C6
- Tools -> Partition Scheme -> Huge APP (3MB No OTA/1MB SPIFFS)

After flashing, the board boots into **setup** mode. Nothing else needs to be
patched per board.

## Bring-up walkthrough (first cluster)

This is the end-to-end checklist for taking a stack of freshly flashed
boards to a working cluster on a new machine. The example uses two boards
(one master + one slave) wired as a `blade` pair.

### 0. Prereqs (one-time)

```bash
# arduino-cli (used by the flash helper)
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
export PATH="$PWD/bin:$PATH"
arduino-cli core update-index
arduino-cli core install esp32:esp32

# Python venv
source .morphle_fast/bin/activate
```

### 1. Flash every board

Plug the XIAO ESP32-C6 into USB, then:

```bash
python -m src.master.devices.oliver.flash_oliver_unified
```

Same command for every board. Repeat after swapping the USB cable to the
next board. After the flash, every board boots into **setup** mode and
advertises as `Oliver_SETUP_<MAC_LAST3>`.

### 2. Power all boards and run the provisioner

```bash
DEVICE_NAME=<your_device> python -m src.master.devices.oliver.provision_oliver_cluster
```

This drops you into a REPL. Inside:

```text
oliver> scan
  Scanning 6s for unprovisioned Oliver boards …
  Reading STA MAC over BLE for each board …
  Found 2 board(s):
    [0] Oliver_SETUP_3FA19C  ble=AA:BB:CC:3F:A1:9C  sta=DC:DA:0C:11:22:33  RSSI=-52
    [1] Oliver_SETUP_3FA1A0  ble=AA:BB:CC:3F:A1:A0  sta=DC:DA:0C:44:55:66  RSSI=-48
```

The `sta` column is the WiFi STA MAC the firmware will use for ESP-NOW.
The provisioner reads it directly from each board's setup info
characteristic, so you do not need a serial monitor for this.

### 3. Identify which physical board is which

```text
oliver> identify 0          # blinks board [0] for 5 s
oliver> identify 1 8000     # blinks board [1] for 8 s
```

Look at the bench: only one board's LED flashes rapidly. That's the entry
behind that index.

### 4. Provision the master

```text
oliver> master 0                          # ble_name defaults to oliver_gateway
oliver> master 0 my_oliver_gateway        # or pass a custom name
```

Output:

```text
  reading STA MAC from board …
  -> master  ble_name='oliver_gateway' wifi_ch=11 sta_mac=DC:DA:0C:11:22:33
  master provisioned. The board is rebooting; rerun `scan` to confirm it
  dropped off the setup list (it now advertises as 'oliver_gateway' on
  the runtime service).
```

The session now has `master_mac` set automatically from the board's STA
MAC. Slaves will use it.

Optional confirmation:

```text
oliver> scan
```

The master should be **gone** from the list (it switched from setup mode
to runtime mode and is no longer advertising the setup service).

### 5. Provision each slave

For a `blade` pair where the master holds the left lever and slave 0 the
right lever:

```text
oliver> identify 1            # confirm physical slave board
oliver> slave 1 0 blade right
```

Output:

```text
  -> slave   id=0 wifi_ch=11 master_mac=DC:DA:0C:11:22:33
```

For more pairs (later, with more boards):

```text
oliver> slave <n> 1 slide left
oliver> slave <n> 2 slide right
oliver> slave <n> 3 chassis_drift left
oliver> slave <n> 4 chassis_drift right
```

### 6. Save the config

```text
oliver> save
  wrote /etc/.../robotome-configs/<DEVICE_NAME>/MASTER/oliver_config.jsonc
```

This is the file the runtime `OliverHub` reads when the gantry process
starts.

### 7. Verify the cluster

Still inside the REPL:

```text
oliver> read                  # READ all encoders
oliver> read blade            # READ_MASK for the blade pair only
```

You should see something like:

```text
  connecting to 'oliver_gateway' …
  READ_MASK 0b00000000000000000000000000000011
  gateway_idx = 7
    M0    166.99005°  status=OK   agc=38 mag=1823
    S0    125.39495°  status=OK   agc=42 mag=2015
```

If a slave shows `OFFLINE`, send `rediscover` from the REPL or run:

```text
oliver> rediscover
oliver> identify_slave 0      # blinks slave 0's LED via the master
oliver> read blade
```

### 8. Done

Quit the REPL (`quit`). The gantry process will pick up the cluster
through `OliverHub` once a consumer is wired to it.

## Provisioning command reference

| Command                               | Behavior                                                                |
|---------------------------------------|-------------------------------------------------------------------------|
| `scan [seconds]`                      | Rescan for unprovisioned boards (default 6 s)                           |
| `identify <n> [ms]`                   | Blink the LED on board #n (default 5000 ms)                             |
| `master <n> [ble_name]`               | Provision board #n as the cluster master                                |
| `slave <n> <slave_id> [pair side]`    | Provision board #n as a slave; populates `pairs` if pair+side given     |
| `standalone <n> [ble_name]`           | Provision board #n as a standalone gateway                              |
| `set master_mac AA:BB:...`            | Override master STA MAC (rarely needed; auto-captured)                  |
| `set wifi_ch 11`                      | Override ESP-NOW channel for new assignments                            |
| `set master_ble_name <name>`          | Override default master BLE name for `master` command                   |
| `read`                                | Connect to master, READ every encoder, print decoded                    |
| `read <pair>`                         | Connect to master, READ_MASK for one pair, print decoded                |
| `rediscover`                          | Tell the master to re-sweep ESP-NOW for slaves                          |
| `identify_slave <id> [ms]`            | Forward an identify blink to slave #id via the master                   |
| `show`                                | Print the in-memory config dict                                         |
| `save`                                | Write `oliver_config.jsonc` to CONFIG_DIR/<DEVICE>/MASTER/              |
| `help`                                | Print this list                                                         |
| `quit`                                | Exit                                                                    |

## Runtime BLE protocol (master + standalone)

GATT (Oliver-1 defaults; same UUIDs the gantry's existing client expects):

| UUID                                       | Purpose         |
|--------------------------------------------|-----------------|
| `4fafc201-1fb5-459e-8fcc-c5c9c331914b`     | service         |
| `beb5483e-36e1-4688-b7f5-ea07361b26a8`     | notify (data)   |
| `8d53dc1d-1db7-4cd3-868b-8a527460aa84`     | command (write) |

Commands:

| Command                       | Behavior                                                                     |
|-------------------------------|------------------------------------------------------------------------------|
| `READ`                        | Read all known encoders, notify with full payload                            |
| `READ_MASK:<u32>`             | Read only encoders whose bit is set (bit 0 = M0, bit 1 = S0, bit 2 = S1, ...) |
| `REDISCOVER`                  | 15 s ESP-NOW discovery sweep                                                 |
| `IDENTIFY:<ms>`               | Blink the master's own LED                                                   |
| `IDENTIFY_SLAVE:<id>:<ms>`    | Forward an identify blink to a slave through ESP-NOW                         |
| `FACTORY_RESET`               | Wipe NVS on the master itself, reboot into setup mode                        |
| `FACTORY_RESET_SLAVE:<id>`    | Forward factory reset to a slave through ESP-NOW                             |

Notify payload (unchanged from the existing protocol so existing parsers work):

```
<gateway_idx>|M0:angle*10000,pkt,agc,mag,magl,magh,cof|S0:...|S1:...
```

Encoders not in the mask are simply omitted from the payload.

## Setup-mode BLE protocol

Service UUID: `5e740001-7b9c-4c2e-9a0f-3c5d6e7f8a90`

| UUID                                       | Property | Purpose                                                |
|--------------------------------------------|----------|--------------------------------------------------------|
| `5e740002-7b9c-4c2e-9a0f-3c5d6e7f8a90`     | read     | JSON: `{"mac":"...", "role":"unset", "fw":"..."}`      |
| `5e740003-7b9c-4c2e-9a0f-3c5d6e7f8a90`     | write    | `BLINK:<ms>` to identify the physical board            |
| `5e740004-7b9c-4c2e-9a0f-3c5d6e7f8a90`     | write    | Config string: `role=...;...` (provisions and reboots) |

## ESP-NOW protocol (between master and slaves)

Tagged frames; first byte selects the type:

| Tag    | Direction       | Payload                                       |
|--------|-----------------|-----------------------------------------------|
| `0x01` | master -> bcast | discovery ping, len=1                         |
| `0x02` | slave  -> master| `[0x02, slave_id]`                            |
| `0x03` | master -> slave | `[0x03, slave_id]` read request               |
| `0x04` | slave  -> master| `[0x04, OliverPayload]` read reply            |
| `0x05` | master -> slave | `[0x05, slave_id, ms_lo, ms_hi]` identify     |
| `0x06` | master -> slave | `[0x06, slave_id]` factory reset              |

## Recovery / re-provisioning

If a board is wedged or you need to repurpose it:

- **From the provisioner REPL** (after the master is up):
  - `factory_reset_slave <id>` resets a specific slave through ESP-NOW.
    *(Not yet a REPL command — use the gateway directly:
    `await OliverGateway('oliver_gateway').factory_reset_slave(<id>)`.)*
- **From the master's BLE command characteristic**:
  - `FACTORY_RESET` resets the master itself.
  - `FACTORY_RESET_SLAVE:<id>` forwards a reset to a slave.
- **Physical fallback**: hold the BOOT button for ~3 s at power-up. The LED
  blinks while the button is held; when 3 s elapse, NVS is wiped and the
  board boots back into setup mode.

After a reset the board reappears as `Oliver_SETUP_<MAC_LAST3>` and can be
re-provisioned with the steps above.

## Where the config goes

`provision_oliver_cluster.py save` writes:

```
$CONFIG_DIR/$DEVICE_NAME/MASTER/oliver_config.jsonc
```

Schema (every field optional unless noted):

```jsonc
{
  "gateway": {
    "device_id":       "oliver_gateway",
    "ble_name":        "oliver_gateway",
    "esp_now_channel": 11,
    "enabled":         true
  },
  "pairs": {
    "blade":         { "left": "M0", "right": "S0", "samples": 5, "enabled": true },
    "slide":         { "left": "S1", "right": "S2", "samples": 5, "enabled": true },
    "chassis_drift": { "left": "S3", "right": "S4", "samples": 5, "enabled": false }
  },
  "encoders": [
    { "field": "M0", "device_id": "oliver_gateway", "role": "master", "mac": "..." },
    { "field": "S0", "device_id": "blade_right",    "role": "slave",  "slave_id": 0 }
  ]
}
```

Edit pair definitions by hand at any time — no reflash needed.

## Pair → READ_MASK reference

| Pair            | Encoders   | READ_MASK     |
|-----------------|------------|---------------|
| `blade`         | M0, S0     | `0b00000011`  |
| `slide`         | S1, S2     | `0b00001100`  |
| `chassis_drift` | S3, S4     | `0b00110000`  |
| (full READ)     | all        | `0xFFFFFFFF`  |

Bit 0 = M0 (master), bit *i* = S(*i*-1) for i = 1..31.
