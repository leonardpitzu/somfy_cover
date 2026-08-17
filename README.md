# ESPSomfy

This repository provides an **external ESPHome component** to control Somfy motorised covers (and future device types). It uses a **hub architecture** where radio hardware is configured once in a `somfy:` hub block, and individual devices reference it via `somfy_id`. This makes it easy to add multiple covers sharing a single radio, and to extend with new platforms (switches, lights, etc.) in the future.

Two protocols are supported:

| Type | Protocol | Frequency | Modulation | Use case |
|------|----------|-----------|------------|----------|
| `rts` | Somfy RTS | 433.42 MHz | OOK (Manchester) | Standard Somfy remotes/motors |
| `iohc` 1W | io-homecontrol 1W | 868.95 MHz | 2-FSK, 38.4 kbaud | Newer Somfy motors (one-way control) |
| `iohc` 2W | io-homecontrol 2W | 868.25/868.95/869.85 MHz | 2-FSK, 38.4 kbaud | Bidirectional with status feedback |

## Architecture

```
somfy:              ← Hub (owns radio hardware)
  type: rts/iohc

cover:              ← Device (references hub)
  platform: somfy
  somfy_id: <hub>
```

The hub handles all radio TX/RX and protocol framing. Device classes (covers today, more in the future) handle device-specific logic (rolling codes, position tracking, pairing) and delegate radio operations to their hub.

## Required hardware

- **ESP32** (any variant)
- **For RTS:** CC1101 433 MHz module + antenna (via ESPHome `remote_transmitter` / `remote_receiver`)
- **For iohc:** CC1101 868 MHz module + antenna (via ESPHome native `cc1101` component in packet mode)

## Installation

```yaml
external_components:
  source: github://leonardpitzu/esphome_somfy@main
  components: [somfy]
  refresh: 600s
```

---

<details>
<summary><h2>RTS configuration</h2></summary>

### Minimal (TX only)

```yaml
remote_transmitter:
  id: transmitter
  pin: GPIO04
  carrier_duty_percent: 100%

somfy:
  - id: rts_radio
    type: rts
    remote_transmitter: transmitter

button:
  - platform: template
    id: program_livingroom_door
    name: "Prog Living Room Door"
    entity_category: config

cover:
  - platform: somfy
    type: rts
    id: livingroom_door
    name: "Living Room Door"
    device_class: shutter
    open_duration: 40s
    close_duration: 40s
    storage_key: KeyLivingDoor
    remote_code: 0x088331
    prog_button: program_livingroom_door
    somfy_id: rts_radio
```

### With receiver (RX — sync with physical remotes)

```yaml
remote_receiver:
  id: receiver
  pin: GPIO03

somfy:
  - id: rts_radio
    type: rts
    remote_transmitter: transmitter
    remote_receiver: receiver

text_sensor:
  - platform: template
    id: somfy_rx_last
    name: "Detected Somfy Remote"

cover:
  - platform: somfy
    type: rts
    id: livingroom_door
    name: "Living Room Door"
    device_class: shutter
    open_duration: 40s
    close_duration: 40s
    storage_key: KeyLivingDoor
    remote_code: 0x088331
    prog_button: program_livingroom_door
    somfy_id: rts_radio
    detected_remote: somfy_rx_last
    allowed_remotes:
      - 0x92FB39
```

### Pairing (PROG)

The ESPHome device acts like a new Somfy RTS remote identified by `remote_code`. Before it can control a motor, that motor must be paired:

1. Put the motor into *programming mode* using an **already paired** physical remote (hold PROG ~2 s until the motor jogs).
2. Within the programming window, press the **Prog …** button in Home Assistant.
3. The motor jogs again to confirm pairing.

### Detecting remote IDs

1. Add a `text_sensor` (as above) and set `detected_remote` on the cover.
2. Press a button on the physical remote.
3. Read the decoded `0x......` ID from the text sensor.
4. Add it to `allowed_remotes` and recompile.

</details>

---

<details>
<summary><h2>iohc configuration (1W — one-way)</h2></summary>

Requires ESPHome 2026.4+ with the native `cc1101` component in packet mode.

```yaml
spi:
  clk_pin: GPIO07
  mosi_pin: GPIO09
  miso_pin: GPIO08

cc1101:
  id: cc1101_radio
  cs_pin: GPIO04
  gdo0_pin: GPIO02
  frequency: 868.95MHz
  modulation_type: 2-FSK
  symbol_rate: 38400
  fsk_deviation: 19.2kHz
  filter_bandwidth: 100kHz
  output_power: 10
  manchester: false
  packet_mode: true
  packet_length: 0
  crc_enable: false
  sync_mode: "16/16"
  sync1: 0x57
  sync0: 0xFD
  # Register value 7 selects the CC1101 maximum 24-byte preamble.
  num_preamble: 7

button:
  - platform: template
    id: program_bedroom_blind
    name: "Prog Bedroom Blind"
    entity_category: config
    disabled_by_default: true
  - platform: template
    id: my_bedroom_blind
    name: "Bedroom Blind MY"

text_sensor:
  - platform: template
    id: bedroom_blind_detected_remote
    name: "Bedroom Blind Detected Remote"

somfy:
  - id: iohc_radio
    type: iohc
    cc1101_id: cc1101_radio

cover:
  - platform: somfy
    type: iohc
    id: bedroom_blind
    name: "Bedroom Blind"
    device_class: shutter
    open_duration: 30s
    close_duration: 30s
    # Estimated position of the motor's stored MY/favourite position.
    my_position: 42%
    my_button: my_bedroom_blind
    storage_namespace: somfy
    storage_key: KeyBedBlind
    # Used only if KeyBedBlind is absent from NVS. Preserve/increase this when
    # restoring an already-paired controller identity to replacement hardware.
    initial_rolling_code: 1
    repeat_command_count: 6
    remote_code: 0xAABBCC
    prog_button: program_bedroom_blind
    somfy_id: iohc_radio
    encryption_key: !secret bedroom_blind_iohc_key
    detected_remote: bedroom_blind_detected_remote
    allowed_remotes:
      - 0x112233
```

### Notes

- 1W commands (open/close/stop/MY) are sent on **868.95 MHz**. Ordinary commands use the virtual controller's private AES key; the public transfer key is used only to install that key during pairing.
- The component handles CRC-16 (Kermit) and AES-128 HMAC in software.
- `MY` sends the hardware-verified STOP/MY command. While idle, the motor recalls its stored favourite; while moving, it stops. `my_position` tells Home Assistant where to animate the estimate because 1W has no position feedback.
- `allowed_remotes` lets commands from already-paired physical remotes update the time-based Home Assistant estimate. An empty list accepts all decoded remote IDs; an explicit list is safer after discovery.
- For bidirectional (2W) support, see the next section.

</details>

---

<details>
<summary><h2>iohc configuration (2W — bidirectional)</h2></summary>

2W mode enables authenticated bidirectional communication with io-homecontrol actuators. The controller sends a command, the actuator replies with a cryptographic challenge, and the controller responds to prove it holds the system key. This provides command acknowledgement and status feedback.

### Requirements

- ESPHome 2026.4+ with native `cc1101` component in packet mode
- The **system key** (per-installation AES-128 key) — obtained during initial pairing or from a paired controller backup
- The actuator's **node address** (3-byte, e.g. `0xABCDEF`)

### Protocol overview

1. Controller sends CMD_EXECUTE (0x00) on 868.95 MHz
2. Actuator replies with challenge (CMD 0x3C, 6 random bytes) on one of 3 RX channels (868.25 / 868.95 / 869.85 MHz)
3. Controller computes AES-128-ECB response from IV (frame data + rolling checksum + challenge) and sends CMD 0x3D
4. Actuator validates and executes; sends status (CMD 0xFE) back

### Example configuration

```yaml
somfy:
  - id: iohc_radio
    type: iohc
    cc1101_id: cc1101_radio

cover:
  - platform: somfy
    type: iohc
    id: living_room_blind
    name: "Living Room Blind"
    device_class: shutter
    open_duration: 25s
    close_duration: 25s
    storage_key: KeyLivBlind
    remote_code: 0xAABBCC
    prog_button: program_living_room
    somfy_id: iohc_radio
    mode: 2w
    target_node: 0xDEADBE
    encryption_key: "YOUR_SYSTEM_KEY_HEX_32_CHARS_HERE"
```

### Additional cover options (2W)

| Option | Required | Description |
|--------|----------|-------------|
| `mode` | no | `1w` (default) or `2w` |
| `target_node` | 2W only | 3-byte hex address of the target actuator |
| `encryption_key` | 2W only | System key (32-char hex AES-128 key) |

### Notes

- TX is always on **868.95 MHz**; RX hops across 3 channels (868.25 / 868.95 / 869.85 MHz) with 2.7 ms dwell time.
- The session has a 3-second timeout with up to 2 retries.
- Unlike 1W, 2W frames do not use a rolling code — authentication is challenge/response based.
- The system key is specific to your installation. It is **not** the public transfer key used by 1W.

</details>

---

## Common options

### Hub (`somfy:`)

| Option | Required | Description |
|--------|----------|-------------|
| `type` | yes | `rts` or `iohc` |
| `remote_transmitter` | RTS only | ESPHome `remote_transmitter` ID |
| `remote_receiver` | no | ESPHome `remote_receiver` ID (enables RX decode) |
| `cc1101_id` | iohc only | ESPHome `cc1101` component ID |

### Cover (`platform: somfy`)

| Option | Required | Description |
|--------|----------|-------------|
| `type` | yes | `rts` or `iohc` |
| `somfy_id` | yes | Reference to the `somfy:` hub |
| `open_duration` | yes | Time for a full open travel |
| `close_duration` | yes | Time for a full close travel |
| `my_position` | no, iohc | Estimated 0–100% position of the motor's stored MY/favourite position |
| `my_button` | no, iohc | Template button that sends native STOP/MY; requires `my_position` |
| `storage_namespace` | no | NVS namespace for rolling-code persistence; defaults to `somfy` (max 15 chars) |
| `storage_key` | yes | NVS key for rolling code persistence (max 15 chars) |
| `initial_rolling_code` | no | Initial code used only when the NVS key is missing; defaults to `1` |
| `remote_code` | yes | Hex address of this virtual remote |
| `prog_button` | yes | Button entity to trigger PROG pairing |
| `repeat_command_count` | no | Ordinary RF command repeat count; defaults to `4` (the tested distant IOHC link uses `6`) |
| `detected_remote` | no | Text sensor for decoded physical-remote IDs |
| `allowed_remotes` | no | Physical remote IDs allowed to update the time-based position estimate |
| `encryption_key` | no | Custom AES key hex string (iohc 1W: defaults to transfer key; iohc 2W: system key, required) |
| `mode` | no | iohc only: `1w` (default) or `2w` |
| `target_node` | 2W only | 3-byte hex address of target actuator |

## Adding shutters safely

Each independently controlled shutter needs its own cover block and calibration. Record all of the following next to that block:

- a stable `id` and entity `name`;
- measured `open_duration` and `close_duration`;
- measured `my_position` and a dedicated `my_button` when the motor has a MY favourite;
- a unique virtual-controller `remote_code` and private `encryption_key`;
- a unique `storage_namespace`/`storage_key` pair and the next known unused `initial_rolling_code`;
- the physical remote ID(s) in `allowed_remotes` for state synchronisation.

Never reuse a `remote_code` or NVS storage key between independent cover entities. One controller identity may intentionally be paired with several shutters, but its broadcast commands will move all of them together; model that as one group cover rather than duplicate entities.

To calibrate a shutter, first drive it fully closed. Time one uninterrupted trip to fully open, then time one uninterrupted trip back to fully closed, and add a small allowance (typically a few tenths of a second) so the estimate reaches the end stop. From a known end stop, recall MY and calculate its percentage from elapsed travel divided by the corresponding full-travel time. Verify 25%, 50%, 75%, MY, physical-remote sync, reboot persistence, and both end stops before relying on automations.

The position is dead reckoning, not motor feedback, in 1W mode. End-stop runs correct accumulated error. A factory erase, renamed NVS key, changed controller ID/key, or accidental PROG operation can lose or consume a scarce pairing slot; back up those values before flashing or replacing hardware.

## Credits

This project builds on prior work:
- https://github.com/HarmEllis/esphome-somfy-cover-remote (basic rts implementation)
- https://github.com/fawick/somfy_cover_2025.12 (same basic rts implementation but using esphome's standard cc1101 component)
- https://github.com/rstrouse/ESPSomfy-RTS (rts rx decoder reference)
- https://github.com/Velocet/iown-homecontrol (io-homecontrol protocol documentation)
- https://github.com/rspaargaren/iohomecontrol (io-homecontrol implementation reference)

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
