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
- STOP and MY are intentionally distinct transmissions even though both use the authenticated `CMD_EXECUTE` main parameter `0xD200`. STOP sends that frame alone, which stops movement but does not recall MY when idle. A dedicated MY action sends that safe stop-only frame first, waits 500 ms for the motor to settle, and then reproduces the real remote's complete short-press sequence: `0xD200`, followed by `cmd=0x20` payloads `02 FF 01 43 02 0C 00 00` and `02 FF 01 43 02 05 FF 00`. Each logical frame has its own rolling code and MAC. MY therefore means “go to the stored favourite” whether the motor was moving or idle, without relying on Home Assistant's estimated state.
- `my_position` only tells Home Assistant where to animate the 1W position estimate when MY recall begins; it is not used to choose the RF command.
- `allowed_remotes` lets commands from already-paired physical remotes update the time-based Home Assistant estimate. An empty list accepts all decoded remote IDs; an explicit list is safer after discovery. `detected_remote` includes the received rolling sequence and an event number, so repeated presses of the same button remain separate Home Assistant history entries while RF copies from one burst are collapsed.
- On Venetian remotes, a wheel gesture can intentionally stop lift movement
  with a `D200` frame before sending its direction-bearing tilt event. The
  receiver correlates that pair into one tilt action for Home Assistant; a
  genuine standalone STOP/MY remains visible and functional. Larger wheel
  rolls carry a signed magnitude around `0xCCE8`; received rolls are converted
  to an estimated effective step count. Home Assistant tilt transmission uses
  hardware-verified two-step compound gestures plus an exact one-step remainder
  instead of unsafe arbitrary-size synthetic rolls.
- For bidirectional (2W) support, see the next section.

### Receive-only raw capture (experimental)

`somfy_iohc_capture` is a diagnostic component for investigating commands from
real io-homecontrol remotes before implementing new device types. It has no
controller key, rolling-code storage, transmit method, pairing action or API
service. Every CRC-valid RF copy is published separately; do not press PROG
during a capture.

Include `somfy_iohc_capture` in the `external_components` list and add:

```yaml
somfy_iohc_capture:
  id: iohc_raw_capture
  somfy_id: iohc_radio
  # Optional after the first capture. Omit to hear every valid iohc frame.
  # remote_code: 0x112233
  capture:
    name: "IOHC Raw Frame Capture"
```

The diagnostic state and the `somfy.iohc.capture` INFO log contain a unique
event number, source/destination, command ID, both frame-control bytes, RSSI,
LQI, complete command data, and the complete logical frame. Raw frames contain
controller identifiers and authenticated values, so treat captures as private
diagnostic material.

</details>

---

<details>
<summary><h2>iohc GUI commissioning (multiple independent shutters)</h2></summary>

`somfy_iohc_manager` turns one ESP32 + CC1101 into a reusable bridge with up
to 20 independently controlled 1W shutter identities by default. Shutters are
added from Home Assistant's UI; adding the second or later shutter does not
require a YAML edit or another firmware build. Install one bridge per radio
coverage zone and add each bridge separately to the HA integration.

The manager deliberately creates a unique controller address, AES key, rolling
code stream, physical-remote allow-list, opening time, closing time, and MY
position for every shutter. It also maintains an AES-GCM encrypted controller
backup after every transmitted rolling code.

### ESPHome

Enable custom API actions, add the manager component, create its two diagnostic
text sensors, and configure a separate 16-byte backup-encryption key:

```yaml
api:
  custom_services: true

external_components:
  source: github://leonardpitzu/esphome_somfy@main
  components: [somfy, somfy_iohc_manager]

text_sensor:
  - platform: template
    id: commissioning_status
    name: "Commissioning Status"
    entity_category: diagnostic
  - platform: template
    id: encrypted_controller_backup
    name: "Encrypted Controller Backup"
    entity_category: diagnostic

somfy_iohc_manager:
  id: shutter_manager
  somfy_id: iohc_radio
  status_sensor: commissioning_status
  backup_sensor: encrypted_controller_backup
  backup_key: !secret somfy_io_backup_key  # exactly 32 hex characters
  max_shutters: 20
```

The `somfy:` hub, SPI bus, and packet-mode CC1101 block are the same as in the
1W example above. See `tests/fixtures/test_iohc_manager.yaml` for a complete
compilable example.

An already-paired controller can be migrated into any chosen slot without any
pairing RF by adding it under `imports:`. Preserve its existing
`storage_namespace` and `storage_key`: the live NVS value remains authoritative
and the YAML `initial_rolling_code` is used only if that NVS key is absent.

The Home Assistant forms use one-based slot numbers **1–20**. Slot moves copy
the controller address, AES key, remote allow-list, calibration and rolling-code
storage reference without transmitting RF. The old slot is disabled and made
durable before the destination is created, so an interrupted move cannot leave
two transmit-capable copies of one controller identity.

Two occupied managed slots can also be swapped directly, without a spare slot.
Before either NVS slot is overwritten, the firmware commits a journal containing
both complete original records and disables both runtime transmitters. Boot-time
recovery can replay that journal after a power loss. The GUI exchanges the two
slot numbers while keeping each shutter's Somfy device, name, area, calibration,
encrypted backup, controller key and rolling-code stream with the same physical
shutter. A swap emits no RF and never needs PROG.

Physical group selections can use an additional receive-only controller
identity. On the hardware-tested Situo 5, **All channels** sends one broadcast
command from that group identity rather than one command for every motor. The
manager stores up to 32 group aliases, each targeting up to 32 managed
shutters. A shutter may belong to multiple aliases. Membership is stored by
permanent controller node identity, so it follows shutters through slot moves
and swaps.

Group discovery accepts only a broadcast OPEN or CLOSE from the physical
remote and stages it until Home Assistant confirms the complete target list.
Adding, editing, or removing an alias sends no RF, never presses PROG, and does
not consume a motor pairing slot. Independent bridge control still uses each
shutter's own paired controller identity; aliases only synchronize received
physical commands to all assigned state estimates. One received group command
is published with its complete target-slot list, so Home Assistant updates
every member without losing intermediate status to state coalescing.

Simultaneous managed MY requests are serialized as complete transactions.
Each shutter receives its uninterrupted STOP, extended execute, press, and
release sequence; the next queued shutter starts after the release burst and a
20 ms radio handoff gap. Duplicate pending MY targets are coalesced because MY
is an idempotent destination. This queue currently covers managed MY requests;
a general scheduler for movement, timed STOP, and Venetian transactions remains
future work.

### Home Assistant

Install the companion
[Somfy IO Shutter Manager](https://github.com/Jordi-14/homeassistant_somfy_io_manager)
custom integration through HACS. It provides graphical pairing, import,
calibration, slot moves, occupied-slot swaps, encrypted recovery, per-shutter
devices, native MY controls, physical-group assignment, and physical-remote
diagnostics.

The physical PROG press cannot be automated: it is the motor's proof that an
already-authorized controller approved the new identity. The wizard sends the
new pairing frame exactly once after explicit confirmation. It persists the
uncertain `pair_sent` state before emitting any pairing RF, so even a power loss
at that boundary cannot trigger an automatic retry. If the second jog is
uncertain, it preserves that identity and offers a resume path instead of
blindly generating or retransmitting controllers.

The firmware status contract is versioned. This manager emits API version `1`;
compatible Home Assistant integration releases validate that version before
using manager events.

Do not run the same restored controller identity on two powered bridges. A
rolling-code identity has one owner; switch the old bridge off before moving a
backup to replacement hardware.

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
| `my_button` | no, iohc | Template button that safely stops, then recalls native MY; requires `my_position` for UI estimation |
| `storage_namespace` | no | NVS namespace for rolling-code persistence; defaults to `somfy` (max 15 chars) |
| `storage_key` | yes | NVS key for rolling code persistence (max 15 chars) |
| `initial_rolling_code` | no | Initial code used only when the NVS key is missing; defaults to `1` |
| `remote_code` | yes | Hex address of this virtual remote |
| `prog_button` | yes | Button entity to trigger PROG pairing |
| `repeat_command_count` | no | Ordinary RF command repeat count; defaults to `4` (the tested distant IOHC link uses `6`) |
| `detected_remote` | no | Text sensor showing the physical remote ID, decoded action, raw parameter, rolling sequence, and event number |
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
