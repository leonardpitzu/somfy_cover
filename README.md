# ESPHome Somfy cover remote

This repository provides an **external ESPHome component** to control Somfy RTS shades using a CC1101 RF module.

It supports:
- **Transmitting** Somfy RTS commands from ESPHome/Home Assistant
- **Receiving/decoding** Somfy RTS frames so Home Assistant stays in sync when you use a physical remote

## Required hardware

- ESP32
- CC1101 RF module (433 MHz)
- 433 MHz antenna (strongly recommended)

## Installation (external_components)

```yaml
external_components:
  source: github://leonardpitzu/somfy_cover@main
  components: [somfy_cover]
  refresh: 600s
```

## Minimal working configuration

This is a known-good baseline. Adjust GPIOs to your board/wiring. The pinout in this example is for the ESP32S3 (https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)

```yaml
substitutions:
  devicename: shades
  devicelabel: Shades

esphome:
  name: ${devicename}

logger:
  # Set to DEBUG temporarily when you want to see RX decode details.
  level: INFO

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
  reboot_timeout: 0s

ota:
  - platform: esphome

cc1101:
  frequency: 433.42MHz
  cs_pin: GPIO6

remote_transmitter:
  id: transmitter
  non_blocking: true
  pin: GPIO04
  carrier_duty_percent: 100%
  on_transmit:
    then:
      - cc1101.begin_tx
  on_complete:
    then:
      - cc1101.begin_rx

remote_receiver:
  id: receiver
  pin: GPIO03
```


## Pairing the ESPHome “base remote” with a motor (PROG / programming)

The ESPHome device acts like a **new Somfy RTS remote** identified by `remote_code`.
Before it can control a given motor, that motor must be **paired/programmed** to accept this new remote.
That’s what `prog_button` is for.

### Requirements

- Each cover **must** have its own unique `remote_code` (hex) unless you intentionally want **group control**.
- Keep `storage_key` stable. It stores the Somfy rolling code in flash; changing it (or flashing to a fresh device without it) can desync the motor.

### How it works

1. Put the motor into *programming mode* using an **already paired physical remote**:
   - Press and hold the physical remote’s **PROG** button (typically ~2 seconds) until the motor “jogs” (brief up/down).
   - (Some motors have a small **P2/PROG** button on the head instead.)
2. Within the programming window (usually a short period), press the matching **Prog …** button in Home Assistant for that cover.
3. The motor should “jog” again to confirm pairing.
4. Now use **Open/Close/Stop** from Home Assistant — the motor will accept commands from this ESPHome remote.

### Defining the PROG buttons in ESPHome

`prog_button` on the cover points to a normal ESPHome `button:` entity. Example:

```yaml
cover:
  - platform: somfy_cover
    id: livingroom_door
    name: "Living Room Door"
    storage_key: KeyLivingDoor
    remote_code: 0x088331
    prog_button: program_livingroom_door
    # ...

button:
  - platform: template
    id: program_livingroom_door
    name: "Prog Living Room Door"
    entity_category: config
```

### Notes / gotchas

- Pairing is per motor: press the **Prog** button for the specific cover you’re programming.
- Don’t spam PROG. Treat it like a configuration action.
- To pair a new remote you need to first put the motor of the shade in pairing mode using an already paired remote.

## Detecting remote IDs (one-time setup)

Create a text sensor that will display the last decoded remote ID and command:

```yaml
text_sensor:
  - platform: template
    id: somfy_rx_last
    name: "Detected Somfy Remote"
```

Then add your covers. The key options are:

- `detected_remote`: where decoded frames are published (for discovery)
- `allowed_remotes`: list of physical remotes allowed to control/sync this cover

```yaml
cover:
  - platform: somfy_cover
    id: livingroom_door
    name: "Living Room Door"
    device_class: shutter
    open_duration: 40s
    close_duration: 40s

    storage_key: KeyLivingDoor

    # The transmitter identity for this ESPHome device (hex)
    remote_code: 0x088331

    prog_button: program_livingroom_door
    remote_transmitter: transmitter
    remote_receiver: receiver

    # Always updated on successful decode (use this to learn remotes)
    detected_remote: somfy_rx_last

    # Physical remotes that should be allowed to sync/control this cover
    allowed_remotes:
      - 0x92FB39
      # - 0xA1B2C3
```

### Workflow

1. Set `logger.level: DEBUG` temporarily (optional, for more RX detail)
2. Press a button on your physical remote
3. Watch `text_sensor.detected_somfy_remote` in Home Assistant / ESPHome logs
4. Copy the shown `0x......` remote ID into `allowed_remotes`
5. Recompile and flash (usually a one-time operation)

## Credits

This project builds on prior work:
- https://github.com/HarmEllis/esphome-somfy-cover-remote
- https://github.com/fawick/somfy_cover_2025.12

The receiver/decoder logic is based on the reference implementation from:
- https://github.com/rstrouse/ESPSomfy-RTS

All credits go to the original authors.

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
