# ESPHome Somfy cover remote

This repository provides an **external ESPHome component** to control Somfy RTS shades using a CC1101 RF module.

It supports:
- **Transmitting** Somfy RTS commands from ESPHome/Home Assistant
- **Receiving/decoding** Somfy RTS frames so Home Assistant stays in sync when you use a physical remote

## Required hardware

- ESP8266 / ESP32 / RP2040 (see ESPHome supported platforms)
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
  dump: all
```

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

Feel free to reuse the code as you please. This is provided as-is; I may fix bugs I encounter, but I cannot guarantee support for every possible edge case.
