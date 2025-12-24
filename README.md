# ESPHome Somfy cover remote
This is an external component for [ESPHOME](https://esphome.io/), to control Somfy RTS covers from for example [Home Assisant](https://www.home-assistant.io/).

## Required hardware
- ESP32
- CC1101 RF module

## Setup
Use the following ESPHome yaml as a base for your Somfy controller. Add one or more covers, depending on your needs.
A shade can be controlled by either this controller or by physical remotes. For the state of the shade to be correctly refleced in HA each shade needs to have the remote control id's listed that control the shade.
```
allowed_remotes:
  - 0x112233
  - 0x445566
```
##  Basic how to guide:
 - generate a random ID for the new (this) remote control
 - pair this remote with the motor of the shade
 - open the web interface of this controller
 - press a button on the remote control that controls the shade
 - copy-paste the remote's id in the ```receive_remote_codes```


## Generate remote code 
The remote code is a three byte hex code.
For example, use the website: https://www.browserling.com/tools/random-hex  
Set to 6 digits and add `0x` in front of the generated hex number.

## Pair the cover
Put your cover in program mode with another remote, then use the `Prog x` button to pair with the ESP. From then on the cover should respond to the ESPHome Somfy controller. You can also connect multiple covers by pairing then one by one with the same `Program x` button.

## Repeating command setting
The *Somfy_Remote_Lib* library defaults to sending a command four times. Some devices do not handle this well and should only reveive the command one time. For these devices the optional parameter `repeat_command_count` can be set in the yaml for the cover.

In order to see more information set the log level to ```DEBUG```

```
substitutions:
  devicename: shades
  devicelabel: Shades

logger:
  level: NONE

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  fast_connect: true
  reboot_timeout: 15min

api:
  reboot_timeout: 0s

ota:
  - platform: esphome

button:
  - platform: restart
    name: "Restart"
    icon: mdi:restart
    entity_category: diagnostic

text_sensor:
  - platform: wifi_info
    ip_address:
      name: "IP Address"
      disabled_by_default: true
      entity_category: diagnostic
      icon: mdi:ip

external_components:
  source: github://leonardpitzu/somfy_cover@main
  components: [somfy_cover]
  refresh: 600s


esphome:
  comment: $devicelabel
  name: $devicename
  platformio_options:
    board_build.flash_mode: dio

esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: esp-idf

web_server:
  port: 80
  log: false
  local: true
  version: 3

spi:
  clk_pin: GPIO07
  mosi_pin: GPIO09
  miso_pin: GPIO08

cc1101:
  frequency: 433.42MHz
  cs_pin: GPIO6

remote_transmitter:
  id: "transmitter"
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
  id: "receiver"
  pin: GPIO03

text_sensor:
  - platform: template
    id: detected_remote
    name: "Detected Remote"

cover:
  - platform: somfy_cover
    id: room1
    name: "Room 1"
    device_class: shutter
    open_duration: 55s
    close_duration: 55s
    storage_key: KeyRoom1
    remote_code: 0xabcde
    prog_button: program_room1
    remote_transmitter: transmitter
    remote_receiver: receiver
    detected_remote: detected_remote
    allowed_remotes:
      - 0x112233
      - 0x445566
  - platform: somfy_cover
    id: room2
    name: "Room 2"
    device_class: shutter
    open_duration: 55s
    close_duration: 55s
    storage_key: KeyRoom2
    remote_code: 0xabcdf
    prog_button: program_room2
    remote_transmitter: transmitter
    remote_receiver: receiver
   detected_remote: detected_remote
    allowed_remotes:
      - 0x778899

button:
  - platform: template
    id: "program_room1"
    name: "Prog Room 1"
    entity_category: config
  - platform: template
    id: "program_room2"
    name: "Prog Room 2"
    entity_category: config
```

## Credits

This is based on the work of https://github.com/HarmEllis/esphome-somfy-cover-remote and https://github.com/fawick/somfy_cover_2025.12
The original esphome implementations were "unidirectional" in the sense that the shades were controlled correctly but the changes in the shade position triggered by physical remotes were not synced back to HA.
The receiver/decoder component was implemented to coer this case based on the RTS decoder component from https://github.com/rstrouse/ESPSomfy-RTS

All credits go to the original authors.
