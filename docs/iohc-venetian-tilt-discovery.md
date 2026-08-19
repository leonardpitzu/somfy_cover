# IOHC 1W Venetian tilt discovery

This document records hardware-verified frames from a Somfy Situo 5 Variation
A/M io Pure II controlling a Venetian blind. The capture was made on the
remote's working channel with a CC1101 receiver. Installation-specific source
identifiers, controller keys, and rolling values are intentionally omitted.

## Verified commands

Each user action produces an authenticated `CMD_EXECUTE` (`0x00`) frame and an
authenticated Somfy private/button-event (`0x20`) frame. The rolling sequence,
MAC, repeat control bit, and CRC vary normally; the clear application payloads
below were repeatable.

| Action | `CMD_EXECUTE` clear data | `0x20` clear data | Private action |
| --- | --- | --- | --- |
| Open button | `01 43 00 00 00 00` | `02 FF 01 43 00 0C 00 00` | `0x00` |
| Close button | `01 43 C8 00 00 00` | `02 FF 01 43 01 0C 00 00` | `0x01` |
| STOP/MY button | `01 43 D2 00 20 D2 00 00` | `02 FF 01 43 02 0C 00 00` | `0x02` |
| Wheel clockwise, one detent | `01 43 D2 00 20 CD 2E 00` | `02 FF 01 43 0D 0C 00 00` | `0x0D` |
| Wheel counterclockwise, one detent | `01 43 D2 00 20 CC A2 00` | `02 FF 01 43 0E 0C 00 00` | `0x0E` |

One wheel detent sends one complete frame pair. No release event was observed
for either wheel direction. Multiple deliberately separated detents advanced
the rolling sequence and reproduced the same clear payloads.

A short STOP/MY press additionally ends with the release event
`02 FF 01 43 02 05 FF 00`. On the tested blind, a press while idle recalled
the MY position and a press while moving stopped the lift. A dedicated STOP
from Home Assistant remains an ordinary short `D200` EXECUTE; a dedicated MY
first sends that safe STOP, waits for the motor to settle, and then sends the
complete native MY sequence shown above.

## Implementation implications

- Tilt remains compatible with the existing IOHC 1W controller identity,
  encryption, rolling-code storage, repeat handling, and pairing path.
- The complete wheel gesture must include both frames. `D200` alone is the
  already-known STOP command and does not identify a tilt direction.
- Receive synchronization must not immediately expose the wheel's leading
  `D200` as STOP/MY. It should briefly correlate it with the following `0x20`
  action; `0x0D` and `0x0E` are tilt detents.
- Sending tilt consumes two rolling codes per detent because both frames are
  independently authenticated.
- The wheel direction that should increase Home Assistant's 0–100 tilt value
  is installation-dependent and must be configurable.
- The motor does not report an absolute slat angle in this 1W path. Setup must
  ask the user to count the wheel detents that actually move the slats between
  the two physical tilt limits, just as it asks for full opening and closing
  times. Do not count an extra click used to verify that an endpoint has been
  reached. The tested blind used eight effective detents; this is calibration
  data, not a protocol-wide constant.
- A 0% or 100% request sends the calibrated full range plus two margin detents.
  The motor ignores commands beyond its physical endpoint, so these requests
  are reliable resynchronisation operations even when the estimate was stale.
- Intermediate requests are quantized to the nearest reachable detent. The
  final published tilt state is that quantized value, not the unachievable raw
  percentage requested by the caller.
- A lift OPEN command immediately rotates the slats to their fully open,
  horizontal attitude. On the tested eight-step blind this is the physical
  midpoint, not either tilt endpoint. A lift CLOSE command immediately rotates
  them to the clockwise closed endpoint. Stopping the lift leaves the slats in
  that attitude; it does not restore their previous tilt.
- A tilt command issued while the lift is moving stops the lift, applies the
  requested detents, and does not resume the interrupted height movement.
- A native MY recall uses the same temporary lift attitude while travelling,
  then the motor restores its saved MY slat attitude at the destination. The
  manager therefore stores a separate MY tilt step, measured clockwise from
  the counterclockwise endpoint, so the estimated tilt state can follow the
  physical motor without sending corrective wheel commands.
- The tested installation used eight effective steps and a saved MY tilt at
  physical step two. Both settings, the Venetian flag, direction mapping, and
  the paired controller identity survived an ESP reboot and were verified
  again with endpoint and MY commands.

## Receiver note

For this capture hardware, tuning the CC1101 to `868.961 MHz` and rejecting
predecode candidates below `-100 dBm` made all four gestures reliable. The
first valid carrier appeared roughly 9.5–12.7 kHz above the nominal
`868.950 MHz`. This is a diagnostic observation, not a production-frequency
change: normal firmware should remain at the standard frequency unless a
specific radio is calibrated.
