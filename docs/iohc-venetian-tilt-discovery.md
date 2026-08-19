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
  action; `0x0D` and `0x0E` are tilt detents. Live capture measured 57 ms
  between the ordinary-looking `D200` and its matching private tilt action.
  The implementation holds a received Venetian `D200` for up to 350 ms: a
  matching tilt action consumes it, a matching STOP/MY action releases it
  immediately, and expiry releases a standalone stop. The motor still stops
  before tilting; only the duplicate Home Assistant event is suppressed.
- Sending tilt consumes two rolling codes per detent because both frames are
  independently authenticated.
- Home Assistant defines 0% tilt as fully closed and 100% as fully open. The
  physical direction that increases that value is installation-dependent and
  must remain configurable. On the tested blind, counterclockwise opens the
  slats and clockwise closes them.
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
- A lift OPEN command immediately rotates the slats to the fully open,
  counterclockwise endpoint (100%). A lift CLOSE command immediately rotates
  them to the clockwise closed endpoint (0%). Stopping the lift leaves the
  slats in that attitude; it does not restore their previous tilt.
- A tilt command issued while the lift is moving stops the lift, applies the
  requested detents, and does not resume the interrupted height movement.
- A native MY recall uses the same temporary lift attitude while travelling,
  then the motor restores its saved MY slat attitude at the destination. The
  manager therefore stores a separate MY tilt step, measured clockwise from
  the counterclockwise endpoint, so the estimated tilt state can follow the
  physical motor without sending corrective wheel commands.
- The tested installation used eight effective steps and a saved MY tilt at
  physical step two from the counterclockwise/open endpoint, which is 75% in
  Home Assistant. Both settings, the Venetian flag, direction mapping, and the
  paired controller identity survived an ESP reboot and were verified again
  with endpoint and MY commands.

## Receiver note

The discovery build first obtained reliable close-range captures near
`868.961 MHz`, with valid energy roughly 9.5–12.7 kHz above the nominal
`868.950 MHz`. This is a per-radio calibration observation, not a
protocol-frequency change; production remains on the configured 1W frequency.

Several apparently reasonable production settings failed at the remote's
normal location. A 35-byte FIFO window—the encoded size of the longest ordinary
application frame—did not let real Situo presses complete. A fixed absolute
carrier threshold and stricter 30/32 sync also rejected useful weak bursts.
Removing carrier qualification recovered frames but left reception vulnerable
to noise matches.

The hardware-validated receiver therefore retains the 60-byte FIFO window,
16/16 carrier-qualified sync, full LNA and DVGA gain, TI's 33 dB magnitude
target, the lowest absolute carrier offset, and a 6 dB relative carrier-rise
threshold. The relative threshold admits a remote that rises above the local
noise floor without allowing continuous weak-noise sync matches to occupy the
FIFO. At the normal test location the full DOWN, STOP/MY, MY, clockwise and
counterclockwise sequence produced 43 raw FIFO packets, 40 CRC-valid frames and
five accepted user actions; the final accepted action measured `-70.5 dBm`.

The correlation path was subsequently verified on the production ESPHome
2026.7.4 build. One deliberately large wheel turn produced two real clockwise
detents and no STOP/MY event. DOWN followed by MY while moving produced exactly
one CLOSE and one STOP/MY event, and a second MY while idle produced exactly
one STOP/MY event and recalled the saved position. Thus the UI sees the user's
logical actions while the motor retains its native stop-then-tilt behaviour.

## Situo group and All-channels mode

The tested Situo 5 does not replay every individual channel when **All
channels** is selected. It sends one normal broadcast `CMD_EXECUTE` from a
distinct group controller identity. That identity differs from the source used
by an individual channel; its exact value is installation-specific and is not
published here.

The manager models these identities as receive-only many-to-many aliases. One
physical group may update any number of managed shutters, and one shutter may
belong to several overlapping groups. Alias membership refers to permanent
controller node identities rather than mutable slot numbers, so it survives
slot moves and swaps. The records are persisted separately from paired
controller identities and rolling-code storage.

Discovery accepts only a broadcast OPEN or CLOSE and stages the source until a
caller confirms the target shutters. Setting or deleting the alias only
refreshes receive filters: it sends no RF, uses no PROG action, consumes no
motor pairing slot, and never changes a transmitter's rolling-code stream.
Hardware testing confirmed that an All-channels DOWN followed by MY was
accepted for an assigned shutter and that the alias remained active after an
ESP reboot.

All-channels wheel control was also hardware verified. Four clockwise detents
physically tilted both assigned Venetian blinds and were decoded as four tilt
actions without false STOP/MY events. When a physical group targets several
managed shutters, the manager publishes one status event containing the full
slot list. This avoids native-API coalescing of back-to-back text-sensor states
and lets every Home Assistant diagnostic sensor observe the same user action.
