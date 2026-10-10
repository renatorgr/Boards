# v5_AIO_UM982_X20P_F9P_TM171_GNHPR_INHPR

Teensy 4.1 (AiO board) firmware for AgOpenWeb — UM982 (dual antenna) **or** u-blox F9P / X20P (single antenna) + TM171 IMU, Cytron or Keya steering.
It sends the "standard set" of NMEA sentences, which AgOpenWeb joins into one fix per epoch.

**Needs an AgOpenWeb build that includes PR #300** (the "standard set" / epoch assembler, `develop` from the evening of 9 Oct 2026 onwards). On an older build these sentences show up as "not accepted".

## Inputs

- GNSS receiver on Serial7, 460800 baud, 10 Hz. The receiver type is detected automatically:
  - **UM982 (dual antenna):** GGA + VTG + HPR (GP or GN talker). KSXT is no longer used (it can stay on; it is ignored).
  - **u-blox F9P / X20P (single antenna):** GGA + VTG. With no HPR, the firmware does not wait for it: each epoch goes out as soon as the GGA and VTG are in.
  - GGA is required (it marks each epoch). A VTG or HPR counts as "sent by the receiver" if one arrived in the last 3 s (`RECEIVER_SEEN_MS`); for the first 3 s after start-up both are waited for.
- TM171 (Serial5, 115200): roll, pitch and yaw.

## Output (UDP 9999): 4 lines per epoch, one per datagram

| Line | Content | AgOpenWeb uses it for |
|---|---|---|
| `$GNGGA` | the receiver's GGA (a UM982 GGA with `00` satellites / HDOP `9999.0` gets the last good values, for up to 2 s) | position, fix, satellites, HDOP, correction age |
| `$GNVTG` | speed and track from the receiver's VTG; if the VTG is empty, computed from the GGA positions | speed |
| `$GNTHS` | dual-antenna heading (the UM982's HPR value, character for character); mode `A` = valid, `V` = no dual heading | heading with two antennas |
| `$INHPR` | TM171 (talker `IN` = inertial sensor): heading aligned to the dual heading, **roll in the pitch field** (HPR convention), pitch in the roll field; QF 4 = TM171 OK, QF 0 = TM171 lost | roll, always; heading without two antennas |

Result in AgOpenWeb (checked against AgOpenWeb's own code):

| Situation | Heading | Roll |
|---|---|---|
| Two antennas (HPR QF 4 fixed, or 5 float) | `$GNTHS` (antennas) | TM171 |
| Second antenna lost / no heading solution | TM171, with AgOpenWeb's single-antenna fusion (fix-to-fix + IMU, reverse detection) | TM171 |
| Single-antenna receiver (F9P / X20P) | TM171, with the same single-antenna fusion | TM171 |
| TM171 lost | antennas (or fix-to-fix without two antennas) | 0 |

AgOpenWeb's status bar shows the family **`GGA+VTG+HPR+THS`** (the "HPR" is the TM171's `$INHPR`).

### Why the dual heading goes out as `$GNTHS` and not as `$GNHPR`

AgOpenWeb starts a new epoch when a sentence type repeats. A `$GNHPR` (antennas) and an `$INHPR` (TM171) are both "HPR", so they could never be in the same fix: the `$INHPR` would open an epoch with no GGA and be dropped. `$GNTHS` carries the same heading as the UM982's HPR and can share the epoch with the `$INHPR`. The firmware still **reads** the UM982's `$GNHPR`.

## Single antenna: u-blox F9P / X20P

- In u-center: UART1 at 460800, GGA + VTG at 10 Hz (100 ms measurement period), high-precision NMEA on, compatibility mode and Limit82 off, RTCM3 input on UART1, all other messages off. NMEA version: any (4.11 recommended). Save to the receiver's memory (BBR/Flash).
- In AgOpenWeb: **"Dual GPS" off**. `$GNTHS` always goes out with `V`; AgOpenWeb uses the TM171 heading with its single-antenna fusion (fix-to-fix + IMU, reverse detection), as with `$PANDA`. Set the antenna height and position in the vehicle profile.
- TM171 yaw direction: without two antennas the firmware cannot learn it and uses +1 (like the official AiO firmware, which uses the TM171 yaw as is). If the heading turns the wrong way in curves, set `TM171_YAW_SIGN -1`.
- The message `[status] Single antenna: no HPR from the receiver ...` confirms the mode.

## UM982 setup

In UPrecise (or by commands), on the port connected to the Teensy, at 10 Hz: `GPGGA 0.1`, `GPVTG 0.1`, `GPHPR 0.1`, then save (`SAVECONFIG`). Keep `CONFIG HEADING OFFSET` as it was (the HPR heading already has that offset applied).

## In AgOpenWeb (UM982, dual antenna)

- **"Dual GPS" on** (when off, AgOpenWeb ignores the antenna heading and uses only the TM171 + fix-to-fix).
- With `AutoDualFix` on, above `DualSwitchSpeed` AgOpenWeb uses the fix-to-fix heading corrected by the antennas instead of the antenna heading itself; turn it off to steer on the antenna heading.
- Antenna orientation: `CONFIG HEADING OFFSET` in the UM982 **or** `DualHeadingOffset` in AgOpenWeb — only one of the two.
- Roll direction / zero: AgOpenWeb's "Roll invert" and "Roll zero" (they apply to the `$INHPR` roll).
- `MinFixQuality` (default 4 = RTK fixed): below it the fix is marked invalid. On the bench set 1 or 2.
- HDOP and correction age are shown again (they come from the GGA).
- NTRIP (ReNEP): port 2101 (legacy 1004/1012) gave RTK fixed with the UM982; port 2102 (MSM5) stayed in float.

## Settings (top of `zHandlers.ino`)

- `HPR_ACCEPT_FLOAT` 1 = dual heading also with HPR QF 5 (float); 0 = only QF 4 (fixed).
- `EPOCH_WAIT_MS` 60: the firmware sends the epoch as soon as the VTG and the HPR with the GGA's time are in; if one is missing, it sends 60 ms after the GGA. A sentence the receiver does not send at all (HPR on an F9P / X20P) is not waited for (`RECEIVER_SEEN_MS` 3000).
- `TM171_SWAP_ROLL_PITCH`: TM171 rotated 90° on the board ("Use Y axis" also swaps roll/pitch).
- `TM171_YAW_SIGN`: 0 = yaw direction learned against the dual heading (only from real turns of the whole rig); +1/-1 forces it.
- `GGA_HOLD_MS`: how long good satellites/HDOP values are repeated when the UM982 sends `00` / `9999.0`.
- Diagnostics (0 on the tractor):
  - `FUSION_DEBUG` (0): one `[fusion] DUAL ...` or `[fusion] SINGLE ...` line per second: HPR QF and heading, `antRoll` (roll measured by the antennas, to compare with the TM171's), TM171 alignment, heading and roll sent, speed in km/h and its source (`vtg` or `positions`).
  - `RAW_NMEA_DEBUG` (0): every GGA, VTG and HPR as the receiver sends it, prefixed `[raw]`.
  - `SEND_TO_USB` (0): the 4 lines sent to AgOpenWeb are also printed on the serial monitor.

## Status messages (`zStatus.ino`)

`[status] ...` lines, **only when something changes** (a new state must last 1 s):

- Ethernet: cable connected (module IP and destination) / cable unplugged
- AgOpenWeb talking (PC IP) / silent for 5 s
- RTCM corrections arriving from AgOpenWeb / stopped 5 s ago
- GPS: first position; fix changes with satellites and correction age; no GGA for 5 s; VTG arriving / no VTG (speed computed from the positions)
- Dual: dual-antenna heading OK (QF 4 fixed / 5 float, sent as `$GNTHS`) / no dual heading
- Single antenna: the receiver sends no HPR (F9P / X20P, or HPR off on the UM982)
- TM171: data OK / no data

LEDs: green = dual heading, red = TM171 heading, red blinking = TM171 lost.
