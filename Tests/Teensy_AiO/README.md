# Teensy AiO harness

Runs the real **Teensy firmware** (`Firmware/v4.5_AiO_WEB`) on a PC, with no board, on a pretend tractor: a WAS,
a GPS receiver, an IMU, a valve or a Keya motor, the steer switch, and AgOpenWeb on the network following a line.
Five kinds of user play against it: one who sets everything up right, one who never sets the module up, one who
types wrong settings, one who wires things wrong, and the failures of a day in the field (a plug that comes out,
the firmware hanging, a power dip, a bad network, a firmware update at the wrong time). For each, it checks what
the steering outputs really do and whether the module tells the user about the mistake.

Each scenario runs on the kits it concerns, out of four: UM982 + TM171 with a Cytron and a lock valve, F9P + BNO085
with an IBT2, UM982 with a Keya motor, and F9P + TM171 with a Danfoss valve. A full run takes about a minute.

This is not a compile for the Teensy: the firmware is built for the PC, with a pretend Teensy core. The Teensy build
itself stays the "Teensy firmware" check and Renato's Arduino IDE.

## Changelog

One row per day, in short; the details are in the sections of this file (the links). Older changes are in the git history.

| Date | What changed |
|---|---|
| 22 Nov 2026 | The five firmware bugs fixed: `known-bugs.txt` is empty, 2 more "not told" are told, and `eeprom-subnet-damaged` is new (54 scenarios, 116 runs, about 765 checks) ([What it found](#what-it-found), [The users](#the-users)). |
| 21 Nov 2026 | First version: 53 scenarios for five users on four kits (115 runs, about 740 checks), a report, and the "Teensy harness" check on GitHub ([How it works](#how-it-works), [The users](#the-users), [What it found](#what-it-found)). |

## Running it

On Linux (or in a cloud session) with a 32-bit capable compiler, because the Teensy is a 32-bit chip and `int`,
`long` and pointers must be 32 bits here too:

```sh
sudo apt install g++-multilib                  # once
Tests/Teensy_AiO/run.sh                        # build everything and run every scenario
Tests/Teensy_AiO/run.sh --only was --kit um982-keya    # some of them
Tests/Teensy_AiO/run.sh --list                 # what there is
```

It prints one line per scenario and kit (`ok`, `told?` when a mistake goes unsaid, `BUG`, `CRASH`) and writes
`build/report.md`. The exit code is 1 when a check that must hold fails (and is not in `known-bugs.txt`) or a
scenario crashes. `SANITIZE=0` builds without the memory checks (faster, finds less). `TEENSY_DUMP_LOG=<folder>`
keeps each run's USB log (what the web page's Messages show).

On GitHub the **Teensy harness** check runs it on every pull request that changes the firmware or the harness, and
puts the report in the run's summary and as a download.

## How it works

- `tools/prep.py` joins the `.ino` files into one C++ file, as the Arduino IDE does (the main `.ino` first, the
  others by name, a prototype of every function), and changes in the copy only what a PC cannot run: the inline
  assembly of the two interrupt entries (`zSafety.ino`), the fixed RAM2 addresses of the crash record, and the flash
  base address (an uploaded `.hex` keeps its real addresses, moved into the pretend flash).
- `fake/` is a pretend Teensy 4.1 + Teensyduino core with only what the firmware uses: pins and PWM, the serial
  ports at their baud rate, I2C (the ADS1115), CAN3, the EEPROM, the 8 MB flash, NativeEthernet / FNET (UDP and
  the web server's TCP), the hardware watchdog with its warning interrupt, the reset reasons and the crash record.
- `run.sh` builds the firmware as a library (`aio.so`) and the bench as a program. A restart (the watchdog, a crash,
  the page's restart, PGN 201, a firmware update) loads the library again, so the RAM starts fresh while the EEPROM
  and the flash keep what was saved, as on a board. The repository's `.hex` is in the pretend flash at power-on, so
  a firmware update keeps a real previous firmware and can go back to it.
- `harness/bench.*` is the test bench, in steps of 1 ms of pretend time. Each `loop()` pass costs 0.2 ms:
  - **the tractor**: wheels that turn at up to 30 deg/s for the valve or the motor, a dead band, a lock valve, a
    hand on the wheel; it drives at the set speed along a straight AB line, starting up to 0.5 m beside it.
  - **the WAS**: 2.5 V in the middle, 0.045 V per degree, read by the pretend ADS1115 at its sample rate. Faults:
    mounted backwards, unplugged (the input floats at 3.9 V), the signal wire on ground, a 3-wire sensor with
    nothing on WAS_L.
  - **the receiver**: a UM982 (GGA, VTG, HPR at 10 Hz) or an F9P (GGA, VTG), bytes at the baud rate it is set to;
    the wrong baud gives garbage, the receiver follows a baud change the module sends it.
  - **the IMU**: a TM171 (Serial5) or a BNO085 in RVC mode; it can freeze.
  - **the Keya**: its heartbeat every 20 ms on CAN, the enable / speed commands, its own 0.5 s stop when the
    commands stop (assumed), faults.
  - **AgOpenWeb**: a hello every second, PGN 254 ten times a second with the angle from a pure-pursuit guidance to the
    line, its steer settings (252, 251) and on request 201, 202. What the module sends reaches it only when the
    network lets it through (the same subnet, or to everyone); it reads the NMEA and checks each checksum, and
    keeps the PGN 221 texts (its screen).
  - **power and restarts**: power off / on, a hang (the loop stops: the watchdog warning, then the reset), a crash
    (the crash hook, then the core's handler until the watchdog), the 49.7-day `millis()` wrap.
- Rules watched every millisecond in every scenario: the outputs are on only while AgOpenWeb asks (within its 2.5 s
  watchdog), off within the time a fault allows, the module never pushes the wheels away from the asked angle, no
  watchdog restart unless the scenario hangs the firmware, and the web page's status is valid JSON.
- Each scenario runs in its own process: a crash, or a memory error the sanitizer catches in the firmware, is a
  finding for that scenario and the others still run.

## The kits

| Kit | Receiver | IMU | Steering | Steer switch |
|---|---|---|---|---|
| `um982-cytron` | UM982 dual | TM171 | Cytron + lock valve | button |
| `f9p-ibt2` | F9P | BNO085 RVC | IBT2 | none |
| `um982-keya` | UM982 dual | none | Keya on CAN | switch |
| `f9p-danfoss` | F9P | TM171 | Danfoss valve | none |

## The users

| User | Scenarios |
|---|---|
| Careful user | `careful-day` (engage, follow the line 40 s within 10 cm, disengage), `careful-silent-program` (the tablet freezes), `careful-restart-page`, `careful-pages` (every page while steering) |
| Never set up | `new-board-no-program`, `no-steer-settings` (AgOpenWeb never sent 252), `no-ads1115` (GPS only, .120), `nothing-plugged`, `cable-late`, `eeprom-damaged`, `eeprom-subnet-damaged` |
| Wrong settings | `was-not-inverted`, `motor-direction`, `ibt2-set-as-cytron`, `danfoss-not-set`, `counts-zero`, `min-above-max-pwm`, `current-sensor-not-fitted`, `switch-mode-no-switch`, `subnet-while-steering`, `subnet-invalid`, `page-settings-range`, `output-mode-for-agopengps` |
| Wrong wiring | `gps-on-gps2-port`, `gps-wrong-baud`, `imu-other-baud`, `was-not-connected`, `was-signal-to-ground`, `was-3wire-differential`, `keya-can-unplugged`, `keya-powered-late`, `tablet-other-subnet` |
| Failures in the field | `was-unplugged-steering`, `was-wire-cut`, `keya-lost`, `keya-fault`, `firmware-hangs`, `firmware-crash`, `crash-outputs-off`, `power-dip`, `too-fast`, `packet-loss`, `short-settings-packet`, `udp-garbage`, `gps-lost`, `rtk-lost`, `imu-frozen`, `web-hammer`, `bad-requests`, `ntrip-flood`, `millis-wrap`, `update-while-steering`, `update-bad-files`, `update-and-back` |

## The checks

- **must**: the module must do this, safely and right (steer only when asked, off after a fault in time, refuse a
  bad value, no crash). A failure is a **bug** and fails the run, unless `known-bugs.txt` lists it.
- **tell**: the user made a mistake or something broke; does the module say so where the user looks (its web page's
  Messages, a PGN 221 text on AgOpenWeb's screen)? A failure is reported as **not told** and never fails the run.
  The check looks for a few words (for the WAS unplugged: "WAS"), so a new message should use them, or the check
  changes with it.
- **note**: numbers for the report (how long until the outputs were off, how far the wheels went).

A check's id is `<scenario>/<key>@<kit>`. `known-bugs.txt` takes `<scenario>/<key>` (every kit) or a full id, with
the reason; the report says when a listed bug passes again, so its line can come out.

## What it found

The first run (21 Nov 2026) found five things in the firmware, listed in `known-bugs.txt` with the fix each needs:

- A firmware update refused because autosteer is on (or the tractor moves) reads the whole file before it answers:
  the steering loop stops for as long as the upload takes (10.6 s for the 2 MB `.hex` at 200 kB/s), the outputs as
  they were. Fixed 22 Nov 2026: the file is read and dropped a piece per pass of the loop.
- PGN 252 (and 251) shorter than its length byte is taken: the missing bytes come from the packet before
  (counts per degree 120 became 1). Fixed 22 Nov 2026: PGN 254, 252 and 251 shorter than 8 data bytes are dropped.
- PGN 201 saves and restarts for any subnet: 0.0.0 (the module then answers nobody), the subnet it is on already,
  and while steering. The XIAO's subnet change (`netSetSubnet`) refuses all three. Fixed 22 Nov 2026: PGN 201 goes
  through `netSetSubnet` too.
- The stored steer settings are taken from the EEPROM without a check: 0xFF bytes after a good mark turn every flag
  on, and an IBT2 is then driven as a Cytron with the relay inverted (full lock, nobody asked). Fixed 22 Nov 2026:
  a value the program cannot send gives a new board's defaults, said in the messages (the subnet too).
- Min To Move above the Max Limit lets the PWM go above the Max Limit. Fixed 22 Nov 2026: the PWM stays at the Max
  Limit, said in the messages.

And 25 "not told" (23 after the fixes of 22 Nov 2026): for example the wheels turning the other way (Invert WAS or Motor direction) is reported as
"did not follow", counts per degree 0 as an angle of millions of degrees, a current sensor ticked but not fitted
stops autosteer with no message, and a restart after a power dip is not shown on AgOpenWeb's screen.

## Adding a scenario

Put it in the `harness/user_*.cpp` of its user:

```cpp
SCENARIO(User::FieldFailures, "my-scenario", "what happens, in one line", { Kit::PumaCytron, Kit::F9pIbt2 })
{
    World w(kit);
    w.startUp();                           // powered, AgOpenWeb's settings sent, 8 s
    steerFor(w, 8, 5000);                  // 8 km/h, AgOpenWeb steering for 5 s
    w.was.unplugged = true;
    w.expectOff("the WAS plug came out", 300);
    const uint64_t off = msUntilOff(w, 5000);
    must("off-fast", off <= 300, "Steering off within 0.3 s", fmt("%llu ms", (unsigned long long)off));
    w.expectOffEnd();
    reportRules(w);                        // the rules watched all along
}
```

A change to what the firmware sends, reads or saves may need the bench to follow (`harness/bench.cpp`), as the web
page simulator does in `Firmware/simulator`. `tools/prep.py` stops with a message when one of the few places it
changes is not found any more.

## What it does not test

- The Teensy build itself (the "Teensy firmware" check does it), the real timing of the 600 MHz chip, and the
  hardware: the pretend core does what the real one is documented to do, nothing more. A flash write here takes
  pretend time but never fails.
- The web page's own JavaScript (the simulator check does that) and the XIAO program.
- The dual F9P on the GPS2 port, the XIAO link (PGN 141-145), the correction radio on Serial3, the TM171
  calibration, the receiver configuration from the page and the machine module tab.
- Hydraulics: the wheels follow the valve in a straight line of speed against PWM, with no oil temperature, relief
  valve or pump flow.
