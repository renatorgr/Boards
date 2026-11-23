# ESP32 module harness

Runs the real **Sprayer_ESP32** and **Seeder_ESP32** programs on a PC, with no board.

It plays five kinds of user against them:

- one who sets everything up right;
- one who never sets the module up;
- one who types wrong values;
- one who puts modules, relays or sensors in the wrong place;
- and a network that misbehaves.

For each, it checks what the relays really do and whether the module tells the user about the mistake.

It covers all four boards:

- the sprayer on the ESP32_Relay_X8 and on a dev module with relay modules;
- the seed drill on the Relay24_ESP32 and on the KC868-A32.

A full run takes about a minute.

## Changelog

One row per day, in short; the details are in the sections of this file (the links). Older changes are in the git history.

| Date | What changed |
|---|---|
| 23 Nov 2026 | The memory that a restart keeps and a power cut clears (`RTC_NOINIT_ATTR`, [How it works](#how-it-works)). `sprayer-restart-says-before`: which watchdog, what the sprayer was doing before, the task on each core. 41 scenarios. |
| 22 Nov 2026 | No router: a pretend XIAO with its WiFi bridge, and a tablet's hotspot ([How it works](#how-it-works), [The users](#the-users)). `sprayer-litres-kept`: litres kept through a brownout, a power cut and the night. 40 scenarios. |
| 21 Nov 2026 | First version: 33 scenarios for five users on the four boards (87 runs, about 350 checks), a report, and the "Module harness" check on GitHub ([How it works](#how-it-works), [The users](#the-users)). A section with no relay: now a must. |

## Running it

On Linux (or in a cloud session) with a 32-bit capable compiler: the ESP32 is a 32-bit chip, and `int` and `long` must be 32 bits here too.

```sh
sudo apt install g++-multilib        # once
Tests/ESP32_Modules/run.sh           # build everything and run every scenario
Tests/ESP32_Modules/run.sh --only seeder --board seeder-kc868    # some of them
Tests/ESP32_Modules/run.sh --list    # what there is
```

What you get:

- One line per scenario and board: `ok`, `told?` when a mistake goes unsaid, `BUG`, `CRASH`.
- The report in `build/report.md`.
- The exit code is 1 when a check that must hold fails (and is not in `known-bugs.txt`) or a scenario crashes.
- `SANITIZE=0` builds without the memory checks (faster, finds less).

On GitHub the **Module harness** check runs it on every pull request that changes either program. It puts the report in the run's summary and as a download.

## How it works

- `run.sh` joins each program's `.ino` files into one C++ file, as the Arduino IDE does.
  - It builds it once per board (`-DBOARD=1` / `2`) as a library, with the address and undefined-behaviour sanitizers on.
- `fake/` is a pretend ESP32 + Arduino core with only what the two programs use:
  - pins, Wi-Fi, the network cable, UDP, the web server, the flash (Preferences), program updates and the I2C chips;
  - every call lands in a `fake::Device`, so the harness can look at it.
- `harness/bench.*` is the test bench, in steps of 1 ms of pretend time (below).
- Each scenario runs in its own process: a crash, or a memory error the sanitizer catches in the firmware, is a finding for that scenario and the others still run.

### The bench

**The board.** The relays are read from the board itself, never from the program's variables:

- the X8's pins (HIGH closes);
- the dev module's (LOW closes);
- the Relay24's 74HC595 chain bit by bit (with OE held off by its pull-up until the program pulls it low);
- the KC868-A32's four PCF8574 chips (a LOW pin closes).

**AgOpenWeb.**

- PGN 239 ten times a second, 229 with it on a tool of more than 16 sections, a hello every second.
- On request: 238, 236, 235, 201, 202.
- What the modules send reaches it only when the network lets it through (the same subnet, or to everyone).

**The sprayer.**

- 8 valves of 6 nozzles (1.6 L/min at 3 bar), a pump at 3 bar.
- A 0-10 bar sensor behind the 10k / 10k divider and a 450 pulses/L flow meter.
- Wiring faults: valves on NC contacts, a valve stuck open, sensors missing or swapped.

**The network.** The tractor's router (it can be switched off), the cable (it can be pulled out), a static IP or one from the router.

**The AiO board's XIAO as the tractor network** (`w.useXiao()`, `onXiaoWifi()`): no router, the tablet on the XIAO's WiFi with its WiFi bridge on.

- As the XIAO program does, it sends each packet of AgOpenWeb to every module it has heard in the last 3 s, at once and to that module's own address.
- It passes on to the tablet what a module sends to it.
- The same packet also arrives "to everyone" from the tablet itself, as on a real WiFi access point: late (`w.xiao.lateMs`) and sometimes never (`w.xiao.loseOneIn`).
- The bridge can be off, or stop.
- An own network and a tractor network with the same first three numbers leave the module sending nowhere, as two such networks do on the chip.

**Power and restarts.**

- A restart (asked by the page, by PGN 201, by `ESP.restart()`) loads the program again. So everything in RAM starts fresh while the flash keeps what was saved, as on a board.
- A reset by itself (a brownout, a crash, the watchdog: `Module::resetBy`) does the same.
- `esp_reset_reason()` gives the program the reason, as the chip does.
- A program's `RTC_NOINIT_ATTR` variable (the sprayer's `lastRun`) is kept through a restart or a reset by itself, and filled with noise at a power cut (`Module::powerOff`), as the chip's RTC memory. The bench finds it by its name in the program.
- A reset by the task watchdog (`Module::resetBy(ESP_RST_TASK_WDT)`) first calls the program's `esp_task_wdt_isr_user_handler`, as the chip's watchdog interrupt does, with the tasks in `Device::coreTask` (`xTaskGetCurrentTaskHandleForCore`, `pcTaskGetName`).

## The users

**Careful user**

- `careful-sprayer`, `careful-seeder`: set up right, a field sprayed / sown, the L/ha and the pressure for the target checked against the sprayer, the program going silent, the cable pulled out, a power cycle.
- `careful-on-xiao-wifi`: no router, through the XIAO's WiFi bridge; the subnet kept, sections switched 20 times, a power cycle.
- `careful-on-xiao-wifi-192-168-4`: the AiO itself on 192.168.4; the own network moves to 192.168.40.1.
- `careful-tablet-hotspot`.
- `sprayer-litres-kept`: the module restarts by itself, a brownout, and loses its power mid-job; a new tank; the next morning.
- `sprayer-restart-says-before`: the watchdog restarts the module twice (once after a flood of flow pulses), then a power cut: the module names the watchdog and says what it was doing, and says nothing after a power on.

**Module never set up**

- `power-on-quiet`: no relay closes at power-on or across a restart.
- `sprayer-never-set-up`, `seeder-never-set-up`.
- `aow-default-pins`: 8-section boom, AgOpenWeb's pin config as it comes.
- `sensors-ticked-none-fitted`, `no-section-widths`.

**Wrong settings**

- `relay-active-high-ticked`.
- `page-typos`: impossible values on the page.
- `flash-garbage`, `long-lift-times`, `wrong-wifi-password`, `wrong-subnet-from-program`.
- `nozzle-flow-typo` (as on 20 Nov), `flow-meter-calibration-typo`, `first-section-past-drill`.
- `xiao-wifi-subnet-forgotten`: on the XIAO's WiFi with the subnet 192.168.4 the first version of the bridge asked for.

**Wrong slots**

- `two-relays-one-section`, `relay-on-missing-section`, `more-sections-than-relays`.
- `two-seeders-right`, `two-seeders-same-first`, `two-seeders-same-ip`.
- `seeder-fewer-sections`, `seeder-pin-config-19`, `sensors-swapped`.
- `other-modules-program`: the other module's .bin, a .merged.bin.

**Bad network**

- `network-drops`.
- `garbage-packets`: random bytes, bad checksums, other senders, cut messages.
- `flood`: 5000 PGN 239 a second.
- `uptime-49-days`: the clock wraps round.
- `stale-229`, `program-restarts`.
- `xiao-wifi-late-broadcasts`: the tablet's own packets 1.2 s late, one in three lost, beside the XIAO's copies.
- `xiao-wifi-bridge-stops`.

## The checks

**must**: the module must do this, safely and right.

- For example: the relays follow the sections, everything closes 2 s after the program goes silent, a bad value is refused, no crash.
- A failure is a **bug** and fails the run, unless `known-bugs.txt` lists it.

**tell**: the user made a mistake; does the module say so where the user looks (its page, its log, a PGN 221 text on AgOpenWeb's screen)?

- A failure is reported as **not told** and never fails the run.
- The check looks for a few words (for a section nobody drives: "no relay"), so a new warning should use them, or the check changes with it.

**note**: numbers for the report (the L/ha the module works out, what AgOpenWeb hears).

A check's id is `<scenario>/<key>@<board>`.

- `known-bugs.txt` takes `<scenario>/<key>` (every board) or a full id, with the reason.
- The report says when a listed bug passes again, so its line can come out.

## Adding a scenario

Put it in the `harness/user_*.cpp` of its user:

```cpp
SCENARIO(User::WrongSlots, "my-scenario", "what the user does, in one line", sprayerBoards())
{
    World w;
    Module& m = readyModule(w, board);          // on the tractor network, set up as the careful user does
    w.aow.startup(false, { 1, 2, 3, 4, 5, 6, 7, 8 });
    std::string bad;
    const int n = driveAndCompare(w, m, first(8), inARow(m.bi.relays, 1, 8), 5000, &bad);
    must("relays-follow", n == 0, "relays 1-8 follow sections 1-8", bad);
}
```

A change to what a program sends or saves may need the bench to follow (`harness/bench.cpp`), as the web page simulator does in `Firmware/simulator`.

## What it does not test

- The Teensy firmware (its own harness: `Tests/Teensy_AiO`) and the XIAO program (its own: `Tests/XIAO_ESP32`, with pretend modules).
  - The XIAO here is a pretend one that follows the same rules.
  - The real programs are not run against each other.
  - How late a real WiFi delivers a packet "to everyone" is a chosen number.
- The web pages' own JavaScript (the simulator check does that).
- The radio, the real timing and the ESP32 core itself: the pretend core does what the real one is documented to do, nothing more.
- A message with the right header whose 8-bit checksum matches by chance (once in 256): the AgOpenGPS protocol takes it as real, so the garbage test only sends messages the protocol rejects.
