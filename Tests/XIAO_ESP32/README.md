# XIAO harness

Runs the real **XIAO program** (`Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32`) on a PC, with no board: a pretend Teensy
on its serial link, phones on its page, a tablet running AgOpenWeb on its WiFi (the WiFi bridge), section modules
on that WiFi and the farm's WiFi around. Five kinds of user play against it: one who sets everything up right, one who never set the XIAO (or
the Teensy beside it) up, one who types wrong settings and chooses wrong files, one who connects things wrong, and
a bad link (a weak WiFi, a noisy serial line, browsers that stall or vanish, too much at once). For each, it checks
what the XIAO really does: what reaches the Teensy, what goes out on the WiFi, when it restarts, and whether it
tells the user about the mistake.

Each scenario runs on the XIAO ESP32C3, and on the ESP32S3 too where the chip matters (updates, start-up, the
bridge). A full run takes about two minutes, most of it the build.

This is not a compile for the XIAO: the program is built for the PC, with a pretend ESP32 core.

## Changelog

One row per day, in short; the details are in the sections of this file (the links). Older changes are in the git history.

| Date | What changed |
|---|---|
| 22 Nov 2026 | First version, then section modules on its WiFi (and in the Machine tab), its network on the module's numbers, the module's page: 75 scenarios, five users, two XIAOs (86 runs) ([How it works](#how-it-works)). Two of four findings fixed ([What it found](#what-it-found)). |

## Running it

On Linux (or in a cloud session) with a 32-bit capable compiler, because the ESP32 is a 32-bit chip and `int`,
`long` and pointers must be 32 bits here too:

```sh
sudo apt install g++-multilib            # once
Tests/XIAO_ESP32/run.sh                  # build everything and run every scenario
Tests/XIAO_ESP32/run.sh --only bridge --kit s3     # some of them
Tests/XIAO_ESP32/run.sh --list           # what there is
```

It prints one line per scenario and XIAO (`ok`, `told?` when a mistake goes unsaid, `BUG`, `CRASH`) and writes
`build/report.md`. The exit code is 1 when a check that must hold fails (and is not in `known-bugs.txt`) or a
scenario crashes. `SANITIZE=0` builds without the memory checks (faster, finds less).

On GitHub the **XIAO harness** check runs it on every pull request that changes the XIAO program, its compiled
`.bin` files or this folder, and puts the report in the run's summary and as a download.

## How it works

- `run.sh` builds the program's `.ino` as one C++ file, as the Arduino IDE does, twice (the ESP32C3, and the ESP32S3
  with `-DXIAO_KIT_S3`), each as a library, with the address and undefined-behaviour sanitizers on. A restart of
  the XIAO loads the library again, so everything in RAM starts fresh and the flash keeps what was saved.
- `fake/` is a pretend ESP32 + Arduino core with only what the program uses. It follows the real core (esp32
  3.3.12) where the program's behaviour hangs on it:
  - **the web server**, byte by byte as `WebServer` reads a request: it waits for a late line or a late piece of a
    form inside `handleClient()`, hands an upload over in pieces of 1436 bytes and waits for the next one up to the
    client's timeout, drops a silent spare connection, answers 408 / 414 / 431;
  - **connections** with a send buffer of 5.7 kB that the browser empties at its link's pace, so an answer to a
    slow phone takes as long as it would;
  - **the serial port to the Teensy** at 460800 baud (46 bytes a millisecond each way) with the driver's buffers:
    a write with no room waits, a receive buffer that is not read loses bytes;
  - **the updater** (`Update`): it writes when 4096 bytes are there, wants the first byte E9, and at the end
    checks the file's parts and checksum for this chip;
  - **the task watchdog** (15 s): the program's own entry, fed only by `esp_task_wdt_reset_user()`, and the loop
    task's (fed once per pass of `loop()` and by `feedLoopWDT()`) when the program switches that one on. When
    one runs out the XIAO restarts with the reason "watchdog";
  - **a second task and a lock** (FreeRTOS: `xTaskCreateUniversal`, a recursive mutex): the program's link task
    gets its turn at every pretend millisecond in which its sleep is over, also in the middle of a wait of the
    loop's, as a task of a higher priority does on the chip; the loop stands still for that turn;
  - WiFi (its own network, joining another, searching), UDP sockets, the flash (Preferences), mDNS, the USB port.
- Pretend time moves a millisecond at a time and only inside the program's own waits (so a scenario's own code does
  not run while the program waits for a phone inside one `loop()` pass: `w.later()` books what must happen in the
  middle of such a wait). Every millisecond the bench
  (`harness/bench.*`, `harness/teensy.cpp`) moves the bytes on the serial link and lets everyone do their part:
  - **the pretend Teensy** follows `zXiao.ino` of the Teensy firmware: the hello it waits for, its status in
    pieces, its log, the info, subnet and firmware-update frames, the WiFi bridge (who has the module: the cable
    first), its 2.5 s steering watchdog, and "XIAO silent" after 10 s without a hello;
  - **AgOpenWeb on a tablet**: its hello every second, PGN 254 a hundred times a second (each one different, so
    the bench knows which arrived), PGN 239 ten times a second, corrections 256 bytes at a time;
  - **section modules on the XIAO's WiFi** (`w.mod`, up to five): each sends its PGN 237 every 200 ms and answers
    the hello, to everyone until it has heard the XIAO and to the XIAO only from then on, as the real programs do
    (`Sprayer_ESP32`, `Seeder_ESP32`: their own harness is `Tests/ESP32_Modules`); one can send far too much, or
    drop off the network;
  - **browsers**: requests as Chrome on Android sends them, on a link with a speed each way, that may stall,
    close, or vanish without a word.
- Each scenario runs in its own process: a crash, or a memory error the sanitizer catches in the program, is a
  finding for that scenario and the others still run.

## The users

| User | Scenarios |
|---|---|
| Careful user | `careful-first-start`, `careful-status-and-messages`, `careful-settings-save`, `careful-other-network`, `careful-update`, `careful-update-other-chip`, `careful-teensy-update`, `careful-teensy-revert-subnet`, `careful-bridge` (a minute of steering through the bridge with corrections), `careful-bridge-network-numbers` (the XIAO's network on the module's numbers, 192.168.5.x, and back), `careful-bridge-modules` (a sprayer and a seed drill module on the XIAO's WiFi), `careful-bridge-page-while-steering`, `careful-module-page` (the module's page at port 8080 while the tablet steers), `careful-usb-and-restart` |
| XIAO never set up | `new-bridge-needs-password`, `new-no-teensy`, `new-gps2-port-not-set`, `new-older-teensy`, `new-wrong-password` (every change with a wrong password), `new-phones-join` (the phones' "is there internet?" checks), `module-page-default-password` |
| Wrong settings | `settings-bad-values`, `settings-special-characters`, `settings-other-network-wrong`, `settings-what-is-kept`, `settings-flash-nonsense`, `settings-forgotten-password` (`wifi-reset`), `settings-subnet-values`, `settings-bridge-router-same-numbers` (an other network at 192.168.5.1 too: left), `settings-bridge-module-subnet` (the module's subnet changed: the XIAO's network follows), `update-wrong-files` (a `.hex`, a cut or damaged `.bin`, the USB file), `update-header-byte-later`, `module-page-firmware` (an update or going back through port 8080: refused) |
| Wrong connections | `conn-two-tablets`, `conn-program-on-other-network`, `conn-forged-address`, `conn-cable-and-wifi`, `conn-not-a-teensy`, `conn-broken-wire`, `conn-teensy-restarts`, `conn-xiao-loses-power`, `conn-odd-packets`, `conn-search-while-steering`, `conn-forged-module`, `conn-modules-bridge-off`, `conn-five-modules` (there are places for four), `conn-module-subnet-and-scan`, `module-page-old-teensy`, `module-page-other-network`, `machine-tab-old-teensy` |
| Bad link | `link-phone-vanishes-mid-page`, `link-spare-connection`, `link-request-in-two-pieces`, `link-page-wait-other-network-away`, `link-request-never-finished`, `link-update-stalls`, `link-update-phone-vanishes`, `link-upload-drips-while-steering`, `link-upload-drips` (a page stuck on one phone for minutes), `link-update-tab-closed`, `link-serial-noise`, `link-teensy-update-lost-answer`, `link-teensy-refuses-update`, `link-hex-while-steering`, `link-wrong-password-big-file`, `link-own-update-while-steering`, `link-flood`, `link-wifi-takes-nothing`, `link-many-phones`, `link-odd-requests`, `link-sensor-and-clock` (the 49-day wrap), `link-restart-at-clock-wrap`, `link-module-goes-wrong` (3000 packets a second from one module while the tablet steers), `module-page-phone-leaves`, `module-page-noisy-link` (never a wrong byte; a cut answer is asked again) |

## The checks

- **must**: the XIAO must do this, safely and right. A failure is a **bug** and fails the run, unless
  `known-bugs.txt` lists it.
- **tell**: the user made a mistake, or something around the XIAO is wrong; does the XIAO say so where the user
  looks (its page, Messages, the Teensy's log)? A failure is reported as **not told** and never fails the run.
- **note**: numbers for the report (delays, gaps, what a page was told).

Every scenario ends with the rules that hold whatever the user does (`rules()` in `harness/helpers.h`); the bench
watches them every millisecond:

- **Nothing reaches the Teensy through the WiFi bridge that should not**: only with the bridge switched on (and a
  password of one's own), only to a Teensy that says it knows the bridge, only AgOpenGPS packets from a program on
  the XIAO's own network, byte for byte as sent and in order, and only the hello while the cable has the module.
- **The Teensy's data for the program goes to that program only**, never to everyone, exactly as the Teensy sent it.
  The same for the section modules: what goes to one is a packet of the program, once and in order, to that
  module's own address; what goes on from them is a packet one of them sent, to the program only.
- **Every answer to a browser is well formed**: HTTP, its length, valid JSON.
- **The XIAO restarts only when the user asked for it**, never by its watchdog (but for the two scenarios where
  the page is stuck on one phone for over a minute: there the watchdog is what must clear it, once).
- **The loop and the link task never give back a lock they do not hold.**
- **Its hello to the Teensy is never late** (every 2 s; the Teensy calls it gone after 10 s).
- **No data from the Teensy is lost** in the XIAO's receive buffer.

While a tablet steers through the bridge (`beginSteer()` … `endSteer()`), a second without steering data at the
Teensy is a bug too: the wheel would hold its last angle that long, and after 2.5 s the Teensy switches autosteer off.

A check's id is `<scenario>/<key>@<kit>` (`c3` or `s3`). `known-bugs.txt` takes `<scenario>/<key>` (both XIAOs) or
a full id, with the reason; the report says when a listed bug passes again, so its line can come out.

## What it found

The first run (22 Nov 2026) found four things in the program. The harness itself changes nothing in the program:
a fix is a pull request of its own, which takes its lines out of `known-bugs.txt`.

1. **The XIAO stood still while it waited for the rest of a request. Fixed 22 Nov 2026.** The core's web server
   waits inside `handleClient()` for a late piece of a request (up to 5 s for each piece of a form or line, 20 s
   for the next piece of an upload), and `loop()` did nothing else meanwhile: no WiFi bridge, no hello, the
   Teensy's data unread. With a tablet steering through the bridge, a phone's Save whose form arrived 3 s late
   stopped the steering data for those 3 s (the Teensy switches autosteer off after 2.5 s). And the 20 s wait of
   an upload was longer than the 15 s loop watchdog: an upload that stalled, or a phone that walked out of reach
   in the middle of one, restarted the XIAO by its watchdog instead of "Program update STOPPED".
   The fix: a second task (the link task) runs the link and the bridge while the loop is inside the core's web
   server, and the watchdog is the program's own entry, answered by that task while the web server waits for a
   browser (`link-request-in-two-pieces`, `link-request-never-finished`, `link-update-stalls`,
   `link-update-phone-vanishes`; `link-upload-drips-while-steering` and `link-upload-drips` were added with it).
   That fix had a mistake of its own, found the same day by reading the code and not by the harness: after such a
   wait the loop went on with the time it had read before it, older than the link task's time stamps, and in the
   one pass in some hundreds that looks at the WiFi the program seemed long gone: with an other network set and
   out of reach, the XIAO searched for it for half a second while the tablet steered. Fixed, with a scenario that
   times its requests to that pass (`link-page-wait-other-network-away`).
2. **A refused file for the Teensy still held the link. Fixed 22 Nov 2026.** `teensyUpdateUpload()` read a
   refused `.hex` to its end without `linkPoll()`: while the file came in there was no bridge and no hello. Sent
   while the tablet steers, the file was refused as it should be, and the steering data stopped all the same
   (`link-hex-while-steering`, `link-teensy-refuses-update`, `link-wrong-password-big-file`).
3. **The bridge goes by the address a packet claims.** Not fixed, in `known-bugs.txt`. A datagram that arrives on
   the other network (the farm's WiFi the XIAO joined) with a sender address of the XIAO's own network
   (192.168.4.x) is passed to the Teensy. It takes someone on the farm's WiFi who forges packets on purpose
   (`conn-forged-address`). The port the section modules send to (added 22 Nov 2026) goes by the claimed address
   the same way: such a sender is taken for a module and its packets go on to the program (`conn-forged-module`,
   in `known-bugs.txt` with the same fix to come).
4. **The chip check of the XIAO's own update looks at more than the first piece.** Not fixed, in
   `known-bugs.txt`. A good `.bin` with the byte E9 exactly 1436 or 2872 bytes in is refused as "other chip".
   Today's `.bin` files have text there, so it does not happen now (`update-header-byte-later`).

It also lists what the XIAO keeps quiet about (**not told**, in the report): a wrong password for the other
network (the page shows "connecting" for ever), a second program or one on the other network calling the module,
a network name with an accent refused as "1 to 32 characters", a subnet the Teensy refuses while the page says
"ok", a new try of a Teensy update in the 15 s the Teensy still holds on to the first one, and the file for the
USB cable accepted when it has another name.

## Adding a scenario

Put it in the `harness/user_*.cpp` of its user:

```cpp
SCENARIO(User::BadLink, "my-scenario", "what happens, in one line", oneKit())
{
    World w(kit);
    w.bridgeSaved();                    // the WiFi bridge was switched on earlier, with a password of one's own
    w.start();                          // power on, 3 s
    if (beginSteer(w))                  // a tablet on the XIAO's WiFi gets the module and steers
    {
        const Reply r = w.post("/api/restart", { { "pw", "trator-2026" } });
        must("refused", r.code == 409, "Restart is refused while the tablet steers", r.error());
        endSteer(w);                    // and no second went by without steering data
    }
    rules(w);                           // the rules that always hold
}
```

`harness/bench.h` lists what a scenario can set and read: the XIAO (`w.xiao`, `w.dev`), the Teensy (`w.teensy`),
the tablets (`w.tab`, `w.tab2`, `w.far` on the other network), the section modules (`w.mod`), browsers (`w.get`, `w.post`, `w.upload` with a
`Link`), the serial wires (`w.noise`, `w.wireCut`), the router (`w.dev.router`), the USB monitor (`w.usb`,
`w.said`), something to do at a time to come (`w.later`). A change to what the program or the Teensy sends over the link needs the pretend Teensy to follow
(`harness/teensy.cpp`), as `Tests/Teensy_AiO` follows it with its pretend XIAO.

## What it does not test

- The page's own JavaScript: what it checks before it sends (a chosen file's name, first byte and chip), what it
  shows. The simulator check opens the page; the harness talks to the XIAO as the page does.
- Two tasks truly running at once. The link task gets its turn only where pretend time moves, and a task that
  would have to wait for a lock is not played. That the loop and the link task are never in the program's own
  code together is the lock's work on the chip; the harness shows what each does with its turn, not a race.
  A mistake that needs a wait to fall in one particular pass of `loop()` (the old time of 22 Nov 2026) shows only
  in a scenario timed for it.
- The radio and the real chip: WiFi range, the delay and loss in the air, a search taking the radio off its
  channel, two networks with the same numbers, real timing, heap, the flash wearing out. The pretend core does
  what the real one's code does, nothing more; the waits of the pretend browsers are chosen numbers.
- The whole check of a program file: the real chip also checks the SHA-256 at its end; here the header, the parts
  and the one-byte checksum. What the real updater does with an empty file, and what the chip does with a file that
  starts with the bootloader, is not known here.
- The Teensy's own work (its harness: `Tests/Teensy_AiO`): the Teensy here is a pretend one that follows the same
  rules. The two real programs are not run against each other.
- The section modules' own programs (their harness: `Tests/ESP32_Modules`, with a pretend XIAO): the modules here
  are pretend ones. What a WiFi access point does with a packet "to everyone" (held for the beacon, not sent
  again when lost), which is why the XIAO sends to one address at a time, is not played here: on the pretend WiFi
  every packet arrives.
- A frame with the right header whose one-byte checksum matches by chance (once in 256): the link takes it as real.
