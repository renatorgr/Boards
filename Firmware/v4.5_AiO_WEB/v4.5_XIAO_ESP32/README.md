# v4.5_XIAO_ESP32

Program for a Seeed **XIAO ESP32C3** or **XIAO ESP32S3** on the GPS2 port of the **AiO v4.5 boards**.

- It goes with the Teensy firmware **v4.5_AiO_WEB**: the two must have the same version number.
- On the Teensy's web page, Settings → **GPS2 port** must be **automatic** (the default): the Teensy finds the XIAO by its hello. The same port takes the second F9P of a dual F9P, so it is one or the other.
- The XIAO creates its own WiFi network with a page for a phone: you can see what the module is doing in the cab or next to the tractor, without the tablet or a network cable.
- It only monitors: it does not change anything in the module or in AgOpenWeb / AgOpenGPS.

## Where it goes

- **AiO v4.5 Micro:** in the GPS2 socket ("Dual-Left"), through the adapter board or the 3D-printed holder.
- **AiO v4.5 Standard:** the adapter and the holder do not fit. Wire it to header P8: pin 1 (RX2) ← XIAO D6, pin 2 (TX2) → XIAO D7, with 3.3 V and GND from another header.
- The two XIAO boards have the same size and the same pins (3V3, GND, D6 = TX, D7 = RX), so the wiring, the adapter board and the holder are the same for both.
- Compile the program for the XIAO you have. It then calls itself `v4.5_XIAO_ESP32C3` or `v4.5_XIAO_ESP32S3`, and the page refuses an update made for the other one.
- On the board the XIAO is powered from the board's 3.3 V: **never plug a USB cable with 5 V into it while it is on the board** (see "USB-C port on the panel").

The Teensy side of the link (pins, frames, PGN 141 / 142 / 143, speed) is described in the README of the Teensy firmware.

## Changelog

Version 4.5, the same number as the Teensy firmware it goes with (v4.5_AiO_WEB). One row per day, in short; the details are in the sections of this file (the links). Older changes are in the git history.

| Date | What changed |
|---|---|
| 22 Nov 2026 | [WiFi bridge](#wifi-bridge): a tablet with no network cable reaches the module through the XIAO, on the [module's numbers](#the-xiaos-network-numbers), with [section modules](#section-modules-on-the-xiaos-wifi) (also in the Teensy's Machine tab) and [the module's page](#the-modules-page-through-the-xiao). `.bin` rebuilt. |
| 21 Nov 2026 | Its timers no longer start a millisecond ahead: a hello to the Teensy could go out twice ([What it does](#what-it-does)). The `.bin` files are rebuilt. |
| 13 Nov 2026 | Loading the page no longer stops the XIAO on weak WiFi ("XIAO silent for 10 s"). Enter in a password field does what its button does. Firmware tab: one card per device, one box for each install ([What it does](#what-it-does)). |
| 11 Nov 2026 | The page checks that it is the page of the program running: after an update an old page a browser kept (old buttons, new data) fetches the new one by itself ([What it does](#what-it-does)). |
| 10 Nov 2026 | Tabs and the light/dark button 44 px high with 8 px corners; the WiFi tab's password to confirm in a card of its own, no longer a field across the page ([What it does](#what-it-does)). Messages give the size of an updated program, not 0 bytes. |
| 9 Nov 2026 | The page's questions (save and restart, install, the module's firmware, subnet) in its own box: the browser's pop-ups show nothing inside an AgOpenWeb web card, so those buttons did nothing there ([What it does](#what-it-does)). |
| 7 Nov 2026 | Add-on card (`_aowcard._tcp`) under the XIAO's own WiFi name ([What it does](#what-it-does)). Program → module shows up to 64 sections (PGN 229); the WiFi settings save in one row; a shorter note in the Module firmware card; short file headers. |
| 4 Nov 2026 | The XIAO's temperature in its card, here and on the Teensy page; the Teensy's clock and temperature in the [Module card](#what-it-does). |
| 3 Nov 2026 | An [update over Wi-Fi](#update-over-wifi-from-a-file) on a weak link no longer stops after a 5 s pause (it waits up to 20 s), and an upload that is cut off is written in Messages. |
| 2 Nov 2026 | [Module (Teensy) firmware](#the-modules-firmware-from-this-page) card on the Firmware tab: install the Teensy's firmware from this page, or go back to its previous one (needs a Teensy firmware of 2 Nov 2026 or later). Red / green messages under the cards. |
| 1 Nov 2026 | Header and tabs stay at the top while the page scrolls. Report card last on the Firmware tab. Plainer **On GitHub** row. |
| 31 Oct 2026 | GPS fix green whenever there is a position. **Download the report** on the Firmware tab. [Install from GitHub](#update-from-github) also sees a new build of the same version. |
| 24 Oct 2026 | The module's subnet changed from this page (WiFi tab). The module's page and this page as links at their real addresses. |
| 20 Oct 2026 | English only, light mode, no repeated rows. The page opens on the farm network under any name. A warning when the Teensy runs another version. |
| 19 Oct 2026 | **Version 4.5**: its own WiFi network and [page](#what-it-does) (module status and messages, what the program sends, WiFi settings, update over WiFi and from GitHub), on the XIAO ESP32C3 and ESP32S3. Link to the Teensy at 460800. |

## What it does

- **Its own WiFi network**, always on: `XIAO-xxxx` (the last 4 characters come from the XIAO's own serial number), password **`agopengps`** until you change it.
  - A phone that joins it opens the page by itself (like a hotel WiFi page). If not, open **http://192.168.4.1/** (with the [WiFi bridge](#wifi-bridge) on, the module's numbers: normally **http://192.168.5.1/**).
  - Up to 4 phones at the same time.
- **Another WiFi network too** (optional): the farm WiFi or a phone hotspot. Then the page is also at **http://xiao.local/** (the name can be changed) or at the IP shown on the page.
  - The XIAO has one radio: its own network moves to the other network's channel when it joins it, so a phone on the XIAO network may lose the connection for a few seconds.
  - `xiao.local` works on iPhone, Mac, Windows 10/11 and Linux; on many Android phones use the IP instead.
  - The XIAO announces its page by mDNS, so AgOpenWeb's web cards (⚙ › **Search the network**) list it in a second when the AgOpenWeb computer is on that network, under the name of its own WiFi network (`XIAO-xxxx`). It is also announced as an AgOpenWeb add-on card (`_aowcard._tcp`, TXT `v=1`), the official format, so the official AgOpenWeb will list it too when its add-on card list ships.
- **The page**, in English, dark or light:
  - the button at the top (☀ / ☾) switches it, and the phone remembers the choice;
  - without a choice, the phone's setting (dark or light mode) decides;
  - when the XIAO restarts (its uptime goes back, or another build runs), the page loads itself again, on the same tab.
  - it goes out without holding up the XIAO: on a weak WiFi link the 53 kB page can take a few seconds, but the link to the Teensy (hello, data) keeps going meanwhile. Before 13 Nov 2026 the XIAO waited for the browser, and the Teensy log showed "XIAO silent for 10 s";
  - the browser keeps the page and asks the XIAO each time whether it changed: the same build answers "not modified" (a few hundred bytes), so only the first visit after an update sends the whole page;
  - a spare connection a browser opens and leaves silent is dropped after 1 s when another one is waiting (it used to hold up the next request for 5 s).

| Tab | What it shows |
|---|---|
| **Status** | **Module (Teensy)**, sent by the Teensy once a second: GNSS (fix, satellites, age of corrections), dual antenna heading, IMU (TM171 or BNO085), the GPS receiver's model, steering, network, the Teensy's clock and temperature. The fix is green whenever the receiver has a position (GPS, DGPS, RTK float, RTK fixed: the name says which) and red only without one. |
| | **Program → module** (named AgOpenWeb or AgOpenGPS when the Teensy knows which program sends the hello): speed, requested steer angle, autosteer on / off, sections (one box each: 16, or up to 64 with PGN 229), hydraulic lift (up / down), tramline, PGN 254 per second. |
| | **XIAO**: uptime, start-up reason, WiFi, the [WiFi bridge](#wifi-bridge), the link to the Teensy, memory, the chip's temperature (its version is in the header and the Firmware tab; the requested steer angle is in Program → module). |
| **Messages** | The module's `[status]` messages (the same as the Messages tab of the Teensy page, last 60), and the XIAO's own events (WiFi, phones joining, updates). |
| **WiFi** | The XIAO's network name and password, and the [WiFi bridge](#wifi-bridge)'s switch. |
| | The other network: a scan lists the networks found, one per line, strongest first; tap one to choose it. |
| | The name on the network (`xiao` → `xiao.local`). |
| | Saved in the XIAO; it restarts to apply them. |
| | **Module network**: the module's (Teensy's) Ethernet IP and its subnet. For a module on a network that is not 192.168.5.x: type that network's first three numbers (and the XIAO password); the Teensy saves it and restarts with the new IP. This page is on WiFi, so it works whatever the module's IP is. |
| **Firmware** | One card per device, the same layout. **XIAO firmware**: installed, on GitHub (tagged *newer*, *installed* or *older*), **Install from GitHub**, **Install from a file…** (a `.bin`, see below), the progress, **Restart XIAO** at the bottom. **Module (Teensy) firmware**: the module's firmware installed from this page over the link, and its previous firmware put back (see below). Each install and the restart are confirmed in one box that says what the file holds against what runs now and asks the XIAO's network password. **Download the report**: the XIAO's state and settings (no passwords), the module status and both message lists in one `.json` file, to send when something goes wrong. |

- Changes (WiFi, update, restart) **ask for the password of the XIAO's own network**, to confirm.
  - The question before each one (save and restart, install, the module's firmware, the subnet) opens in a box of the page itself, not a browser pop-up: inside an AgOpenWeb web card the browser's pop-ups show nothing, so those buttons did nothing there.
  - **Enter** in a password field does what its button does, and the question still comes first. On the Firmware tab the password is typed in the box that confirms the action, where Enter is its **Install** or **Restart**. An empty field does nothing, and a held Enter does not also answer the box it opened.
  - The box's buttons stay in sight at its bottom when it is taller than the screen (an AgOpenWeb web card of 480 x 320): the rest scrolls.
  - The tabs and the light/dark button at the top are 44 px high, with the same 8 px corners as that box's buttons: a size a finger hits on the cab's screen.
  - On the WiFi tab the password and **Save and restart the XIAO** have a card of their own, *Confirm and save*, as wide as the other two.
  - The page carries the date its own program was compiled and compares it with the one the XIAO reports. When they differ (an old page the browser kept after an update), it fetches the page again at an address the browser has never seen (`/?b=<build date>`); if that page is still old, a notice says to reload it by hand.
  - Typed once, in the WiFi tab or in a Firmware box, it is offered again until the page is closed.
  - The page never shows or stores it.
  - Just viewing the page needs nothing more than being on the network.
- It tells the Teensy its version and WiFi state: they show in the **XIAO** card of the Teensy page (normally http://192.168.5.126/; the module's IP is shown on the Status tab). Its events also go to the Teensy log as `[xiao] > ...`. From the WiFi tab it can set the module's subnet (PGN 144 to the Teensy).
- **Watchdog**: if the program ever stops for 15 s, the XIAO restarts by itself.
- The Teensy sends it the program's packets and the module status; the XIAO sends back only its hello, its state and text lines (and, with the [WiFi bridge](#wifi-bridge) on, the packets of a program on its own network).
  - Its timers start with the time itself (`msStamp`, never 0). Until 21 Nov 2026 they started one millisecond ahead half the time: a hello could go out twice in that millisecond, and "heard from the Teensy lately" read no for it.

## WiFi bridge

For a tablet with **no network cable to the module** (no USB-Ethernet adapter): the tablet joins the XIAO's own network and AgOpenWeb (or AgOpenGPS / AgIO) reaches the module through the XIAO. Off from new. With a network cable to the tablet, leave it off: the cable is the better link.

Why the XIAO's own network, and not the farm's or the tractor router's WiFi: on a router's WiFi the module's packets are broadcasts, which the router holds for its beacon. Measured on a tractor (22 Nov 2026): 8 at a time every 0.2 s, each 1.5 s late, and half of them lost. The XIAO sends every datagram to the tablet's own address, which is not held (a ping over that link: 3 ms).

**Switching it on**

1. WiFi tab: give the XIAO's network **a password of your own**. Whoever is on this network can then steer the tractor, so the bridge is refused with the password every XIAO starts with (`agopengps`), and `wifi-reset` switches it off again.
2. Tick **WiFi bridge**, type the current password, **Save and restart the XIAO**. The page says what changes: the XIAO's network moves to the [module's numbers](#the-xiaos-network-numbers) (192.168.5.x), and its page to http://192.168.5.1/.
3. The Teensy needs the firmware of 22 Nov 2026 or later. The Status tab's **WiFi bridge** row says when the Teensy does not answer it.
4. On the tablet: join the XIAO's network again and start the program. AgOpenWeb should find the module by itself (it shows at 192.168.5.1); AgOpenGPS (AgIO) keeps its subnet, 192.168.5, as on the network cable.

**The rules**

| Rule | Why |
|---|---|
| A program on the module's **network cable comes first** | One program has the module at a time. While one talks on the cable (and for 3 s after) the one on the WiFi is not answered; only its hello goes to the Teensy, which says in Messages that it is calling. |
| **One way at a time** | While the program on the WiFi has the module, nothing for the program goes out on the network cable: a tablet on both would get everything twice, from two addresses. |
| **One program** on the WiFi | The first one heard, until it has been silent for 3 s. |
| Only from the XIAO's **own network** (the module's numbers, above) | Never from the other network the XIAO joined. |
| The module's **subnet (PGN 201) and scan (PGN 202) are not done** | They are for the module's network cable, and a subnet change would restart the module under the tablet. Messages says so. |
| While the program has the module, **no search for networks** | The XIAO has one radio: a search (for the other network when it is out of reach, or the WiFi tab's button) takes it to every channel in turn, away from the steering data. The search for the other network starts again 10 s after the program left; a network already joined stays joined. |
| With **autosteer on** over the bridge, the page refuses a save, an update (the XIAO's or the module's) and a restart | Each would stop the steering in the middle of the field. Switch autosteer off first. |

**When the link stops** (the XIAO restarts or loses power, the tablet goes out of range): the Teensy gets no steering data and its own watchdog turns the outputs off, as with a pulled network cable (within 2.5 s). After 3 s of silence the module is free again and its GPS goes out on the network cable as before.

**Limits**

- The XIAO's network has **no internet**. The corrections (NTRIP) must come through the tablet's own mobile data while it is joined to this network. Not tested yet: it depends on the tablet keeping mobile data in use on a WiFi with no internet.
- The program reaches this module (GPS, IMU, steering, the Teensy's own machine outputs) and the [section modules on the XIAO's WiFi](#section-modules-on-the-xiaos-wifi). A section-control module on the **network cable** (sprayer, seed drill) is not reached through the bridge.
- Datagrams of up to 251 bytes (every AgOpenGPS PGN is shorter); a longer one is dropped and counted.
- The range is the XIAO's: in an aluminium box use the external antenna.

**Where it shows:** Status → XIAO → **WiFi bridge**: off, on with no program, standing by (a program is on the cable), or the program's address with the packets to the module, to the program, and those lost; under it **Other modules on this network**. **Program → module** shows the program's data as before. The Teensy page has the same row in its XIAO card, and "on the XIAO's WiFi" beside the program's IP. Both write each change in Messages.

**How it travels** (PGN 146-149 on the link, never on the network): the XIAO listens on UDP 8888 and 2233 of its own network (and on 9999 for the [section modules](#section-modules-on-the-xiaos-wifi)), hands each datagram to the Teensy (PGN 146 with the sender's address, PGN 147 for the corrections) and sends what the Teensy has for the program (PGN 148) to that program's UDP 9999. The Teensy says once a second who has the module (PGN 149); without that, from an older Teensy firmware, the XIAO hands nothing over. The XIAO never waits for the link: with its transmit buffer full a datagram is dropped and counted.

## The XIAO's network numbers

With the bridge on, the XIAO's own network takes the module's numbers: **192.168.5.x** by definition (the XIAO at 192.168.5.1, the devices that join it .2 to .12), the first three numbers of the Teensy's IP. Without the bridge it stays at 192.168.4.x, as every ESP32's own network.

- The program keeps the subnet it has on the network cable, and a [sprayer or seed drill module](#section-modules-on-the-xiaos-wifi) joins this network with its own unchanged: its page is at its usual address (http://192.168.5.123/).
- The XIAO reads the module's numbers in the Teensy's status and keeps them. When they change (WiFi tab, **Module network**, or the module's own page) the XIAO restarts and its network follows; Messages says so. Devices on it join again.
- An **other network** with the same numbers (a tractor's router at 192.168.5.1) is left as soon as the XIAO gets its address there, and not joined again while the bridge is on: with two networks of the same numbers the XIAO could send the program's packets out on the wrong one. Status says "left: it has this network's numbers".

## The module's page through the XIAO

The module's (Teensy's) own page is on its network cable. A phone or a tablet on the XIAO's own WiFi opens it at **http://192.168.5.1:8080/** with the bridge on (http://192.168.4.1:8080/ without it): the XIAO's address, port **8080**. It is the module's own page, every tab, as on the cable; the Status tab's module card has the link. The XIAO passes each request to the Teensy over the link (PGN 150) and the answer back (PGN 151). The page takes about 4 s the first time (136 kB at the link's 46 kB/s), then the browser keeps it.

- Only from the XIAO's own WiFi, and only once that network has a **password of one's own** (as the bridge): whoever opens the module's page can change how the tractor steers. Otherwise the browser gets a short page that says why.
- With little free memory (under 32 kB) the page **waits**: no new request, no answer bytes sent on, until the memory is back (Messages says so, once a minute at most). On a busy WiFi a C3 fell to 2 kB free while this page loaded (bench, 22 Nov 2026); out of memory the XIAO would restart, and the bridge with it.
- The module's firmware update and going back to its previous firmware are **not** done there: use this page's Firmware tab, [The module's firmware from this page](#the-modules-firmware-from-this-page).
- It needs a Teensy firmware that knows it (22 Nov 2026, "the page through the XIAO"): with an older one the browser is told to install the newer firmware first. Nothing is sent to an older Teensy.
- One request at a time, as the module serves them; the steering through the bridge comes first on the link, both ways. A phone that leaves half way is given up after 15 s at most, and the next one is served.
- The section modules' pages (sprayer, seed drill) need none of this: they open at their own address on the XIAO's WiFi (http://192.168.5.123/).

## Section modules on the XIAO's WiFi

For a tractor with **no router**: the tablet is on the XIAO's network (the [WiFi bridge](#wifi-bridge)) and a sprayer or seed drill module joins that network too (`Sprayer_ESP32` 0.10.15, `Seeder_ESP32` 0.2.15, or later). Since 22 Nov 2026 the XIAO passes each packet of the program on to every such module, and each packet of a module on to the program.

**Setting it up**

1. Switch the WiFi bridge on, as above.
2. On the module's own page, Module tab → *Network settings*: tick *Join the tractor network* and give it the XIAO's network name and password. Leave the *Program subnet* as it is: the module's numbers (192.168.5), which the XIAO's network has with the bridge on ([The XIAO's network numbers](#the-xiaos-network-numbers)). Save and restart the module.
3. From the tablet, on the XIAO's network, the module's page is at its usual address (`http://192.168.5.123/` for the sprayer).

A module set to the subnet **192.168.4** for the first version of this (XIAO `.bin` of 22 Nov 2026, 16:03 UTC): set it back to 192.168.5.

**How it works**

| What | Why |
|---|---|
| The XIAO knows a module from the packets it sends by itself (five a second to UDP 9999, program or not) and forgets it 3 s after the last | Nothing to set on the XIAO. Up to 4 modules; a fifth is named in Messages and not served. |
| Every packet goes to **one address**: the program's to each module, a module's to the program | A packet "to everyone" waits at a WiFi access point for the next beacon and is not sent again when lost: sections would switch late. |
| The module answers the XIAO only, and leaves alone the program's own packets "to everyone" that arrive beside the XIAO's copies | They are the same packets, later: an old section state after a newer one would switch a relay back for a moment. The module's page says "through the access point" and counts them. |
| Not passed on to the modules: the subnet (PGN 201), the scan (PGN 202), the steering's own (PGN 254, 252, 251) | A subnet change would take the module off this network; the steering data, a hundred packets a second, is of no use to it. |
| Of one module's packets, 60 a second at most go on to the program (ours send about 8) | A module gone wrong must not fill the WiFi that carries the steering. Messages says so once. |
| The XIAO's network takes 8 devices (4 before) | The tablet, the modules, a phone or two on the pages. |

**When it stops:** with the XIAO's copies gone (the XIAO restarts, or the Teensy stops answering the bridge) the module closes every section within 2 s, as with any lost link. A second later it listens to the program's own packets again and answers everyone, which works with delays until the XIAO passes on again.

**Limits**

- Nothing is passed on while the Teensy does not answer the bridge (an older Teensy firmware, or the Teensy silent): the XIAO takes no program then.
- A program on the Teensy's **network cable** does not reach a module on the XIAO's WiFi.
- The Teensy page's **Machine tab** shows it: the XIAO passes the module's own packets to the Teensy too (PGN 152: its address and the packet; only the hello reply and PGN 237, 160, 161, and only when the link has room), with a program or without. Needs a Teensy firmware that says bridge version 3 (22 Nov 2026 evening); an older one gets none of them.
- The program's scan for modules does not list it; it answers the hello like any machine module, so the program shows it as connected.
- **Not yet tried with real modules**: see [What was tested](#what-was-tested).

**Where it shows:** Status → XIAO → **Other modules on this network**: each module's address with the packets sent to it and passed on from it. Messages says when a module comes, goes silent, or sends too much. On the module's page: Module tab, the *Program* row.

## A slow browser

A phone on a weak WiFi link can take seconds to send a request, or stop in the middle of an upload. The XIAO's web server waits for the rest, and until 22 Nov 2026 everything else waited with it: the hello to the Teensy, its data, and the [WiFi bridge](#wifi-bridge). A tablet steering through the bridge lost the module for that long, and the Teensy switched autosteer off after 2.5 s (found by the harness, `Tests/XIAO_ESP32`).

Since 22 Nov 2026 a second task (the link task) keeps the link to the Teensy and the WiFi bridge going while the web server waits for a browser:

- A request that arrives late or in pieces, an upload that pauses, a file that is refused but still read to its end: the bridge and the hello go on.
- An upload may pause for up to 20 s. Before, the XIAO's 15 s watchdog restarted it in such a pause.
- The page still answers one browser at a time: while it waits for one, the others wait too.
- A page stuck on one browser for over a minute (bytes that keep coming and never make a whole piece of a file) ends in a restart by the watchdog, so the page answers the others again. **Not while a tablet steers through the bridge**: the restart would take the WiFi away from it, so it waits until autosteer is off.

One mistake in the first version of it (found the same day, by reading the code): the loop went on with the time it had read before the wait, which is older than the link task's notes of "the program was here". In the one pass in some hundreds that also looks at the WiFi, the program then seemed long gone, and the XIAO searched for the other network for half a second with the tablet steering (only with an other network set and out of reach). The time is now read after the web server.

Not yet tried on a real XIAO. In the harness the steering packets kept passing 10 ms apart while the page waited two minutes for an upload that came a few bytes at a time.

## Install the first time (USB-C, off the board)

**Without the Arduino IDE:** the firmware folder has the program already compiled for both XIAOs (`v4.5_XIAO_ESP32_bin`, see its README).

- `v4.5_XIAO_ESP32C3_USB.bin` / `v4.5_XIAO_ESP32S3_USB.bin`: the first install over USB, from the browser (https://espressif.github.io/esptool-js/, address 0x0).
- `v4.5_XIAO_ESP32C3.bin` / `v4.5_XIAO_ESP32S3.bin`: for the update over WiFi.

**With the Arduino IDE:**

1. Arduino IDE 2: File > Preferences > Additional boards manager URLs:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Tools > Board > Boards Manager: install **esp32 by Espressif Systems** (3.x; tested with 3.0.7).
3. Open `v4.5_XIAO_ESP32.ino` (the 4 files must stay in the folder `v4.5_XIAO_ESP32`).
4. Tools:

| Setting | XIAO ESP32C3 | XIAO ESP32S3 |
|---|---|---|
| Board | **XIAO_ESP32C3** | **XIAO_ESP32S3** |
| USB CDC On Boot | **Enabled** | **Enabled** |
| USB Mode | (not shown) | Hardware CDC and JTAG |
| Partition Scheme | Default 4MB with spiffs | Default with spiffs (3MB APP/1.5MB SPIFFS) |
| PSRAM | (not shown) | not needed (Disabled is fine) |

   - If USB CDC On Boot is not enabled, or the board is another ESP32, compiling stops with a message saying so.
   - The default partition schemes have room for updates over WiFi.
5. XIAO **off the board**, USB-C to the PC, choose its port, Upload.
   - If the port does not show up: hold BOOT, press and release RESET, release BOOT, then Upload.
6. Serial Monitor (any speed): it prints the network name and the commands.

No other libraries are needed: everything is in the esp32 package.

## Update from GitHub

- The page looks for the newest version in the `Firmware` folder of **github.com/renatorgr/Boards**: when it opens, every 6 hours while it stays open, and with **Check now** (Firmware tab).
- A newer one shows at the top of Status and in Firmware. **A new build of the same version counts as new**: the page compares when the `.bin` for this XIAO's chip (C3 or S3) on GitHub was compiled with this XIAO's **Built**. That date is read from the file each time it reaches `main` and published with the simulator (`https://renatorgr.github.io/Boards/firmware.json`); without it, the page compares the version number only.
- **Install from GitHub** (a box shows the version on GitHub against the one running, with the password of the XIAO's network):
  - the browser downloads the `.bin` for this XIAO (in the version folder `vX.Y_AiO_WEB`: `vX.Y_XIAO_ESP32_bin/vX.Y_XIAO_ESP32C3.bin` or `...S3.bin`, chosen from the chip it reports);
  - it checks that the file is for this chip and that its size matches GitHub;
  - it sends it to the XIAO like a chosen file.
- **The phone (or PC) needs internet.** On the XIAO's own network alone it usually has none: connect the XIAO to the farm network or to the phone's hotspot ("Other network" in WiFi), and open the page over that network (`http://xiao.local/` or the IP shown).
- Without internet the page says "no answer from GitHub" and nothing else changes.

## Update over WiFi (from a file)

1. Arduino IDE: Sketch > **Export Compiled Binary**. It writes a `build` folder inside the sketch folder.
2. Use **`v4.5_XIAO_ESP32.ino.bin`** (or `vX.Y_XIAO_ESP32C3.bin` / `S3.bin` from GitHub). Not the files with `merged`, `bootloader`, `partitions` or `_USB` in the name: those are for the first install over USB-C, and the page refuses them.
   - A `.bin` compiled for the XIAO ESP32C3 does not run on the ESP32S3, and the other way round. The page reads the start of the file and says so before sending it.
   - With both XIAOs, rename the files after exporting (for example `…_C3.bin` and `…_S3.bin`).
3. Phone or PC on the XIAO's network → page → **Firmware** → **Install from a file…** → choose the file. A box says what is inside (version, build date) against what runs now → password → **Install**.
   - A `.bin` without the XIAO firmware's name inside (another program) is shown with a warning, and **Install** waits for **Install it anyway**: this page goes with it, so the next update would be over USB-C.
4. It takes less than a minute and the XIAO restarts with the new program. The WiFi settings are kept.

If the update fails halfway (WiFi lost), the old program keeps running: just try again. Messages then shows "Program update STOPPED after … kB: the browser's connection dropped". With a weak signal, connect the phone or PC to the XIAO's own network and install from http://192.168.4.1/: download the `.bin` first (Install from GitHub needs internet, and the XIAO's own network has none).

## The module's firmware from this page

For when the module's (Teensy's) own page does not open: its network cable or network, a fault in the page, the box closed, while this XIAO still answers on WiFi. Firmware tab → **Module (Teensy) firmware**.

- The card shows the firmware the module runs and when it was built, the `.hex` on GitHub (newer, the same, or older than the module's), and the **previous firmware** the Teensy keeps (the one that ran before its last update; "none kept yet" until the module's first update with a firmware of 2 Nov 2026 or later).
- **Install from GitHub** downloads the Teensy `.hex` of the newest version folder in the browser (the phone needs internet, as for the XIAO's own update), **Install from a file…** takes a file from the phone or PC. Either way one box says what is inside (version, build date, whether it is a Teensy 4.1 AiO_WEB firmware) against what the module runs, and asks the network password; **Install** sends it to the XIAO, and the XIAO passes it on to the Teensy over the serial link in pieces, each one confirmed by the Teensy before the next: about a minute for the 2 MB file. The progress bar is the file leaving the phone; then the Teensy checks the whole file as it does one from its own page, installs it and restarts.
- **Go back to the previous firmware** (at the bottom of the card, confirmed in the same box) asks the Teensy to put its kept copy back: no file and no internet needed. The copy is used up by it.
- Only with **autosteer off and the tractor stopped**: the Teensy refuses otherwise, and keeps autosteer off while it receives. If anything fails the module's current firmware keeps running and the card says why; a Teensy that does not answer (an older firmware, or the link down) is reported after a few seconds.
- The card then waits for the module to come back (its time on starts again, or another build runs) and says which firmware it runs. The module's own messages are on the Messages tab.
- A `.hex` without the update page inside (another firmware for the Teensy 4.1) waits in the box for **Install it anyway**; the next update is then by USB.
- Not yet tried on a board (2 Nov 2026): try it first on the bench with the box open and the USB cable at hand.

## USB serial commands

Serial Monitor at any speed, line ending "Newline" (or any):

| Command | What it does |
|---|---|
| `status` | version, WiFi, phones connected, Teensy link counters |
| `wifi-reset` | WiFi settings back to the defaults (`XIAO-xxxx` / `agopengps`, other network off) and restart: for a **forgotten password** |
| `restart` | restart |

## USB-C port on the panel

To set up or recover the XIAO without opening the box: a panel-mount USB-C socket with a short cable (right-angle USB-C plug) into the XIAO.

- Wires: **D+ → D+, D− → D−, GND → GND**. Same name to same name: they are not crossed.
- **Do not connect VBUS (5 V)**: on the board the XIAO is powered by the board's 3.3 V.
  - Without VBUS the XIAO's USB still works while the board is on.
  - With the board off the XIAO has no power: the panel port then does nothing.
- The panel socket needs the two **5.1 kΩ resistors from CC1 and CC2 to GND** (most panel sockets with a board have them), or a C-to-C cable from the PC will not work. A USB-A to USB-C cable works without them.
- Hold the cable so it does not pull on the XIAO's socket.
- The XIAO's USB data wires reach only its USB-C socket: they are not on its pins or on the pads underneath.

## XIAO ESP32S3: what is different

- Faster (two cores, 8 MB flash, 8 MB PSRAM). This program does not need it, but there is more room to grow (a WiFi bridge for the tablet, for example).
- D6 / D7 are GPIO43 / GPIO44 (on the C3, GPIO21 / GPIO20). Same pads, so nothing changes in the wiring.
- Seeed quotes about 100 mA with WiFi on, and an operating range of −20 to 65 °C. Mount the box where it does not sit in the sun.
- The antenna socket (U.FL) is at the other end from the USB-C, next to D6. Route the D6 wire so it does not press on the antenna plug.
- Underneath it has more gold pads and a large thermal pad.
  - On the 3D holder they sit on plastic.
  - On the adapter board they sit on solder mask, and there is no via or track under them.
  - Solder only the pads at the edge, as with the C3.
- If it has a small black connector underneath (the "Sense" version, for the camera board), it does not sit flat on the adapter or in the holder: the holder then needs a hole for it.

## The other files

| File | What it has |
|---|---|
| `v4.5_XIAO_ESP32.ino` | WiFi, web server, settings, update, USB commands, setup and loop |
| `xLink.h` | the link to the Teensy (460800 baud, frames, status, log, the firmware pieces and the Teensy's answers, PGN 145). Plain C++ so it can be tested on a PC |
| `xPage.h` | the web page (HTML, CSS and JavaScript in one text) |
| `xVersion.h` | the version number and the board (C3 or S3, from the board chosen in the Arduino IDE). When the number changes, the folder and the `.ino` get the new name too |
| `COPYING` | the licence: GNU GPL v3 |

`LINK_BAUD` here (`xLink.h`) must be the same as `XIAO_BAUD` in the Teensy firmware (`zXiao.h`).

## What was tested

- It compiles with esp32 3.0.7 with no warnings:
  - XIAO_ESP32C3: 1,049,140 bytes (80 % of the space for a program with WiFi updates);
  - XIAO_ESP32S3: 1,002,413 bytes (29 %).
- On a PC: `xLink.h` talking to the Teensy's `zXiao.ino` through a simulated 460800 line.
  - 60 s of AgOpenWeb at 100 Hz with 64 sections, plus the status every second: nothing dropped, every status whole, the log lines in order, a Teensy restart detected.
  - `xLink.h` was also fed random bytes with memory checks on.
- The page was checked in a browser at phone and PC size with test data.
- **Not yet tested on a real XIAO.** The first time, watch the Serial Monitor and the Messages tab.
- The [WiFi bridge](#wifi-bridge) (22 Nov 2026):
  - it compiles with esp32 3.3.12 with no warnings in the program: XIAO_ESP32C3 1,167,723 bytes (89 %), XIAO_ESP32S3 1,053,300 bytes (31 %);
  - the Teensy's side runs in the Teensy harness (`Tests/Teensy_AiO`) with a pretend XIAO and a pretend tablet on its WiFi: steering 40 s on a line through the bridge on the four kits, 100 steering packets a second with nothing dropped, the cable's program first, the XIAO or the tablet gone while steering (outputs off within 2.5 s);
  - the pages were opened in the simulator's browser check;
  - **not yet tried on a real XIAO, tablet and tractor**: the WiFi itself, the tablet's mobile data for the corrections, and AgOpenWeb / AgOpenGPS finding the module on this network are what the first bench test has to show.
- The harness (22 Nov 2026, `Tests/XIAO_ESP32`, the "XIAO harness" check on GitHub): this program itself runs on a PC on a pretend XIAO, with a pretend Teensy on its link, phones on its page and a tablet steering through the bridge.
  - Five kinds of user (careful, never set up, wrong settings, wrong connections, a bad link), 55 scenarios on the ESP32C3 and the ESP32S3, with memory checks on.
  - What it found is in its README: four things in this program. The two that matter in the field are fixed ([A slow browser](#a-slow-browser)); the other two are in its `known-bugs.txt`.
- The link task (22 Nov 2026, [A slow browser](#a-slow-browser)):
  - it compiles with esp32 3.3.12 with no warnings in the program: XIAO_ESP32C3 1,170,649 bytes (89 %), XIAO_ESP32S3 1,055,772 bytes (31 %);
  - in the harness (58 scenarios now): a Save that arrives 3 s late, an upload that pauses 18 s, a phone that walks away in the middle of one, a refused `.hex` of 600 kB, an upload that drips in for two minutes, all with a tablet steering through the bridge or the Teensy watching for the hello: nothing held up, no restart by the watchdog but the one after a minute stuck; and requests timed to wait in the very pass that looks at the WiFi, with the other network out of reach: no search for it;
  - the harness plays the second task one turn at a time: two tasks truly running at once, and the timing on the chip, are **not yet tried on a real XIAO**.
- The [section modules on the XIAO's WiFi](#section-modules-on-the-xiaos-wifi) (22 Nov 2026):
  - it compiles with esp32 3.3.12 with no warnings in the program: XIAO_ESP32C3 1,172,471 bytes (89 %), XIAO_ESP32S3 1,057,536 bytes (31 %);
  - in the harness (64 scenarios now) with pretend modules: a sprayer and a seed drill module served while the tablet steers (each packet of the program to each of them once and in order, their packets to the program only, no gap in the steering data); five modules (four served, the fifth named in Messages); the bridge off; the program's scan and subnet, not passed on; a module that sends 3000 packets a second and then drops off the network (the steering goes on, 60 a second pass, Messages says so once);
  - the two real module programs run in their own harness (`Tests/ESP32_Modules`) with a pretend XIAO: late and lost packets beside the XIAO's copies, the bridge stopping, a power cycle;
  - **not yet tried with a real XIAO, tablet and modules**: that the modules join the XIAO's network and stay on it, the delays in the air, and the program showing the module are what the first bench test has to show.
- [The XIAO's network numbers](#the-xiaos-network-numbers) (22 Nov 2026, after the first bench test):
  - it compiles with esp32 3.3.12 with no warnings in the program: XIAO_ESP32C3 1,175,103 bytes (89 %), XIAO_ESP32S3 1,060,044 bytes (31 %);
  - in the harness (67 scenarios now, 78 runs, about 1010 checks): the bridge switched on and off (the page moves between 192.168.4.1 and 192.168.5.1, the tablet steers on 192.168.5.x), a module on another subnet (the XIAO follows it after a restart, not while a tablet steers), an other network with the same numbers (left, said on the page and in Messages); the modules' harness keeps the module on subnet 192.168.5;
  - **not yet tried on a real XIAO**.

## Credits and licence

Written in 2026 by Claude (Anthropic) for renatorgr, to go with the Teensy firmware v4.5_AiO_WEB.

- The link to the Teensy uses the same protocol as the ESP32 bridge of the official AgOpenGPS Teensy firmware v26 "AiO New Dawn" (AgOpenGPS-Official).
- Built with the Arduino ESP32 core (Espressif) and its libraries.

**Licence:** GNU General Public License v3 or later (file `COPYING`), WITHOUT ANY WARRANTY.
