# The XIAO program, already compiled (v4.5)

The same program (`v4.5_XIAO_ESP32`, for the Teensy firmware v4.5_AiO_WEB), compiled for both XIAO boards.

The ESP32C3 and the ESP32S3 are different chips: each needs its own file.

**Built:** 31 Oct 2026, from the program as it is after pull request #77 (green GPS with any fix, Download the report, the Teensy module's address as a link).
When the program in `v4.5_XIAO_ESP32` changes, compile these files again and upload them over the old ones, with the same names. **Install from GitHub** on the XIAO page offers a file when it was compiled later than the program running on the XIAO (the date is read from the file).

| File | What for |
|---|---|
| `v4.5_XIAO_ESP32C3.bin` | XIAO ESP32C3: update over WiFi (XIAO page → Firmware) |
| `v4.5_XIAO_ESP32S3.bin` | XIAO ESP32S3: update over WiFi (XIAO page → Firmware) |
| `v4.5_XIAO_ESP32C3_USB.bin` | XIAO ESP32C3: first install over USB-C, at address 0x0 |
| `v4.5_XIAO_ESP32S3_USB.bin` | XIAO ESP32S3: first install over USB-C, at address 0x0 |

If you choose the file for the other XIAO, the page refuses it before sending it.

## First install over USB-C, without the Arduino IDE

On the computer, with Chrome or Edge:

1. XIAO **off the AiO board**, connected to the computer over USB-C.
2. Open https://espressif.github.io/esptool-js/
3. Baudrate 921600 → **Connect** → choose the XIAO's port.
   - If the port does not show up: unplug the cable, hold the XIAO's **BOOT** button, plug the cable in with BOOT held, then let go.
4. In **Flash Address** type `0x0` (the page may show another value) and choose the `…_USB.bin` file for your XIAO (C3 or S3).
5. **Program**. When it is done, press the XIAO's **RESET** button (or unplug the cable and plug it in again).
6. The XIAO creates the network `XIAO-xxxx`, password `agopengps`.

It can also be installed from the Arduino IDE, by compiling the program (see the program's README).

## Later: update over WiFi

- Phone on the XIAO's network → page → **Firmware** → choose the `.bin` **without** "_USB" for your XIAO → password → **Install**.
- The WiFi settings are kept.
- Or, when the phone has internet: **Firmware → Install from GitHub** downloads the right file from github.com/renatorgr/Boards by itself (see the program's README).

## Credits and licence

- Program written by Claude (Anthropic) for renatorgr.
- The link to the Teensy uses the protocol of the ESP32 bridge of the official AgOpenGPS firmware v26.
- **Licence:** GNU GPL v3 or later (file `COPYING` in the program folder `v4.5_XIAO_ESP32`), WITHOUT ANY WARRANTY.
