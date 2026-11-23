# Hand tests: the computer's part

Scripts for the hand tests on the bench and the tractor that a computer can do or watch.
You still do what needs hands (a cable out, the power off). The script does the rest and says what it saw.

- Windows PowerShell 5.1: nothing to install.
- Copy this whole folder to the tablet (or the PC), then double-click the `.cmd` of the test.
- The script asks before every step that needs you, and before any step that needs a password.
- A password is used for one request and never saved.

## The tests

| File | Hand test | Where it runs | How long |
|------|-----------|---------------|----------|
| `test-07-gps-temperature.cmd` | 7: the UM982's temperature from a cold start | tablet on the tractor network, right after switching on | 20 min |
| `test-17-sprayer-relays-come-back.cmd` | 17: the sprayer's relays drop and come back | a computer that stays on the network | 5 min |
| `test-42-xiao-page-keeps-bridge.cmd` | 42: the XIAO page no longer stops the bridge | a computer on the XIAO's Wi-Fi | 12 min |
| `test-44-sprayer-litres-kept.cmd` | 44: the sprayer's litres kept over a power cut | a computer that reaches the sprayer module | 5 min |
| `test-46-teensy-page-through-xiao.cmd` | 46: the Teensy page through the XIAO (port 8080) | the tablet on the XIAO's Wi-Fi | 1 min |
| `test-48-xiao-sign-in-notice.cmd` | 48: the XIAO's sign-in notice | a Windows computer on the XIAO's Wi-Fi | 3 min |

What each script leaves to you:

- **17:** take the tablet's cable (or Wi-Fi) out and back, 2 or 3 times.
- **42:** step d) (Save refused while AutoSteer is on), with test 41.
- **44:** run water or blow into the flow meter, switch sections on and off, cut the power.
- **46:** step b) (change a setting, Save, reload): it needs the module's password.
- **48:** step c) (a new phone still gets the page by itself). With Windows' Location off, step b) (leave the Wi-Fi and join it
  again) is yours too: Windows then hides the Wi-Fi's name from the script.

## The result

- Every line says **OK**, **CHECK** (look at it) or **PROBLEM** (wrong, with what to do).
- The report goes to `reports\<date>_test-<number>.zip`, next to the scripts. Send that file.

## Other addresses

The scripts look for the boards at their usual addresses:

- Teensy (AiO): `192.168.5.126`
- XIAO: `192.168.5.1` (bridge on) or `192.168.4.1`
- Sprayer module: `192.168.5.123`

For another one, add it after the test in a command window, for example:
`powershell -ExecutionPolicy Bypass -File hand-tests.ps1 -Test 44 -Module 192.168.5.124`

## Self-test (no boards)

`selftest\run-selftest.ps1` runs every test against a pretend board (`selftest\fake-boards.js`, needs Node.js).
It only shows that the scripts run on Windows. The harnesses in `Tests\` check the boards' programs.
It talks to `localhost` only: nothing goes out on the network, and the Wi-Fi is not touched.
