@echo off
rem Hand test 17: the sprayer's relays when the program's network drops (a computer that stays on the network, 5 minutes).
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0hand-tests.ps1" -Test 17 %*
pause
