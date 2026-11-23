@echo off
rem Hand test 46: the Teensy page through the XIAO, port 8080 (the tablet on the XIAO's Wi-Fi, 1 minute).
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0hand-tests.ps1" -Test 46 %*
pause
