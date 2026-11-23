@echo off
rem Hand test 42: the XIAO page no longer stops the bridge (a computer on the XIAO's Wi-Fi, about 12 minutes).
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0hand-tests.ps1" -Test 42 %*
pause
