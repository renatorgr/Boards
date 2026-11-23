@echo off
rem Hand test 48: the XIAO's sign-in notice (a Windows computer on the XIAO's Wi-Fi, 3 minutes).
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0hand-tests.ps1" -Test 48 %*
pause
