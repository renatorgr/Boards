@echo off
rem Hand test 7: the UM982's temperature from a cold start (tablet on the tractor network, 20 minutes).
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0hand-tests.ps1" -Test 7 %*
pause
