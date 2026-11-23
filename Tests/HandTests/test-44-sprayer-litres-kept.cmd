@echo off
rem Hand test 44: the sprayer's litres kept over a power cut (a computer that reaches the module; you cut the power when it says).
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0hand-tests.ps1" -Test 44 %*
pause
