# run-selftest.ps1 - every test of hand-tests.ps1 against selftest\fake-boards.js (Node.js), answering yes to everything.
# Checks the script parses and runs to its RESULT on Windows PowerShell 5.1; the fake is not a board.
#   powershell -NoProfile -ExecutionPolicy Bypass -File run-selftest.ps1 [-Node <node.exe>] [-Only 7,17,42,44,46,48]
# SPDX-License-Identifier: GPL-3.0-or-later
param([string]$Node = 'node', [string]$Only = '7,17,42,44,46,48')
$here = $PSScriptRoot
$tool = Join-Path (Split-Path $here -Parent) 'hand-tests.ps1'
$errs = $null
[void][System.Management.Automation.Language.Parser]::ParseFile($tool, [ref]$null, [ref]$errs)
if ($errs.Count -gt 0) { $errs | ForEach-Object { Write-Host ('PARSE ' + $_.Extent.StartLineNumber + ': ' + $_.Message) -ForegroundColor Red }; exit 1 }
Write-Host 'hand-tests.ps1 parses' -ForegroundColor Green
$minutes = @{ '7' = 1; '17' = 1; '42' = 1; '44' = 0; '46' = 0; '48' = 0 }
foreach ($t in ($Only -split ',')) {
    $t = $t.Trim()
    $fake = Start-Process -FilePath $Node -ArgumentList @((Join-Path $here 'fake-boards.js'), $t, '240') -PassThru -WindowStyle Hidden
    Start-Sleep -Seconds 2
    Write-Host ('---------------- test ' + $t) -ForegroundColor Cyan
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $tool -Test $t -BaseUrl 'http://localhost:18090' -Minutes $minutes[$t] -Yes 2>&1 | ForEach-Object { "$_" }
    Stop-Process -Id $fake.Id -Force -ErrorAction SilentlyContinue
}
