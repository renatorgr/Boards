# hand-tests.ps1 - the parts of Renato's hand tests that a computer can do or watch, on the bench or the tractor:
#   7  the GPS receiver's temperature from a cold start      17 sprayer relays drop and come back with the program
#   42 the XIAO page no longer stops the bridge               44 sprayer litres kept over a power cut
#   46 the Teensy page through the XIAO (port 8080)           48 the XIAO's sign-in notice ("Done on this device")
# Windows PowerShell 5.1, nothing to install. Started from the .cmd files next to it (README.md says which and where).
# Each run writes reports\<date>_<test>\ (summary.txt, the answers it read) and a .zip of it to send.
# A password it asks for is used for that one request and never written anywhere.
# History: the git log.  SPDX-License-Identifier: GPL-3.0-or-later
param(
    [Parameter(Mandatory = $true)][ValidateSet('7', '17', '42', '44', '46', '48')][string]$Test,
    [string]$Teensy = '192.168.5.126',  # the module (AiO) on the tractor network
    [string]$Xiao = '',                 # the XIAO; empty = found at 192.168.5.1 (bridge on) or 192.168.4.1
    [string]$Module = '192.168.5.123',  # the sprayer module
    [int]$Minutes = 0,                  # 0 = the test's own length
    [string]$BaseUrl = '',              # self-test only: every board answers at this address (selftest\fake-boards.js)
    [switch]$Yes                        # self-test only: answer every question with yes and Enter
)

$ErrorActionPreference = 'Continue'
$ProgressPreference = 'SilentlyContinue'
$inv = [System.Globalization.CultureInfo]::InvariantCulture
$utf8 = New-Object System.Text.UTF8Encoding($false)
$BaseUrl = $BaseUrl.TrimEnd('/')

$stamp = (Get-Date).ToString('yyyy-MM-dd_HHmmss')
$outDir = Join-Path (Join-Path $PSScriptRoot 'reports') ($stamp + '_test-' + $Test)
[void](New-Item -ItemType Directory -Force -Path $outDir)
$script:lines = New-Object System.Collections.ArrayList
$script:problems = 0
$script:checks = 0
$script:oks = 0

# ---------------------------------------------------------------- what the screen and summary.txt show
function Say([string]$text, [string]$color = 'Gray') { Write-Host $text -ForegroundColor $color; [void]$script:lines.Add($text) }
function Step([string]$text) { Say ''; Say ('== ' + $text) 'Cyan' }
function Ok([string]$text) { $script:oks++; Say ('  OK       ' + $text) 'Green' }
function Info([string]$text) { Say ('  info     ' + $text) 'Gray' }
function Check([string]$text) { $script:checks++; Say ('  CHECK    ' + $text) 'Yellow' }
function Problem([string]$text) { $script:problems++; Say ('  PROBLEM  ' + $text) 'Red' }
function Todo([string]$text) { Say ('  YOU      ' + $text) 'White' }
function F1($x) { return ([double]$x).ToString('0.0', $inv) }
function F2($x) { return ([double]$x).ToString('0.00', $inv) }
function Now { return (Get-Date).ToString('HH:mm:ss') }

function Ask([string]$question) {
    if ($Yes) { Say ('  ? ' + $question + ' -> (self-test) y'); return 'y' }
    Write-Host ('  ? ' + $question + ' ') -ForegroundColor White -NoNewline
    $a = Read-Host
    [void]$script:lines.Add('  ? ' + $question + ' -> ' + $a)
    return $a
}
function AskYes([string]$question) {
    $a = (Ask ($question + ' (y/n)')).Trim().ToLower()
    return ($a.StartsWith('y') -or $a.StartsWith('s'))   # yes / sim
}
function WaitEnter([string]$what) {
    if ($Yes) { Say ('  > ' + $what + ' (self-test: no wait)'); return }
    Write-Host ('  > ' + $what + ', then press Enter ') -ForegroundColor White -NoNewline
    [void](Read-Host)
    [void]$script:lines.Add('  > ' + $what + ' (Enter at ' + (Now) + ')')
}
function ReadPassword([string]$what) {
    if ($Yes) { return 'selftest' }
    $s = Read-Host ('  ? ' + $what + ' (not saved anywhere)') -AsSecureString
    $b = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($s)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringBSTR($b) } finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($b) }
}

# ---------------------------------------------------------------- talking to the boards' pages
function Url([string]$addr, [string]$path) {
    if ($BaseUrl -ne '') { return $BaseUrl + $path }
    return 'http://' + $addr + $path
}
# One request. Answers @{ code; text; ms } (code 0 = no answer). 4xx and 5xx answers are read too: a refusal is a result.
function Http([string]$method, [string]$url, [string]$body = $null, [int]$timeoutMs = 4000, [string]$type = 'application/x-www-form-urlencoded') {
    $watch = [Diagnostics.Stopwatch]::StartNew()
    try {
        $rq = [System.Net.HttpWebRequest]::Create($url)
        $rq.Method = $method
        $rq.Timeout = $timeoutMs
        $rq.ReadWriteTimeout = $timeoutMs
        $rq.KeepAlive = $false
        $rq.AllowAutoRedirect = $false
        if ($null -ne $body) {
            $bytes = $utf8.GetBytes($body)
            $rq.ContentType = $type
            $rq.ContentLength = $bytes.Length
            $st = $rq.GetRequestStream(); $st.Write($bytes, 0, $bytes.Length); $st.Close()
        }
        $rs = $rq.GetResponse()
    } catch [System.Net.WebException] {
        $rs = $_.Exception.Response
        if ($null -eq $rs) { return @{ code = 0; text = $_.Exception.Message; ms = $watch.ElapsedMilliseconds } }
    } catch {
        return @{ code = 0; text = $_.Exception.Message; ms = $watch.ElapsedMilliseconds }
    }
    $code = [int]$rs.StatusCode
    $loc = $rs.Headers['Location']
    $rd = New-Object System.IO.StreamReader($rs.GetResponseStream(), $utf8)
    $text = $rd.ReadToEnd(); $rd.Close(); $rs.Close()
    return @{ code = $code; text = $text; ms = $watch.ElapsedMilliseconds; location = $loc }
}
# GET of a JSON answer, written to answers.txt (the path only: a password never goes in a GET here)
function GetJson([string]$url, [int]$timeoutMs = 3000) {
    $r = Http 'GET' $url $null $timeoutMs
    $when = (Get-Date).ToString('HH:mm:ss.fff')
    $line = $when + ' GET ' + ([uri]$url).AbsolutePath + ' -> ' + $r.code + ' ' + ($r.text -replace "[\r\n]+", ' ')
    if ($line.Length -gt 3000) { $line = $line.Substring(0, 3000) + '...' }
    [System.IO.File]::AppendAllText((Join-Path $outDir 'answers.txt'), $line + "`r`n", $utf8)
    if ($r.code -ne 200 -or -not $r.text.TrimStart().StartsWith('{')) { return $null }
    try { return ($r.text | ConvertFrom-Json) } catch { return $null }
}
function Has($obj, [string]$name) { return ($null -ne $obj -and $null -ne $obj.PSObject.Properties[$name]) }
function Enc([string]$s) { return [uri]::EscapeDataString($s) }

# "Nov 22 2026 16:03:02" (one-digit days have two spaces) -> a date, or $null
function BuiltDate([string]$built) {
    try { return [datetime]::ParseExact((($built -replace '\s+', ' ').Trim()), 'MMM d yyyy HH:mm:ss', $inv) } catch { return $null }
}
# '0.10.16' >= '0.10.16'
function VersionAtLeast([string]$have, [string]$need) {
    try {
        $h = [version](($have -replace '[^0-9.]', '').Trim('.'))
        return ($h -ge [version]$need)
    } catch { return $false }
}

function FindXiao {
    $list = @()
    if ($Xiao -ne '') { $list = @($Xiao) } else { $list = @('192.168.5.1', '192.168.4.1') }
    foreach ($a in $list) {
        $s = GetJson (Url $a '/api/state')
        if ((Has $s 'bridge') -and (Has $s 'wifi')) { return @{ addr = $a; state = $s } }
        if ($null -eq $s -and (Http 'GET' (Url $a '/')).code -eq 200) { Info ($a + ' answers, but with another page than the XIAO''s: a router?') }
    }
    return $null
}
# The XIAO's own events and the Teensy's lines it carries, from "after" on: @{ t = @(...); x = @(...); tlast; xlast }
function XiaoLog([string]$addr, [long]$tAfter, [long]$xAfter) {
    $t = New-Object System.Collections.ArrayList
    $x = New-Object System.Collections.ArrayList
    for ($i = 0; $i -lt 10; $i++) {
        $j = GetJson (Url $addr ('/api/log?after=' + $tAfter + '&xafter=' + $xAfter))
        if ($null -eq $j) { break }
        foreach ($l in @($j.t)) { if ($null -ne $l) { [void]$t.Add($l); if ([long]$l.s -gt $tAfter) { $tAfter = [long]$l.s } } }
        foreach ($l in @($j.x)) { if ($null -ne $l) { [void]$x.Add($l); if ([long]$l.s -gt $xAfter) { $xAfter = [long]$l.s } } }
        if (-not $j.more) { break }
    }
    return @{ t = $t; x = $x; tlast = $tAfter; xlast = $xAfter }
}

function Finish {
    Step 'RESULT'
    Say ('  ' + $script:oks + ' OK, ' + $script:checks + ' CHECK, ' + $script:problems + ' PROBLEM') $(if ($script:problems) { 'Red' } elseif ($script:checks) { 'Yellow' } else { 'Green' })
    [System.IO.File]::WriteAllLines((Join-Path $outDir 'summary.txt'), [string[]]$script:lines, $utf8)
    $zip = $outDir + '.zip'
    try {
        Add-Type -AssemblyName System.IO.Compression.FileSystem
        if (Test-Path $zip) { Remove-Item $zip -Force }
        [System.IO.Compression.ZipFile]::CreateFromDirectory($outDir, $zip)
        Say ('  Send this file: ' + $zip) 'White'
    } catch { Say ('  The report is in ' + $outDir) 'White' }
}

Say ('Hand test ' + $Test + ', ' + (Get-Date).ToString('yyyy-MM-dd HH:mm:ss') + ', on ' + $env:COMPUTERNAME) 'White'

# ================================================================= 7: the GPS receiver's temperature from a cold start
function Test7 {
    $mins = $(if ($Minutes -gt 0) { $Minutes } else { 20 })
    Step ('Test 7: the UM982''s temperature, once a minute for ' + $mins + ' minutes')
    Info 'Start it straight after switching the tractor on, on a cold morning. AgOpenWeb can stay open.'
    $st = GetJson (Url $Teensy '/api/status')
    if ($null -eq $st) { Problem ('no answer from the module at ' + $Teensy + ': is this computer on the tractor network?'); return }
    $gps = GetJson (Url $Teensy '/api/gps')
    if ($null -eq $gps) { Problem 'the module answers but not its GPS tab (/api/gps): an old firmware?'; return }
    Info ('module ' + $st.fw + ' built ' + $st.built + ', up ' + [int]($st.up / 60) + ' min; receiver ' + $gps.kind + ' ' + $gps.model)
    if ($gps.kind -ne 'um982') { Check ('the receiver is "' + $gps.kind + '", not a UM982: this test is for the Puma''s UM982') }
    if ([double]$st.up -gt 600) { Check ('the module has been on ' + [int]($st.up / 60) + ' min: for a cold start switch the tractor off and on, then run this again') }
    $rows = New-Object System.Collections.ArrayList
    $end = (Get-Date).AddMinutes($mins)
    Say '  time      on for   temperature'
    while ((Get-Date) -lt $end) {
        $s = GetJson (Url $Teensy '/api/status')
        $g = GetJson (Url $Teensy '/api/gps')
        if ($null -ne $s -and $null -ne $g) {
            $t = $(if (Has $g 'temp') { [double]$g.temp } else { $null })
            $r = [pscustomobject]@{ at = (Now); upMin = [double]$s.up / 60; temp = $t; st = $g.tempSt }
            [void]$rows.Add($r)
            $tt = $(if ($null -ne $t) { (F1 $t) + ' C' } else { '(' + $g.tempSt + ')' })
            Say ('  ' + $r.at + '  ' + (F1 $r.upMin).PadLeft(5) + ' min  ' + $tt)
        } else { Say ('  ' + (Now) + '  no answer') }
        if ($Yes) { Start-Sleep -Seconds 1 } else { Start-Sleep -Seconds 60 }
    }
    # the first line the module wrote in Messages with the number as it came
    $log = GetJson (Url $Teensy '/api/log?after=0')
    if ($null -ne $log) { foreach ($l in @($log.lines)) { if ($l.x -match 'GPS receiver: temperature') { Info ('Messages: ' + $l.x) } } }
    $got = @($rows | Where-Object { $null -ne $_.temp })
    if ($got.Count -eq 0) { Problem 'no temperature came (the GPS tab says "not reported"): send this report'; return }
    $first = $got[0]; $last = $got[$got.Count - 1]
    Ok ('first reading ' + (F1 $first.temp) + ' C at ' + (F1 $first.upMin) + ' min after switching on; last ' + (F1 $last.temp) + ' C at ' + (F1 $last.upMin) + ' min')
    if ($last.temp -gt $first.temp + 2) { Ok ('it rose ' + (F1 ($last.temp - $first.temp)) + ' C while the box warmed up') }
    else { Check 'it did not rise more than 2 C: was the box already warm?' }
}

# ================================================================= 17: sprayer relays drop and come back with the program
function Test17 {
    $mins = $(if ($Minutes -gt 0) { $Minutes } else { 5 })
    Step ('Test 17: the sprayer''s relays when the program''s network drops, watched for ' + $mins + ' minutes')
    Info 'Run this on a computer that STAYS on the network (not the tablet whose cable or Wi-Fi you take out).'
    $s = GetJson (Url $Module '/api/status')
    if ($null -eq $s) { Problem ('no answer from the sprayer module at ' + $Module); return }
    Info ('module ' + $s.fw + ' ' + $s.ver + ', up ' + $s.up + ' s, last start: ' + $s.start)
    Todo 'Switch 2 or 3 sections ON in AgOpenWeb. Then, while this runs: take the tablet''s cable (or Wi-Fi) out for about 10 s and put it back. Do it 2 or 3 times, 30 s apart.'
    $end = (Get-Date).AddMinutes($mins)
    $prev = $null; $silentAt = $null; $offAt = $null; $backAt = $null; $cuts = 0; $restarts = 0
    $dropTimes = New-Object System.Collections.ArrayList; $backTimes = New-Object System.Collections.ArrayList
    $lastBeat = Get-Date
    $script:still = $null
    while ((Get-Date) -lt $end) {
        $j = GetJson (Url $Module '/api/status') 1500
        $tnow = Get-Date
        if ($null -eq $j) { if (($tnow - $lastBeat).TotalSeconds -gt 3) { Say ('  ' + (Now) + '  the module does not answer this computer'); $lastBeat = $tnow }; Start-Sleep -Milliseconds 250; continue }
        $n239 = [long]$j.aog.n239
        $on = [Convert]::ToUInt64([string]$j.out.on, 16)
        $req = [Convert]::ToUInt64([string]$j.aog.req, 16)
        if ($null -ne $prev) {
            if ([long]$j.up -lt [long]$prev.up) { $restarts++; Problem ((Now) + ' the module RESTARTED (start: ' + $j.start + ')') }
            $moving = ($n239 -ne [long]$prev.n239)
            if (-not $moving -and $null -eq $silentAt -and ($tnow - $prev.t).TotalSeconds -lt 2) {
                if ($null -eq $script:still) { $script:still = $tnow } elseif (($tnow - $script:still).TotalMilliseconds -ge 600) { $silentAt = $script:still; $cuts++; Say ('  ' + (Now) + '  cut ' + $cuts + ': the program''s packets stopped') 'White' }
            } elseif ($moving) { $script:still = $null }
            if ($null -ne $silentAt -and $null -eq $offAt -and $on -eq 0) {
                $offAt = $tnow; $d = ($offAt - $silentAt).TotalSeconds; [void]$dropTimes.Add($d)
                Say ('  ' + (Now) + '  relays OFF ' + (F1 $d) + ' s after the packets stopped') 'White'
            }
            if ($null -ne $silentAt -and $moving) {
                if ($null -eq $backAt) { $backAt = $tnow; Say ('  ' + (Now) + '  the program''s packets are back') 'White' }
            }
            if ($null -ne $backAt -and $req -ne 0 -and $on -eq $req) {
                $d = ($tnow - $backAt).TotalSeconds; [void]$backTimes.Add($d)
                Say ('  ' + (Now) + '  relays back ON by themselves ' + (F1 $d) + ' s after the packets came back') 'White'
                if ($null -eq $offAt) { Check ('cut ' + $cuts + ': the relays never went off (a cut shorter than 2 s?)') }
                $silentAt = $null; $offAt = $null; $backAt = $null
            } elseif ($null -ne $backAt -and ($tnow - $backAt).TotalSeconds -gt 15) {
                Problem ('cut ' + $cuts + ': the packets came back 15 s ago and the relays did not follow (asked ' + $j.aog.req + ', on ' + $j.out.on + ')')
                $silentAt = $null; $offAt = $null; $backAt = $null
            }
        }
        $prev = [pscustomobject]@{ up = $j.up; n239 = $n239; t = $tnow }
        Start-Sleep -Milliseconds 250
    }
    if ($cuts -eq 0) { Check 'no cut was seen: the program''s packets never stopped while this watched'; return }
    foreach ($d in $dropTimes) { if ($d -le 3.5) { Ok ('relays off ' + (F1 $d) + ' s after the program went silent (the module waits 2 s)') } else { Problem ('relays off only ' + (F1 $d) + ' s after the program went silent (should be about 2 s)') } }
    if ($backTimes.Count -gt 0) { Ok ($backTimes.Count.ToString() + ' of ' + $cuts + ' cuts: the sections came back by themselves, nothing restarted') }
    if ($restarts -eq 0) { Ok 'the module never restarted' }
}

# ================================================================= 42: the XIAO page no longer stops the bridge
function Test42 {
    $mins = $(if ($Minutes -gt 0) { $Minutes } else { 10 })
    Step 'Test 42 a): the XIAO''s program and its messages'
    $x = FindXiao
    if ($null -eq $x) { Problem 'no XIAO at 192.168.5.1 or 192.168.4.1: is this computer on the XIAO''s Wi-Fi?'; return }
    $a = $x.addr; $s = $x.state
    Info ('XIAO ' + $s.fw + ' (' + $s.chip + ') built ' + $s.built + ', up ' + $s.up + ' s, last start: ' + $s.reset)
    $b = BuiltDate $s.built
    if ($null -ne $b -and $b -lt [datetime]'2026-11-22 16:03:00') { Problem 'its program is older than 22 Nov 16:03 UTC: Firmware > Install from GitHub first' } else { Ok 'program of 22 Nov 16:03 UTC or newer' }
    $lg = XiaoLog $a 0 0
    $bad = @($lg.x | Where-Object { $_.x -match 'link task did NOT start' })
    if ($bad.Count -gt 0) { Problem ('Messages: "' + $bad[0].x + '"') } else { Ok 'no "The link task did NOT start" in its messages' }

    Step ('Test 42 b): the page open for ' + $mins + ' minutes (this computer asks it what the page asks, every second)')
    $tLast = $lg.tlast; $xLast = $lg.xlast
    $end = (Get-Date).AddMinutes($mins)
    $prevUp = [long]$s.up; $restarts = 0; $silent = 0; $noAnswer = 0; $asked = 0; $slow = 0; $n = 0
    while ((Get-Date) -lt $end) {
        $n++
        $st = GetJson (Url $a '/api/state') 3000; $asked++
        if ($null -eq $st) { $noAnswer++ } else {
            if ([long]$st.up -lt $prevUp) { $restarts++; Problem ((Now) + ' the XIAO RESTARTED (last start: ' + $st.reset + ')') }
            $prevUp = [long]$st.up
            if (-not $st.link.talking) { $slow++ }
        }
        $l = XiaoLog $a $tLast $xLast; $tLast = $l.tlast; $xLast = $l.xlast
        foreach ($t in $l.t) { if ($t.x -match 'XIAO silent') { $silent++; Problem ((Now) + ' Teensy: ' + $t.x) } }
        if ($n % 10 -eq 0) { [void](Http 'GET' (Url $a '/') $null 5000) }   # the page itself, as a phone that opens it again
        if ($n % 60 -eq 0) { Say ('  ' + (Now) + '  ' + [int](($end - (Get-Date)).TotalMinutes) + ' min to go, XIAO up ' + $prevUp + ' s') }
        if ($Yes) { Start-Sleep -Milliseconds 100 } else { Start-Sleep -Seconds 1 }
    }
    if ($restarts -eq 0) { Ok ('the XIAO did not restart in ' + $mins + ' min') }
    if ($silent -eq 0) { Ok 'no "XIAO silent" in the Teensy''s messages' }
    if ($noAnswer -gt 0) { Check ($noAnswer.ToString() + ' of ' + $asked + ' questions to the XIAO got no answer within 3 s') }
    if ($slow -gt 0) { Check ($slow.ToString() + ' times the XIAO said the Teensy was not talking') }

    Step 'Test 42 c): a program upload cut half way (as Wi-Fi switched off during "Install from a file")'
    if (-not (AskYes 'Do step c now? It needs the XIAO page password; nothing is installed')) { Info 'step c skipped'; return }
    $st = GetJson (Url $a '/api/state')
    if ($null -ne $st -and $st.bridge.steer) { Problem 'AutoSteer is on through the bridge: the XIAO refuses an update then. Switch AutoSteer off and run again.'; return }
    $pw = ReadPassword 'XIAO page password'
    $upBefore = [long]$st.up
    $l0 = XiaoLog $a $tLast $xLast; $tLast = $l0.tlast; $xLast = $l0.xlast
    # A program header for this XIAO's chip (so the chip check lets it in) and then 0xFF: only 96 kB of 1.2 MB are sent,
    # the upload never ends, the XIAO never installs it (and a 0xFF image would fail its own check at the end anyway).
    $chipId = $(if ($s.chip -match 'S3') { 9 } else { 5 })
    $total = 1200000; $send = 96 * 1024
    $boundary = '----handtest42' + [Guid]::NewGuid().ToString('N')
    $head = '--' + $boundary + "`r`nContent-Disposition: form-data; name=`"update`"; filename=`"hand-test-42-cut.bin`"`r`nContent-Type: application/octet-stream`r`n`r`n"
    $tail = "`r`n--" + $boundary + "--`r`n"
    $len = $utf8.GetByteCount($head) + $total + $utf8.GetByteCount($tail)
    $hostName = $a; $port = 80
    if ($BaseUrl -ne '') { $u = [uri]$BaseUrl; $hostName = $u.Host; $port = $u.Port }
    $answer = ''
    try {
        $tcp = New-Object System.Net.Sockets.TcpClient
        $tcp.Connect($hostName, $port)
        $ns = $tcp.GetStream()
        $req = 'POST /update?pw=' + (Enc $pw) + " HTTP/1.1`r`nHost: " + $hostName + "`r`nContent-Type: multipart/form-data; boundary=" + $boundary + "`r`nContent-Length: " + $len + "`r`nConnection: close`r`n`r`n" + $head
        $rb = $utf8.GetBytes($req); $ns.Write($rb, 0, $rb.Length)
        $buf = New-Object byte[] 4096
        for ($i = 0; $i -lt $buf.Length; $i++) { $buf[$i] = 0xFF }
        $hdr = New-Object byte[] 4096
        for ($i = 0; $i -lt $hdr.Length; $i++) { $hdr[$i] = 0xFF }
        $hdr[0] = 0xE9; $hdr[1] = 1; $hdr[12] = [byte]$chipId; $hdr[13] = 0
        $ns.Write($hdr, 0, $hdr.Length)
        for ($sent = 4096; $sent -lt $send; $sent += 4096) { $ns.Write($buf, 0, 4096); Start-Sleep -Milliseconds 20 }
        Info ((Now) + ' sent ' + [int]($send / 1024) + ' kB of the file and stopped sending (the connection stays open)')
        $ns.ReadTimeout = 1000
        $waitS = $(if ($Yes) { 4 } else { 35 })
        $t0 = Get-Date
        while (((Get-Date) - $t0).TotalSeconds -lt $waitS) {
            try { $k = $ns.Read($buf, 0, $buf.Length); if ($k -gt 0) { $answer += $utf8.GetString($buf, 0, $k) } elseif ($k -eq 0) { break } } catch { }
        }
        $tcp.Close()
    } catch { Check ('the upload connection: ' + $_.Exception.Message) }
    $pw = $null
    $answer = ($answer -split "`r`n")[0]
    if ($answer -ne '') { Info ('the XIAO answered: ' + $answer) }
    Start-Sleep -Seconds 3
    $st2 = GetJson (Url $a '/api/state')
    $l = XiaoLog $a $tLast $xLast
    foreach ($e in $l.x) { Info ('XIAO: ' + $e.x) }
    foreach ($t in $l.t) { if ($t.x -match 'XIAO silent') { Problem ('Teensy: ' + $t.x) } }
    if (@($l.x | Where-Object { $_.x -match 'wrong password' }).Count -gt 0) { Problem 'the password was wrong: run again'; return }
    if (@($l.x | Where-Object { $_.x -match 'Program update STOPPED' }).Count -gt 0) { Ok '"Program update STOPPED" in the XIAO''s messages' } else { Problem 'no "Program update STOPPED" in the XIAO''s messages' }
    if ($null -eq $st2) { Problem 'the XIAO does not answer after the cut upload' }
    elseif ([long]$st2.up -lt $upBefore) { Problem ('the XIAO RESTARTED (last start: ' + $st2.reset + '): before #167 the watchdog did that') }
    else { Ok ('the XIAO did not restart (up ' + $st2.up + ' s) and kept its program') }
    Todo 'Step d) (Save refused while AutoSteer is on through the bridge) stays by hand, with test 41.'
}

# ================================================================= 44: sprayer litres kept over a power cut
function SprayerWaitRestart([long]$upBefore) {
    # waits for the module to go and come back with a lower "up"; answers its status, or $null after 3 minutes
    $t0 = Get-Date; $gone = $false
    while (((Get-Date) - $t0).TotalMinutes -lt 3) {
        $j = GetJson (Url $Module '/api/status') 1500
        if ($null -eq $j) { if (-not $gone) { Info ((Now) + ' the module is off') }; $gone = $true }
        elseif ([long]$j.up -lt $upBefore -or ($Yes -and $gone)) { Info ((Now) + ' the module is back, up ' + $j.up + ' s'); return $j }
        Start-Sleep -Milliseconds 500
    }
    return $null
}
function Test44 {
    Step 'Test 44 a): the sprayer module''s version'
    $s = GetJson (Url $Module '/api/status')
    if ($null -eq $s) { Problem ('no answer from the sprayer module at ' + $Module); return }
    Info ('module ' + $s.fw + ' ' + $s.ver + ', last start: ' + $s.start + ', applied ' + (F1 $s.spr.total) + ' L')
    if (-not (VersionAtLeast $s.ver '0.10.16')) { Problem 'older than 0.10.16: Firmware > Install from GitHub first'; return } else { Ok ('version ' + $s.ver) }
    if (-not (Has $s 'start')) { Problem 'its status has no "Last start": an old program'; return }

    Step 'Test 44 b): a power cut with every section off'
    $t0 = [double]$s.spr.total
    Todo 'Make Applied go up: run water, or blow into the flow meter (this waits up to 2 minutes).'
    $t1 = Get-Date; $grew = $false
    while (((Get-Date) - $t1).TotalMinutes -lt 2) {
        $j = GetJson (Url $Module '/api/status')
        if ($null -ne $j -and [double]$j.spr.total -gt $t0 + 0.05) { $grew = $true; break }
        if ($Yes) { break }
        Start-Sleep -Seconds 1
    }
    if ($grew) { Ok ('Applied went up: ' + (F1 $t0) + ' -> ' + (F1 $j.spr.total) + ' L') } else { Check 'Applied did not go up: the test goes on with the litres it has' }
    Info 'waiting 15 s, so the module saves the litres'
    if (-not $Yes) { Start-Sleep -Seconds 15 }
    WaitEnter 'Switch EVERY section OFF in AgOpenWeb'
    $s = GetJson (Url $Module '/api/status')
    if ($null -eq $s) { Problem 'the module stopped answering'; return }
    if ([Convert]::ToUInt64([string]$s.out.on, 16) -ne 0) { Check ('the module still has sections on (' + $s.out.on + ')') }
    $before = [double]$s.spr.total
    Todo ('Applied now ' + (F1 $before) + ' L. Take the power off the module, wait 5 s, and put it back.')
    $j = SprayerWaitRestart ([long]$s.up)
    if ($null -eq $j) { Problem 'the module did not go off and come back within 3 minutes'; return }
    $after = [double]$j.spr.total
    if ($after -ge $before - 0.05) { Ok ('Applied kept: ' + (F1 $before) + ' -> ' + (F1 $after) + ' L') }
    else { $lost = $before - $after; if ($lost -le [math]::Max(0.5, [double]$s.spr.lpm * 10 / 60)) { Check ('Applied ' + (F1 $before) + ' -> ' + (F1 $after) + ' L: lost ' + (F2 $lost) + ' L (up to 10 s of spraying is allowed)') } else { Problem ('Applied ' + (F1 $before) + ' -> ' + (F1 $after) + ' L: lost ' + (F2 $lost) + ' L') } }
    if ($j.start -eq 'power on' -and -not $j.startBad) { Ok 'Last start: "power on", not red' } else { Problem ('Last start: "' + $j.start + '"' + $(if ($j.startBad) { ', red' } else { '' }) + ' (should be "power on")') }
    if (AskYes 'Did AgOpenWeb show NO warning about the sprayer module?') { Ok 'AgOpenWeb: no warning' } else { Problem 'AgOpenWeb showed a warning after a power cut with the sections off' }

    Step 'Test 44 c): a power cut while spraying'
    WaitEnter 'Switch sections ON in AgOpenWeb'
    $s = GetJson (Url $Module '/api/status')
    if ($null -eq $s) { Problem 'the module stopped answering'; return }
    $before = [double]$s.spr.total
    Todo 'Take the power off the module, wait 5 s, and put it back. Watch the AgOpenWeb screen.'
    $j = SprayerWaitRestart ([long]$s.up)
    if ($null -eq $j) { Problem 'the module did not go off and come back within 3 minutes'; return }
    Start-Sleep -Seconds 3
    $j2 = GetJson (Url $Module '/api/status'); if ($null -ne $j2) { $j = $j2 }
    if ($j.start -match 'while spraying' -and $j.startBad) { Ok ('Last start: "' + $j.start + '", in red') } else { Problem ('Last start: "' + $j.start + '"' + $(if ($j.startBad) { ', red' } else { ', not red' }) + ' (should be "power on while spraying (power cut?)", red)') }
    if ([double]$j.spr.total -ge $before - 0.5) { Ok ('litres still there: ' + (F1 $j.spr.total) + ' L') } else { Problem ('litres ' + (F1 $before) + ' -> ' + (F1 $j.spr.total) + ' L') }
    if (AskYes 'Did AgOpenWeb show "Sprayer module restarted: power on while spraying (power cut?)" for about 30 s?') { Ok 'AgOpenWeb showed the warning' } else { Problem 'AgOpenWeb did not show the warning' }

    Step 'Test 44 d): Total back to 0, then Restart module'
    if (-not (AskYes 'Do step d now? It sets the module''s Applied to 0 and needs the module page password')) { Info 'step d skipped'; return }
    $pw = ReadPassword 'sprayer module page password'
    $r = Http 'POST' (Url $Module '/api/total') ('pw=' + (Enc $pw))
    if ($r.code -ne 200) { $pw = $null; Problem ('Total back to 0 refused: ' + $r.code + ' ' + $r.text); return }
    $s = GetJson (Url $Module '/api/status')
    $r = Http 'POST' (Url $Module '/api/restart') ('pw=' + (Enc $pw))
    $pw = $null
    if ($r.code -ne 200) { Problem ('Restart module refused: ' + $r.code + ' ' + $r.text); return }
    $j = SprayerWaitRestart ([long]$s.up)
    if ($null -eq $j) { Problem 'the module did not come back within 3 minutes'; return }
    if ([double]$j.spr.total -lt 0.05) { Ok 'Applied is 0 after the restart' } else { Problem ('Applied is ' + (F1 $j.spr.total) + ' L after Total back to 0 and a restart') }
    if ($j.start -eq 'restart asked') { Ok 'Last start: "restart asked"' } else { Check ('Last start: "' + $j.start + '"') }
}

# ================================================================= 46: the Teensy page through the XIAO (port 8080)
function Test46 {
    Step 'Test 46: the Teensy''s page through the XIAO''s Wi-Fi'
    $x = FindXiao
    if ($null -eq $x) { Problem 'no XIAO at 192.168.5.1 or 192.168.4.1: is this computer on the XIAO''s Wi-Fi?'; return }
    $a = $x.addr; $s = $x.state
    Info ('XIAO ' + $s.fw + ' built ' + $s.built + ', bridge ' + $(if ($s.bridge.on) { 'on' } else { 'off' }))
    $b = BuiltDate $s.built
    if ($null -ne $b -and $b -lt [datetime]'2026-11-22 19:15:00') { Problem 'its program is older than 22 Nov 19:15 UTC: Firmware > Install from GitHub first'; return }
    if ((Has $s 'captive') -and -not $s.captive.ap) { Problem 'this computer reaches the XIAO through another network: join the XIAO''s own Wi-Fi'; return }
    $page = $(if ($BaseUrl -ne '') { $BaseUrl } else { 'http://' + $a + ':8080' })

    Step 'e) the XIAO''s Status: the module''s card has the link'
    if (-not (Has $s 'tpage')) { Problem 'the XIAO says nothing about port 8080: an old program'; return }
    if ($s.tpage.ready) { Ok 'the XIAO says the module''s page is ready on port 8080' } else { Problem ('the XIAO says port 8080 is not ready: ' + $s.tpage.why) }
    $xp = Http 'GET' (Url $a '/') $null 8000
    if ($xp.text -match '8080') { Ok 'the XIAO page has the :8080 link' } else { Check 'no ":8080" found in the XIAO page (the link may be built when the page runs)' }

    Step 'a) the page opens, with every tab'
    $r = Http 'GET' ($page + '/') $null 20000
    if ($r.code -ne 200) { Problem ('no page on port 8080: ' + $r.code + ' ' + $r.text); return }
    $secs = $r.ms / 1000.0
    if ($secs -le 6) { Ok ('page in ' + (F1 $secs) + ' s (' + [int]($r.text.Length / 1024) + ' kB)') } else { Check ('page took ' + (F1 $secs) + ' s (about 4 s expected)') }
    $missing = @('status', 'log', 'settings', 'gps', 'imu', 'machine', 'firmware') | Where-Object { $r.text -notmatch ('data-t="' + $_ + '"') }
    if ($missing.Count -eq 0) { Ok 'every tab is there (Status, Messages, Settings, GPS, IMU, Machine, Firmware)' } else { Problem ('tabs missing: ' + ($missing -join ', ')) }
    foreach ($p in @('/api/status', '/api/settings', '/api/log', '/api/gps')) {
        $j = GetJson ($page + $p) 10000
        if ($null -ne $j) { Ok ($p + ' answers through the XIAO') } else { Problem ($p + ' does not answer through the XIAO') }
    }

    Step 'c) the page opened again while AgOpenWeb steers through the bridge'
    $s = GetJson (Url $a '/api/state')
    if ($null -eq $s -or -not $s.bridge.steer) {
        Check 'AutoSteer was not on through the bridge, so c) was not checked: switch it on in AgOpenWeb and run this again'
    } else {
        $offs = 0
        for ($k = 1; $k -le 3; $k++) {
            $job = [System.Net.HttpWebRequest]::Create($page + '/'); $job.Timeout = 20000
            $task = $job.GetResponseAsync()
            while (-not $task.IsCompleted) {
                $st = GetJson (Url $a '/api/state') 2000
                if ($null -ne $st -and -not $st.bridge.steer) { $offs++ }
                Start-Sleep -Milliseconds 300
            }
            try { $task.Result.Close() } catch { }
        }
        $ts = GetJson ($page + '/api/status') 10000
        if ($offs -eq 0) { Ok 'AutoSteer stayed on through the bridge while the page loaded 3 times' } else { Problem ('AutoSteer went off ' + $offs + ' times while the page loaded') }
        if ($null -ne $ts -and (Has $ts 'steer')) { Info ('Teensy: autosteer ' + $(if ($ts.steer.active) { 'on' } else { 'off' }) + ', set ' + $ts.steer.set + ', WAS ' + $ts.steer.was) }
    }

    Step 'd) a firmware update through port 8080 is refused'
    $r = Http 'POST' ($page + '/api/ota') 'not a firmware' 10000 'application/octet-stream'
    if ($r.code -ge 400 -and $r.code -lt 600) { Ok ('refused (' + $r.code + '): ' + (($r.text -replace '<[^>]+>', ' ' -replace '\s+', ' ').Trim())) }
    else { Problem ('not refused: ' + $r.code + ' ' + $r.text) }
    $r = Http 'POST' ($page + '/api/revert') '' 10000
    if ($r.code -ge 400 -and $r.code -lt 600) { Ok ('"back to the previous firmware" refused too (' + $r.code + ')') } else { Problem ('/api/revert not refused: ' + $r.code) }
    Todo 'Step b) (change a value in Settings, Save, reload: it stays) stays by hand: it needs the module''s password.'
}

# ================================================================= 48: the XIAO's sign-in notice
function WifiNow {
    # the Wi-Fi this computer is on: @{ ifName; ssid; profile; state }
    $o = @{ ifName = ''; ssid = ''; profile = ''; state = '' }
    $txt = (netsh wlan show interfaces) 2>$null
    foreach ($l in $txt) {
        if ($l -match '^\s*(Name|Nome)\s*:\s*(.+)$' -and $o.ifName -eq '') { $o.ifName = $matches[2].Trim() }
        elseif ($l -match '^\s*SSID\s*:\s*(.+)$' -and $o.ssid -eq '') { $o.ssid = $matches[1].Trim() }
        elseif ($l -match '^\s*(Profile|Perfil)\s*:\s*(.+)$') { $o.profile = $matches[2].Trim() }
        elseif ($l -match '^\s*(State|Estado)\s*:\s*(.+)$') { $o.state = $matches[2].Trim() }
    }
    return $o
}
function Connectivity([string]$ssid) {
    # Windows' own verdict for that network: InternetAccess, ConstrainedInternetAccess (= "sign in"), LocalAccess (= "No internet")
    try {
        [void][Windows.Networking.Connectivity.NetworkInformation, Windows.Networking.Connectivity, ContentType = WindowsRuntime]
        foreach ($p in [Windows.Networking.Connectivity.NetworkInformation]::GetConnectionProfiles()) {
            if ($p.ProfileName -eq $ssid -or ($p.IsWlanConnectionProfile -and $p.WlanConnectionProfileDetails.GetConnectedSsid() -eq $ssid)) { return [string]$p.GetNetworkConnectivityLevel() }
        }
    } catch { }
    return '?'
}
function SignInAsked([string]$dnsServer) {
    # what Windows' check (www.msftconnecttest.com) gets from the XIAO: 'page' = sent to the sign-in page, 'none' = no such name
    try {
        $r = Resolve-DnsName -Name 'www.msftconnecttest.com' -Server $dnsServer -Type A -DnsOnly -QuickTimeout -ErrorAction Stop
        $ip = @($r | Where-Object { $_.Type -eq 'A' } | ForEach-Object { $_.IPAddress })
        if ($ip.Count -gt 0) { return 'page ' + $ip[0] }
        return 'none'
    } catch { if ($_.Exception.Message -match 'does not exist|DNS name does not exist|9003') { return 'none' }; return 'error ' + $_.Exception.Message }
}
function Test48 {
    Step 'Test 48 a): "Done on this device" on this Windows computer'
    $w = WifiNow
    Info ('Wi-Fi: ' + $w.ssid + ' (' + $w.state + ') on "' + $w.ifName + '"')
    $x = FindXiao
    if ($null -eq $x) { Problem 'no XIAO at 192.168.5.1 or 192.168.4.1: is this computer on the XIAO''s Wi-Fi?'; return }
    $a = $x.addr; $s = $x.state
    Info ('XIAO ' + $s.fw + ' built ' + $s.built)
    $b = BuiltDate $s.built
    if ($null -ne $b -and $b -lt [datetime]'2026-11-23 04:43:00') { Problem 'its program is older than 23 Nov 04:43 UTC: Firmware > Install from GitHub first'; return }
    if (-not (Has $s 'captive')) { Problem 'the XIAO says nothing about the sign-in page: an old program'; return }
    if (-not $s.captive.ap) { Problem 'this computer reaches the XIAO through another network: join the XIAO''s own Wi-Fi'; return }
    Info ('before: Windows says ' + (Connectivity $w.ssid) + '; the XIAO answers Windows'' internet check with ' + (SignInAsked $a) + '; done on this device: ' + $s.captive.done)
    $r = Http 'POST' (Url $a '/api/captive') ''
    if ($r.code -ne 200) { Problem ('"Done on this device" refused: ' + $r.code + ' ' + $r.text); return }
    Ok ((Now) + ' "Done on this device" taken')
    $t0 = Get-Date; $lvl = '?'; $dns = '?'
    while (((Get-Date) - $t0).TotalSeconds -lt 75) {
        $dns = SignInAsked $a; $lvl = Connectivity $w.ssid
        if ($dns -eq 'none' -and $lvl -eq 'LocalAccess') { break }
        if ($Yes -and $dns -eq 'none') { break }
        Start-Sleep -Seconds 3
    }
    $secs = [int]((Get-Date) - $t0).TotalSeconds
    if ($dns -eq 'none') { Ok 'the XIAO now tells Windows'' check "no such name" (no sign-in page for this computer)' } else { Problem ('the XIAO still answers Windows'' check with ' + $dns) }
    if ($lvl -eq 'LocalAccess') { Ok ('Windows says "No internet" (LocalAccess) after ' + $secs + ' s, no sign-in') }
    elseif ($lvl -eq 'ConstrainedInternetAccess') { Problem 'Windows still says "sign in" (ConstrainedInternetAccess) after 75 s' }
    else { Check ('Windows says ' + $lvl + ' (LocalAccess expected)') }

    Step 'b) leave the XIAO''s Wi-Fi and join it again'
    $st = GetJson (Url $a '/api/state')
    if ($null -ne $st -and $st.bridge.steer) { Problem 'AutoSteer is on through the bridge: switch it off before b) (leaving the Wi-Fi stops it)'; return }
    if ($w.profile -eq '' -or $BaseUrl -ne '') { Check 'no Wi-Fi profile found (or self-test): b) by hand: Wi-Fi off and on, the notice must not come back' }
    else {
        Info ((Now) + ' leaving "' + $w.ssid + '"')
        [void](netsh wlan disconnect interface="$($w.ifName)")
        Start-Sleep -Seconds 5
        [void](netsh wlan connect name="$($w.profile)" ssid="$($w.ssid)" interface="$($w.ifName)")
        $t0 = Get-Date
        while (((Get-Date) - $t0).TotalSeconds -lt 40) { $w2 = WifiNow; if ($w2.ssid -eq $w.ssid -and $null -ne (GetJson (Url $a '/api/state'))) { break }; Start-Sleep -Seconds 2 }
        Info ((Now) + ' back on "' + (WifiNow).ssid + '"')
        $t0 = Get-Date
        while (((Get-Date) - $t0).TotalSeconds -lt 60) { $lvl = Connectivity $w.ssid; if ($lvl -eq 'LocalAccess' -or $lvl -eq 'ConstrainedInternetAccess') { break }; Start-Sleep -Seconds 3 }
        $dns = SignInAsked $a
        if ($dns -eq 'none') { Ok 'after joining again the XIAO still gives this computer no sign-in page' } else { Problem ('after joining again the XIAO answers Windows'' check with ' + $dns) }
        if ($lvl -eq 'LocalAccess') { Ok 'Windows says "No internet" again, no sign-in notice' } else { Problem ('Windows says ' + $lvl + ' after joining again') }
    }

    Step 'd) AgOpenWeb still talks to the module through the bridge'
    $s1 = GetJson (Url $a '/api/state'); Start-Sleep -Seconds 5; $s2 = GetJson (Url $a '/api/state')
    if ($null -eq $s1 -or $null -eq $s2) { Problem 'the XIAO stopped answering' }
    elseif (-not $s2.bridge.on) { Check 'the bridge is off on the XIAO: d) needs the WiFi bridge on' }
    elseif ([long]$s2.bridge.in -gt [long]$s1.bridge.in -and [long]$s2.bridge.out -gt [long]$s1.bridge.out) { Ok ('packets both ways through the bridge (' + ([long]$s2.bridge.in - [long]$s1.bridge.in) + ' in, ' + ([long]$s2.bridge.out - [long]$s1.bridge.out) + ' out in 5 s; program ' + $s2.bridge.prog + ')') }
    else { Problem 'no packets through the bridge in 5 s: is AgOpenWeb open on this computer?' }
    Todo 'Step c) (a new phone on the XIAO''s Wi-Fi still opens the page by itself) stays by hand.'
}

switch ($Test) {
    '7'  { Test7 }
    '17' { Test17 }
    '42' { Test42 }
    '44' { Test44 }
    '46' { Test46 }
    '48' { Test48 }
}
Finish
