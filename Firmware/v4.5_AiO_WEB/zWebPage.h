/* zWebPage.h - the module's web page (English).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   25 Oct 2026: Machine tab for any standard machine module (official Machine_UDP_v5 or sprayer Nano): sections from
   the hello reply, Sprayer card only with PGN 160, up to 16 sections.
   26 Oct 2026: the buttons the module refuses while autosteer is on (or the tractor moves) are greyed out then,
   with a line at the top of the tab saying why. Machine card fits a phone screen: the Hello replies row wraps,
   the checksum wording is a note under the list. The GPS2 port is automatic or off (Settings), and says what it found,
   in one short line. The section squares stay as many as the highest section used since the module started (they
   fell back to 5 when every section was off).
   27 Oct 2026: the Sprayer card also hides when the module stops sending PGN 160 (a sprayer module set to no sensors,
   or another module) and still sends the rest; its wording says module, not Nano.
   27 Oct 2026: up to 64 section squares on the Machine tab (PGN 229 from the program, the seed drill module's
   PGN 237); cmd and sec are hex digits now. The sections of any module (its PGN 237 in the AiO v26 layout) and of
   several modules together; their IPs in the IP row.
   27 Oct 2026: a negative WAS reading on the differential input (nothing on AmpSeal pin 3): the WAS row and the
   warning at the top say so and to choose Single in the program; the WAS check setting mentions it.
   27 Oct 2026: "Last turned off" by the program says that its screen gives the reason (button, boundary, speed, RTK).
   31 Oct 2026: the GPS fix is green whenever the receiver has a position (GPS, DGPS, RTK float or fixed), red only without one.
   31 Oct 2026: the GitHub check also compares when the firmware was compiled (firmware.json on the simulator's site), so
   a new build of the same version (4.5) shows as new; Install from GitHub takes vX.Y_AiO_WEB.hex or .ino.hex.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zWebPage.h
// The module's web page (served by zWeb.ino): one self-contained HTML file, no internet needed.
// Kept in flash (PROGMEM). In English (the Portuguese version and its language button are gone, 20 Oct 2026).
// Dark, or light when the device asks for it; the header button switches and this browser remembers it.
// Tabs: Status (GNSS, dual heading from the UM982 or a second F9P, IMU, steering with the Keya, the
// steering settings from the program, connections, RTCM corrections from the program and the radio,
// sentences sent, the XIAO; warnings at the top), Messages, Settings (with the GPS2 port and the
// simulator mode; the module's subnet is changed from the XIAO's WiFi page or by the program, not here), GPS (the
// receiver's model, zReceiver.ino; the u-blox F9P / X20P settings; the UM982 settings, read with
// (the AgOpenGPS settings of the F9P are AOGConfigOMatic's, those of the X20P X20P.txt, zRxConfig.h)
// CONFIG, MASK, UNILOGLIST and MODE and written one command at a time, then SAVECONFIG, with the
// console below; only the options that matter to AgOpenWeb / AgOpenGPS, and the KSXT; the other
// NMEA sentences / messages are only named when the receiver sends them; the receiver check, zRxCheck.ino:
// data at 460800, baud rate search, what the Teensy needs, the AgOpenGPS settings loaded) and Firmware (start reason, watchdog, a newer version on GitHub - renatorgr/Boards,
// folder Firmware - installed through the same update as a file, zOTA.ino; a chosen .hex shows its version
// and build date and time before it is sent; the module itself never
// goes to the internet), and a report for an analysis (every answer of the module in one .json file).
#ifndef ZWEBPAGE_H
#define ZWEBPAGE_H

const char WEB_PAGE[] PROGMEM = R"WEBPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark light">
<title>Teensy 4.1 Settings</title>
<style>
/* dark (as before); light when the device asks for it, or chosen with the header button (data-theme) */
:root{color-scheme:dark;--bg:#121511;--card:#1b1f1a;--ink:#e7ebe3;--mute:#9aa395;--line:#2e352b;--ok:#66bb6a;--warn:#ffb74d;--bad:#ef5350;--acc:#8cc068;--accInk:#10140f;--soft:#232a21;--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
:root[data-theme=light]{color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}
@media (prefers-color-scheme:light){:root:not([data-theme]){color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);accent-color:var(--acc);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{display:flex;align-items:center;gap:10px;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line);position:sticky;top:0;z-index:3}
header h1{font-size:16px;margin:0;font-weight:650;white-space:nowrap}
header .sub{color:var(--mute);font-size:13px;flex:1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.dot{width:11px;height:11px;border-radius:50%;background:var(--bad);flex:none}
.dot.on{background:var(--ok)}
nav{display:flex;gap:6px;padding:8px 12px;overflow-x:auto;background:var(--card);border-bottom:1px solid var(--line)}
nav button{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:999px;padding:7px 14px;font:inherit;font-size:14px;cursor:pointer;white-space:nowrap}
nav button.on{background:var(--acc);border-color:var(--acc);color:var(--accInk)}
main{padding:14px 16px 40px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:12px}
.grid.fill{grid-template-columns:repeat(auto-fill,minmax(270px,1fr))}
.grid.gps>form{display:contents}
.grid.gps>form>.row,.grid.gps>form>p,.grid.gps>form>details{grid-column:1/-1;margin:0}
#fs .card{margin:0 0 12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;min-width:0}
.card h2{font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--mute);margin:0 0 8px;font-weight:650}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:0}
dt{color:var(--mute)}
dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
dd.nw{white-space:nowrap;overflow-wrap:normal}
.big{font-size:22px;font-weight:650;font-variant-numeric:tabular-nums}
.pill{display:inline-block;padding:1px 9px;border-radius:999px;font-size:13px;font-weight:600;background:var(--soft)}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.pill.ok{background:color-mix(in srgb,var(--ok) 16%,transparent)}
.pill.warn{background:color-mix(in srgb,var(--warn) 18%,transparent)}
.pill.bad{background:color-mix(in srgb,var(--bad) 16%,transparent)}
.mono{font-family:var(--mono);font-size:12.5px;white-space:pre-wrap;overflow-wrap:anywhere}
.note{color:var(--mute);font-size:13.5px;margin:6px 0 0}
#ck0 .note.bad,#ck1 .note.bad,.note.bad{color:var(--bad)}.note.warn{color:var(--warn)}.note.ok{color:var(--ok)}
.box{background:var(--soft);border-radius:8px;padding:8px 10px;margin-top:8px}
/* Machine tab sections: squares in the AgOpenWeb section colours (its SECTION_COLORS) */
.msecs{display:grid;grid-template-columns:repeat(var(--n,5),1fr);gap:4px;margin-top:8px}
.msecs span{height:34px;border-radius:5px;background:var(--soft);color:var(--mute);font-weight:700;display:flex;align-items:center;justify-content:center}
.msecs.old span{opacity:.4}
.s-off{background:rgba(242,51,51,.85)!important;color:#111!important}
.s-on{background:rgba(0,242,0,.85)!important;color:#111!important}
.s-goingon{background:rgba(255,165,0,.9)!important;color:#111!important}
.s-goingoff{background:rgba(0,222,222,.85)!important;color:#111!important}
.mleg{display:flex;flex-wrap:wrap;gap:4px 12px;margin-top:8px;font-size:12.5px;color:var(--mute)}
.mleg i{display:inline-block;width:11px;height:11px;border-radius:3px;margin-right:5px;vertical-align:-1px}
#log,#umOut{height:62vh;overflow:auto;background:var(--card);border:1px solid var(--line);border-radius:12px;padding:8px 10px}
#umOut{height:30vh}
#umOut:empty{height:6.5em}
.ll{display:flex;gap:10px;padding:2px 0;border-bottom:1px dashed var(--line)}
.ll .t{color:var(--mute);flex:none;font-variant-numeric:tabular-nums;width:86px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}
button.act,label.act{border:0;background:var(--acc);color:var(--accInk);border-radius:8px;padding:9px 16px;font:inherit;font-weight:600;cursor:pointer}
button.sec{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button.danger{border:1px solid var(--bad);background:transparent;color:var(--bad);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button:disabled{opacity:.45;cursor:default}
button.locked{opacity:.45;cursor:not-allowed}
.card.dim{opacity:.55}
input:disabled,select:disabled,label.opt:has(input:disabled){opacity:.5;cursor:default}
input[type=text]{font:inherit;font-family:var(--mono);padding:9px 10px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);flex:1;min-width:180px}
.opt{display:flex;gap:10px;align-items:flex-start;padding:5px 0}
.opt input{margin-top:4px}
.opt small{display:block;color:var(--mute)}
.opt small.warn{color:var(--warn)}
select{font:inherit;padding:6px 8px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink)}
input[type=number]{font:inherit;padding:6px 8px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);width:6.2em;margin-top:0!important}
.pair{display:flex;gap:6px;align-items:center;white-space:nowrap}
.opt>select{max-width:58%}
details.card summary{cursor:pointer;color:var(--mute);font-size:13.5px}
progress{width:100%;height:14px}
.hidden{display:none!important}
.alert{background:color-mix(in srgb,var(--bad) 18%,transparent);border:1px solid var(--bad);border-radius:12px;padding:9px 12px;margin:0 0 12px;white-space:pre-line;font-weight:600}
.info{background:color-mix(in srgb,var(--acc) 16%,transparent);border:1px solid var(--acc);border-radius:12px;padding:9px 12px;margin:0 0 12px;font-weight:600}
a{color:var(--acc)}
.tbtn{margin-left:auto;flex:none;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:999px;padding:3px 11px;font:inherit;font-size:13px;font-weight:650;cursor:pointer}
@media (max-width:520px){header .sub{display:none}}
</style>
</head>
<body>
<header><span class="dot" id="dot" title="link to the module"></span><h1>Teensy 4.1 Settings</h1><span class="sub" id="hdr">connecting…</span><button class="tbtn" id="theme" type="button" title="Light / dark">☀</button></header>
<nav id="nav"><button data-t="status" class="on">Status</button><button data-t="log">Messages</button><button data-t="settings">Settings</button><button data-t="gps">GPS</button><button data-t="imu">IMU</button><button data-t="machine" id="navMach" class="hidden">Machine</button><button data-t="firmware">Firmware</button></nav>
<main>

<section id="t-status">
<div id="alert" class="alert hidden"></div>
<div id="ghNote" class="info hidden"></div>
<div class="grid">
 <div class="card"><h2>GNSS</h2>
  <div class="big" id="fixName">–</div>
  <dl style="margin-top:6px">
   <dt>Satellites</dt><dd id="sats">–</dd><dt>HDOP</dt><dd id="hdop">–</dd>
   <dt>Correction age</dt><dd id="age">–</dd><dt>Speed</dt><dd id="kmh">–</dd>
   <dt>Latitude</dt><dd id="lat" class="nw">–</dd><dt>Longitude</dt><dd id="lon" class="nw">–</dd><dt>Altitude</dt><dd id="alt" class="nw">–</dd><dt>Checksum errors</dt><dd id="crc">–</dd><dt>Receiver</dt><dd id="rxModel">–</dd>
   <dt id="g1SetT" class="hidden">Setup</dt><dd id="g1Set" class="hidden">–</dd>
  </dl>
  <p class="note hidden" id="g1Go">Receiver check and settings: <a href="#" data-go="gps">GPS tab</a></p></div>
 <div class="card" id="dualCard"><h2>Dual-antenna heading</h2>
  <div class="big" id="dualHdg">–</div>
  <dl style="margin-top:6px"><dt>Status</dt><dd id="dualState">–</dd><dt>Source</dt><dd id="dualSrc">–</dd><dt>Quality (QF)</dt><dd id="qf">–</dd><dt>Antenna roll</dt><dd id="antRoll">–</dd><dt>Antenna distance</dt><dd id="dualBase">–</dd>
   <dt id="f2T" class="hidden">Second F9P</dt><dd id="f2" class="hidden">–</dd><dt id="f2SetT" class="hidden">Setup</dt><dd id="f2Set" class="hidden">–</dd></dl>
  <div class="box mono" id="hprRaw">–</div>
  <p class="note hidden" id="f2Go">Baud rate, check and AgOpenGPS settings: <a href="#" data-go="gps">GPS tab</a></p></div>
 <div class="card"><h2>IMU</h2>
  <div class="big" id="tmHdg">–</div>
  <dl style="margin-top:6px"><dt>Status</dt><dd id="tmState">–</dd><dt>Type</dt><dd id="imuType">–</dd><dt>Roll</dt><dd id="roll">–</dd><dt>Pitch</dt><dd id="pitch">–</dd>
   <dt>Raw yaw</dt><dd id="yaw">–</dd><dt>Alignment</dt><dd id="align">–</dd><dt>Yaw direction</dt><dd id="sign">–</dd>
   <dt id="tmCalT" class="hidden">Gyro calibration</dt><dd id="tmCal" class="hidden">–</dd>
   <dt id="tmSetupT" class="hidden">Setup</dt><dd id="tmSetup" class="hidden">–</dd></dl>
  <p class="note hidden" id="tmGo">Calibration and settings: <a href="#" data-go="imu">IMU tab</a></p></div>
 <div class="card"><h2>Steering</h2>
  <div class="big" id="was">–</div>
  <dl style="margin-top:6px"><dt>Requested angle</dt><dd id="setp">–</dd><dt>PWM</dt><dd id="pwm">–</dd><dt>Autosteer</dt><dd id="active">–</dd>
   <dt>Switch</dt><dd id="sw">–</dd><dt>Speed (program)</dt><dd id="aogKmh">–</dd>
   <dt>WAS sensor</dt><dd id="wasV">–</dd><dt id="sensT">Pressure sensor</dt><dd id="sens">–</dd>
   <dt>Last turned off</dt><dd id="off">–</dd><dt>Message to the program</dt><dd id="hwMsg">–</dd>
   <dt>Keya motor (CAN)</dt><dd id="keya">–</dd><dt>Angle from the GPS</dt><dd id="gpsWas">–</dd></dl></div>
 <div class="card"><h2>Steering settings (from the program)</h2>
  <dl><dt>Received</dt><dd id="scGot">–</dd><dt>Gain (Kp)</dt><dd id="scKp">–</dd><dt>Maximum PWM</dt><dd id="scHi">–</dd><dt>Minimum PWM</dt><dd id="scMin">–</dd>
   <dt>Low PWM</dt><dd id="scLow">–</dd><dt>Counts per degree</dt><dd id="scCounts">–</dd><dt>WAS offset</dt><dd id="scOff">–</dd><dt>Ackermann</dt><dd id="scAck">–</dd>
   <dt>WAS</dt><dd id="scWas">–</dd><dt>Motor</dt><dd id="scMotor">–</dd><dt>Relay active</dt><dd id="scRelay">–</dd><dt>Switch type</dt><dd id="scSw">–</dd>
   <dt>Sensor</dt><dd id="scSens">–</dd><dt>Kickout limit</dt><dd id="scKick">–</dd><dt>IMU axis (roll)</dt><dd id="scAxis">–</dd><dt>Wheelbase</dt><dd id="scWb">–</dd></dl></div>
 <div class="card"><h2>Connections</h2>
  <dl><dt>Ethernet cable</dt><dd id="link" class="nw">–</dd><dt>Program</dt><dd id="aog" class="nw">–</dd><dt>Program IP</dt><dd id="aogIp" class="nw">–</dd>
   <dt>Up for</dt><dd id="up">–</dd></dl>
  <p class="note">The receiver sends (last 5 s):</p><div class="box mono" id="rx">–</div></div>
 <div class="card"><h2>RTCM corrections</h2>
  <dl><dt>NTRIP (program)</dt><dd id="rtcm" class="nw">–</dd><dt>Received</dt><dd id="rtcmKB" class="nw">–</dd><dt>RTCM messages</dt><dd id="rtcmMsg">–</dd><dt>Types</dt><dd id="rtcmTypes">–</dd>
   <dt>Radio (XBee)</dt><dd id="radio" class="nw">–</dd><dt>Received</dt><dd id="radioKB" class="nw">–</dd><dt>RTCM messages</dt><dd id="radioMsg">–</dd><dt>Types</dt><dd id="radioTypes">–</dd></dl>
  <p class="note">Messages with errors: bytes lost or changed on the way (weak radio signal, wrong port speed).</p></div>
 <div class="card"><h2>Sentences sent</h2>
  <dl><dt>Mode</dt><dd id="mode" class="nw">–</dd><dt>Choice</dt><dd id="modeSel">–</dd><dt>Heading sent</dt><dd id="cSrc">–</dd><dt>In AgOpenGPS</dt><dd id="aogSrc">–</dd><dt>Epochs sent</dt><dd id="epochs">–</dd></dl>
  <div class="box mono" id="sent">–</div></div>
 <div class="card" id="xiaoCard"><h2>XIAO ESP32 (Serial2)</h2>
  <div class="big" id="xState">–</div>
  <dl style="margin-top:6px"><dt>Last hello</dt><dd id="xHello" class="nw">–</dd>
   <dt>Packets to the XIAO</dt><dd id="xTo">–</dd><dt>PGNs from the XIAO</dt><dd id="xFrom">–</dd><dt>Last PGN from the XIAO</dt><dd id="xLast">–</dd>
   <dt>Checksum errors</dt><dd id="xCrc">–</dd><dt>Lost</dt><dd id="xDrop">–</dd><dt>Received / sent</dt><dd id="xKB" class="nw">–</dd>
   <dt>XIAO version</dt><dd id="xProg">–</dd><dt>XIAO WiFi</dt><dd id="xAp">–</dd><dt>WiFi network</dt><dd id="xSta">–</dd><dt>IP on the WiFi network</dt><dd id="xIp">–</dd></dl>
  <p class="note hidden" id="xNote"></p>
  </div>
</div>
</section>

<section id="t-machine" class="hidden">
<div class="grid">
 <div class="card"><h2>Machine module</h2>
  <div class="big" id="mState">–</div>
  <dl style="margin-top:6px"><dt>IP</dt><dd id="mIp" class="nw">–</dd><dt>Last packet</dt><dd id="mAgo" class="nw">–</dd>
   <dt>Packets</dt><dd id="mPkts">–</dd><dt>Checksum errors</dt><dd id="mBad">–</dd><dt>Last PGN</dt><dd id="mPgn">–</dd>
   <dt>Hello replies</dt><dd id="mHello">–</dd><dt>Sections from</dt><dd id="mSecFrom" class="nw">–</dd></dl>
  <p class="note hidden" id="mNoCrc" style="margin-top:8px">The hello replies come without a checksum, as the official Machine_UDP_v5 sends them: taken as they are, not counted as checksum errors.</p>
  <p class="note" style="margin-top:8px">Sections: what the program asks for (PGN 254; above 16, PGN 229) and what the machine modules report open, in their PGN 237 (else PGN 160 or the hello reply); several modules together</p>
  <div class="msecs" id="mSec"></div>
  <div class="mleg"><span><i class="s-on"></i>open</span><span><i class="s-off"></i>closed</span><span><i class="s-goingon"></i>asked on, still closed</span><span><i class="s-goingoff"></i>asked off, still open</span></div>
  <p class="note">What the machine module sends to UDP 9999 (to the whole network), as the Teensy hears it: the official AgOpenGPS Machine_UDP_v5, the AiO v26's machine part, the sprayer or seed drill module, or anything that talks like them. Only to look at: nothing here changes the module or the steering.</p></div>
 <div class="card hidden" id="mSprayCard"><h2>Sprayer (PGN 160)</h2>
  <div class="big" id="mBar">–</div>
  <dl style="margin-top:6px"><dt>Flow</dt><dd id="mFlow">–</dd><dt>Total since the module started</dt><dd id="mTotal">–</dd>
   <dt>Main switch</dt><dd id="mMain">–</dd><dt>Last PGN 160</dt><dd id="mSprayAgo" class="nw">–</dd></dl></div>
</div>
</section>

<section id="t-log" class="hidden">
<div class="row"><button class="sec" id="logPause">Pause</button><button class="sec" id="logClear">Clear screen</button><span class="note">[status] messages from the firmware, only when something changes. The module keeps the last 60.</span></div>
<div id="log" class="mono"></div>
</section>

<section id="t-settings" class="hidden">
<form id="fs">
<div class="card"><h2>Operating mode</h2>
 <label class="opt"><input type="radio" name="outputMode" value="2"><span>Automatic<small>Finds out which program is connected, from the «hello» it sends every second, and sends that program's sentences. Until then, the AgOpenGPS ones, which AgOpenWeb reads too.</small></span></label>
 <label class="opt"><input type="radio" name="outputMode" value="0"><span>AgOpenWeb<small>$GNGGA, $GNVTG, $GNTHS (antenna heading) and $INHPR (IMU) every epoch.</small></span></label>
 <label class="opt"><input type="radio" name="outputMode" value="1"><span>AgOpenGPS<small>$PAOGI (UM982 or dual F9P) or $PANDA (X20P / F9P) every epoch.</small></span></label>
 <label class="opt"><input type="checkbox" name="simMode" value="1"><span>Simulator<small class="warn">Only for tests with the program's simulator: the GPS is not sent to the program (AgOpenWeb switches its simulator off while it gets GPS, with or without a fix). Switch it off before going to the field.</small></span></label>
</div>
<div class="card"><h2>Dual-antenna heading (UM982 / dual F9P)</h2>
 <label class="opt"><input type="checkbox" name="hprAcceptFloat" value="1"><span>Accept a float heading (QF 5)<small>Off: only with QF 4 (fixed). UM982 only: the dual F9P uses only the fixed one, as the AiO firmware.</small></span></label>
 <div class="opt"><span style="flex:1">Longest wait for the VTG and heading after the GGA<small>Default 60 ms. Longer delays the epoch; shorter may miss the heading (HPR or RELPOSNED).</small></span><select name="epochWaitMs"><option>20</option><option>30</option><option>40</option><option>50</option><option>60</option><option>70</option><option>80</option><option>100</option></select></div>
</div>
<div class="card"><h2>IMU (TM171 / BNO085)</h2>
 <label class="opt"><input type="checkbox" name="tm171SwapRollPitch" value="1"><span>Swap roll and pitch<small>IMU turned 90° on the board. The "Use Y axis" of AgOpenGPS swaps them too.</small></span></label>
 <div class="opt"><span style="flex:1">Yaw direction<small>Automatic: learned in turns, from the antenna heading or, without it, from the GPS course.</small></span><select name="tm171YawSign"><option value="0">automatic</option><option value="1">+1</option><option value="-1">−1</option></select></div>
</div>
<div class="card"><h2>GPS2 port (Serial2)</h2>
 <div class="opt"><span style="flex:1">On the GPS2 port: <b id="gps2Now">–</b><small>Automatic finds a XIAO or a second F9P by itself. Off ignores the port.</small></span><select name="xiaoOff"><option value="0">automatic</option><option value="1">off</option></select></div>
</div>
</div>
<div class="card"><h2>Autosteer safety</h2>
 <label class="opt"><input type="checkbox" name="wasCheckOn" value="1"><span>Check the WAS sensor<small>If the WAS voltage leaves 0.2–4.8 V (broken wire, short circuit, loose plug), autosteer switches off and stays blocked until the reading is normal again (the steering wheel always works). Switch it off only on a bench with no WAS connected: the Status page and the program warn while it is off, and with the WAS out of range the wheel angle goes to the program as 0°. With the program's A/D converter on Differential, a reading below −0.5 V is a fault too: nothing is wired to AmpSeal pin 3.</small></span></label>
 <label class="opt"><input type="checkbox" name="wdtOn" value="1"><span>Watchdog<small>Restarts the Teensy if the firmware stops for 2 s; half a second before, it switches the autosteer outputs (valve or motor) off. Switching it off takes effect after the Teensy restarts.</small></span></label>
</div>
<div class="card"><h2>Steering</h2>
 <div class="opt"><span style="flex:1">Minimum speed<small>Below it (the program's speed) the wheels are not turned. Default 0.2 km/h.</small></span><select name="steerMinKmhX10"><option value="1">0.1 km/h</option><option value="2">0.2 km/h</option><option value="3">0.3 km/h</option><option value="5">0.5 km/h</option><option value="10">1.0 km/h</option><option value="15">1.5 km/h</option><option value="20">2.0 km/h</option><option value="30">3.0 km/h</option></select></div>
 <div class="opt"><span style="flex:1">Maximum speed<small>Above it autosteer switches off. Default 25 km/h.</small></span><select name="steerMaxKmh"><option value="10">10 km/h</option><option value="15">15 km/h</option><option value="20">20 km/h</option><option value="25">25 km/h</option><option value="30">30 km/h</option><option value="35">35 km/h</option><option value="40">40 km/h</option></select></div>
 <div class="opt"><span style="flex:1">Soft start<small>When autosteer engages, the valve or motor output rises over this time instead of jumping; on a big step or a change of direction, over half of it. Off: as before (default). Try 0.5 s in the field.</small></span><select name="softStartX10ms"><option value="0">off</option><option value="20">0.2 s</option><option value="30">0.3 s</option><option value="50">0.5 s</option><option value="75">0.75 s</option><option value="100">1 s</option></select></div>
 <div class="opt"><span style="flex:1">Current kickout delay<small>Current sensor only (not the Keya): autosteer switches off only when the current stays above the limit this long, so a short peak does not stop it. Default 0.5 s; 0: at once, as before.</small></span><select name="currentKickX10ms"><option value="0">0 (at once)</option><option value="25">0.25 s</option><option value="50">0.5 s</option><option value="100">1 s</option></select></div>
</div><div class="row"><button class="act" type="submit">Save</button><button class="sec" type="button" id="defaults">Restore defaults</button><span id="setMsg" class="note"></span></div>
<p class="note">Saved in the Teensy (EEPROM) and applied at once, without a restart. The steering settings still come from AgOpenWeb / AgOpenGPS.</p>
</form>
</section>

<section id="t-gps" class="hidden">
<p class="note warn hidden" data-locknote="e" style="margin:0 0 10px"></p>
<div class="grid fill gps">
<div class="card"><h2>GPS receiver</h2>
 <div class="big" id="gModel">–</div>
 <dl style="margin-top:6px"><dt>Make</dt><dd id="gKind">–</dd><dt>Firmware</dt><dd id="gFw">–</dd><dt id="gProtT">Protocol</dt><dd id="gProt">–</dd><dt id="gNmeaT" class="hidden">NMEA version</dt><dd id="gNmea" class="hidden">–</dd><dt>Identified</dt><dd id="gId">–</dd></dl>
 <p id="ck0FwNote" class="note bad hidden">Wrong firmware: AgOpenGPS uses HPG 1.13 (the AOGConfigOMatic settings are for it). Download it and install it on the F9P with AOGConfigOMatic («Flash firmware») or u-center: <a href="https://github.com/lansalot/AOGConfigOMatic/raw/main/AOGConfigOMatic/UBX113.bin" target="_blank" rel="noopener">UBX113.bin (HPG 1.13)</a> · <a href="https://github.com/lansalot/AOGConfigOMatic/releases" target="_blank" rel="noopener">AOGConfigOMatic</a></p>
 <div class="row"><button class="sec" type="button" id="gIdent">Ask again</button><button class="sec hidden" type="button" id="gUbxOn">Switch the UBX output on (u-blox)</button></div>
 <p class="note">The Teensy asks the receiver what it is: VERSIONA to the UM982, UBX-MON-VER to the F9P / X20P. The sentences sent to the program do not depend on it.</p></div>
<div class="card" id="ck0"><h2>Receiver check</h2>
 <dl><dt>Data at 460800 baud</dt><dd id="ck0Data">–</dd><dt>Reaching the Teensy</dt><dd id="ck0Hz">–</dd><dt>What the Teensy needs</dt><dd id="ck0Need">–</dd><dt>AgOpenGPS settings</dt><dd id="ck0Ago">–</dd></dl>
 <p id="ck0Warn" class="note bad hidden"></p>
 <div id="ck0Vals" class="hidden"><p class="note">Values read</p><div id="ck0Bad" class="mono warn" style="margin-top:4px"></div>
  <details id="ck0Det"><summary class="note" id="ck0OkSum">–</summary><div id="ck0List" class="mono" style="margin-top:6px"></div></details></div>
 <details id="ck0Cmd" class="hidden"><summary class="note">AgOpenGPS commands (✓ already in the receiver)</summary><div id="ck0CmdList" class="mono" style="margin-top:6px"></div></details>
 <div class="row"><select id="ck0Um" class="hidden"><option value="d">UM982 with two antennas</option><option value="s">UM982 with one antenna</option></select></div>
 <p id="ck0UmNote" class="note hidden">One antenna: Status leaves the dual-antenna heading out. Kept in the Teensy's settings.</p>
 <div class="row"><button class="act hidden" type="button" id="ck0Baud" data-lock="e">Set to 460800 and save</button><button class="sec" type="button" id="ck0Check">Compare with AOGConfigOMatic</button><button class="act hidden" type="button" id="ck0Load" data-lock="e">Load the AgOpenGPS settings</button>
 <!-- "Read all settings" (zRxCheck.ino, GET /api/rxdump) is kept in the Teensy but taken off the page (Renato, 2026-10-24): it
      was made to get X20P.txt, which is in the firmware now (zRxConfig.h). To put it back: this button, the ck0DumpRow block
      below, the same two in the second F9P card, and the lines marked "Read all settings" in renderCk and in the button handlers.
 <button class="sec hidden" type="button" id="ck0Dump">Read all settings</button> --></div>
 <p id="ck0Msg" class="note"></p>
 <!-- <div class="hidden" id="ck0DumpRow"><div class="row"><button class="act" type="button" id="ck0Dl">Download</button></div>
  <p class="note">Every setting of the receiver, read from its RAM and compared with the factory defaults: the ones that differ, as a u-center configuration file (the form of the AOGConfigOMatic files). Put into the firmware (zRxConfig.h), it becomes the AgOpenGPS file of a receiver AOGConfigOMatic has none for.</p></div> -->
 <p class="note" id="ck0Note">Baud rate checked at start-up (460800). Change only with autosteer off.</p></div>
<div class="card hidden" id="ck1"><h2>Second F9P check</h2>
 <dl><dt>Data at 460800 baud</dt><dd id="ck1Data">–</dd><dt>Reaching the Teensy</dt><dd id="ck1Hz">–</dd><dt>Receiver</dt><dd id="ck1Model">–</dd><dt>Firmware</dt><dd id="ck1Fw">–</dd><dt>What the Teensy needs</dt><dd id="ck1Need">–</dd><dt>AgOpenGPS settings</dt><dd id="ck1Ago">–</dd></dl>
 <p id="ck1FwNote" class="note bad hidden">Wrong firmware: AgOpenGPS uses HPG 1.13 (the AOGConfigOMatic settings are for it). Download it and install it on the F9P with AOGConfigOMatic («Flash firmware») or u-center: <a href="https://github.com/lansalot/AOGConfigOMatic/raw/main/AOGConfigOMatic/UBX113.bin" target="_blank" rel="noopener">UBX113.bin (HPG 1.13)</a> · <a href="https://github.com/lansalot/AOGConfigOMatic/releases" target="_blank" rel="noopener">AOGConfigOMatic</a></p>
 <p id="ck1Warn" class="note bad hidden"></p>
 <div id="ck1Vals" class="hidden"><p class="note">Values read</p><div id="ck1Bad" class="mono warn" style="margin-top:4px"></div>
  <details id="ck1Det"><summary class="note" id="ck1OkSum">–</summary><div id="ck1List" class="mono" style="margin-top:6px"></div></details></div>
 <div class="row"><button class="act hidden" type="button" id="ck1Baud" data-lock="e">Set to 460800 and save</button><button class="sec" type="button" id="ck1Check">Compare with AOGConfigOMatic</button><button class="act hidden" type="button" id="ck1Load" data-lock="e">Load the AgOpenGPS settings</button>
 <!-- "Read all settings", see the first card: <button class="sec hidden" type="button" id="ck1Dump">Read all settings</button> --></div>
 <p id="ck1Msg" class="note"></p>
 <!-- <div class="hidden" id="ck1DumpRow"><div class="row"><button class="act" type="button" id="ck1Dl">Download</button></div>
  <p class="note">Every setting of the receiver, read from its RAM and compared with the factory defaults: the ones that differ, as a u-center configuration file (the form of the AOGConfigOMatic files).</p></div> -->
 <p class="note">The second F9P (GPS2 port) sends the RELPOSNED to the Teensy at 460800 baud. Its AgOpenGPS settings are those of DualRelPos.txt.</p></div>
<form id="fu" class="hidden">
<div class="card"><h2>Position</h2>
 <div class="opt"><span style="flex:1">Rate<small>AgOpenGPS and AgOpenWeb use 10 Hz.</small></span><select name="rate"><option value="1000">1 Hz</option><option value="500">2 Hz</option><option value="200">5 Hz</option><option value="125">8 Hz</option><option value="100">10 Hz</option><option value="50">20 Hz</option></select></div>
 <div class="opt"><span style="flex:1">Dynamic model<small>The movement the receiver expects.</small></span><select name="dyn"><option value="0">portable</option><option value="2">stationary</option><option value="3">pedestrian</option><option value="4">automotive</option><option value="5">sea</option><option value="6">airborne &lt;1 g</option><option value="7">airborne &lt;2 g</option><option value="8">airborne &lt;4 g</option><option value="9">wrist</option><option value="10">bike</option><option value="11">lawn mower</option><option value="12">e-scooter</option></select></div>
 <label class="opt"><input type="checkbox" name="hp" value="1"><span>High-precision NMEA<small>More decimals in the latitude, longitude and altitude.</small></span></label>
</div>
<div class="card"><h2>UART1 (wired to the Teensy)</h2>
 <label class="opt"><input type="checkbox" name="gga" value="1"><span>GGA<small>Position. The Teensy needs it.</small></span></label>
 <label class="opt"><input type="checkbox" name="vtg" value="1"><span>VTG<small>Speed and course. The Teensy needs it.</small></span></label>
 <label class="opt"><input type="checkbox" name="inRtcm" value="1"><span>Take RTCM3 corrections<small>From NTRIP and the radio, through the Teensy. Off: no RTK.</small></span></label>
 <p class="note">The Teensy only uses the GGA and the VTG.</p>
 <p id="uOther" class="note warn hidden"></p>
</div>
<div class="card"><h2>Constellations</h2>
 <label class="opt"><input type="checkbox" name="gps" value="1"><span>GPS</span></label>
 <label class="opt"><input type="checkbox" name="glo" value="1"><span>GLONASS</span></label>
 <label class="opt"><input type="checkbox" name="gal" value="1"><span>Galileo</span></label>
 <label class="opt"><input type="checkbox" name="bds" value="1"><span>BeiDou</span></label>
 <label class="opt"><input type="checkbox" name="qzss" value="1"><span>QZSS</span></label>
 <label class="opt"><input type="checkbox" name="sbas" value="1"><span>SBAS</span></label>
 <p class="note">Changing the constellations restarts the receiver's GNSS: RTK is lost for a few seconds.</p>
</div>
<div class="row"><button class="sec" type="button" id="uRead">Read from the receiver</button><button class="act" type="submit" id="uWrite" data-lock="e">Write to the receiver</button><span id="uMsg" class="note"></span></div>
<p class="note">Reads what the receiver is using. Writes only what changed, to the receiver's RAM, BBR and flash (kept after a power off). Only with autosteer off.</p>
</form>
<form id="fm" class="hidden" novalidate>
<div class="card"><h2>Position (UM982)</h2>
 <div class="opt"><span style="flex:1">Rate of the GGA, VTG and HPR<small>AgOpenGPS and AgOpenWeb use 10 Hz. The three always go together.</small></span><select name="rate"><option value="">–</option><option value="1000">1 Hz</option><option value="500">2 Hz</option><option value="200">5 Hz</option><option value="100">10 Hz</option><option value="50">20 Hz</option></select></div>
 <p id="mRateWarn" class="note warn hidden">The VTG or the HPR does not come at the GGA's rate: writing sets them right.</p>
 <div class="opt"><span style="flex:1">Working mode<small>MODE ROVER. Unicore gives survey for precision agriculture.</small></span><select name="mode"><option value="">–</option><option value="0">UAV (default)</option><option value="1">automotive</option><option value="2">survey</option><option value="3">survey, lawn mower</option></select></div>
 <div class="opt"><span style="flex:1">Longest age of the RTK corrections<small>With no corrections for longer, the receiver leaves RTK. UM982 default: 600 s.</small></span><select name="rtkTo"><option value="">–</option><option value="10">10 s</option><option value="30">30 s</option><option value="60">60 s</option><option value="120">120 s</option><option value="300">300 s</option><option value="600">600 s</option></select></div>
 <div class="opt"><span style="flex:1">SBAS<small>Corrections by satellite (EGNOS in Europe), useful only without RTK.</small></span><select name="sbas"><option value="">–</option><option value="0">off</option><option value="1">automatic</option><option value="2">EGNOS</option></select></div>
</div>
<div class="card" id="mHd"><h2>Dual-antenna heading</h2>
 <div class="opt"><span style="flex:1">Heading mode<small>CONFIG HEADING. Default: fixed length.</small></span><select name="hdMode"><option value="">–</option><option value="0">fixed length</option><option value="1">variable length</option><option value="2">static antennas</option><option value="3">low dynamics</option><option value="4">tractor</option></select></div>
 <div class="opt"><span style="flex:1">Distance between the antennas<small>Centre to centre, in cm, and the tolerance. Empty: automatic. Wrong: no heading.</small></span><span class="pair"><input type="number" name="hdLen" min="10" max="20000" step="1" placeholder="auto" style="width:5.2em"><span>±</span><input type="number" name="hdTol" min="1" max="1000" step="1" placeholder="3" style="width:3.9em"><span>cm</span></span></div>
 <div class="opt"><span style="flex:1">Heading correction<small>CONFIG HEADING OFFSET, in degrees. Use this or AgOpenWeb's DualHeadingOffset, not both.</small></span><input type="number" name="hdOff" min="-180" max="180" step="0.01"></div>
 <div class="opt"><span style="flex:1">Pitch correction<small>In degrees.</small></span><input type="number" name="ptOff" min="-90" max="90" step="0.01"></div>
</div>
<div class="card"><h2>Constellations (UM982)</h2>
 <label class="opt"><input type="checkbox" name="gps" value="1"><span>GPS</span></label>
 <label class="opt"><input type="checkbox" name="glo" value="1"><span>GLONASS</span></label>
 <label class="opt"><input type="checkbox" name="gal" value="1"><span>Galileo</span></label>
 <label class="opt"><input type="checkbox" name="bds" value="1"><span>BeiDou</span></label>
 <label class="opt"><input type="checkbox" name="qzss" value="1"><span>QZSS</span></label>
 <p class="note">MASK / UNMASK. Changing the constellations can lose RTK for a few seconds.</p>
</div>
<div class="card"><h2>Messages to the Teensy (<span id="mPortName">–</span>)</h2>
 <label class="opt"><input type="checkbox" name="ksxt" value="1"><span>KSXT<small>KSXT 0.1 (10 Hz) on the Teensy's port. The GGA, VTG and HPR the Teensy needs are in the receiver check.</small></span></label>
 <p id="mKsxtWarn" class="note warn hidden"></p>
 <p id="mMsgOther" class="note warn hidden"></p>
</div>
<div class="row"><button class="sec" type="button" id="mRead">Read from the receiver</button><button class="act" type="submit" id="mWrite" data-lock="e">Write to the receiver</button><span id="mMsg" class="note"></span></div>
<p class="note">Reads with CONFIG, MASK, UNILOGLIST and MODE. Writes only what changed, one command at a time, then SAVECONFIG (kept after a power off). «–»: the receiver did not give the value. Only with autosteer off.</p>
</form>
</div>
<div id="gUm">
<div class="card" style="margin-top:12px">
 <h2>Command for the UM982</h2>
 <div class="row"><input type="text" id="umCmd" placeholder="e.g. UNILOGLIST" autocomplete="off" autocapitalize="characters" spellcheck="false"><button class="act" id="umSend">Send</button></div>
 <div class="row"><span class="note">Fill in:</span><button class="sec" data-c="CONFIG">CONFIG</button><button class="sec" data-c="UNILOGLIST">UNILOGLIST</button><button class="sec" data-c="SAVECONFIG">SAVECONFIG</button></div>
 <p class="note">The command goes to the UM982 port wired to the Teensy. <b>CONFIG</b> shows the configuration and <b>UNILOGLIST</b> the active messages. A change is kept only after <b>SAVECONFIG</b>. Careful with FRESET (back to the factory settings, port speed included) and with changing the port speed: the Teensy stops hearing the receiver.</p>
</div>
<p class="note" style="margin-top:12px">UM982 replies (for 5 s after each command):</p>
<div id="umOut" class="mono"></div>
</div>
</section>

<section id="t-imu" class="hidden">
<p class="note warn hidden" data-locknote="m" style="margin:0 0 10px"></p>
<div class="grid">
 <div class="card"><h2>IMU</h2>
  <dl><dt>Type</dt><dd id="imuType2">–</dd><dt>Status</dt><dd id="tmState2">–</dd><dt>Heading</dt><dd id="tmHdg2">–</dd></dl>
  <p class="note hidden" id="imuNone">–</p>
  <p class="note hidden" id="imuWire">Everything on this tab sends commands to the TM171, so it needs one wire from Teensy pin 20 (TX5) to the TM171 RX pin. The board only wires the other direction.</p></div>
 <div class="card hidden" id="tmCalCard"><h2>TM171 calibration</h2>
  <dl><dt>Gyro calibration</dt><dd id="tmCal2">–</dd></dl>
  <div class="row hidden" id="tmCalRow"><button class="sec" type="button" id="tmCalBtn" data-lock="m">Calibrate gyro</button><span class="note">Takes 20 s. Park the tractor and switch autosteer off; the engine can stay running. The result is kept until the TM171 is powered off.</span></div>
  <div class="row hidden" id="tmInstRow"><button class="sec" type="button" id="tmInstBtn" data-lock="m">New install calibration</button><span class="note">Stored in the TM171's flash memory. Run it once after installing the TM171, or after a large temperature change. Takes about 20 s with the tractor parked and autosteer off. Do not cut power while it runs. When it finishes, switch the whole module off and on (the TM171 has no power switch of its own).</span></div>
 </div>
 <div class="card hidden" id="tmSetCard"><h2>TM171 setup</h2>
  <dl><dt>Setup</dt><dd id="tmSetup2">–</dd></dl>
  <div class="row hidden" id="tmFixRow"><button class="act" type="button" id="tmFixBtn" data-lock="e">Fix TM171 setup</button><span class="note" id="tmFixNote">–</span></div>
  <details id="tmSetDet"><summary class="note" id="tmSetSum">TM171 settings</summary>
   <div style="margin-top:8px">
   <label class="opt"><input type="checkbox" id="tsMag"><span>Enable magnetometer<small>Leave off for AgOpenGPS: heading is aligned to GPS instead.</small></span></label>
   <label class="opt"><input type="checkbox" id="tsGyroErr"><span>GyroErrFilter<small>Accelerometer gain: <input type="number" id="tsAcc" min="0.01" max="20" step="0.01"> (the dial next to GyroErrFilter in ImuAssistant; 3.50 recommended)</small></span></label>
   <label class="opt"><input type="checkbox" id="tsSelfAdapt"><span>Self-Adapt Filter<small>Magnetometer gain: <input type="number" id="tsMagG" min="0.01" max="20" step="0.01"> (the dial next to Self-Adapt; 1.00 recommended)</small></span></label>
   <label class="opt"><input type="checkbox" id="tsStatus"><span>Status output<small>Temperature and signal quality (QoS). Roll-pitch-yaw and UART output are always left on.</small></span></label>
   <div class="row"><span>Boot mode</span><select id="tsBoot"><option value="0">Auto</option><option value="1">Static</option><option value="2">Dynamic</option></select><span class="note">Applies after the module is switched off and on; requires "Save to flash".</span></div>
   <div class="row"><span>Output data rate</span><select id="tsRate"><option value="100">100 Hz</option><option value="50">50 Hz</option><option value="25">25 Hz</option></select><span class="note">25 Hz is the AgOpenGPS forum set-up, 100 Hz what the AiO v6 firmware uses: this firmware takes the newest sample at each GPS fix, so a higher rate only makes that sample fresher. Currently measured: <span id="tsRateNow">–</span>. The TM171's inhibit time (the shortest gap between two packets) is left as read: <span id="tsInhibit">–</span>.</span></div>
   <label class="opt"><input type="checkbox" id="tsSave" checked><span>Save to flash<small>If unchecked, the settings are lost when the TM171 is powered off.</small></span></label>
   <div class="row"><button class="sec" type="button" id="tsRead">Read from TM171</button><button class="sec" type="button" id="tsAog">AgOpenGPS defaults</button><button class="act" type="button" id="tsWrite" data-lock="e">Write to TM171</button></div>
   <p class="note" id="tsMsg">–</p>
   <p class="note">Settings are read from the TM171 at start-up (requires the TX5 wire). UART baud rate: <span id="tsBaud">–</span>. The baud rate is only changed by "Fix TM171 setup".</p>
   </div></details>
 </div>
</div>
</section>

<section id="t-firmware" class="hidden">
<p class="note warn hidden" data-locknote="m" style="margin:0 0 10px"></p>
<div class="grid">
<div class="card"><h2>Running firmware</h2>
 <dl><dt>Firmware</dt><dd id="fw" class="mono">–</dd><dt>Built</dt><dd id="built">–</dd><dt>Board</dt><dd id="target">–</dd><dt>Last start</dt><dd id="bootWhy">–</dd><dt>Watchdog</dt><dd id="wdt">–</dd></dl>
 <div class="row"><button class="danger" id="reboot">Restart Teensy</button></div></div>
<div class="card"><h2>Version on GitHub</h2>
 <dl><dt>On GitHub</dt><dd id="ghVer">–</dd></dl>
 <div class="row"><button class="act" id="ghInstall" data-lock="m" disabled>Install from GitHub</button><button class="sec" id="ghCheck">Check now</button><a id="ghLink" href="https://github.com/renatorgr/Boards/tree/main/Firmware" target="_blank" rel="noopener">See on GitHub</a></div>
 <p class="note">The device this page is open on (tablet, phone, PC) needs internet. The Teensy does not go to the internet: it gets the file from this device and checks it as when a file is installed.</p></div>
<div class="card"><h2>Update firmware</h2>
 <div class="row"><label class="act">Choose .hex<input type="file" id="hexFile" accept=".hex" class="hidden"></label><span id="hexName" class="note">no file</span></div>
 <p id="hexInfo" class="note hidden"></p>
 <label class="opt"><input type="checkbox" id="force"><span>Other firmware<small>Installs it even without this update page inside (the next update will then have to be by USB).</small></span></label>
 <div class="row"><button class="act" id="install" data-lock="m" disabled>Install</button></div>
 <progress id="prog" max="100" value="0" class="hidden"></progress>
 <p id="fwMsg" class="note"></p>
 <p class="note">Only with autosteer off and the tractor stopped (on a bench, with the simulator mode on or no WAS, also while the program's simulator drives). The whole file is checked before it replaces the current firmware; if anything fails, the current one keeps running. In the last seconds of the installation <b>do not switch the power off</b>. The saved settings are kept.</p></div>
<div class="card"><h2>Report for an analysis</h2>
 <p class="note">Puts the status, the messages, the settings, the GPS receiver and the Teensy's inner workings in one file, to send when something goes wrong. The file is downloaded by this browser.</p>
 <div class="row"><button class="sec" type="button" id="dumpBtn">Download the report</button><span id="dumpMsg" class="note"></span></div>
</div>
</div>
</section>
</main>
<script>
"use strict";
const $=id=>document.getElementById(id);
let lastS=null,lastSet=null,tab="status",online=false,logAfter=0,logPaused=false,umAfter=0,umUntil=0,lastUp=0,upBefore=0,waitingBoot=false,busy=false;
const fmt=(v,d,u)=>(v===null||v===undefined||Number.isNaN(v))?"–":Number(v).toFixed(d)+(u||"");
// The kickout sensor (pressure, current, Keya slip) and its trip point go 0-255 between the program and the
// module; AgOpenWeb / AgOpenGPS show them in % (the trip point x 2.55 = the byte), so the page does too.
const pct255=v=>fmt(v===null||v===undefined?v:v*100/255,0," %");
function set(id,txt,cls){const e=$(id);if(!e)return;e.textContent=txt;e.classList.remove("ok","warn","bad");if(cls)e.classList.add(cls);}
// a value with a link after it (the XIAO's page, the module's page at its real IP): url "" = text only
function setLink(id,txt,url,cls){set(id,txt,cls);if(!url)return;const e=$(id),a=document.createElement("a");a.href=url;a.textContent=url;e.appendChild(document.createTextNode(txt?" · ":""));e.appendChild(a);}
function pill(id,txt,cls){const e=$(id);e.textContent="";const s=document.createElement("span");s.className="pill "+(cls||"");s.textContent=txt;e.appendChild(s);}
async function getJ(u,o){const r=await fetch(u,Object.assign({cache:"no-store"},o||{}));let j={};try{j=await r.json();}catch(e){}if(!r.ok&&!j.error)j.error="HTTP "+r.status;return j;}
function setOnline(v){online=v;$("dot").classList.toggle("on",v);}
function dur(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?h+" h "+m+" min":m?m+" min "+(s%60)+" s":s+" s";}
const OFF={keya:"Keya motor slip",keyafault:"Keya motor fault",angle:"wheel angle above 50°",fast:"speed above the maximum",keyalost:"Keya not responding",switch:"steer switch",button:"steer button",pressure:"pressure kickout",current:"current kickout",encoder:"wheel encoder",was:"WAS fault",silent:"the program stopped sending",program:"by the program (its screen says why: its button, boundary, speed, RTK)",speed:"speed below 0.1 km/h",other:"other"};
const BW={power:"power on (or button / USB)",upload:"programming",restart:"restart requested (page, update or program)",watchdog:"watchdog (the firmware stopped)",crash:"firmware fault","crash-wdt":"firmware fault, restarted by the watchdog",temp:"high temperature",other:"other",unknown:"other"};
const WHYF={1:"no RELPOSNED from the second F9P (GPS2 port: UART1 at 460800, UBX-NAV-RELPOSNED on)",2:"the RELPOSNED does not come with the GGA",3:"the second F9P has no RTK fixed solution between the antennas",4:"empty heading"};
const WHY={1:"no HPR from the receiver (single antenna, or HPR off in the UM982)",2:"the HPR does not arrive with the GGA",3:"the UM982 has no heading solution from its two antennas",4:"empty heading in the HPR"};

const ago=s=>(dur(s)+" ago");

// ---- light / dark: the device's setting until the header button is used, then this browser's choice
function themeNow(){return document.documentElement.dataset.theme||(matchMedia("(prefers-color-scheme: light)").matches?"light":"dark");}
function themeShow(){$("theme").textContent=themeNow()==="light"?"☾":"☀";}
try{const t=localStorage.getItem("theme");if(t==="light"||t==="dark")document.documentElement.dataset.theme=t;}catch(e){}
$("theme").addEventListener("click",()=>{const t=themeNow()==="light"?"dark":"light";document.documentElement.dataset.theme=t;try{localStorage.setItem("theme",t);}catch(e){}themeShow();});
themeShow();

// what the Teensy needs from a receiver (the receiver check's summary in /api/status, zRxCheck.ino): "gga, vtg" as
// key ids for a u-blox, plain text for the UM982
function ckMissText(c){return c.um?c.miss:c.miss.split(",").map(x=>ckName(x.trim())).join(", ");}
// the Setup row of the GNSS and dual-antenna cards: [text, class]
function ckSetup(c){
 const by=c.auto?"the Teensy is ":"";
 if(c.op==="baud")return [by+"setting it to 460800 baud…",""];
 if(c.op==="load")return [by+"loading the AgOpenGPS settings…"+(c.at&&c.at!==460800?" (at "+c.at+" baud; they set 460800)":""),""];
 if(c.op==="scan")return ["finding its baud rate…",""];
 if(c.op==="check")return ["checking…",""];
 if(c.noAnswer)return ["no answer to the check"+(c.um?"":" (UBX output off on its UART1?)"),"bad"];
 if(c.load==="load-fail")return ["the AgOpenGPS settings were not taken"+(c.autoDid==="load"?" (the Teensy tried)":""),"bad"];
 if(!c.checked)return [c.health==="ok"?"checking…":"not checked yet",""];
 if(c.miss)return ["missing: "+ckMissText(c),"bad"];
 return ["all good"+(c.autoDid==="load"&&(c.load==="loaded"||c.load==="load-part")?" · AgOpenGPS settings loaded by the Teensy "+ago(c.loadAgo):""),"ok"];}
// the warnings at the top of Status for a receiver being set up, or not set up: who = "The GPS receiver" / "The second F9P"
function ckAlerts(c,who,al){
 if(!c)return;
 const miss=c.miss?ckMissText(c):"";
 if(c.op==="baud"&&c.auto)al.push(who+" answered at another baud rate: the Teensy is setting it to 460800 baud and saving that in the receiver.");
 else if(c.op==="load"&&c.auto)al.push(who+" lacks settings the Teensy needs ("+miss+"): the Teensy is loading the AgOpenGPS settings ("+c.file+") into it"+(c.at&&c.at!==460800?", at "+c.at+" baud; they set it to 460800":"")+". The GPS can drop out for a few seconds.");
 else if(c.at&&c.at!==460800&&!c.op)al.push(who+" answers at "+c.at+" baud: the port stays at that speed while the Teensy identifies it and loads the AgOpenGPS settings (they set 460800).");
 else if(c.load==="load-fail"&&c.autoDid==="load")al.push("The Teensy tried to load the AgOpenGPS settings ("+c.file+") into "+who.replace(/^The /,"the ")+" and it did not take them. See the GPS tab.");
 else if(c.checked&&c.miss)al.push(c.autoDid==="load"?"The Teensy loaded the AgOpenGPS settings into "+who.replace(/^The /,"the ")+" ("+c.loadOk+" of "+c.loadTotal+" taken), but it still lacks: "+miss+". See the GPS tab.":who+" is not set up for AgOpenGPS (missing: "+miss+"). The Teensy loads the AgOpenGPS settings ("+c.file+") into it as soon as autosteer is off; or see the GPS tab.");}

function render(s){
 lastS=s;lastUp=s.up;
 $("hdr").textContent=(s.fw.lastIndexOf(" ")>0?s.fw.slice(0,s.fw.lastIndexOf(" ")):s.fw)+" · "+s.ip;
 const g=s.gnss,fixCls=g.gga&&g.fix>0?"ok":"bad";   // green with a position (the name says how good), red without one
 set("fixName",g.gga?(g.fixName):"no data from the receiver",fixCls);
 set("sats",g.sats||"–");set("hdop",g.hdop||"–");set("age",g.age?g.age+" s":"–");
 set("kmh",fmt(g.kmh,1," km/h")+(g.speedSrc!=="-"?" ("+(g.speedSrc)+")":""));
 set("lat",g.lat!==null?g.lat.toFixed(7)+"°":"–");set("lon",g.lon!==null?g.lon.toFixed(7)+"°":"–");
 set("alt",(g.lat!==null&&g.alt)?Number(g.alt).toFixed(2)+" m":"–");
 set("crc",String(g.crcErr),g.crcErr?"warn":"");
 set("rxModel",g.model?g.model+(g.rxFw?" · "+g.rxFw:""):"–");
 const d=s.dual,f2=d.f9p2||null;
 // AiO_WEB: the dual heading comes from the UM982 (HPR) or from a second F9P on the GPS2 port (RELPOSNED)
 const f9=d.src==="f9p"||(!d.src&&!!f2&&f2.on);
 $("dualCard").classList.toggle("hidden",!!g.single);   /* one antenna (F9P / X20P alone, UM980 / UM981, UM982 set to one): no card */
 set("dualHdg",d.ok?Number(d.heading).toFixed(2)+"°":"–",d.ok?"ok":"");
 if(d.ok)pill("dualState",d.qf===4?"OK (fixed)":"OK (float)","ok");
 else if(!d.hpr)pill("dualState",(f9?"no RELPOSNED":"no HPR"),"warn");
 else pill("dualState","no heading","bad");
 set("dualSrc",f9?"second F9P (RELPOSNED)":d.src?"UM982 (HPR)":"–");
 set("qf",d.hpr?String(d.lastQf):"–");set("antRoll",d.hpr?fmt(d.antRoll,2,"°"):"–");
 set("dualBase",f9&&f2&&f2.frames?fmt(f2.baseline,1," cm")+" · carrSoln "+(f2.carr===2?"fixed":f2.carr===1?"float":"no solution"):"–",f9&&f2&&f2.frames&&f2.carr!==2?"warn":"");
 const W=f9?WHYF:WHY,nmeaOnly=!!(f9&&f2&&f2.nmea&&!f2.frames);   /* the F9P sends NMEA: not set up as the second F9P */
 $("hprRaw").textContent=d.ok?(d.raw||"–"):(W[d.why]?(W[d.why])+"\n":"")+(d.raw||(f9?"no RELPOSNED received"+(f2&&f2.bytes?" ("+f2.bytes+" bytes, "+f2.bad+" with errors)":"")+(nmeaOnly?"\nIt sends NMEA sentences instead: not set up as the second F9P yet.":""):"no HPR received"));
 // the second F9P itself: its data, model and firmware, and whether it has what the Teensy needs (zRxCheck.ino's summary)
 const c2=f9&&f2&&f2.on?d.ck2:null;
 for(const id of ["f2T","f2","f2SetT","f2Set","f2Go"])$(id).classList.toggle("hidden",!c2);
 if(c2){
  let t="–",cls="";
  if(c2.health==="ok"){t=c2.model?c2.model+(c2.fw?" · "+c2.fw:""):"answering (model not read yet)";cls=c2.fwSt==="older"||c2.fwSt==="newer"?"warn":"ok";}
  else if(c2.health==="junk"){t=c2.found&&c2.found!==460800?"wrong baud rate (it answers at "+c2.found+" baud)":"wrong baud rate (nothing valid at 460800)";cls="bad";}
  else if(c2.health==="none"){t=c2.notFound?"nothing arriving (no baud rate gave data)":"nothing arriving";cls="warn";}
  set("f2",t,cls);
  const u=ckSetup(c2);set("f2Set",u[0],u[1]);}
 // the GPS receiver's own Setup row (its check's summary), once it is identified as a u-blox or a UM982
 const c1=g.ck&&(g.rxKind==="ublox"||g.rxKind==="um982")?g.ck:null;
 for(const id of ["g1SetT","g1Set","g1Go"])$(id).classList.toggle("hidden",!c1);
 if(c1){const u=ckSetup(c1);set("g1Set",u[0],u[1]);}
 const t=s.tm;
 set("tmHdg",t.ok?fmt(t.out,2,"°"):"–",t.ok?"":"bad");
 pill("tmState",t.ok?"OK":t.seen?"no data":"not detected",t.ok?"ok":"bad");
 set("imuType",t.type?t.type+" · "+t.port:"not detected",t.type?"":"bad");
 set("roll",t.ok?fmt(t.roll,2,"°"):"–");set("pitch",t.ok?fmt(t.pitch,2,"°"):"–");set("yaw",t.ok?fmt(t.yaw,2,"°"):"–");
 set("align",t.aligned?"with the antennas"+" ("+fmt(t.offset,1,"°")+")":"not aligned");
 {const tm=t.type==="TM171",busy=["starting","running","finishing"].includes(t.calib),inst=t.calibKind==="install";
  for(const id of ["tmCalT","tmCal","tmSetupT","tmSetup","tmGo","tmCalRow","tmInstRow","tmCalCard","tmSetCard","imuWire"])$(id).classList.toggle("hidden",!tm);
  $("imuNone").classList.toggle("hidden",tm);set("imuNone",t.type?t.type+": nothing to set up here.":"No IMU found.");
  set("imuType2",t.type?t.type+" · "+t.port:"none");pill("tmState2",t.ok?"OK":t.seen?"no data":"not detected",t.ok?"ok":"bad");set("tmHdg2",t.ok?fmt(t.out,2,"°"):"–");
  if(tm&&t.set)renderTmSet(t.set);
  const noWire=tm&&t.set&&t.set.noWire,why=tm&&t.set?t.set.setupWhy:"",fixed=tm&&t.set&&!why&&t.set.fixAgo>=0&&t.set.fixAgo<120;
  $("tmFixRow").classList.toggle("hidden",!(why||noWire||fixed));
  let su="–",suc="";
  if(noWire){su="RX pin not wired to Teensy TX5: nothing can be changed from here";suc="bad";
   set("tmFixNote","No reply from the TM171: its RX pin is not wired to Teensy TX5 (pin 20). The TM171 still sends roll, pitch and yaw, but calibration, settings and setup cannot be changed from this page. Solder one wire from Teensy pin 20 to the TM171 RX pin, then restart the module.","bad");$("tmFixBtn").disabled=true;}
  else if(why){su="not set up for AgOpenGPS: "+why;suc="warn";
   if(t.set.pending)set("tmFixNote","Writing…","");
   else set("tmFixNote",t.set.valid?"Click Fix TM171 setup: it writes 115200 baud and the AgOpenGPS settings to the TM171's flash and reads them back. Then switch the whole module off and on and run New install calibration.":"Waiting for the TM171's settings (needs the TX5 wire). Until they are read, nothing can be written.","warn");
   $("tmFixBtn").disabled=!t.set.valid||!!t.set.pending;}
  else if(fixed){su="written and saved: switch the module off and on, then run New install calibration";suc="ok";set("tmFixNote","TM171 setup written and saved. Switch the whole module off and on, then run New install calibration (tractor parked, engine running).","ok");$("tmFixBtn").disabled=true;}
  else if(tm&&t.set&&t.set.valid){su="AgOpenGPS";suc="ok";}
  else if(tm&&t.set&&t.set.pending){su="reading…";}
  else if(tm){su="not read yet";}
  set("tmSetup",su,suc);set("tmSetup2",su,suc);
  const CAL=inst?{running:"new install calibration in progress. Keep still, do not cut power…",ok:"new install calibration saved to flash. Switch the whole module off and on now.",failed:"new install calibration failed (did the tractor move?). Switch the module off and on, then try again.",noanswer:"no reply from the TM171 (is the TX5 wire connected?)"}
   :{starting:"starting…",running:"in progress, "+(t.calibLeft>=0?t.calibLeft+" s left":""),finishing:"finishing…",ok:"OK",failed:"failed (the tractor moved, or too much vibration)",noanswer:"no reply from the TM171 (is the TX5 wire connected?)",aborted:"stopped"};
  const calTxt=t.calib?(CAL[t.calib]||t.calib)+(!busy&&t.calibAgo>=0?" ("+(t.calibAgo<90?t.calibAgo+" s":Math.round(t.calibAgo/60)+" min")+" ago)":""):"not run since power-on",calCls=t.calib==="ok"?"ok":busy?"":t.calib?"warn":"";
  set("tmCal",calTxt,calCls);set("tmCal2",calTxt,calCls);
  $("tmCalBtn").textContent=busy&&!inst?"Stop":"Calibrate gyro";$("tmCalBtn").disabled=busy&&inst;$("tmInstBtn").disabled=busy;}
 set("sign",(t.sign>0?"+1":"−1")+" ("+(t.locked?"fixed":"learning")+")");
 const st=s.steer;
 set("was",fmt(st.was,1,"°"));set("setp",fmt(st.set,1,"°"));set("pwm",String(st.pwm));
 pill("active",(st.active?"steering":"off"),st.active?"ok":"");
 set("sw",(st.switchOn?"on":"off"));set("aogKmh",fmt(st.aogKmh,1," km/h"));
 // WAS check, kickout sensor, why it went off, the steering settings, warnings at the top
 const sc=s.steerCfg,b=s.boot;
 if(st.wasless)set("wasV","no WAS: angle from the Keya position");
 else if(!st.adc)set("wasV","no ADS1115","bad");
 else{const neg=st.wasOut&&sc&&!sc.singleWas&&st.wasV<-0.5;   // differential input, WAS_H under WAS_L: nothing on pin 3
  set("wasV",fmt(st.wasV,2," V")+" · "+st.wasRaw+(neg?" · negative on the differential input: nothing on pin 3? Choose Single in the program":st.wasOut?" · "+"out of range":"")+(!st.wasCheck?" · "+"check off":"")+(st.wasZero?" · "+"angle sent: 0°":""),st.wasOut?"bad":!st.wasCheck?"warn":"");}
 set("sensT",(st.sensor==="current"?(st.motor==="Keya"?"Keya slip":"Current sensor"):"Pressure sensor"));
 set("sens",pct255(st.sensorV),st.sensor&&st.sensorV>=st.kick?"bad":"");   // the value read; the cut is in the steering settings card
 let ot="–";
 if(st.off){ot=(OFF[st.off]||st.off);if(st.off==="pressure"||st.off==="current"||st.off==="keya")ot+=" ("+pct255(st.offVal)+" / "+pct255(st.kick)+")";else if(st.off==="keyafault")ot+=" (0x"+(st.offVal>>>0).toString(16).toUpperCase().padStart(4,"0")+")";else if(st.off==="angle")ot+=" ("+st.offVal+"°)";else if(st.off==="fast")ot+=" ("+st.offVal+" km/h)";else if(st.off==="encoder")ot+=" ("+st.offVal+" pulses"+")";if(st.offAgo>=0)ot+=" ("+ago(st.offAgo)+")";}
 set("off",ot,st.off==="keyafault"?"bad":["pressure","current","encoder","was","silent","keya","angle","fast","keyalost"].includes(st.off)?"warn":"");
 set("hwMsg",b&&b.hwMsg?b.hwMsg:"–");
 const k=st.keya;
 if(!k||!k.found){set("keya","not detected");set("gpsWas","–");}
 else{const live=k.ago>=0&&k.ago<1000;
  set("keya",(k.lost?"NOT RESPONDING"+" · ":live?"":"no heartbeat"+" · ")+(k.lostCount?"lost "+k.lostCount+"× · ":"")+"position "+k.pos+" · "+"current "+k.current+" · "+"slip "+pct255(k.slip)+(k.faults?" · "+k.faults:""),k.faults||!live?"bad":"");
  set("gpsWas",k.gpsWasOk?fmt(k.gpsWas,1,"°"):"only above 1 km/h, with a heading");}
 if(sc){
  set("scGot",sc.got252>=0?ago(sc.got252):"not yet (saved in the Teensy)");
  set("scKp",String(sc.kp));set("scHi",String(sc.hiPwm));set("scMin",String(sc.minPwm));set("scLow",String(sc.lowPwm));
  set("scCounts",fmt(sc.counts,0));set("scOff",String(sc.offset));set("scAck",fmt(sc.ackerman,0));
  set("scWb",fmt(sc.wheelbase,2," m")+" · "+(sc.got247>=0?"from AgOpenWeb, "+ago(sc.got247):"firmware default (AgOpenWeb has not sent it)"));
  set("scWas",(sc.invertWas?"inverted":"normal")+" · "+(sc.singleWas?"single input":"differential"));
  set("scMotor",st.motor+(sc.danfoss&&st.motor!=="Danfoss"?" · Danfoss":"")+(sc.motorDir?" · "+"direction reversed":""));
  set("scRelay",(sc.relayHigh?"high":"low"));
  set("scSw",(sc.sw?"steer switch":sc.btn?"steer button":"none (the program's button)"));
  set("scSens",[sc.pressure?"pressure":"",sc.current?"current":"",sc.encoder?"encoder":""].filter(x=>x).join(" · ")||"none");
  set("scKick",sc.pressure||sc.current?pct255(sc.kickMax):sc.encoder?sc.kickMax+" pulses":"–");
  set("scAxis",sc.yAxis?"Y":"X");
 }
 const al=[];
 if(!st.adc)al.push("ADS1115 not found: no WAS reading, autosteer is off.");
 else if(st.wasBlock)al.push(sc&&!sc.singleWas&&st.wasV<-0.5?"Autosteer blocked: WAS "+fmt(st.wasV,2," V")+" on the differential input. A negative reading means nothing is wired to WAS_L (AmpSeal pin 3): a 3-wire sensor needs the program's A/D converter set to Single (AutoSteer, Hardware), or its ground wired to pin 3 too. The steering wheel works as usual.":("Autosteer blocked: WAS fault ("+fmt(st.wasV,2," V")+"). The steering wheel works as usual. Check the sensor, its wire and its plug."));
 if(st.keyaBlock)al.push(st.keyaLost?"Autosteer blocked: the Keya motor does not answer on the CAN bus. Check the CAN cable, its plug and the motor's power.":"Keya answers again: turn the steer switch off and on to engage autosteer.");
 if(!st.wasCheck)al.push("WAS check off (bench): a broken WAS wire does not switch autosteer off.");
 if(s.out.sim)al.push("Simulator mode on (for tests only): the GPS is not sent to the program. Switch it off in Settings before going to the field.");
 if(b&&/watchdog|crash/.test(b.why))al.push("The Teensy restarted after a firmware fault (see Firmware and Messages).");
 if(g.ckH==="junk")al.push("The GPS receiver is at the wrong baud rate (nothing valid at 460800 baud). See GPS, Receiver check.");
 if(g.ck2H==="junk")al.push("The second F9P is at the wrong baud rate (nothing valid at 460800 baud). See GPS, Second F9P check.");
 if(g.ck&&(g.rxKind==="ublox"||g.rxKind==="um982"))ckAlerts(g.ck,"The GPS receiver",al);
 const f2a=s.dual&&s.dual.f9p2,c2a=s.dual&&s.dual.ck2;
 if(f2a&&f2a.on&&c2a){
  if(c2a.health==="none"&&s.up>20)al.push("Nothing arrives from the second F9P on the GPS2 port. Check its power and its wiring to Serial2 (Teensy pins 7 / 8). See GPS, Second F9P check.");
  ckAlerts(c2a,"The second F9P",al);}
 if(f2a&&!f2a.on&&f2a.seen)al.push(f2a.port==="off"?"A GNSS receiver is sending data on the GPS2 port, but the port is off. For a dual F9P, set GPS2 port to «automatic» in Settings."
  :f2a.port==="XIAO"?"Valid GNSS data on the GPS2 port while a XIAO says hello there: one thing at a time on the port."
  :"A GNSS receiver is sending data on the GPS2 port: it becomes the second F9P once autosteer is off.");
 if(s.tm&&s.tm.type==="TM171"&&s.tm.set&&s.tm.set.setupWhy)al.push("TM171 not set up for AgOpenGPS ("+s.tm.set.setupWhy+"). See the IMU tab, Fix TM171 setup.");
 if(s.tm&&s.tm.type==="TM171"&&s.tm.set&&s.tm.set.noWire)al.push("TM171 RX pin not wired to Teensy TX5 (pin 20): roll, pitch and yaw still work, but gyro calibration, new install calibration, settings and Fix TM171 setup are unavailable. Solder one wire from Teensy pin 20 to the TM171 RX pin (see the IMU tab).");
 const n=s.net;
 pill("link",n.link?"connected":"unplugged",n.link?"ok":"bad");
 const hn=n.host==="AgOpenWeb"||n.host==="AgOpenGPS"?n.host:"";
 pill("aog",!n.aog?"no communication":n.hostBoth?"two programs":hn?hn+" · "+"talking":"talking",!n.aog||n.hostBoth?"warn":"ok");set("aogIp",n.aog?n.aogIp:"–");
 pill("rtcm",(n.rtcm?"arriving":"stopped"),n.rtcm?"ok":"warn");set("rtcmKB",n.rtcmKB.toFixed(0)+" kB"+(n.rtcm?" · "+n.rtcmBps+" B/s":""));
 set("rtcmMsg",n.rtcmOk+" · "+n.rtcmBad+" "+"with errors",n.rtcmBad?"warn":"");set("rtcmTypes",n.rtcmTypes||"–");
 pill("radio",(n.radio?"arriving":n.radioKB>0?"stopped":"nothing received"),n.radio?"ok":n.radioKB>0?"warn":"");
 set("radioKB",n.radioKB.toFixed(0)+" kB"+(n.radio?" · "+n.radioBps+" B/s":""));
 set("radioMsg",n.radioOk+" · "+n.radioBad+" "+"with errors",n.radioBad?"warn":"");set("radioTypes",n.radioTypes||"–");
 set("up",dur(s.up));
 $("rx").textContent=g.rx?g.rx.split(", ").join("\n"):"–";
 const cl=s.out.mode==="classic";
 set("mode",cl?(s.out.kind===3?"AgOpenGPS ($PANDA)":"AgOpenGPS ($PAOGI)"):"AgOpenWeb");
 set("modeSel",(s.out.auto?(n.host?"automatic":"automatic, waiting for the hello"):!cl&&n.host==="AgOpenGPS"?"manual: AgOpenGPS does not read these sentences":"manual"),!s.out.auto&&!cl&&n.host==="AgOpenGPS"?"warn":"");
 set("cSrc",s.out.mode!=="classic"?"–":(s.out.kind===3?"IMU (AgOpenGPS does the fusion)":(["–","antennas","IMU (no antennas)","GPS course (no antennas, no IMU)","held (stopped / IMU not aligned yet)"][s.out.classicSrc]||"–")),s.out.classicSrc>=2?"warn":"");
 set("aogSrc",s.out.aogSrc?"heading source"+": "+s.out.aogSrc:"–");
 set("epochs",String(s.out.epochs)+(s.out.sim?" · "+s.out.held+" "+"held back (simulator mode)":""),s.out.simHeld?"warn":"");
 $("sent").textContent=s.out.lines.length?s.out.lines.join("\n"):"–";
 const x=s.xiao;
 if(x){
  const xs=x.port===2?["GPS2 port: second F9P",""]:x.port===1?["GPS2 port off",""]:!x.on?["bridge off",""]:x.det?["connected","ok"]:x.hellos?["no hello","bad"]:["not detected",""];
  set("xState",xs[0],xs[1]);
  set("gps2Now",x.port===1?"ignored (off)":x.port===2?"a second F9P":x.det?"a XIAO ESP32":"nothing found yet");
  $("xiaoCard").classList.toggle("dim",x.port===2||x.port===1);   /* the port has the second F9P, or is off: the card is greyed */
  set("xHello",x.helloAgo<0?"never":ago(x.helloAgo));
  set("xTo",String(x.to));set("xFrom",String(x.from));set("xLast",x.lastPgn<0?"–":String(x.lastPgn));
  set("xCrc",String(x.crcErr),x.crcErr?"warn":"");set("xDrop",String(x.drop+x.cut),(x.drop+x.cut)?"warn":"");
  set("xKB",x.rxKB.toFixed(1)+" / "+x.txKB.toFixed(1)+" kB");
  let xi=null;try{xi=x.info?JSON.parse(x.info):null;}catch(e){}
  if(xi){set("xProg",(xi.v||"?")+(xi.c?" · "+xi.c:"")+(x.infoAgo>15?" ("+ago(x.infoAgo)+")":""));
   setLink("xAp",(xi.ap||"–")+(xi.cl!==undefined?" · "+xi.cl+" "+(xi.cl==1?"client":"clients"):""),xi.apIp?"http://"+xi.apIp+"/":"");   /* its page, on its own network */
   set("xSta",!xi.sta?"not set":xi.st==="ok"?xi.sta+" · "+xi.rssi+" dBm":xi.sta+" · "+"not connected",xi.sta&&xi.st!=="ok"?"warn":"");
   setLink("xIp",xi.sta&&xi.st==="ok"?"":"–",xi.sta&&xi.st==="ok"&&xi.ip?"http://"+xi.ip+"/":"");}   /* its page on that network */
  else{set("xProg","–");set("xAp","–");set("xSta","–");set("xIp","–");}
  // the XIAO program must be the one of this firmware version (same number): a new XIAO has to be programmed first
  const tv=s.fw.slice(s.fw.lastIndexOf(" ")+1),bins="https://github.com/renatorgr/Boards/tree/main/Firmware/v"+tv+"_AiO_WEB/v"+tv+"_XIAO_ESP32_bin";
  let xn="",xnc="";
  if(x.port===0&&x.on&&xi&&xi.v&&xi.v!==tv){xn="The XIAO runs program "+xi.v+"; this firmware expects "+tv+". Update it on the XIAO's own page (its WiFi network, Firmware, Install from GitHub), or over USB-C with v"+tv+"_XIAO_ESP32C3_USB.bin / S3_USB.bin: ";xnc="warn";
   al.push("The XIAO runs program "+xi.v+", this firmware expects "+tv+": update it (see the XIAO card).");}
  else if(x.port===0&&x.on&&!x.det&&!x.hellos&&s.up>20){xn="Nothing found on the GPS2 port (no XIAO hello, no GNSS data). A new XIAO has to be programmed first, over USB-C: v"+tv+"_XIAO_ESP32C3_USB.bin or v"+tv+"_XIAO_ESP32S3_USB.bin at address 0x0 (the README there says how). Already programmed: check its power and the wires XIAO D6 → Teensy RX2, D7 ← TX2: ";xnc="";}
  const xe=$("xNote");xe.textContent=xn;xe.className="note "+xnc+(xn?"":" hidden");
  if(xn){const a=document.createElement("a");a.href=bins;a.target="_blank";a.rel="noopener";a.textContent="the files on GitHub";xe.appendChild(a);xe.appendChild(document.createTextNode("."));}
 }
 $("alert").textContent=al.join("\n");$("alert").classList.toggle("hidden",!al.length);   /* (after the XIAO card: its version warning) */
 renderMach(s.mach);
 renderGh();
 renderLock(s);
 const sp=s.fw.lastIndexOf(" ");set("fw",sp>0?s.fw.slice(0,sp):s.fw);set("built",s.built);set("target",s.target);
 if(b){set("bootWhy",(BW[b.why]||b.why)+(b.step?" · "+b.step:"")+(b.addr?" · "+b.addr:"")+(/other|unknown/.test(b.why)?" ("+b.srsr+")":""),/watchdog|crash/.test(b.why)?"bad":"");
  set("wdt",b.wdt?(b.wdtOff?"on until the next restart":"on (2 s)"):"off",b.wdt?"":"warn");}
}

// Buttons the module refuses while autosteer is engaged (data-lock "e"), or also while the tractor is moving ("m"; zWeb.ino
// answers 409): greyed out then, and a click does nothing. A running gyro calibration keeps its Stop button.
function renderLock(s){
 const st=s.steer||{},t=s.tm||{},eng=!!st.active,mov=!!st.moving;
 const calStop=["starting","running","finishing"].includes(t.calib)&&t.calibKind!=="install";
 for(const b of document.querySelectorAll("[data-lock]"))
  b.classList.toggle("locked",(eng||(mov&&b.dataset.lock==="m"))&&!(b.id==="tmCalBtn"&&calStop));
 for(const p of document.querySelectorAll("[data-locknote]")){
  const w=eng?"Some buttons are greyed out while autosteer is on.":mov&&p.dataset.locknote==="m"?"Some buttons are greyed out while the tractor is moving.":"";
  p.textContent=w;p.classList.toggle("hidden",!w);}
}
document.addEventListener("click",e=>{const b=e.target.closest&&e.target.closest("button.locked");if(b){e.preventDefault();e.stopImmediatePropagation();}},true);

// The machine module (zMachine.ino): s.mach only once the Teensy has heard it, then the tab stays.
// Any standard module: the official Machine_UDP_v5 (sections in its hello reply, no PGN 160), the sprayer module
// (PGN 160) or the seed drill module (PGN 237 with sections 1-64).
function renderMach(m){
 if(!m)return;
 $("navMach").classList.remove("hidden");
 set("mState",m.here?"sending":"silent",m.here?"ok":"bad");
 set("mIp",m.ip);set("mAgo",ago(m.ago),m.here?"":"warn");
 set("mPkts",String(m.pkts));set("mBad",String(m.bad),m.bad?"warn":"");set("mPgn",m.lastPgn<0?"–":String(m.lastPgn));
 set("mHello",m.helloAgo<0?"none yet":ago(m.helloAgo)+" · "+m.helloN);$("mNoCrc").classList.toggle("hidden",!m.helloNoCrc);
 const have=m.secAgo>=0&&m.secAgo<5;
 set("mSecFrom",m.secAgo<0?"nothing yet":(m.secPgn===160||m.secPgn===237?"PGN "+m.secPgn:"hello reply")+", "+ago(m.secAgo),have?"":"warn");
 const box=$("mSec");machSecs(box,m.cmd,m.sec,have,m.secN);box.classList.toggle("old",m.secAgo>=0&&!have);
 // Sprayer card only while the module sends PGN 160: never heard, or not for 30 s while the module still talks
 // (a sprayer module set to no sensors, or another module): hidden. All silent: the last readings stay, in orange.
 const noSpray=m.sprayAgo<0||(m.here&&m.sprayAgo>=30);
 $("mSprayCard").classList.toggle("hidden",noSpray);
 if(noSpray)return;
 const old=m.sprayAgo>=5;
 set("mBar",fmt(m.bar,2," bar"),old?"warn":"");set("mFlow",fmt(m.lmin,2," L/min"));set("mTotal",fmt(m.total,1," L"));
 set("mMain",m.main?"on":"off",m.main?"ok":"");set("mSprayAgo",ago(m.sprayAgo)+" · "+m.sprayN,old?"warn":"");
}
// One numbered square per section (5, or up to 64 when the program or the module uses more, 8 per row; as
// many as the highest section used since the Teensy started, secN, so they do not shrink when all are off),
// coloured as AgOpenWeb colours its section buttons: green open, red closed, orange asked on but still
// closed, cyan asked off but still open. No word from the module yet: grey.
// cmd and sec: sections 1-64 as hex digits, section 1 = the lowest bit.
function hexBit(h,k){const i=h.length-1-(k>>2);return i>=0?(parseInt(h[i],16)>>(k&3))&1:0;}
function machSecs(box,cmd,sec,have,top){
 cmd=String(cmd||"0");sec=String(sec||"0");
 let n=Math.max(5,top||0);for(let k=n;k<64;k++)if(hexBit(cmd,k)|hexBit(sec,k))n=k+1;
 if(box.childElementCount!==n){box.textContent="";box.style.setProperty("--n",Math.min(n,8));
  for(let k=1;k<=n;k++){const e=document.createElement("span");e.textContent=k;box.appendChild(e);}}
 for(let k=0;k<n;k++){const a=hexBit(cmd,k),o=hexBit(sec,k),e=box.children[k];
  e.className=!have?"":a&&o?"s-on":!a&&!o?"s-off":a?"s-goingon":"s-goingoff";
  e.title="Section "+(k+1)+": "+(a?"asked on":"asked off")+(have?", "+(o?"open":"closed"):"");}
}

// The Teensy restarted (its time on went back, or another build runs): the whole page again from the
// module, on the same tab - a new firmware brings its own page, and every card, list and form starts afresh.
function reloadAfterRestart(msg){
 try{sessionStorage.setItem("tab",tab);if(msg)sessionStorage.setItem("fwMsg",msg);}catch(e){}
 location.reload();
}
async function pollStatus(){
 try{const s=await getJ("/api/status");if(s.error)throw 0;
  if(lastS&&(s.up+3<lastS.up||s.built!==lastS.built)){reloadAfterRestart(waitingBoot?"Updated. Running: "+s.fw+", built "+s.built+".":"");return;}
  if(waitingBoot&&s.up<upBefore){waitingBoot=false;busy=false;$("prog").classList.add("hidden");$("install").disabled=!$("hexFile").files[0];
   fwMsg(("Updated. Running: "+s.fw+", built "+s.built+"."),"ok");}
  setOnline(true);render(s);
 }catch(e){setOnline(false);}
}
function addLines(box,lines,withTime){
 const near=box.scrollTop+box.clientHeight>=box.scrollHeight-30;
 for(const l of lines){const d=document.createElement("div");d.className="ll";
  if(withTime){const t=document.createElement("span");t.className="t";t.textContent=dur(Math.floor(l.t/1000));d.appendChild(t);}
  const x=document.createElement("span");x.textContent=withTime?l.x:l;
  if(withTime&&/NO DATA|REFUSED|no dual|NOT detected|bad checksum|late|unplugged|silent|stopped|no GGA|dropped|OFF|WATCHDOG|CRASH|blocked|out of range|no whole/.test(l.x))x.className="warn";
  d.appendChild(x);box.appendChild(d);}
 while(box.childElementCount>400)box.removeChild(box.firstChild);
 if(near)box.scrollTop=box.scrollHeight;
}
async function pollLog(){
 if(logPaused)return;
 try{const j=await getJ("/api/log?after="+logAfter);if(j.error)return;
  if(j.next<=logAfter){logAfter=0;return;}           // the module restarted: read from the start
  addLines($("log"),j.lines,true);if(j.lines.length)logAfter=j.lines[j.lines.length-1].s;}catch(e){}
}
// ---- the GPS receiver (zReceiver.ino): what it is; the u-blox settings
const UBX_SEL=["rate","dyn"],UBX_CB=["hp","gga","vtg","gps","glo","gal","bds","qzss","sbas","inRtcm"],UBX_SIG=["gps","glo","gal","bds","qzss","sbas"],
 UBX_OTHER=["rmc","gsa","gsv","gll","gst","zda"];   /* sentences the Teensy does not use: not on the page, only named when the receiver sends them */
let gpsJ=null,ubxShown=-1,ubxAsked=false,uLocal=null,uLocalAt=0;
function uSay(t,c){uLocal=[t,c];uLocalAt=Date.now();set("uMsg",t,c);}   /* a message of this page, shown for 5 s */
async function pollGps(){try{const j=await getJ("/api/gps");if(j.error)return;gpsJ=j;renderGps(j);}catch(e){}}
function ubxMsg(u){
 if(u.op==="read")return ["Reading from the receiver…",""];
 if(u.op==="write")return ["Writing to the receiver…",""];
 if(u.op==="ubx-on")return ["Switching the UBX output on…",""];
 switch(u.res){
  case "read":return ["Read from the receiver "+(u.readAgo>=0?ago(u.readAgo):"")+".","ok"];
  case "read-none":return ["The receiver did not answer (UBX output off on UART1?).","bad"];
  case "write":return ["Written to the receiver (RAM, BBR and flash) and read back.","ok"];
  case "write-ram":return ["Written to the receiver's RAM and BBR; the flash refused (lost without the backup battery).","warn"];
  case "write-nak":return ["The receiver refused the changes: nothing was changed.","bad"];
  case "write-none":return ["The receiver did not answer: the write was not confirmed.","bad"];
  default:return ["",""];}
}
function otherOn(id,v,names){      /* the receiver sends sentences the Teensy does not use (not on the page); every poll */
 const list=names.filter(n=>v&&v[n]>0).map(n=>n.toUpperCase());
 set(id,list.length?"Also on in the receiver (the Teensy does not use them):"+" "+list.join(", ")+".":"","warn");$(id).classList.toggle("hidden",!list.length);}
function fillUbx(u){
 const f=$("fu"),v=u.v||{};
 for(const n of UBX_SEL){const s=f[n];
  if(v[n]!==undefined){const val=String(v[n]);if(![...s.options].some(o=>o.value===val)){const o=document.createElement("option");o.value=val;o.textContent=n==="rate"?(1000/v[n]).toFixed(1)+" Hz":val;s.appendChild(o);}s.value=val;}
  s.disabled=v[n]===undefined;}
 for(const n of UBX_CB){const c=f[n];c.checked=!!v[n];c.disabled=v[n]===undefined||((n==="gga"||n==="vtg")&&v[n]>0);}
}
function renderGps(j){
 const k=j.kind,asking=j.id==="wait"||j.id==="ask";
 set("gModel",j.model||(k==="um982"?"Unicore":asking&&j.talks?"asking…":"not identified"),j.model||k?"":"warn");
 set("gKind",k==="ublox"?"u-blox":k==="um982"?"Unicore":"–");
 set("gFw",j.fw||"–");set("gProt",j.prot||"–");$("gProtT").classList.toggle("hidden",k!=="ublox");$("gProt").classList.toggle("hidden",k!=="ublox");   /* u-blox only */
 const um=k==="um982";                                  /* NMEA version: UM982 only, from its CONFIG (zReceiver.ino) */
 set("gNmea",um&&j.um&&j.um.nmea||"–");$("gNmeaT").classList.toggle("hidden",!um);$("gNmea").classList.toggle("hidden",!um);
 set("gId",!j.talks?"the receiver sends nothing":j.id==="done"?ago(j.idAgo):j.id==="none"?(j.hpr?"no answer (it sends HPR: Unicore)":"no answer"):"asking…",!j.talks||j.id==="none"?"warn":"");
 $("gUbxOn").classList.toggle("hidden",!(j.id==="none"&&!j.hpr&&j.talks));
 $("fu").classList.toggle("hidden",k!=="ublox");
 $("fm").classList.toggle("hidden",k!=="um982");
 $("gUm").classList.toggle("hidden",k==="ublox");
 renderCk(j);
 $("gIdent").disabled=ckBusy0(j);
 $("mHd").classList.toggle("hidden",k==="um982"&&!!((j.ck&&j.ck.umSingle)||(j.model&&j.model!=="UM982")));   /* one antenna: no heading setup */
 if(k==="um982")renderUm(j.um);
 if(k!=="ublox")return;
 const u=j.ubx;
 if(u.seq!==ubxShown&&!u.op){ubxShown=u.seq;fillUbx(u);}
 otherOn("uOther",u.v,UBX_OTHER);
 $("uRead").disabled=$("uWrite").disabled=!!u.op||ckBusy0(j);
 const m=uLocal&&Date.now()-uLocalAt<5000?uLocal:ubxMsg(u);set("uMsg",m[0],m[1]);
 if(!u.op&&u.readAgo<0&&!ubxAsked&&tab==="gps"&&!ckBusy0(j)){ubxAsked=true;gpsAct("act=read");}   /* read once, the first time the tab shows a u-blox */
}
async function gpsAct(body,say){
 say=say||uSay;
 try{const j=await getJ("/api/gps",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:body});
  if(!j.ok){say("Error"+": "+ptErr(j.error),"bad");return false;}
  uLocal=mLocal=null;ckLocal=[null,null];await pollGps();return true;}catch(e){say("No connection to the module.","bad");return false;}
}
let tsSeen=null;
function tsFill(s){$("tsMag").checked=!!s.mag;$("tsGyroErr").checked=!!s.gyroErr;$("tsSelfAdapt").checked=!!s.selfAdapt;$("tsStatus").checked=!!s.status;
 $("tsBoot").value=String(s.boot||0);$("tsAcc").value=(s.accGain/100).toFixed(2);$("tsMagG").value=(s.magGain/100).toFixed(2);
 tsRateSet(s.odr||0);}
// the rate list holds 25, 50 and 100 Hz; a rate read from the TM171 that is not in it is added "as read"
function tsRateSet(v){const sel=$("tsRate");if(!v){sel.value="25";return;}if(![...sel.options].some(o=>+o.value===v)){const o=document.createElement("option");o.value=v;o.textContent=v+" Hz (as read)";sel.appendChild(o);}sel.value=v;}
function renderTmSet(s){
 set("tsRateNow",s.rate+" Hz");set("tsInhibit",s.inhibit>=0?s.inhibit+" ms":"–");set("tsBaud",(s.baud?s.baud+" bps":"–")+(s.baudNow&&s.baudNow!==115200?" (Teensy listening at "+s.baudNow+")":""));
 const key=s.age>=0?s.sw+":"+s.accGain+":"+s.magGain+":"+s.odr+":"+(s.age<2?"n":"o"):null;
 if(s.age>=0&&key!==tsSeen&&(tsSeen===null||s.age<2)){tsSeen=key;tsFill(s);}
 let m="",cls="";
 if(s.pending)m="waiting for the TM171…";
 else if(s.odd){m="Settings received, but the layout is not recognised (switch word 0x"+(s.sw>>>0).toString(16)+", bytes "+(s.raw||"?")+"). Writing is disabled; use ImuAssistant instead, and send these bytes to the firmware's author.";cls="bad";}
 else if(s.age<0){m=s.noAnswer?"No reply from the TM171 (is the TX5 wire connected?)":"not read yet";cls=s.noAnswer?"warn":"";}
 else{m="Read "+(s.age<90?s.age+" s":Math.round(s.age/60)+" min")+" ago"+(s.ackAge>=0?". Last write acknowledged by the TM171 "+(s.ackAge<90?s.ackAge+" s":Math.round(s.ackAge/60)+" min")+" ago":"");cls=s.ackAge>=0&&s.ackAge<30?"ok":"";}
 if(!$("tsMsg").dataset.err)set("tsMsg",m,cls);
 $("tsWrite").disabled=!s.valid||s.pending;$("tsAog").disabled=!s.valid;$("tsRead").disabled=!!s.pending;
 for(const e of $("tmSetDet").querySelectorAll("input,select"))e.disabled=!s.valid;}   /* nothing read: nothing to choose from */
$("tsRead").addEventListener("click",()=>{tsSeen=null;delete $("tsMsg").dataset.err;tmCalPost("setread");});
$("tsAog").addEventListener("click",()=>{tsFill({mag:0,gyroErr:1,selfAdapt:1,status:1,boot:0,accGain:350,magGain:100,odr:25});set("tsMsg","AgOpenGPS defaults loaded into the form (magnetometer off, GyroErrFilter 3.50, Self-Adapt 1.00, RPY + Status, 25 Hz, Auto boot). Click \"Write to TM171\" to apply.","warn");$("tsMsg").dataset.err="1";});
$("tsWrite").addEventListener("click",async()=>{
 if(!confirm("Write these settings to the TM171"+($("tsSave").checked?" and save them to its flash memory":"")+"?"))return;
 const n=id=>Math.round(parseFloat($(id).value)||0),rate=n("tsRate");
 const b="act=setwrite&mag="+(+$("tsMag").checked)+"&gyroErr="+(+$("tsGyroErr").checked)+"&selfAdapt="+(+$("tsSelfAdapt").checked)+"&status="+(+$("tsStatus").checked)+"&save="+(+$("tsSave").checked)
  +"&boot="+$("tsBoot").value+"&accGain="+Math.round(parseFloat($("tsAcc").value)*100)+"&magGain="+Math.round(parseFloat($("tsMagG").value)*100)+"&odr="+rate;
 delete $("tsMsg").dataset.err;tsSeen=null;
 try{const j=await getJ("/api/imu",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:b});
  if(!j.ok){set("tsMsg","Error"+": "+ptErr(j.error),"bad");$("tsMsg").dataset.err="1";}else set("tsMsg","Sent. Reading back…","");}
 catch(e){set("tsMsg","Cannot reach the module.","bad");$("tsMsg").dataset.err="1";}});
async function tmCalPost(act){
 try{const j=await getJ("/api/imu",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"act="+act});
  if(!j.ok)set("tmCal","Error"+": "+ptErr(j.error),"bad");}catch(e){set("tmCal","Cannot reach the module.","bad");}}
$("tmCalBtn").addEventListener("click",()=>{
 const busy=$("tmCalBtn").textContent==="Stop";
 if(!busy&&!confirm("Calibrate the TM171 gyro?\nKeep the tractor parked for 20 s (the engine can stay running). Autosteer must be off."))return;
 tmCalPost(busy?"calibstop":"calib");});
$("tmFixBtn").addEventListener("click",()=>{
 if(!confirm("Write the AgOpenGPS setup to the TM171?\nUART 115200, magnetometer off, GyroErrFilter 3.50, Self-Adapt 1.00, RPY + Status output, 25 Hz, Auto boot. Saved to the TM171's flash. Autosteer must be off."))return;
 tmCalPost("fixsetup");});
$("tmInstBtn").addEventListener("click",()=>{
 if(!confirm("Run the TM171 new install calibration?\nKeep the tractor parked for about 20 s and do NOT cut power while it runs (SYD warns this can damage the TM171). It cannot be stopped once started. The result is saved to flash; switch the whole module off and on afterwards."))return;
 tmCalPost("install");});
$("gIdent").addEventListener("click",()=>{ubxAsked=false;ubxShown=-1;umAsked=false;umShown=-1;gpsAct("act=identify");});
$("gUbxOn").addEventListener("click",()=>{if(confirm("Switch the UBX output of the receiver's UART1 on (u-blox only)? It is saved in the receiver."))gpsAct("act=ubxon");});
$("uRead").addEventListener("click",()=>gpsAct("act=read"));
$("fu").addEventListener("submit",e=>{e.preventDefault();
 if(!gpsJ||!gpsJ.ubx)return;
 const f=$("fu"),v=gpsJ.ubx.v||{},p=new URLSearchParams();p.set("act","write");const ch=[];
 for(const n of UBX_SEL){if(v[n]===undefined||f[n].disabled)continue;const nv=Number(f[n].value);if(nv!==v[n]){p.set(n,String(nv));ch.push(n);}}
 for(const n of UBX_CB){if(v[n]===undefined||f[n].disabled)continue;if(f[n].checked!==!!v[n]){p.set(n,f[n].checked?"1":"0");ch.push(n);}}
 if(!ch.length){uSay("Nothing changed.","");return;}
 let q=("Write "+ch.length+(ch.length===1?" change":" changes")+" to the receiver?");
 if(ch.some(n=>UBX_SIG.includes(n)))q+="\n\n"+"Changing the constellations restarts the receiver's GNSS: RTK is lost for a few seconds.";
 if(p.get("inRtcm")==="0")q+="\n\n"+"Without RTCM3 corrections the receiver gets no RTK.";
 if(!confirm(q))return;
 gpsAct(p.toString());});
// ---- the UM982 settings (zReceiver.ino)
const UM_SEL=["rate","mode","rtkTo","sbas","hdMode"],UM_NUM=["hdLen","hdTol","hdOff","ptOff"],
 UM_MSG=["ksxt"],UM_SYSN=["gps","glo","gal","bds","qzss"],
 UM_OTHER=["rmc","gsa","gsv","gst","zda","ths"];   /* messages the Teensy does not use: not on the page, only named when the UM982 sends them */
let umShown=-1,umAsked=false,mLocal=null,mLocalAt=0;
function mSay(t,c){mLocal=[t,c];mLocalAt=Date.now();set("mMsg",t,c);}   /* a message of this page, shown for 5 s */
function hz(ms){const f=1000/ms;return (f>=1?String(Math.round(f*10)/10):f.toFixed(2))+" Hz";}
function umMsg(u){
 if(u.op==="read")return ["Reading from the receiver…",""];
 if(u.op==="write")return ["Writing to the receiver…",""];
 const f=(u.fail||[]).length?" "+"Refused:"+" "+u.fail.join("; "):"";
 switch(u.res){
  case "read":return ["Read from the receiver "+(u.readAgo>=0?ago(u.readAgo):"")+".","ok"];
  case "read-none":return ["The UM982 did not answer CONFIG.","bad"];
  case "write":return ["Written to the UM982 (SAVECONFIG) and read back.","ok"];
  case "write-part":return [("Partly written: "+u.okN+" of "+u.cmds+" commands taken (SAVECONFIG done).")+f,"warn"];
  case "write-nosave":return ["Changed, but SAVECONFIG was not confirmed: lost at a power off."+f,"warn"];
  case "write-none":return ["The UM982 took no command: nothing was changed."+f,"bad"];
  default:return ["",""];}
}
function fillUm(u){
 const f=$("fm"),v=u.v||{};
 for(const n of UM_SEL){const s=f[n],has=v[n]!==undefined;
  if(has){const val=String(v[n]);if(![...s.options].some(o=>o.value===val)){const o=document.createElement("option");o.value=val;
   o.textContent=n==="rate"?hz(v[n]):n==="rtkTo"?val+" s":n==="sbas"?"other (see the answers)":val;s.appendChild(o);}}
  s.querySelector('option[value=""]').hidden=has;s.value=has?String(v[n]):"";}
 f.hdLen.value=v.hdLen>0?String(v.hdLen):"";f.hdTol.value=v.hdLen>0&&v.hdTol!==undefined?String(v.hdTol):"";
 f.hdOff.value=v.hdOff!==undefined?String(v.hdOff):"";f.ptOff.value=v.ptOff!==undefined?String(v.ptOff):"";
 f.hdOff.placeholder=f.ptOff.placeholder=v.hdOff===undefined?"–":"";
 for(const n of UM_MSG){const c=f[n];c.checked=v[n]>0;c.dataset.na=v[n]===undefined?"1":"";}
 for(const n of UM_SYSN){const c=f[n];c.checked=v[n]===1;c.dataset.na=v[n]===undefined?"1":"";}
 $("mRateWarn").classList.toggle("hidden",!(v.rate!==undefined&&(v.vtg!==v.rate||v.hpr!==v.rate)));
 const kx=v.ksxt>0&&v.ksxt!==100;   /* a KSXT at another rate: "Write to the receiver" sets it to 10 Hz */
 set("mKsxtWarn",kx?("The KSXT comes at "+hz(v.ksxt)+": writing sets it to 10 Hz."):"","warn");$("mKsxtWarn").classList.toggle("hidden",!kx);
}
function umInfo(u){            /* the read-only part, every poll */
 const port=u.port>0?"COM"+u.port:"";
 set("mPortName",port||"–");
}
function renderUm(u){
 if(!u)return;
 if(u.seq!==umShown&&!u.op){umShown=u.seq;fillUm(u);}
 umInfo(u);
 otherOn("mMsgOther",u.v,UM_OTHER);
 const dis=!!u.op||u.readAgo<0;
 for(const e of $("fm").elements)if(e.name)e.disabled=dis||e.dataset.na==="1";
 $("mRead").disabled=!!u.op||ckBusy0(gpsJ);$("mWrite").disabled=dis||ckBusy0(gpsJ);
 const m=mLocal&&Date.now()-mLocalAt<5000?mLocal:umMsg(u);set("mMsg",m[0],m[1]);
 if(!u.op&&u.readAgo<0&&!umAsked&&tab==="gps"&&!ckBusy0(gpsJ)){umAsked=true;gpsAct("act=umread",mSay);}   /* read once, the first time the tab shows a UM982 */
}
$("mRead").addEventListener("click",()=>gpsAct("act=umread",mSay));
$("fm").addEventListener("submit",e=>{e.preventDefault();
 if(!gpsJ||!gpsJ.um)return;
 const f=$("fm"),u=gpsJ.um,v=u.v||{},p=new URLSearchParams();p.set("act","umwrite");const ch=[];
 for(const n of UM_SEL){const s=f[n];if(s.disabled||s.value==="")continue;const nv=Number(s.value);if(nv!==v[n]){p.set(n,String(nv));ch.push(n);}}
 if(!ch.includes("rate")&&v.rate!==undefined&&(v.vtg!==v.rate||v.hpr!==v.rate)){p.set("rate",String(v.rate));ch.push("rate");}
 const val=n=>f[n].value.trim(),bad=n=>val(n)!==""&&!f[n].checkValidity();
 for(const n of UM_NUM)if(bad(n)){mSay("Value out of range."+" ("+n+")","bad");f[n].focus();return;}
 const len=val("hdLen"),tol=val("hdTol"),curLen=v.hdLen>0?String(v.hdLen):"",curTol=v.hdLen>0&&v.hdTol!==undefined?String(v.hdTol):"";
 if(!f.hdLen.disabled&&(len!==curLen||(len!==""&&tol!==curTol))){
  if(len===""){if(v.hdLen>0){p.set("hdLen","0");ch.push("hdLen");}}
  else{p.set("hdLen",len);p.set("hdTol",tol||"3");ch.push("hdLen");}}
 const off=val("hdOff"),pt=val("ptOff"),curOff=v.hdOff!==undefined?String(v.hdOff):"",curPt=v.ptOff!==undefined?String(v.ptOff):"";
 if(!f.hdOff.disabled&&(off!==curOff||pt!==curPt)&&(off!==""||pt!=="")){p.set("hdOff",off||"0");p.set("ptOff",pt||"0");ch.push("hdOff");}
 for(const n of UM_MSG.concat(UM_SYSN)){const c=f[n];if(c.disabled)continue;const sys=UM_SYSN.includes(n),on=sys?v[n]===1:v[n]>0;
  if(c.checked!==on||(!sys&&c.checked&&v[n]!==100)){p.set(n,c.checked?(sys?"1":"100"):"0");ch.push(n);}}   /* the KSXT at 10 Hz (KSXT 0.1) */
 if(!ch.length){mSay("Nothing changed.","");return;}
 let q=("Write "+ch.length+(ch.length===1?" change":" changes")+" to the UM982 (and SAVECONFIG)?");
 if(ch.some(n=>UM_SYSN.includes(n))||ch.includes("mode"))q+="\n\n"+"Changing the constellations or the working mode can lose RTK for a few seconds.";
 if(ch.includes("hdLen")&&p.get("hdLen")!=="0")q+="\n\n"+"With a wrong distance between the antennas, the UM982 gives no heading.";
 if(!confirm(q))return;
 gpsAct(p.toString(),mSay);});
async function pollUm(){
 try{const j=await getJ("/api/um982?after="+umAfter);if(j.error)return;
  if(j.next<=umAfter){umAfter=0;return;}
  addLines($("umOut"),j.lines,false);umAfter=j.next-1;}catch(e){}
}
// ---- the receiver check (zRxCheck.ino): the data at 460800, the baud rate, what the Teensy needs, the AgOpenGPS settings
const CK_N={ubxIn:"UBX in on UART1",rtcmIn:"RTCM3 in on UART1",ubxOut:"UBX out on UART1",nmeaOut:"NMEA out on UART1",gga:"GGA on UART1",vtg:"VTG on UART1",
 rate:"time between measurements",navRate:"measurements per solution",baud1:"UART1 baud rate",rtcmOut2:"RTCM3 out on UART2",baud2:"UART2 baud rate",
 relpos:"RELPOSNED on UART1",rtcmIn2:"RTCM3 in on UART2"},
 CK_BOOL=["ubxIn","rtcmIn","ubxOut","nmeaOut","rtcmOut2","rtcmIn2"];
let ckLocal=[null,null],ckLocalAt=[0,0];
const ckBusy0=j=>!!(j&&j.ck&&j.ck.op&&j.ck.port===0);     /* the check has the receiver on Serial7: the GPS tab's reads wait */
function ckName(id){const m=/^r(\d+)$/.exec(id);if(m){const n=m[1]==="4072"?"4072.0":m[1];return ("RTCM "+n+" on UART2");}return (CK_N[id]||id);}
function ckVal(id,v){
 if(CK_BOOL.includes(id))return (v?"on":"off");
 if(id==="rate")return v+" ms";
 if(id==="baud1"||id==="baud2")return v+" baud";
 if(id==="navRate")return String(v);
 return v>0?(v===1?"on":("on, 1 in "+v)):"off";}
const hz10=x=>(x/10).toFixed(x%10?1:0)+" Hz";   /* (no line break before "Hz") */
function ckSay(port,t,c){ckLocal[port]=[t,c];ckLocalAt[port]=Date.now();set("ck"+port+"Msg",t,c);}
function ckHealth(id,p){
 if(!p.on||p.health==="wait"){set(id,"–");return;}
 if(p.health==="ok")pill(id,"valid"+" · "+(p.good+" in 5 s")+(p.at&&p.at!==460800?" (at "+p.at+" baud for now)":""),"ok");
 else if(p.health==="junk")pill(id,"wrong baud rate","bad");
 else pill(id,"nothing arriving","warn");}
// the UM982 (zReceiver.ino's read): GGA, VTG and HPR at 10 Hz on the Teensy's port, that port at 460800
function umNeed(u,single){
 if(!u||u.readAgo<0)return null;
 const v=u.v||{},miss=[];
 for(const n of single?["gga","vtg"]:["gga","vtg","hpr"])if(v[n]!==100)miss.push(n.toUpperCase()+" "+"at 10 Hz");
 const b=u.port>0?(u.baud||{})["COM"+u.port]:0;
 if(b&&b!==460800)miss.push("port at 460800");
 return miss;}
function ckMsgText(c,p,port,um){
 if(c.op&&c.port===port){
  if(c.op==="scan")return [("Finding the baud rate: "+c.baudTry+" baud…"),""];
  if(c.op==="baud")return ["Changing the receiver's baud rate…",""];
  if(c.op==="load")return [c.msgs?("Loading: message "+(c.msg+1)+" of "+c.msgs+"…"):"Loading the commands into the UM982…",""];
  if(c.op==="dump")return [("Reading all the settings: "+c.dumpRead+" read, "+c.dumpN+" differ from the factory defaults…"),""];
  return [c.msgs?("Comparing with the AgOpenGPS settings: "+(c.msg+1)+" of "+c.msgs+"…"):"Checking…",""];}
 const out=[];let cls="";
 const kn={ublox:"u-blox",unicore:"Unicore",nmea:"NMEA"}[p.foundKind]||"";
 const boot=p.boot==="scan"&&p.baud!=="baud-ok"?"At start-up: ":"";
 if(p.scan==="found"){out.push(boot+("the receiver answers at "+p.found+" baud ("+kn+")."));
  if(p.found!==460800){out.push(p.at===p.found?"The port stays at that speed for now: the AgOpenGPS settings, once loaded, set 460800. Or press «Set to 460800 and save».":"The Teensy needs 460800: press «Set to 460800 and save».");cls="warn";}}
 if(p.scan==="notfound"){out.push(boot+"no baud rate gave valid data: nothing connected, a silent receiver, or TX / RX swapped.");cls="bad";}
 if(p.baud==="baud-ok"){out.push("The receiver is at 460800 baud now.");cls="ok";}
 if(p.baud==="baud-fail"){out.push(("No valid data at 460800 after the change: is the receiver still at "+p.found+" baud?"));cls="bad";}
 if(p.chk==="noanswer"){out.push("The receiver did not answer the check (UBX output off on UART1?).");cls="bad";}
 const tot=p.loadTotal,ok=p.loadOk+p.loadRam,w=um?"commands":"messages";
 if(p.load==="loaded"){out.push(("AgOpenGPS settings loaded ("+p.loadFile+"): "+ok+" of "+tot+" "+w+" taken"+(p.loadRam?", "+p.loadRam+" in RAM and BBR only":"")+"."));cls="ok";}
 if(p.load==="load-part"){out.push(("AgOpenGPS settings partly loaded ("+p.loadFile+"): "+ok+" of "+tot+" "+w+" taken, "+p.loadNak+" refused, "+p.loadNone+" not answered"+(um&&p.loadNak+p.loadNone===0?", SAVECONFIG not confirmed":"")+"."));cls="warn";}
 if(p.load==="load-fail"){out.push(("The AgOpenGPS settings were not loaded ("+p.loadFile+"): the receiver did not take them."));cls="bad";}
 if(p.dump==="dumped"){out.push(("All settings read: "+p.dumpN+" of "+p.dumpRead+" differ from the factory defaults")+(p.dumpFull?" (more than the Teensy keeps: the file is incomplete).":"."));cls=p.dumpFull?"warn":"ok";}
 if(p.dump==="dump-fail"){out.push("The settings could not all be read: the receiver refuses the all-items key of UBX-CFG-VALGET.");cls="bad";}
 if(p.dump==="noanswer"){out.push("The receiver did not answer the read of all its settings (UBX output off on UART1?).");cls="bad";}
 if([p.scan,p.baud,p.chk,p.load,p.dump].includes("stopped")){out.push("Stopped: the program (AgIO) took the receiver, or the GPS2 port changed.");cls="warn";}
 return [out.join(" "),cls];}
function ckDumpName(p){return ((p.model||"").replace(/^ZED-/,"")||"receiver")+".txt";}
function renderCk(j){
 const c=j.ck;if(!c)return;
 const ublox=j.kind==="ublox",um=j.kind==="um982";
 for(const port of [0,1]){
  const p=c.ports[port],id="ck"+port;
  if(port===1){$("ck1").classList.toggle("hidden",!p.on);if(!p.on)continue;set("ck1Model",p.model||"–");}
  ckHealth(id+"Data",p);
  const fast=p.health==="ok"&&Math.abs(p.hz[0]-100)>5?"warn":"";
  if(port===0){const n=["GGA","VTG","HPR"].slice(0,um||p.hz[2]?3:2),hzs=n.map((x,i)=>p.hz[i]);
   set(id+"Hz",p.health!=="ok"?"–":hzs.every(x=>x===hzs[0])?n.join(", ")+": "+hz10(hzs[0]):n.map((x,i)=>x+" "+hz10(hzs[i])).join(" · "),fast);}
  else set(id+"Hz",p.health!=="ok"?"–":p.hz[0]?"RELPOSNED "+hz10(p.hz[0]):p.hz[1]?"NMEA "+hz10(p.hz[1])+", no RELPOSNED":"nothing counted yet",p.health==="ok"&&!p.hz[0]?"warn":fast);
  // the firmware its AgOpenGPS settings are made for (F9P: HPG 1.13, AOGConfigOMatic's; X20P: HPG 2.11, X20P.txt): another
  // one in orange; on an F9P in red, with where to download HPG 1.13
  const fwBad=p.fwSt==="older"||p.fwSt==="newer",f9p=/F9P/.test(p.model||"");
  set(port===0?"gFw":id+"Fw",(port===0&&j.fw)||p.fw||"–",p.fwSt==="same"?"ok":fwBad?(f9p?"bad":"warn"):"");   /* the first receiver's: in the GPS receiver card */
  $(id+"FwNote").classList.toggle("hidden",!(fwBad&&f9p));
  if(port===0)set("ck0Note","Baud rate checked at start-up (460800). AgOpenGPS settings: "+(!ublox&&!um?"–":um?"AOGConfigOMatic (lansalot).":!p.set?"none for this receiver (ZED-F9P and ZED-X20P only).":p.file==="X20P.txt"?"X20P.txt, read from a working ZED-X20P (firmware "+p.setFw+").":"AOGConfigOMatic (lansalot) "+p.file+", for F9P firmware "+p.setFw+".")+" Change only with autosteer off.");
  // what the Teensy needs: u-blox from its keys, the UM982 from the GPS tab's read
  // values read: the wrong ones first (orange), the right ones below in a closed list
  const ub=port===1||ublox,list=[],bad=[];let miss=null;
  if(ub&&p.essAgo>=0){miss=[];
   for(const e of p.ess){const good=e.st===1&&e.v===e.want;if(!good)miss.push(ckName(e.id));
    (good?list:bad).push((good?"✓ ":e.st===2?"? ":"✗ ")+ckName(e.id)+": "+(e.st===1?ckVal(e.id,e.v):e.st===2?"unknown to the receiver":"not read")+(good||e.st===2?"":" — AgOpenGPS: "+ckVal(e.id,e.want)));}}
  if(port===0&&um)miss=umNeed(j.um,c.umSingle);
  set(id+"Need",miss===null?(ub||um?"not checked yet":"–"):miss.length?"missing:"+" "+miss.join(", "):"all good",miss===null?"":miss.length?"bad":"ok");
  $(id+"Vals").classList.toggle("hidden",!list.length&&!bad.length);
  $(id+"Bad").textContent=bad.join("\n");$(id+"Bad").classList.toggle("hidden",!bad.length);
  $(id+"Det").classList.toggle("hidden",!list.length);$(id+"List").textContent=list.join("\n");
  $(id+"OkSum").textContent=("Correct ("+list.length+")");
  // the AgOpenGPS settings: u-blox (F9P, X20P) compared key by key, the UM982 command by command
  let ago="–",agoC="";
  if(ub){if(!p.set)ago="none for this receiver";
   else if(p.cmpAgo>=0){ago=(p.same+" the same, "+p.diff+" different"+(p.unk?", "+p.unk+" not compared":""))+" ("+p.file+")";agoC=p.diff||p.unk?"warn":"ok";}
   else ago="not compared";}
  let differs=ub&&p.set&&p.cmpAgo>=0&&(p.diff>0||p.unk>0);
  if(port===0&&um&&document.activeElement!==$("ck0Um")&&Date.now()-ckUmAt>3000)$("ck0Um").value=c.umSingle?"s":"d";   /* the setting */
  if(!ub&&um){const s=$("ck0Um").value==="s",cmds=s?c.um982s:c.um982d,mask=s?p.umS:p.umD;
   const HM=/^CONFIG HEADING (FIXLENGTH|VARIABLELENGTH|STATIC|LOWDYNAMIC|TRACTOR)$/i,   /* one heading mode at a time: */
    gone=cmds.map((x,i)=>HM.test(x)&&cmds.slice(i+1).some(y=>HM.test(y)));             /* the file's TRACTOR is replaced by its VARIABLELENGTH */
   if(p.umListed){const tot=cmds.filter((x,i)=>!gone[i]).length,n=cmds.filter((x,i)=>!gone[i]&&mask&(1<<i)).length;
    ago=(n+" of "+tot+" in the receiver");agoC=n===tot?"ok":"warn";differs=n<tot;}
   else ago="not read";
   $("ck0CmdList").textContent=cmds.map((x,i)=>(gone[i]?"– ":p.umListed&&mask&(1<<i)?"✓ ":"· ")+x+(gone[i]?" "+"(replaced by the next one)":"")).join("\n");}
  set(id+"Ago",ago,agoC);
  if(port===0){for(const e of ["ck0Um","ck0UmNote","ck0Cmd"])$(e).classList.toggle("hidden",!um);}
  const w=p.health==="junk"?(p.bytes+" bytes in 5 s with nothing valid at 460800 baud: the receiver is at another baud rate (or has other settings, or is wired wrong)."):
   miss&&miss.length?"The receiver lacks settings the Teensy needs. «Load the AgOpenGPS settings» sets them.":"";
  set(id+"Warn",w,"bad");$(id+"Warn").classList.toggle("hidden",!w);
  // the buttons: "set to 460800" when the start-up search found another speed; "load" when something is not as AgOpenGPS wants
  const busy=!!c.op,canLoad=port===1?p.set:(ublox&&p.set)||um;
  $(id+"Check").disabled=busy||p.health==="junk"||(port===0&&!ublox&&!um)||(ub&&!p.set);
  $(id+"Check").textContent=ub&&p.set&&p.file==="X20P.txt"?"Compare with X20P.txt":"Compare with AOGConfigOMatic";
  $(id+"Baud").classList.toggle("hidden",!(p.scan==="found"&&p.found!==460800));$(id+"Baud").disabled=busy;
  $(id+"Load").classList.toggle("hidden",!(canLoad&&((miss&&miss.length)||differs)));
  $(id+"Load").disabled=busy||p.health==="junk";
  // "Read all settings" (taken off the page, see the HTML): u-blox only; the download row once it is done
  // $(id+"Dump").classList.toggle("hidden",!ub);$(id+"Dump").disabled=busy||p.health==="junk";
  // $(id+"DumpRow").classList.toggle("hidden",p.dump!=="dumped");$(id+"Dl").textContent="Download "+ckDumpName(p);
  const m=ckLocal[port]&&Date.now()-ckLocalAt[port]<5000?ckLocal[port]:ckMsgText(c,p,port,port===0&&um);set(id+"Msg",m[0],m[1]);
 }
}
const ckAct=(port,act,extra)=>gpsAct("act=ck"+act+"&port="+port+(extra||""),(t,c)=>ckSay(port,t,c));
function ckLoad(port){
 const j=gpsJ;if(!j||!j.ck)return;const p=j.ck.ports[port];
 if(port===0&&j.kind==="um982"){
  const s=$("ck0Um").value==="s",n=(s?j.ck.um982s:j.ck.um982d).length,f=s?"ConfigUM982S.txt":"ConfigUM982D.txt";
  if(confirm(("Load the AgOpenGPS settings into the UM982 ("+(s?"one antenna":"two antennas")+")?\n\nAOGConfigOMatic "+f+": "+n+" commands, one at a time, then SAVECONFIG (kept after a power off). They replace the receiver's settings these commands change. The GPS can drop out for a few seconds.")))ckAct(0,"load","&um="+(s?"s":"d"));
  return;}
 let q=("Load the AgOpenGPS settings into the receiver?\n\nAOGConfigOMatic "+p.file+" ("+p.role+"): "+p.keys+" settings, written to the receiver's RAM, BBR and flash (kept after a power off). They replace its current settings: rate, constellations, messages and ports. The GPS can drop out for a few seconds.");
 if(p.fwSt&&p.fwSt!=="same")q+="\n\n"+("These settings are for firmware "+p.setFw+"; the receiver has "+p.fw+".");
 if(confirm(q))ckAct(port,"load");}
for(const port of [0,1]){
 $("ck"+port+"Check").addEventListener("click",()=>ckAct(port,"check"));
 $("ck"+port+"Baud").addEventListener("click",()=>{const p=gpsJ&&gpsJ.ck&&gpsJ.ck.ports[port];if(!p)return;
  const uni=port===0&&p.foundKind!=="ublox",ubx=p.foundKind!=="unicore";
  let q=("Set the receiver to 460800 baud and save? It is at "+p.found+" baud.");
  if(ubx)q+="\n\n"+"u-blox: the UART1 speed is written to all the receiver's layers (RAM, BBR and flash).";
  if(uni)q+="\n\n"+"Unicore: CONFIG COM1, COM2 and COM3 460800, then SAVECONFIG.";
  if(confirm(q+"\n\n"+"Only with autosteer off: the GPS stops for a few seconds."))ckAct(port,"baud");});
 $("ck"+port+"Load").addEventListener("click",()=>ckLoad(port));
 // "Read all settings" (taken off the page, see the HTML)
 // $("ck"+port+"Dump").addEventListener("click",()=>ckAct(port,"dump"));
 // $("ck"+port+"Dl").addEventListener("click",()=>ckDownload(port));
}
// the file of "Read all settings" (GET /api/rxdump), downloaded by this browser as <model>.txt (the button is
// taken off the page for now, see the HTML; ckDumpName and this stay for when it comes back)
async function ckDownload(port){
 const p=gpsJ&&gpsJ.ck&&gpsJ.ck.ports[port];if(!p)return;
 try{const r=await fetch("/api/rxdump",{cache:"no-store"});if(!r.ok){ckSay(port,"Error"+": HTTP "+r.status,"bad");return;}
  const t=await r.text(),name=ckDumpName(p);
  const a=document.createElement("a");a.href=URL.createObjectURL(new Blob([t],{type:"text/plain"}));a.download=name;
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),10000);
  ckSay(port,"Downloaded: "+name,"ok");}catch(e){ckSay(port,"No connection to the module.","bad");}}
let ckUmAt=0;
$("ck0Um").addEventListener("change",async()=>{     /* UM982 antennas: a setting (Settings block), also the AgOpenGPS file */
 ckUmAt=Date.now();if(gpsJ)renderCk(gpsJ);
 try{const j=await getJ("/api/settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"umSingle="+($("ck0Um").value==="s"?1:0)});
  if(!j.ok)ckSay(0,"Error"+": "+ptErr(j.error),"bad");else await pollGps();}catch(e){ckSay(0,"No connection to the module.","bad");}});
function show(t){
 tab=t;for(const b of document.querySelectorAll("nav button"))b.classList.toggle("on",b.dataset.t===t);
 for(const s of document.querySelectorAll("main>section"))s.classList.toggle("hidden",s.id!=="t-"+t);
 if(t==="settings")loadSettings();
 if(t==="gps")pollGps();
 if(t==="log"){const b=$("log");b.scrollTop=b.scrollHeight;}
}
$("nav").addEventListener("click",e=>{if(e.target.dataset.t)show(e.target.dataset.t);});
document.addEventListener("click",e=>{const a=e.target.closest("a[data-go]");if(a){e.preventDefault();show(a.dataset.go);}});

// ---- settings
function selVal(e,v){e.value=String(v);if(e.value!==String(v)){const o=document.createElement("option");o.value=String(v);o.textContent=String(v);e.appendChild(o);e.value=String(v);}}
function fillSettings(s){
 const f=$("fs");
 f.querySelector('input[name=outputMode][value="'+s.outputMode+'"]').checked=true;
 f.hprAcceptFloat.checked=s.hprAcceptFloat==1;f.tm171SwapRollPitch.checked=s.tm171SwapRollPitch==1;
 f.epochWaitMs.value=String(s.epochWaitMs);
 if(f.epochWaitMs.value!==String(s.epochWaitMs)){const o=document.createElement("option");o.textContent=s.epochWaitMs;f.epochWaitMs.appendChild(o);f.epochWaitMs.value=String(s.epochWaitMs);}
 f.tm171YawSign.value=String(s.tm171YawSign);f.xiaoOff.value=String(s.xiaoOff);
 f.wasCheckOn.checked=s.wasCheckOff!=1;f.wdtOn.checked=s.wdtOff!=1;f.simMode.checked=s.simMode==1;
 selVal(f.steerMinKmhX10,s.steerMinKmhX10);selVal(f.steerMaxKmh,s.steerMaxKmh);selVal(f.softStartX10ms,s.softStartX10ms);selVal(f.currentKickX10ms,s.currentKickX10ms);lastSet=s;
}
async function loadSettings(){try{const j=await getJ("/api/settings");if(j.ok){fillSettings(j.settings);const m=$("setMsg");m.className="note";m.textContent=j.saved?"":"(defaults: nothing saved yet)";}}catch(e){}}
async function postSettings(body){
 const m=$("setMsg");
 try{const j=await getJ("/api/settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:body});
  if(j.ok){fillSettings(j.settings);m.textContent="Saved.";m.className="note ok";}else{m.textContent="Error"+": "+ptErr(j.error);m.className="note bad";}
 }catch(e){m.textContent="No connection to the module.";m.className="note bad";}
}
$("fs").addEventListener("submit",e=>{e.preventDefault();const f=$("fs");
 const p=new URLSearchParams();p.set("outputMode",f.querySelector("input[name=outputMode]:checked").value);
 p.set("hprAcceptFloat",f.hprAcceptFloat.checked?"1":"0");p.set("epochWaitMs",f.epochWaitMs.value);
 p.set("tm171SwapRollPitch",f.tm171SwapRollPitch.checked?"1":"0");p.set("tm171YawSign",f.tm171YawSign.value);
 p.set("xiaoOff",f.xiaoOff.value);
 if(!f.wasCheckOn.checked&&(!lastSet||lastSet.wasCheckOff!=1)&&!confirm("Switch the WAS check off? Do it only on a bench with no WAS connected."))return;
 p.set("wasCheckOff",f.wasCheckOn.checked?"0":"1");p.set("wdtOff",f.wdtOn.checked?"0":"1");
 if(f.simMode.checked&&(!lastSet||lastSet.simMode!=1)&&!confirm("Switch the simulator mode on? Only for tests with the program's simulator: the GPS is no longer sent to the program."))return;
 p.set("simMode",f.simMode.checked?"1":"0");
 p.set("steerMinKmhX10",f.steerMinKmhX10.value);p.set("steerMaxKmh",f.steerMaxKmh.value);
 p.set("softStartX10ms",f.softStartX10ms.value);p.set("currentKickX10ms",f.currentKickX10ms.value);
 postSettings(p.toString());});
$("defaults").addEventListener("click",()=>{if(confirm("Restore the defaults of this page?"))postSettings("defaults=1");});

// ---- UM982
for(const b of document.querySelectorAll("[data-c]"))b.addEventListener("click",()=>{$("umCmd").value=b.dataset.c;$("umCmd").focus();});
async function umSend(){
 const c=$("umCmd").value.trim();if(!c)return;
 if(/^(FRESET|RESET|UNLOG\b)/i.test(c)||/^CONFIG\s+COM\d\s+\d+/i.test(c)){if(!confirm("This command can cut the link between the UM982 and the Teensy. Send it anyway?"))return;}
 addLines($("umOut"),["> "+c],false);
 try{const j=await getJ("/api/um982",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"cmd="+encodeURIComponent(c)});
  if(!j.ok)addLines($("umOut"),["error"+": "+ptErr(j.error)],false);}catch(e){addLines($("umOut"),["no connection to the module"],false);}
 umUntil=Date.now()+6000;$("umCmd").select();
}
$("umSend").addEventListener("click",umSend);
$("umCmd").addEventListener("keydown",e=>{if(e.key==="Enter")umSend();});

// ---- firmware
function ptErr(e){return String(e||"");}
function fwMsg(t,c){const m=$("fwMsg");m.textContent=t;m.className="note "+(c||"");}
// What is inside a chosen .hex (Renato, 2026-10-24): the data records of the Intel HEX file put together, then the
// "[boot] vX.Y_AiO_WEB, built Mmm dd yyyy hh:mm:ss" line the firmware prints at start-up gives the version and the
// build date and time; "fw_teensy41" says it is a Teensy 4.1 firmware and the family marker (zOTA.ino) that it has
// this update page inside. Read in this browser, nothing is sent. hexBuild: the result for the chosen file (null: none).
let hexBuild=null;
function hexRead(text){
 let s="";
 for(const ln of text.split("\n")){if(ln.charCodeAt(0)!==58||ln.substr(7,2)!=="00")continue;const n=parseInt(ln.substr(1,2),16);let o="";for(let i=0;i<n;i++)o+=String.fromCharCode(parseInt(ln.substr(9+2*i,2),16));s+=o;}
 const m=/\[boot\] (v[\w.]+_AiO_WEB), built ([A-Z][a-z]{2} [ \d]\d \d{4} \d\d:\d\d:\d\d)/.exec(s);
 return {fw:m?m[1]:"",built:m?m[2].replace("  "," "):"",t41:s.includes("fw_teensy41"),marker:s.includes("AiO-WEB-OTA-1")||s.includes("AOG-UM982-TM171-WEB-OTA-1")};
}
$("hexFile").addEventListener("change",()=>{const f=$("hexFile").files[0];hexBuild=null;
 $("hexName").textContent=f?f.name+" ("+Math.round(f.size/1024)+" kB)":"no file";$("install").disabled=!f||busy;fwMsg("");
 const inf=$("hexInfo");inf.classList.toggle("hidden",!f);if(!f)return;
 inf.textContent="Reading the file…";inf.className="note";
 const r=new FileReader();
 r.onload=()=>{const b=hexBuild=hexRead(r.result);let t,c="";
  if(!b.t41){t="Not a Teensy 4.1 firmware (no fw_teensy41 inside): the Teensy refuses it.";c="bad";}
  else if(!b.fw){t="No AiO_WEB build data inside: another firmware. It needs «Other firmware» to install"+(b.marker?".":", and it has no update page: the next update would be by USB.");c="warn";}
  else{const same=!!lastS&&lastS.built===b.built;t=b.fw+", built "+b.built+(same?" (the firmware running now)":"")+(b.marker?"":"; no update page inside: needs «Other firmware»");c=same?"":"ok";}
  inf.textContent=t;inf.className="note "+c;};
 r.onerror=()=>{inf.textContent="The file could not be read.";inf.className="note bad";};
 r.readAsText(f);
});
$("install").addEventListener("click",()=>{
 const f=$("hexFile").files[0];if(!f||busy)return;
 if(!f.name.toLowerCase().endsWith(".hex")){fwMsg("Choose a .hex file.","bad");return;}
 const b=hexBuild&&hexBuild.fw?" ("+hexBuild.fw+", built "+hexBuild.built+")":"";
 if(!confirm(("Install "+f.name+b+"?\n\nAutosteer must be off and the tractor stopped. Autosteer is off during the update.")))return;
 busy=true;$("install").disabled=true;$("ghInstall").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;
 sendHex(f,$("force").checked,0);
});
// The .hex to the Teensy (a chosen file, or one downloaded from GitHub); progress from p0 to 95 %
function sendHex(blob,force,p0){
 const p=$("prog");fwMsg("Sending the file…");
 const x=new XMLHttpRequest();
 x.open("POST","/api/ota"+(force?"?force=1":""));
 x.setRequestHeader("Content-Type","text/plain");
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=p0+Math.round(e.loaded/e.total*(95-p0));};
 x.upload.onload=()=>fwMsg("File sent; the Teensy is checking and writing it…");
 const fail=t=>{busy=false;$("install").disabled=!$("hexFile").files[0];renderGh();p.classList.add("hidden");fwMsg(t,"bad");};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;upBefore=lastUp;waitingBoot=true;setOnline(false);
   fwMsg(("Checked ("+Math.round(j.size/1024)+" kB). Installing and restarting — do not switch the power off…"),"warn");}
  else fail("Not installed: "+ptErr(j.error||("error"+" "+x.status))+". The current firmware keeps running.");};
 x.onerror=()=>fail("The connection failed. The current firmware keeps running.");
 x.send(blob);
}

// ---- the report for an analysis: every answer of the module in one .json file, downloaded by this browser
$("dumpBtn").addEventListener("click",async()=>{
 const b=$("dumpBtn"),m=$("dumpMsg");b.disabled=true;m.className="note";m.textContent="Gathering…";
 const r={made:new Date().toString(),page:location.href,browser:navigator.userAgent};
 for(const [k,u] of [["diag","/api/diag"],["status","/api/status"],["settings","/api/settings"],["gps","/api/gps"]]){try{r[k]=await getJ(u);}catch(e){r[k]={error:String(e)};}}
 const log=[];let after=0;
 for(let i=0;i<30;i++){let j;try{j=await getJ("/api/log?after="+after);}catch(e){break;}
  if(!j.lines||!j.lines.length)break;for(const l of j.lines)log.push(dur(Math.floor(l.t/1000))+"  "+l.x);
  after=j.lines[j.lines.length-1].s;if(after>=j.next-1)break;}
 r.log=log;
 try{const j=await getJ("/api/um982?after=0");r.um982Console=j.lines||j;}catch(e){}
 const d=new Date(),z=n=>String(n).padStart(2,"0"),fw=((r.status&&r.status.fw)||"teensy").split(" ")[0];
 const name=fw+"_report_"+d.getFullYear()+"-"+z(d.getMonth()+1)+"-"+z(d.getDate())+"_"+z(d.getHours())+z(d.getMinutes())+".json";
 const a=document.createElement("a");a.href=URL.createObjectURL(new Blob([JSON.stringify(r,null,1)],{type:"application/json"}));a.download=name;
 document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),10000);
 m.className="note ok";m.textContent="Downloaded: "+name;b.disabled=false;});

// ---- a newer version on GitHub (renatorgr/Boards, folder Firmware, one folder per version:
// vX.Y_AiO_WEB with vX.Y_AiO_WEB.hex in it).
// Asked by this browser when the page opens, every 6 hours, and with "Check now".
// The newest version folder on GitHub and, from firmware.json (written by Firmware/simulator/build.py on every
// change on main and published with the simulator), the .hex in it and when it was compiled: a new build of the
// same version shows as new. Without firmware.json, the folder names alone (the version number only).
const GH_API="https://api.github.com/repos/renatorgr/Boards/contents/Firmware",GH_WEB="https://github.com/renatorgr/Boards/tree/main/Firmware";
const GH_PUB="https://renatorgr.github.io/Boards/firmware.json";
const GH_RX=/^v(\d+(?:\.\d+)*)_AiO_WEB$/,MON="JanFebMarAprMayJunJulAugSepOctNovDec";
let gh=null,ghState="",ghAt=0;
function verCmp(a,b){const x=String(a).split(".").map(Number),y=String(b).split(".").map(Number);for(let i=0;i<Math.max(x.length,y.length);i++){const d=(x[i]||0)-(y[i]||0);if(d)return d;}return 0;}
function builtKey(b){const m=/^([A-Z][a-z]{2}) +(\d{1,2}) (\d{4}) (\d\d):(\d\d):(\d\d)$/.exec(String(b||"").trim()),i=m?MON.indexOf(m[1]):-1;   // "Oct 31 2026 19:28:53" -> "20261031192853"
 return i<0||i%3?"":m[3]+String(i/3+1).padStart(2,"0")+m[2].padStart(2,"0")+m[4]+m[5]+m[6];}
function builtCmp(a,b){a=builtKey(a);b=builtKey(b);return a&&b?(a>b)-(a<b):0;}
function verText(v,b){const k=builtKey(b);return v+(k?", built "+(+k.slice(6,8))+" "+MON.substr(k.slice(4,6)*3-3,3)+" "+k.slice(0,4)+" "+k.slice(8,10)+":"+k.slice(10,12):"");}
function runningVer(){const m=lastS&&/^v(\d+(?:\.\d+)*)_/.exec(lastS.fw);return m?m[1]:"";}
async function ghList(path){const r=await fetch(GH_API+(path?"/"+path:"")+"?ref=main");/* a simple request: no CORS preflight */if(!r.ok)throw new Error("HTTP "+r.status);return r.json();}
async function ghCheck(){
 if(ghState==="busy")return;
 ghState="busy";renderGh();
 let best=null;
 try{const r=await fetch(GH_PUB,{cache:"no-store"}),j=r.ok?await r.json():null,m=j&&j.teensy&&GH_RX.exec(j.folder||"");
  if(m)best={ver:m[1],dir:j.folder,file:j.teensy.file,built:j.teensy.built||""};}catch(e){}
 try{if(!best){const l=await ghList("");
   for(const e of l)if(e.type==="dir"){const m=GH_RX.exec(e.name);if(m&&(!best||verCmp(m[1],best.ver)>0))best={ver:m[1],dir:e.name};}}
  gh=best;ghState=best?"ok":"none";}
 catch(e){ghState="fail";}
 ghAt=Date.now();renderGh();
}
function ghCmp(){const r=runningVer();return gh&&r?verCmp(gh.ver,r)||builtCmp(gh.built,lastS.built):NaN;}   // >0: GitHub's is newer
function ghNewer(){return ghCmp()>0;}
function renderGh(){
 const c=ghCmp(),nw=c>0,inst=lastS?verText(runningVer(),lastS.built):"";
 set("ghVer",ghState==="busy"?"checking…":ghState==="fail"?"no answer from GitHub (does this device have internet?)":ghState==="none"?"no version on GitHub":
  !gh?"–":verText(gh.ver,gh.built)+(nw?(" — new (installed: "+inst+")"):c===0?" — latest version installed":c<0?(" — the installed one ("+inst+") is newer"):""),
  nw?"ok":ghState==="fail"?"warn":"");
 $("ghInstall").disabled=!nw||busy;
 $("ghLink").href=gh?GH_WEB+"/"+gh.dir:GH_WEB;
 const n=$("ghNote");n.textContent=nw?("There is a new firmware on GitHub: "+verText(gh.ver,gh.built)+" (installed: "+inst+"). Install it in the Firmware tab."):"";
 n.classList.toggle("hidden",!nw);
}
async function ghInstall(){
 if(!ghNewer()||busy)return;
 const names=[gh.file,gh.dir+".hex",gh.dir+".ino.hex"].filter(Boolean);   // the Arduino IDE exports vX.Y_AiO_WEB.ino.hex
 if(!confirm(("Install "+verText(gh.ver,gh.built)+" from GitHub?\n\nAutosteer must be off and the tractor stopped. Autosteer is off during the update.")))return;
 busy=true;$("install").disabled=true;$("ghInstall").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;
 fwMsg("Downloading from GitHub…");
 try{
  const l=await ghList(gh.dir),f=names.map(n=>l.find(e=>e.type==="file"&&e.name===n)).find(Boolean);
  if(!f)throw new Error(("the file "+names[0]+" is not on GitHub"));
  const res=await fetch(f.download_url);if(!res.ok)throw new Error("GitHub: HTTP "+res.status);
  const rd=res.body.getReader(),parts=[];let got=0;
  for(;;){const {done,value}=await rd.read();if(done)break;parts.push(value);got+=value.length;p.value=Math.round(got/f.size*45);}
  const blob=new Blob(parts);
  if(blob.size!==f.size)throw new Error(("file incomplete ("+blob.size+" of "+f.size+" bytes)"));
  sendHex(blob,false,45);
 }catch(e){busy=false;$("install").disabled=!$("hexFile").files[0];p.classList.add("hidden");renderGh();
  fwMsg("Not downloaded: "+e.message+". The current firmware keeps running.","bad");}
}
$("ghCheck").addEventListener("click",ghCheck);
$("ghInstall").addEventListener("click",ghInstall);
$("reboot").addEventListener("click",async()=>{if(!confirm("Restart the Teensy? Autosteer and GPS stop for a few seconds."))return;
 try{await getJ("/api/reboot",{method:"POST"});}catch(e){}fwMsg("Restarting…","warn");});
$("logPause").addEventListener("click",()=>{logPaused=!logPaused;$("logPause").textContent=(logPaused?"Resume":"Pause");});
$("logClear").addEventListener("click",()=>{$("log").textContent="";});

// ---- one request at a time to the module, every second
let tickN=0;
async function tick(){
 tickN++;
 if(busy&&!waitingBoot)return;                        // the upload has the module to itself
 await pollStatus();
 if(lastS&&!busy&&(ghAt===0||Date.now()-ghAt>6*3600e3))ghCheck();   // GitHub, not waited for
 if(!online)return;
 if(tab==="log"||tickN%5===0)await pollLog();
 if(tab==="gps"){await pollGps();if(Date.now()<umUntil)await pollUm();}
}
// after a reload for a restart: the same tab, and the update's result
try{const t=sessionStorage.getItem("tab"),m=sessionStorage.getItem("fwMsg");sessionStorage.removeItem("tab");sessionStorage.removeItem("fwMsg");
 if(t&&$("t-"+t))show(t);if(m)fwMsg(m,"ok");}catch(e){}
(async function loop(){await tick();setTimeout(loop,1000);})();
</script>
</body>
</html>
)WEBPAGE";

#endif
