/* sPage.h - the sprayer module's web page (English).
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Look and feel from the XIAO page of the AiO v4.5 boards (Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32/xPage.h).
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One self-contained HTML file, no internet needed, made for a phone (and for an AgOpenWeb web card). Dark, or light
// when the phone asks for it. It asks the module for /api/status once a second. The tab is in the address (#settings,
// #module, #firmware), so a reload or a web card keeps it. Only the update check needs internet: the browser asks
// GitHub, never the module.
#pragma once

const char SPRAYER_PAGE[] PROGMEM = R"SPRAYERPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark light">
<title>Sprayer module</title>
<style>
:root{color-scheme:dark;--bg:#121511;--card:#1b1f1a;--ink:#e7ebe3;--mute:#9aa395;--line:#2e352b;--ok:#66bb6a;--warn:#ffb74d;--bad:#ef5350;--acc:#8cc068;--accInk:#10140f;--soft:#232a21}
@media (prefers-color-scheme:light){:root{color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);accent-color:var(--acc);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
#top{position:sticky;top:0;z-index:3}
header{display:flex;align-items:center;gap:10px;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0;font-weight:650;white-space:nowrap}
header .sub{color:var(--mute);font-size:13px;flex:1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.dot{width:11px;height:11px;border-radius:50%;background:var(--bad);flex:none}
.dot.on{background:var(--ok)}
nav{display:flex;gap:6px;padding:4px 12px;overflow-x:auto;background:var(--card);border-bottom:1px solid var(--line)}
nav button{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;min-height:44px;padding:7px 14px;font:inherit;font-size:14px;cursor:pointer;white-space:nowrap}
nav button.on{background:var(--acc);border-color:var(--acc);color:var(--accInk)}
nav button.new::after{content:"";display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--ok);margin-left:6px;vertical-align:middle}
nav button.on.new::after{background:var(--accInk)}
main{padding:14px 16px 40px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;min-width:0}
.card h2{font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--mute);margin:0 0 8px;font-weight:650}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:0}
dt{color:var(--mute)}
dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.big{font-size:26px;font-weight:650;font-variant-numeric:tabular-nums}
.big small{font-size:14px;color:var(--mute);font-weight:400;margin-left:4px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}
.tile{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:10px 14px;min-width:0}
.tile .lbl{font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--mute);font-weight:650}
.huge{font-size:52px;line-height:1.1;font-weight:700;font-variant-numeric:tabular-nums;white-space:nowrap}
.huge small{font-size:18px;color:var(--mute);font-weight:400;margin-left:6px}
.huge.on-target{color:var(--ok)}.huge.off-target{color:var(--warn)}
.tile .hint{font-size:15px;min-height:1.45em}
.alarm{background:var(--bad);color:#fff;border-radius:12px;padding:10px 14px;font-size:18px;font-weight:650;margin-top:12px}
.silent{border:2px solid var(--bad);color:var(--bad);border-radius:12px;padding:8px 14px;font-weight:650;margin-top:12px}
.pill{display:inline-block;padding:1px 9px;border-radius:999px;font-size:13px;font-weight:600;background:var(--soft)}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.secs{display:grid;grid-template-columns:repeat(8,1fr);gap:4px;margin-top:6px}
.secs span{height:26px;border-radius:5px;background:var(--soft);font-size:12px;display:flex;align-items:center;justify-content:center;color:var(--mute)}
.secs span.req{outline:2px solid var(--warn);outline-offset:-2px}
.secs span.on{background:var(--ok);color:var(--accInk);font-weight:700}
.secs.work span{height:48px;font-size:17px;border-radius:8px}
.secs.rl span{height:38px;flex-direction:column;line-height:1.1;font-size:11.5px;white-space:nowrap;overflow:hidden}
.secs.rl small{font-size:9.5px;font-weight:400;color:var(--mute)}
.secs.rl span.on small{color:inherit;opacity:.8}
.note{color:var(--mute);font-size:13.5px;margin:6px 0 0}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:10px 0 0}
button.act,label.act{border:0;background:var(--acc);color:var(--accInk);border-radius:8px;padding:9px 16px;font:inherit;font-weight:600;cursor:pointer}
button.sec{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button.danger{border:1px solid var(--bad);background:transparent;color:var(--bad);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button:disabled{opacity:.45;cursor:default}
.field{display:flex;flex-direction:column;gap:3px;margin:8px 0}
.field span{color:var(--mute);font-size:13.5px}
.two{display:grid;grid-template-columns:1fr 1fr;gap:0 10px}
input[type=text],input[type=password],input[type=number]{font:inherit;padding:9px 10px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);width:100%}
.opt{display:flex;gap:10px;align-items:center;padding:5px 0}
label.opt{align-items:flex-start}
label.opt input{margin-top:4px}
.opt small{display:block;color:var(--mute)}
.mono{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12.5px}
.nets{display:flex;flex-direction:column;gap:4px;margin:2px 0 6px}
.nets button{display:flex;justify-content:space-between;align-items:center;gap:10px;width:100%;text-align:left;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:8px;padding:8px 10px;font:inherit;cursor:pointer}
.nets button.sel{border-color:var(--acc);box-shadow:inset 0 0 0 1px var(--acc)}
.nets .n{overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.nets .r{color:var(--mute);font-size:13px;font-variant-numeric:tabular-nums;white-space:nowrap}
.log{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12.5px;white-space:pre-wrap;overflow-wrap:anywhere;color:var(--mute)}
.msg{font-size:13.5px;margin-top:6px;min-height:1.2em}
p.grp{margin:14px 0 2px;padding-top:10px;border-top:1px solid var(--line);font-weight:600;font-size:13.5px}hr.grp{border:0;border-top:1px solid var(--line);margin:14px 0 4px}
progress{width:100%;height:14px;margin-top:10px;accent-color:var(--acc)}
a{color:var(--acc)}
[hidden]{display:none!important}
/* the page's own question box (ask / askText in the script): the browser's confirm() and prompt() show nothing inside an
   AgOpenWeb web card (its frame has no allow-modals), so the page never uses them; buttons 44 px high, as the cards ask */
.ask{position:fixed;inset:0;z-index:9;background:rgba(0,0,0,.55);display:flex;align-items:center;justify-content:center;padding:12px}
.ask[hidden],.ask [hidden]{display:none!important}
.askBox{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px;width:100%;max-width:440px;max-height:100%;overflow:auto;box-shadow:0 8px 30px rgba(0,0,0,.35)}
.askBox p{margin:0 0 12px;white-space:pre-line}
.askBox input{display:block;width:100%;font:inherit;padding:9px 10px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);margin:0 0 12px}
.askBtns{display:flex;gap:10px;justify-content:flex-end}
.askBtns button{min-height:44px;min-width:96px}
/* the question box's other parts (askShow): a title, rows of what is in a file, the verdict, a tick, the password's label */
.askBox h3{margin:0 0 10px;font-size:17px}
.askBox dl{margin:0 0 10px}
.askBox p.verdict{font-weight:650}
.askBox label.opt{display:flex;gap:10px;align-items:center;margin:0 0 12px;font-weight:600}
.askBox label.opt input{display:inline-block;width:22px;height:22px;margin:0}
#askPwL{display:block;color:var(--mute);font-size:13.5px;margin:0 0 3px}
/* the Firmware tab: one card for the firmware, Check now / See on GitHub in a line, the restart at the card's foot */
.fwgrid{grid-template-columns:repeat(auto-fit,minmax(min(400px,100%),1fr))}
label.sec{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
label.sec.off{opacity:.45;cursor:default}
button.lnk{border:0;background:none;padding:0;font:inherit;color:var(--acc);text-decoration:underline;cursor:pointer}
p.chk{text-align:right;margin:4px 0 0}
.foot{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:12px 0 0;padding-top:10px;border-top:1px solid var(--line)}
.pill.ok{background:color-mix(in srgb,var(--ok) 16%,transparent)}.pill.warn{background:color-mix(in srgb,var(--warn) 18%,transparent)}
</style>
</head>
<body>
<div id="top">
<header><div class="dot" id="dot"></div><h1>Sprayer module</h1><div class="sub" id="sub">connecting…</div></header>
<nav id="nav"><button data-t="work" class="on">Spraying</button><button data-t="settings">Settings</button><button data-t="module">Module</button><button data-t="firmware">Firmware</button></nav>
</div>
<main>

<section id="t-work">
<div class="tiles" id="sprCard">
<div class="tile rf"><div class="lbl">Rate</div><div class="huge" id="lha">–<small>L/ha</small></div><div class="hint" id="target">–</div></div>
<div class="tile rp"><div class="lbl">Pressure</div><div class="huge" id="bar">–<small>bar</small></div><div class="hint" id="need"></div></div>
</div>
<div class="alarm" id="alarms" hidden></div>
<div class="silent" id="silent" hidden>No messages from AgOpenWeb: every section is closed.</div>
<div class="card" style="margin-top:12px"><h2>Sections</h2>
<div class="secs work" id="secs"></div>
<p class="note">Green = open. Orange outline = asked for by the program but not open (master switch off, or program silent).</p>
<p class="note" id="detail">–</p>
</div>
</section>

<section id="t-settings" hidden>
<div class="grid">

<div class="card"><h2>Spraying</h2>
<div class="field"><span>Target rate (L/ha, 0 = none). Only shown and checked: the pressure is set by hand</span><input type="number" id="rate" step="5" min="0"></div>
<div class="field"><span>Nozzle spacing (m): a section's nozzles = its width ÷ this</span><input type="number" id="nzSp" step="0.05" min="0"></div>
<div class="two">
<div class="field"><span>One nozzle (L/min, 0 = no flow alarm)</span><input type="number" id="nzFl" step="0.01" min="0"></div>
<div class="field"><span>at (bar), from the nozzle chart</span><input type="number" id="nzBar" step="0.1" min="0.1"></div>
<div class="field"><span>Minimum pressure (bar, 0 = off)</span><input type="number" id="pLow" step="0.1" min="0"></div>
<div class="field"><span>Maximum pressure (bar, 0 = off)</span><input type="number" id="pHigh" step="0.1" min="0"></div>
</div>
<div class="field"><span>Flow tolerance (%): alarm when the flow is this far from what the open nozzles should give</span><input type="number" id="fTol" step="1" min="1" max="100"></div>
<div class="row"><button class="act" onclick="saveSpray()">Save</button></div>
<div class="msg" id="msgSpray"></div>
<p class="note">An alarm comes on after 3 s: pressure below the minimum while spraying, above the maximum, flow outside the tolerance, or flow with every section closed. AgOpenWeb shows it and sounds it.</p>
</div>

<div class="card"><h2>Sections from AgOpenWeb</h2>
<p id="tool" class="big" style="font-size:20px">–</p>
<p class="note" id="toolNote">The section widths come from AgOpenWeb (Tool → Sections) and are kept here. They give the open width for the L/ha.</p>
</div>

<div class="card"><h2>Sensors</h2>
<div class="opt"><input type="checkbox" id="snP" onchange="showFitted()"><label for="snP">Pressure sensor fitted (G34)</label></div>
<div class="opt"><input type="checkbox" id="snF" onchange="showFitted()"><label for="snF">Flow meter fitted (G35)</label></div>
<p class="note" id="noSensors" hidden>No sensors: the module sends no sprayer readings (PGN 160 and 161), so AgOpenWeb and the AiO module page show none.</p>
<div class="field cf"><span>Flow meter pulses per litre (YF-S201: about 450)</span><input type="number" id="ppl" step="0.1"></div>
<div class="field cp"><span>Pressure sensor full scale (bar, at 4.5 V)</span><input type="number" id="pmax" step="0.1"></div>
<div class="field cp"><span>Divider on the pressure signal (10k / (10k + 10k) = 0.5)</span><input type="number" id="pdiv" step="0.001"></div>
<div class="row"><button class="act" onclick="saveCalib()">Save</button></div>
<div class="msg" id="msgCalib"></div>
<dl style="margin-top:8px"><dt class="rp">Sensor voltage</dt><dd class="rp" id="volts">–</dd><dt class="rf">Flow</dt><dd class="rf" id="lpm">–</dd><dt class="rf">Flow pulses</dt><dd class="rf" id="pulses">–</dd><dt class="rf">Applied</dt><dd class="rf" id="total">–</dd><dt>Master switch</dt><dd id="master">–</dd></dl>
<div class="row rf"><button class="sec" onclick="post('/api/total',{},'msgTotal')">Total back to 0</button></div>
<div class="msg" id="msgTotal"></div>
<p class="note cf">Bucket test: with one nozzle open, fill 10 L and note the litres counted. New pulses per litre = old × (counted ÷ 10).</p>
</div>

</div>
</section>

<section id="t-module" hidden>
<div class="grid">

<div class="card"><h2>AgOpenWeb</h2>
<dl>
<dt>Program</dt><dd id="link">–</dd>
<dt>Last section message</dt><dd id="ago239">–</dd>
<dt>Last hello</dt><dd id="agoHello">–</dd>
<dt>Speed</dt><dd id="kmh">–</dd>
<dt>Hydraulic lift</dt><dd id="hyd">–</dd>
<dt>Tramline</dt><dd id="tram">–</dd>
<dt>Messages</dt><dd id="counts">–</dd>
<dt>Output polarity</dt><dd id="invert">–</dd>
</dl>
</div>

<div class="card"><h2>Relays</h2>
<div class="secs rl" id="relays"></div>
<p class="note" id="relayNote">What each relay does comes from the program (Tool → Machine → Pin config).</p>
</div>

<div class="card"><h2>Module</h2>
<dl>
<dt>Board</dt><dd id="board">–</dd>
<dt>Name on the network</dt><dd id="name">–</dd>
<dt>Up</dt><dd id="up">–</dd>
<dt>Free memory</dt><dd id="heap">–</dd>
<dt>Own Wi-Fi network</dt><dd id="ap">–</dd>
<dt>Tractor network</dt><dd id="sta">–</dd>
<dt>Module IP</dt><dd id="ip">–</dd>
<dt>Signal</dt><dd id="rssi">–</dd>
<dt>Program subnet</dt><dd id="subnet">–</dd>
</dl>
<p class="note">The program looks for the machine module at &lt;subnet&gt;.123 on the tractor network. The subnet comes from the program (PGN 201), or is set below.</p>
</div>

<div class="card"><h2>Network settings</h2>
<p class="grp">Own Wi-Fi network</p>
<div class="field"><span>Network name</span><input type="text" id="apSsid" maxlength="32"></div>
<div class="field"><span>New password (only to change it: 8 to 63 characters; blank keeps the current one)</span><input type="password" id="apPass" maxlength="63"></div>
<p class="note">Always on, at 192.168.4.1. Its password is also the one for changing the settings and the firmware.</p>
<p class="grp">Tractor Wi-Fi network</p>
<label class="opt"><input type="checkbox" id="staOn"><span>Join the tractor network<small>The router the tablet uses. The page is then also at http://<b id="hostShow">sprayer</b>.local/ or at the module's IP on that network.</small></span></label>
<div class="field"><span>Network name</span><input type="text" id="staSsid" maxlength="32"></div>
<div class="row" style="margin:2px 0"><button class="sec" type="button" id="scan" onclick="scanNets()">Search for networks</button><span id="scanMsg" class="note"></span></div>
<div id="netList" class="nets"></div>
<div class="field"><span>That network's password (blank: keeps the current one; for a new network, blank = open network)</span><input type="password" id="staPass" maxlength="63"></div>
<div class="field"><span>Module name on that network</span><input type="text" id="host" maxlength="24" oninput="$('hostShow').textContent=this.value||'sprayer'"></div>
<p class="grp">Program network</p>
<div class="field"><span>Program subnet: the first three numbers of the tablet's IP on the tractor network</span><input type="text" id="subnetIn" maxlength="11" placeholder="192.168.5" inputmode="decimal"></div>
<div class="field"><span>Module IP, last number: 123 = the machine module; another number (124, 125…) for a second module. Not 126, the AiO steer module's</span><input type="number" id="ipLast" min="2" max="254" step="1"></div>
<hr class="grp">
<div class="field"><span>Current password of the own Wi-Fi network (to confirm)</span><input type="password" id="pwW" maxlength="63"></div>
<div class="row"><button class="act" type="button" onclick="saveWifi()">Save and restart the module</button></div>
<div class="msg" id="msgWifi"></div>
<p class="note">The module restarts to apply them: every section closes for a few seconds. If you change the name or the password of the own network, connect the phone to the new network again. Forgot the password? Hold the IO0 button 5 s, or in the serial monitor (USB): <span class="mono">wifi-reset</span>.</p>
</div>

<div class="card"><h2>Events</h2><div class="log" id="log"></div></div>

</div>
</section>

<section id="t-firmware" hidden>
<div class="grid fwgrid">

<div class="card"><h2>Sprayer firmware</h2>
<dl><dt>Installed</dt><dd id="ver">–</dd><dt>On GitHub</dt><dd id="ghVer">–</dd></dl>
<p class="note chk"><span id="ghAt"></span><button type="button" class="lnk" id="ghCheck" onclick="ghCheck()">Check now</button> · <a id="ghLink" href="https://github.com/renatorgr/Boards/tree/main/Sprayer/Sprayer_ESP32" target="_blank" rel="noopener">See on GitHub</a></p>
<div class="row"><button class="act" id="ghInstall" onclick="ghInstall()" disabled>Install from GitHub</button><label class="sec" id="fileBtn">Install from a file…<input type="file" id="bin" accept=".bin" hidden onchange="binChosen()"></label></div>
<progress id="prog" max="100" value="0" hidden></progress>
<div class="msg" id="msgProg"></div>
<p class="note">The sections close during the update. If it fails, the current firmware keeps running. Install from GitHub needs internet on this phone or tablet: open the page over the tractor network (the router with internet), not the module's own Wi-Fi. A file of your own: Arduino IDE, Sketch → Export compiled binary, then Sprayer_ESP32.ino.bin.</p>
<div class="foot"><button class="danger" id="restart" onclick="restartModule()">Restart module</button></div>
</div>

<div class="card"><h2>Report for an analysis</h2>
<p class="note">Puts the module's state, its settings, the readings, the link to AgOpenWeb and the events in one file, to send when something goes wrong. Passwords are not included. The file is downloaded by this phone or computer.</p>
<div class="row"><button class="sec" type="button" id="dumpBtn" onclick="report()">Download the report</button><span id="dumpMsg" class="note"></span></div>
</div>

</div>
</section>

</main>
<div id="ask" class="ask" hidden><div class="askBox" id="askBox" role="dialog" aria-modal="true" aria-labelledby="askText"><h3 id="askTitle" hidden></h3><dl id="askRows" hidden></dl><p id="askVerdict" class="verdict" hidden></p><p id="askText"></p><label id="askForce" class="opt" hidden><input type="checkbox" id="askForceIn"><span id="askForceT"></span></label><label id="askPwL" for="askIn" hidden></label><input id="askIn" type="password" autocomplete="current-password" hidden><div class="askBtns"><button type="button" class="sec" id="askNo">Cancel</button><button type="button" class="act" id="askYes">OK</button></div></div></div>
<script>
const $=id=>document.getElementById(id);
// ---- the page's own question box, instead of the browser's confirm() and prompt(): inside an AgOpenWeb web card the frame has
// no allow-modals, so those return false / null without showing anything (AgOpenWeb Docs/ADD_ON_CARDS.md). ask(text) answers
// true for OK and false for Cancel; askText(text) the text typed, or null for Cancel. A new question closes the one still open.
// askShow(o) is both, with more: o.text the question; o.input the password field (o.pw fills it, o.pwLabel goes above it);
// o.title, o.rows [[name, value]] and o.verdict [text, class] (what is in a file, compared with what runs now); o.force a tick
// that OK waits for; o.okText (null: no OK button) and o.noText the buttons' words. Answers true for OK, false otherwise.
let askDone=null;
function askShow(o){
  if(askDone)askEnd(false);
  const t=$('askTitle'),r=$('askRows'),v=$('askVerdict'),x=$('askText'),l=$('askPwL'),i=$('askIn'),y=$('askYes');
  t.textContent=o.title||'';t.hidden=!o.title;
  r.replaceChildren();for(const [k,w] of o.rows||[]){const a=document.createElement('dt'),b=document.createElement('dd');a.textContent=k;b.textContent=w;r.append(a,b)}r.hidden=!r.firstChild;
  v.hidden=!o.verdict;v.textContent=o.verdict?o.verdict[0]:'';v.className='verdict '+(o.verdict&&o.verdict[1]||'');
  x.textContent=o.text||'';x.hidden=!o.text;
  $('askForce').hidden=!o.force;$('askForceT').textContent=o.force||'';$('askForceIn').checked=false;
  l.textContent=o.pwLabel||'';l.hidden=!(o.input&&o.pwLabel);
  i.hidden=!o.input;i.value=o.pw||'';
  y.textContent=o.okText||'OK';y.hidden=o.okText===null;y.disabled=!!o.force;$('askNo').textContent=o.noText||'Cancel';
  $('askBox').setAttribute('aria-labelledby',o.title?'askTitle':'askText');$('ask').hidden=false;
  (o.input&&!i.value?i:y.hidden||y.disabled?$('askNo'):y).focus();
  return new Promise(res=>{askDone=res})}
function askEnd(v){const r=askDone;askDone=null;$('ask').hidden=true;if(r)r(v)}
const ask=text=>askShow({text});
const askText=text=>askShow({text,input:true}).then(ok=>ok?$('askIn').value:null);
$('askForceIn').onchange=()=>{$('askYes').disabled=!$('askForceIn').checked};
$('askYes').onclick=()=>askEnd(true);$('askNo').onclick=()=>askEnd(false);
$('ask').onclick=e=>{if(e.target===$('ask'))askEnd(false)};
$('ask').onkeydown=e=>{if(e.key==='Escape')askEnd(false);else if(e.key==='Enter'&&e.target===$('askIn')&&!$('askYes').disabled&&!$('askYes').hidden)askEnd(true)};
// relay functions as the module sends them: 1-64 a section, 101-105 the program's other functions
const FN={0:'–',101:'Lift ↓',102:'Lift ↑',103:'Tram R',104:'Tram L',105:'Geo'};
function fnName(f){return f>=1&&f<=64?'S'+f:(FN[f]||'?')}
// the sprayer alarms (rate.alarms bits), as AgOpenWeb words them
const ALARMS=[[1,'Low pressure'],[2,'High pressure'],[4,'Flow too low'],[8,'Flow too high'],[16,'Flow with the sections off']];
const RATE_TOLERANCE=0.10;   // the rate is green within 10 % of the target, orange outside (as AgOpenWeb's strip)
// sections come as hex digits, section 1 = the lowest bit ("7ffff" = sections 1-19)
function bit(h,i){const k=h.length-1-(i>>2);return k>=0&&(parseInt(h[k],16)>>(i&3)&1)===1}
const pw=()=>askText('Password of the own Wi-Fi network');   // a Promise: await it
function cells(id,n,from){let h='';for(let i=0;i<n;i++)h+='<span id="'+id+(from+i)+'">'+(from+i+1)+'</span>';return h}
let nSecs=0,nRelays=0;
function ago(s){return s<0?'never':s===0?'now':s+' s ago'}
const f1=v=>v.toFixed(1);
// show what belongs to the sensors fitted (1 pressure, 2 flow): class p = pressure, f = flow
function fitted(pre,s){for(const e of document.querySelectorAll('.'+pre+'p'))e.hidden=!(s&1);for(const e of document.querySelectorAll('.'+pre+'f'))e.hidden=!(s&2)}
function ticked(){return($('snP').checked?1:0)|($('snF').checked?2:0)}
function showFitted(){const s=ticked();fitted('c',s);$('noSensors').hidden=s!==0}
// tabs: the one in the address (#settings, #module, #firmware), else Spraying
function show(t){
  if(!['work','settings','module','firmware'].includes(t))t='work';
  for(const b of document.querySelectorAll('nav button'))b.classList.toggle('on',b.dataset.t===t);
  for(const s of document.querySelectorAll('main>section'))s.hidden=s.id!=='t-'+t;
  history.replaceState(null,'',t==='work'?location.pathname:'#'+t);
}
$('nav').addEventListener('click',e=>{if(e.target.dataset.t)show(e.target.dataset.t)});
show(location.hash.slice(1));
// the tool's sections from AgOpenWeb (PGN 235): "8 sections of 1.00 m", or each width when they differ
function toolText(t){
  if(!t.n)return null;
  const w=t.cm.map(c=>(c/100).toFixed(2));
  if(w.length&&w.every(x=>x===w[0]))return t.n+' section'+(t.n>1?'s':'')+' of '+w[0]+' m';
  return t.n+' sections: '+w.join(' · ')+' m';
}
let settingsLoaded=false;
async function refresh(){
  if(busy&&!waitBoot)return;   // sending a program: the module has no time for anything else
  try{
    const r=await fetch('/api/status',{cache:'no-store'});const d=await r.json();
    $('dot').className='dot'+(d.aog.link?' on':'');
    $('sub').textContent=(d.wifi.up?'on '+d.wifi.sta+' as '+d.wifi.ip:'not on the tractor network')+(d.aog.link?' · program talking':' · program silent');
    const sn=d.cal.sensors===undefined?3:d.cal.sensors;$('sprCard').hidden=!sn;fitted('r',sn);
    // Spraying tab: rate against the target, pressure and what gives the target, alarms, sections
    const ra=d.rate||{lha:-1,target:0,needBar:-1,hintKmh:-1,alarms:0,width:0,nozzles:0,expected:-1},tool=d.tool||{n:0,cm:[]};
    const lha=$('lha');lha.innerHTML=(ra.lha>=0?Math.round(ra.lha):'–')+'<small>L/ha</small>';
    const off=ra.target>0&&ra.lha>=0?Math.abs(ra.lha-ra.target)>ra.target*RATE_TOLERANCE:null;
    lha.className='huge'+(off===null?'':off?' off-target':' on-target');
    $('target').textContent=!tool.n?'no section widths from AgOpenWeb yet (Settings)':(ra.target>0?'target '+Math.round(ra.target)+' L/ha':'no target set (Settings)')+(ra.lha<0&&d.aog.link?' · stopped':'');
    $('bar').innerHTML=f1(d.spr.bar)+'<small>bar</small>';
    let need='';
    if(ra.needBar>0&&ra.hintKmh>0)need=f1(ra.needBar)+' bar needed, '+(ra.needBar>ra.maxBar?'slow down':'speed up')+' to '+f1(ra.hintKmh)+' km/h';
    else if(ra.needBar>0)need='set '+f1(ra.needBar)+' bar for the target';
    $('need').textContent=need;
    const al=ALARMS.filter(([b])=>ra.alarms&b).map(([,t])=>t);
    $('alarms').hidden=!al.length;$('alarms').textContent=al.join(' · ');
    $('silent').hidden=d.aog.link;
    // as many sections as the program has asked for (secTop), at least those the relays follow (up to 16)
    let mapped=0,nSec=0;for(const f of d.out.fn)if(f>=1&&f<=64){nSec++;mapped=Math.max(mapped,f)}
    const n=Math.max(tool.n||8,d.aog.secTop||0,Math.min(16,mapped));
    if(nSecs!==n){nSecs=n;$('secs').innerHTML=cells('s',n,0);$('secs').style.gridTemplateColumns='repeat('+Math.min(n,8)+',1fr)'}
    for(let i=0;i<nSecs;i++){const e=$('s'+i),on=bit(d.out.on,i);e.className=(on?'on':'')+(bit(d.aog.req,i)&&!on?' req':'')}
    const det=[];
    if(sn&2)det.push(f1(d.spr.lpm)+' L/min');
    det.push(f1(d.aog.kmh)+' km/h');
    if(tool.n)det.push(ra.width.toFixed(2)+' m open'+(ra.nozzles?' ('+ra.nozzles+' nozzles)':''));
    if(ra.expected>=0&&ra.nozzles)det.push('expected '+f1(ra.expected)+' L/min');
    if(sn&2)det.push(f1(d.spr.total)+' L applied');
    if(!d.spr.master)det.push('master switch off');
    $('detail').textContent=det.join(' · ');
    // Settings tab
    const tt=toolText(tool);$('tool').textContent=tt||'not received yet';$('tool').className='big'+(tt?'':' warn');
    $('toolNote').textContent=tt?'From AgOpenWeb (Tool → Sections), kept here. They give the open width for the L/ha.'
      :'AgOpenWeb sends the section widths when it starts and when a tool profile is saved or loaded. With the module on the network: in AgOpenWeb, save the tool profile (or restart AgOpenWeb). Until then there is no L/ha.';
    $('volts').textContent=d.spr.volts.toFixed(2)+' V';
    $('lpm').textContent=f1(d.spr.lpm)+' L/min';
    $('pulses').textContent=d.spr.pulses;
    $('total').innerHTML=f1(d.spr.total)+' L';
    $('master').innerHTML=d.spr.master?'<span class="ok">on</span>':'<span class="bad">off</span>';
    // Module tab
    if(nRelays!=d.out.fn.length){nRelays=d.out.fn.length;$('relays').innerHTML=cells('r',nRelays,0)}
    for(let i=0;i<nRelays;i++){const e=$('r'+i),f=d.out.fn[i];e.innerHTML='<small>'+(i+1)+'</small>'+fnName(f);e.className=d.out.bits>>>i&1?'on':''}
    $('relayNote').textContent=nRelays+' relays, '+nSec+' of them sections: what each relay does comes from the program (Tool → Machine → Pin config).';
    $('link').innerHTML=d.aog.link?'<span class="ok">talking</span>':'<span class="bad">silent, sections closed</span>';
    $('ago239').textContent=ago(d.aog.ago239);$('agoHello').textContent=ago(d.aog.agoHello);
    $('kmh').textContent=f1(d.aog.kmh)+' km/h';
    $('hyd').textContent=(d.aog.hydEn?'enabled, ':'disabled, ')+(d.aog.hyd==1?'lowering':d.aog.hyd==2?'raising':'idle')+' (raise '+d.aog.raise+' s, lower '+d.aog.lower+' s)';
    $('tram').textContent=(d.aog.tram&1?'right ':'')+(d.aog.tram&2?'left':'')||'off';
    $('counts').textContent=d.aog.n239+' sections'+(d.aog.n229?' ('+d.aog.n229+' with 17-64)':'')+', '+d.aog.nHello+' hellos, '+(tool.n235||0)+' section widths, '+d.aog.crcErr+' bad checksums';
    $('invert').innerHTML=(d.aog.invert?'<span class="bad">inverted ("relay active high" ticked in the program)</span>':'<span class="ok">normal</span>')+', relay on = pin '+((d.out.onHigh!==false)!=!!d.aog.invert?'high':'low');
    $('ap').textContent=d.wifi.ap+' · '+d.wifi.apCl+' device(s)';
    $('sta').innerHTML=!d.wifi.staOn?'<span class="warn">not set</span>':d.wifi.up?'<span class="ok">'+d.wifi.sta+'</span>':'<span class="bad">'+d.wifi.sta+' (not joined)</span>';
    $('ip').textContent=d.wifi.up?d.wifi.ip:'–';$('rssi').textContent=d.wifi.up?d.wifi.rssi+' dBm':'–';
    $('subnet').textContent=d.wifi.subnet+' → module '+d.wifi.subnet+'.'+(d.wifi.ipLast||123);
    $('log').textContent=d.log.join('\n');
    // Firmware tab
    $('ver').textContent=d.fw+' '+verText(d.ver,d.built);$('board').textContent=d.board||'–';
    $('up').textContent=Math.floor(d.up/60)+' min';$('heap').textContent=Math.round(d.heap/1024)+' kB';$('name').textContent=d.name;
    running(d);
    if(!settingsLoaded){settingsLoaded=true;loadSettings()}
  }catch(e){$('sub').textContent='no answer from the module';$('dot').className='dot';if(waitBoot)progMsg('The module is restarting…','warn')}
}
async function loadSettings(){
  const d=await (await fetch('/api/settings',{cache:'no-store'})).json();
  $('apSsid').value=d.apSsid;$('staOn').checked=d.staOn;$('staSsid').value=d.staSsid;$('host').value=d.host;$('subnetIn').value=d.subnet;
  $('hostShow').textContent=d.host;$('ipLast').value=d.ipLast||123;
  $('apPass').value='';$('staPass').value='';$('staPass').placeholder=d.staPassSet?'(saved)':'';
  $('ppl').value=d.ppl;$('pmax').value=d.pmax;$('pdiv').value=d.pdiv;
  const sn=d.sensors===undefined?3:d.sensors;$('snP').checked=!!(sn&1);$('snF').checked=!!(sn&2);showFitted();
  for(const k of ['rate','nzSp','nzFl','nzBar','pLow','pHigh','fTol'])if(d[k]!==undefined)$(k).value=d[k];
}
async function post(url,fields,msgId,pass){
  const p=pass!==undefined?pass:await pw();if(p===null)return;
  const body=new URLSearchParams(Object.assign({pw:p},fields));
  const m=$(msgId);m.textContent='…';
  try{const r=await fetch(url,{method:'POST',body});const d=await r.json();
    // saved: the fields show what the module kept, before "done" (a restart: once it is back)
    if(d.ok&&d.restart)settingsLoaded=false;
    else if(d.ok)try{await loadSettings()}catch(e){}
    m.innerHTML=d.ok?'<span class="ok">done'+(d.restart?', restarting: reconnect in 10 s':'')+'</span>':'<span class="bad">'+d.error+'</span>';
    return d;
  }catch(e){m.innerHTML='<span class="bad">no answer</span>'}
}
// Wi-Fi settings, as on the XIAO page: the password typed in the Save card, a question before the restart
async function saveWifi(){const m=$('msgWifi');
  if($('apPass').value&&$('apPass').value.length<8){m.innerHTML='<span class="bad">The password must have 8 characters or more.</span>';return}
  if(!$('pwW').value){m.innerHTML='<span class="bad">Type the password of the own Wi-Fi network.</span>';$('pwW').focus();return}
  if(!await ask('Save and restart the module? Its Wi-Fi goes off for a few seconds and every section closes.'))return;
  post('/api/wifi',{apSsid:$('apSsid').value,apPass:$('apPass').value,staOn:$('staOn').checked?'1':'0',staSsid:$('staSsid').value,staPass:$('staPass').value,host:$('host').value,subnet:$('subnetIn').value.trim(),ipLast:$('ipLast').value},'msgWifi',$('pwW').value)}
// "Search for networks": the networks the module hears (GET /api/scan, asked again until the list comes), one per
// line, strongest first; a tap chooses one (as on the XIAO page)
function sig(r){return r>=-60?'strong':r>=-72?'medium':'weak'}
function showNets(list){const box=$('netList');box.textContent='';
  for(const n of list){const b=document.createElement('button');b.type='button';b.classList.toggle('sel',n.s===$('staSsid').value);
    const a=document.createElement('span');a.className='n';a.textContent=n.s;b.appendChild(a);
    const r=document.createElement('span');r.className='r';r.textContent=n.r+' dBm · '+sig(n.r)+(n.open?' · open':'');b.appendChild(r);
    b.addEventListener('click',()=>{const same=$('staSsid').value===n.s;$('staSsid').value=n.s;$('staOn').checked=true;
      for(const o of box.children)o.classList.toggle('sel',o===b);
      if(!same){$('staPass').value='';$('staPass').placeholder=n.open?'(open network)':'';if(!n.open)$('staPass').focus()}});
    box.appendChild(b)}}
async function scanNets(){const m=$('scanMsg'),b=$('scan');m.textContent='searching…';m.className='note';$('netList').textContent='';b.disabled=true;
  try{for(let i=0;i<20;i++){
      try{const j=await (await fetch('/api/scan',{cache:'no-store'})).json();
        if(j.list){j.list.sort((x,y)=>y.r-x.r);showNets(j.list);const n=j.list.length;
          m.textContent=n?(n===1?'1 network':n+' networks')+' · tap one to choose it':'no network found';return}}catch(e){}
      await new Promise(r=>setTimeout(r,800))}
    m.textContent='no answer'}
  finally{b.disabled=false}}
function saveCalib(){post('/api/calib',{sensors:ticked(),ppl:$('ppl').value,pmax:$('pmax').value,pdiv:$('pdiv').value},'msgCalib')}
function saveSpray(){const f={};for(const k of ['rate','nzSp','nzFl','nzBar','pLow','pHigh','fTol'])f[k]=$(k).value.trim();post('/api/spray',f,'msgSpray')}
// the report for an analysis: every answer of the module in one .json file, downloaded by this browser
async function report(){
  const b=$('dumpBtn'),m=$('dumpMsg');b.disabled=true;m.className='note';m.textContent='Gathering…';
  const r={made:new Date().toString(),page:location.href,browser:navigator.userAgent};
  for(const [k,u] of [['status','/api/status'],['settings','/api/settings']]){
    try{r[k]=await (await fetch(u,{cache:'no-store'})).json()}catch(e){r[k]={error:String(e)}}}
  const d=new Date(),z=n=>String(n).padStart(2,'0'),fw=(r.status&&r.status.fw)||'Sprayer_ESP32';
  const name=fw+'_report_'+d.getFullYear()+'-'+z(d.getMonth()+1)+'-'+z(d.getDate())+'_'+z(d.getHours())+z(d.getMinutes())+'.json';
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(r,null,1)],{type:'application/json'}));a.download=name;
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),10000);
  m.className='note ok';m.textContent='Downloaded: '+name;b.disabled=false;
}
// ---- the firmware: the one on GitHub, or a chosen .bin; both are checked here before they are sent, and every installation
// is confirmed in one box (fwAsk) that shows what is in the file, compared with what runs now, and asks the own network's password.
// What a .bin is: an ESP32 program (first byte 0xE9, chip 0 = the plain ESP32 of these boards) and, from the text every
// build carries (the start of its /api/status answer, zNet.ino), which program, version, board and build.
async function binInfo(blob,name){
  if(/merged|bootloader|partitions/i.test(name))return {error:'This is the wrong file: use Sprayer_ESP32.ino.bin, not the '+name+'.'};
  const h=new Uint8Array(await blob.slice(0,16).arrayBuffer());
  if(h[0]!==0xE9)return {error:'This file is not an ESP32 program (wrong .bin).'};
  if((h[12]|h[13]<<8)!==0)return {error:'This program is for another kind of ESP32 (S3, C3…), not for this module.'};
  const t=new TextDecoder('latin1').decode(await blob.arrayBuffer());
  const m=/\{"fw":"([^"]+)","ver":"([^"]*)","built":"([^"]*)"/.exec(t),b=/,"board":"([^"]+)"/.exec(t);
  const i={fw:m?m[1]:'',ver:m?m[2]:'',built:m?m[3]:'',board:b?b[1]:''};
  if(!fwNow)return i;
  if(i.fw!==fwNow.fw)i.error=i.fw?'This is the program '+i.fw+', not this module\'s ('+fwNow.fw+').':'This file is not the '+fwNow.fw+' program.';
  else if(i.board&&fwNow.board&&i.board!==fwNow.board)i.error='This program is for the '+i.board+' board, but this module is on the '+fwNow.board+' board (its relays are on other pins).';
  return i;
}
function infoText(i){return i.fw+' '+verText(i.ver,i.built)+(i.board?' for the '+i.board:'')}
function progMsg(t,c){const m=$('msgProg');m.className='msg'+(c?' '+c:'');m.textContent=t}
// pwKeep: the password typed in the box, offered again while this page stays open (forgotten when the module refuses it)
let busy=false,waitBoot=false,upBefore=0,fwNow=null,pwKeep='';
// what the module runs; after an installation, the restart is seen when it is up again for less time than before
function running(d){
  const k=d.fw+' '+d.ver+' '+d.built+' '+d.board,was=fwNow&&fwNow.key;
  fwNow={fw:d.fw,ver:d.ver,built:d.built,board:d.board,key:k};
  if(waitBoot&&d.up<upBefore){waitBoot=false;busy=false;$('prog').hidden=true;
    progMsg('Back: it runs '+infoText(fwNow)+'.','ok');fwButtons()}
  upBefore=Math.max(upBefore,d.up);
  if(k!==was)renderGh();
  if(!busy&&(ghAt===0||Date.now()-ghAt>6*3600e3))ghCheck();   // GitHub, not waited for
}
// the card's buttons: off while an installation runs
function fwButtons(){
  $('ghInstall').disabled=!ghNewer()||busy;$('bin').disabled=busy;$('fileBtn').classList.toggle('off',busy);$('restart').disabled=busy;
}
// newer, the same or older than the firmware running now
function cmpNow(ver,built){return fwNow?(ver&&fwNow.ver?verCmp(ver,fwNow.ver):0)||builtCmp(built,fwNow.built):NaN}
function verdict(c){return c>0?['✓ Newer than the one installed.','ok']:c===0?['The same as the one installed.','']:c<0?['Older than the one installed.','warn']:null}
// the one confirmation box of this tab, with the own network's password: answers the password, or null for Cancel
async function fwAsk(o){
  if(!await askShow(Object.assign({input:true,pw:pwKeep,pwLabel:'Password of the own Wi-Fi network'},o)))return null;
  pwKeep=$('askIn').value;return pwKeep;
}
// Install from a file: the file is read as soon as it is chosen, and the box says what it is
async function binChosen(){
  const inp=$('bin'),f=inp.files[0];inp.value='';   // choosing the same file again is a new choice
  if(!f||busy)return;
  if(!fwNow){progMsg('No answer from the module yet.','bad');return}
  progMsg('');
  const i=await binInfo(f,f.name),file=f.name+' · '+Math.round(f.size/1024)+' kB';
  if(i.error){
    if(await askShow({title:'This file cannot be installed',rows:[['File',file]],verdict:['✗ '+i.error,'bad'],okText:'Choose another file',noText:'Close'}))inp.click();
    return}
  const p=await fwAsk({title:'Install this file?',rows:[['File',file],['Inside',infoText(i)],['Runs now',infoText(fwNow)]],
    verdict:verdict(cmpNow(i.ver,i.built)),text:'The sections close during the update.',okText:'Install'});
  if(p===null)return;
  busy=true;fwButtons();const g=$('prog');g.hidden=false;g.value=0;
  sendBin(f,f.name,p,0);
}
// the .bin to the module, progress from p0 to 95 %
function sendBin(blob,name,pass,p0){
  const g=$('prog');progMsg('Sending '+name+'…');
  const fd=new FormData();fd.append('bin',blob,name);
  const x=new XMLHttpRequest();x.open('POST','/api/update?pw='+encodeURIComponent(pass));
  x.upload.onprogress=e=>{if(e.lengthComputable)g.value=p0+Math.round(e.loaded/e.total*(95-p0))};
  const fail=t=>{busy=false;g.hidden=true;progMsg(t,'bad');fwButtons()};
  x.onload=()=>{let j={};try{j=JSON.parse(x.responseText)}catch(e){}
    if(x.status===200&&j.ok){g.value=100;waitBoot=true;progMsg('Installed. The module is restarting…','warn')}
    else{if(x.status===403)pwKeep='';fail('Not installed: '+(j.error||('error '+x.status))+'. The current firmware keeps running.')}};
  x.onerror=()=>fail('The connection failed. The current firmware keeps running.');
  x.send(fd);
}
// Restart module: the same box, with the password
async function restartModule(){
  if(busy)return;
  const p=await fwAsk({title:'Restart the module?',text:'Every section closes for a few seconds.',okText:'Restart'});
  if(p===null)return;
  const d=await post('/api/restart',{},'msgProg',p);
  if(d&&!d.ok&&/password/.test(d.error||''))pwKeep='';
}

// ---- a newer firmware on GitHub (renatorgr/Boards, Sprayer/Sprayer_ESP32/Sprayer_ESP32.ino.bin). firmware.json, written
// by Firmware/simulator/build.py on every change on main and published with the simulator, says which version that .bin
// is, for which board and when it was compiled (read from the .bin itself), so a new build of the same version shows as
// new. Asked by this browser when the page opens, every 6 hours, and with "Check now".
const GH_PUB='https://renatorgr.github.io/Boards/firmware.json',GH_API='https://api.github.com/repos/renatorgr/Boards/contents/';
const GH_WEB='https://github.com/renatorgr/Boards/tree/main/',MON='JanFebMarAprMayJunJulAugSepOctNovDec';
let gh=null,ghState='',ghAt=0;
function verCmp(a,b){const x=String(a).split('.').map(Number),y=String(b).split('.').map(Number);for(let i=0;i<Math.max(x.length,y.length);i++){const d=(x[i]||0)-(y[i]||0);if(d)return d}return 0}
function builtKey(b){const m=/^([A-Z][a-z]{2}) +(\d{1,2}) (\d{4}) (\d\d):(\d\d):(\d\d)$/.exec(String(b||'').trim()),i=m?MON.indexOf(m[1]):-1;   // "Nov  2 2026 10:41:07" -> "20261102104107"
  return i<0||i%3?'':m[3]+String(i/3+1).padStart(2,'0')+m[2].padStart(2,'0')+m[4]+m[5]+m[6]}
function builtCmp(a,b){a=builtKey(a);b=builtKey(b);return a&&b?(a>b)-(a<b):0}
function builtText(b){const k=builtKey(b);return k?(+k.slice(6,8))+' '+MON.substr(k.slice(4,6)*3-3,3)+' '+k.slice(0,4)+' '+k.slice(8,10)+':'+k.slice(10,12):''}   // "2 Nov 2026 10:41"
function verText(v,b){const t=builtText(b);return (v?'v'+v:'')+(t?(v?', ':'')+'built '+t:'')}
async function ghCheck(){
  if(ghState==='busy')return;
  ghState='busy';renderGh();
  try{const r=await fetch(GH_PUB,{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);
    const j=await r.json();gh=j.sprayer&&j.sprayer.file?j.sprayer:null;ghState=gh?'ok':'none'}
  catch(e){ghState='fail'}
  ghAt=Date.now();renderGh();
}
function ghBoardOk(){return !gh||!gh.board||!fwNow||!fwNow.board||gh.board===fwNow.board}
function ghCmp(){return gh?cmpNow(gh.version,gh.built):NaN}   // >0: GitHub's is newer
function ghNewer(){return ghCmp()>0&&ghBoardOk()}
// On GitHub: the version and a pill (newer, installed, older); when it was asked, in the line under it
function renderGh(){
  const c=ghCmp(),nw=ghNewer(),e=$('ghVer');let t='–',cls='',pill=null;
  if(ghState==='busy')t='searching…';
  else if(ghState==='fail'){t='no answer from GitHub (does this phone or tablet have internet?)';cls='warn'}
  else if(ghState==='none')t='no sprayer firmware on GitHub';
  else if(gh){t=verText(gh.version,gh.built)||'a firmware';
    if(!ghBoardOk()){t+=' — for the '+gh.board+' board, but this module is on the '+fwNow.board+' board';cls='warn'}
    else if(c>0){pill=['newer','ok'];cls='ok'}
    else if(c===0)pill=['installed',''];
    else if(c<0)pill=['older','warn']}
  e.textContent=t;e.className=cls;
  if(pill){const s=document.createElement('span');s.className='pill '+pill[1];s.textContent=pill[0];
    if(c<0)s.title='The new .bin is not on GitHub yet';e.append(' ',s)}
  const at=ghAt?new Date(ghAt):null;
  $('ghAt').textContent=ghState!=='busy'&&at?'checked at '+String(at.getHours()).padStart(2,'0')+':'+String(at.getMinutes()).padStart(2,'0')+' · ':'';
  fwButtons();
  $('ghLink').href=GH_WEB+(gh?gh.file.replace(/\/[^/]*$/,''):'Sprayer/Sprayer_ESP32');
  const b=document.querySelector('nav button[data-t="firmware"]');b.classList.toggle('new',nw);b.title=nw?'A newer firmware is on GitHub':'';
}
async function ghInstall(){
  if(!ghNewer()||busy)return;
  const name=gh.file.split('/').pop(),want=gh.size||0;
  const pass=await fwAsk({title:'Install from GitHub?',rows:[['On GitHub',fwNow.fw+' '+verText(gh.version,gh.built)],['Runs now',infoText(fwNow)]],
    verdict:verdict(ghCmp()),text:'The sections close during the update.',okText:'Install'});
  if(pass===null)return;
  busy=true;fwButtons();const g=$('prog');g.hidden=false;g.value=0;
  progMsg('Downloading from GitHub…');
  try{
    const dir=gh.file.replace(/\/[^/]*$/,''),r=await fetch(GH_API+dir+'?ref=main');if(!r.ok)throw new Error('GitHub: HTTP '+r.status);
    const f=(await r.json()).find(e=>e.type==='file'&&e.name===name);
    if(!f)throw new Error('the file '+name+' is not on GitHub');
    if(want&&f.size!==want)throw new Error('GitHub is still publishing the new file: try again in a few minutes');
    const res=await fetch(f.download_url);if(!res.ok)throw new Error('GitHub: HTTP '+res.status);
    const rd=res.body.getReader(),parts=[];let got=0;
    for(;;){const {done,value}=await rd.read();if(done)break;parts.push(value);got+=value.length;g.value=Math.round(got/f.size*45)}
    const blob=new Blob(parts);
    if(blob.size!==f.size)throw new Error('file incomplete ('+blob.size+' of '+f.size+' bytes)');
    const i=await binInfo(blob,name);if(i.error)throw new Error(i.error);
    if(gh.built&&i.built!==gh.built)throw new Error('GitHub is still publishing the new file: try again in a few minutes');
    sendBin(blob,name,pass,45);
  }catch(e){busy=false;g.hidden=true;fwButtons();
    progMsg('Not downloaded: '+e.message+'. The current firmware keeps running.','bad')}
}
refresh();setInterval(refresh,1000);
</script>
</body>
</html>
)SPRAYERPAGE";
