/* sPage.h - the sprayer module's web page (English).
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Look and feel from the XIAO page of the AiO v4.5 boards (Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32/xPage.h).
   2026-10-26 (Claude): relay cells show the relay number above its function; 16 relays were cramped.
   2026-10-27 (Claude): the divider field is for the pressure signal only (10k / 10k = 0.5).
   2026-10-27 (Claude): the sensors fitted are ticked in Calibration; readings and fields only for those (none: no
   Sprayer card, as on a seed drill).
   2026-10-27 (Claude): sections sized to the machine (up to 64), the module IP's last number. The seed drill has its
   own page now (Seeder/Seeder_ESP32/dPage.h).
   2026-11-02 (Claude): v0.10.0: three tabs. Spraying: the L/ha (actual and target), the pressure and what gives the
   target, the sections and the alarms, big enough to read in the cab. Settings: target rate, nozzles, pressure limits,
   flow tolerance, the section widths from the program, the sensors and their calibration. Module: AgOpenWeb, relays,
   Wi-Fi, program, events, and a "Download the report" button (state, settings and events in a .json file to send when
   something goes wrong), as on the Teensy and XIAO pages.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One self-contained HTML file, no internet needed, made for a phone (and for an AgOpenWeb web card). Dark, or light
// when the phone asks for it. It asks the module for /api/status once a second. The tab is in the address (#settings,
// #module), so a reload or a web card keeps it.
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
nav{display:flex;gap:6px;padding:8px 12px;overflow-x:auto;background:var(--card);border-bottom:1px solid var(--line)}
nav button{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:999px;padding:7px 14px;font:inherit;font-size:14px;cursor:pointer;white-space:nowrap}
nav button.on{background:var(--acc);border-color:var(--acc);color:var(--accInk)}
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
.log{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12.5px;white-space:pre-wrap;overflow-wrap:anywhere;color:var(--mute)}
.msg{font-size:13.5px;margin-top:6px;min-height:1.2em}
[hidden]{display:none!important}
</style>
</head>
<body>
<div id="top">
<header><div class="dot" id="dot"></div><h1>Sprayer module</h1><div class="sub" id="sub">connecting…</div></header>
<nav id="nav"><button data-t="work" class="on">Spraying</button><button data-t="settings">Settings</button><button data-t="module">Module</button></nav>
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

<div class="card"><h2>Wi-Fi</h2>
<dl>
<dt>Own Wi-Fi network</dt><dd id="ap">–</dd>
<dt>Tractor network</dt><dd id="sta">–</dd>
<dt>Module IP</dt><dd id="ip">–</dd>
<dt>Signal</dt><dd id="rssi">–</dd>
<dt>Program subnet</dt><dd id="subnet">–</dd>
</dl>
<p class="note">The program looks for the machine module at &lt;subnet&gt;.123 on the tractor network. The subnet comes from the program (PGN 201), or is set below.</p>
</div>

<div class="card"><h2>Wi-Fi settings</h2>
<div class="field"><span>Own network name</span><input type="text" id="apSsid" maxlength="32"></div>
<div class="field"><span>New own-network password (leave empty to keep it)</span><input type="password" id="apPass" maxlength="63"></div>
<div class="opt"><input type="checkbox" id="staOn"><label for="staOn">Join the tractor network (the router the tablet uses)</label></div>
<div class="field"><span>Tractor network name</span><input type="text" id="staSsid" maxlength="32"></div>
<div class="field"><span>Tractor network password</span><input type="password" id="staPass" maxlength="63"></div>
<div class="field"><span>Module name (http://name.local/)</span><input type="text" id="host" maxlength="24"></div>
<div class="field"><span>Program subnet: the first three numbers of the tablet's IP on the tractor network</span><input type="text" id="subnetIn" maxlength="11" placeholder="192.168.5"></div>
<div class="field"><span>Module IP, last number: 123 = the machine module; another number (124, 125…) for a second module. Not 126, the AiO steer module's</span><input type="number" id="ipLast" min="2" max="254" step="1"></div>
<div class="row"><button class="act" onclick="saveWifi()">Save and restart</button></div>
<div class="msg" id="msgWifi"></div>
</div>

<div class="card"><h2>Program</h2>
<dl><dt>Version</dt><dd id="ver">–</dd><dt>Built</dt><dd id="built">–</dd><dt>Up</dt><dd id="up">–</dd><dt>Free memory</dt><dd id="heap">–</dd><dt>Module name</dt><dd id="name">–</dd></dl>
<div class="row"><label class="act">Install a new program (.bin)<input type="file" id="bin" accept=".bin" style="display:none" onchange="update()"></label><button class="danger" onclick="post('/api/restart',{},'msgProg')">Restart</button></div>
<div class="msg" id="msgProg"></div>
<p class="note">Arduino IDE: Sketch → Export compiled binary, then choose the .bin from the sketch folder. The sections close during the update.</p>
</div>

<div class="card"><h2>Events</h2><div class="log" id="log"></div></div>

<div class="card"><h2>Report for an analysis</h2>
<p class="note">Puts the module's state, its settings, the readings, the link to AgOpenWeb and the events in one file, to send when something goes wrong. Passwords are not included. The file is downloaded by this phone or computer.</p>
<div class="row"><button class="sec" type="button" id="dumpBtn" onclick="report()">Download the report</button><span id="dumpMsg" class="note"></span></div>
</div>

</div>
</section>

</main>
<script>
const $=id=>document.getElementById(id);
// relay functions as the module sends them: 1-64 a section, 101-105 the program's other functions
const FN={0:'–',101:'Lift ↓',102:'Lift ↑',103:'Tram R',104:'Tram L',105:'Geo'};
function fnName(f){return f>=1&&f<=64?'S'+f:(FN[f]||'?')}
// the sprayer alarms (rate.alarms bits), as AgOpenWeb words them
const ALARMS=[[1,'Low pressure'],[2,'High pressure'],[4,'Flow too low'],[8,'Flow too high'],[16,'Flow with the sections off']];
const RATE_TOLERANCE=0.10;   // the rate is green within 10 % of the target, orange outside (as AgOpenWeb's strip)
// sections come as hex digits, section 1 = the lowest bit ("7ffff" = sections 1-19)
function bit(h,i){const k=h.length-1-(i>>2);return k>=0&&(parseInt(h[k],16)>>(i&3)&1)===1}
function pw(){return prompt('Password of the own Wi-Fi network')}
function cells(id,n,from){let h='';for(let i=0;i<n;i++)h+='<span id="'+id+(from+i)+'">'+(from+i+1)+'</span>';return h}
let nSecs=0,nRelays=0;
function ago(s){return s<0?'never':s===0?'now':s+' s ago'}
const f1=v=>v.toFixed(1);
// show what belongs to the sensors fitted (1 pressure, 2 flow): class p = pressure, f = flow
function fitted(pre,s){for(const e of document.querySelectorAll('.'+pre+'p'))e.hidden=!(s&1);for(const e of document.querySelectorAll('.'+pre+'f'))e.hidden=!(s&2)}
function ticked(){return($('snP').checked?1:0)|($('snF').checked?2:0)}
function showFitted(){const s=ticked();fitted('c',s);$('noSensors').hidden=s!==0}
// tabs: the one in the address (#settings, #module), else Spraying
function show(t){
  if(!['work','settings','module'].includes(t))t='work';
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
    $('ver').textContent=d.fw+' v'+d.ver+(d.board?' · '+d.board:'');$('built').textContent=d.built;$('up').textContent=Math.floor(d.up/60)+' min';
    $('heap').textContent=Math.round(d.heap/1024)+' kB';$('name').textContent=d.name;
    $('log').textContent=d.log.join('\n');
    if(!settingsLoaded){settingsLoaded=true;loadSettings()}
  }catch(e){$('sub').textContent='no answer from the module';$('dot').className='dot'}
}
async function loadSettings(){
  const d=await (await fetch('/api/settings',{cache:'no-store'})).json();
  $('apSsid').value=d.apSsid;$('staOn').checked=d.staOn;$('staSsid').value=d.staSsid;$('host').value=d.host;$('subnetIn').value=d.subnet;
  $('ipLast').value=d.ipLast||123;
  $('staPass').placeholder=d.staPassSet?'(saved, leave empty to keep it)':'';
  $('ppl').value=d.ppl;$('pmax').value=d.pmax;$('pdiv').value=d.pdiv;
  const sn=d.sensors===undefined?3:d.sensors;$('snP').checked=!!(sn&1);$('snF').checked=!!(sn&2);showFitted();
  for(const k of ['rate','nzSp','nzFl','nzBar','pLow','pHigh','fTol'])if(d[k]!==undefined)$(k).value=d[k];
}
async function post(url,fields,msgId){
  const p=pw();if(p===null)return;
  const body=new URLSearchParams(Object.assign({pw:p},fields));
  const m=$(msgId);m.textContent='…';
  try{const r=await fetch(url,{method:'POST',body});const d=await r.json();
    // saved: the fields show what the module kept, before "done" (a restart: once it is back)
    if(d.ok&&d.restart)settingsLoaded=false;
    else if(d.ok)try{await loadSettings()}catch(e){}
    m.innerHTML=d.ok?'<span class="ok">done'+(d.restart?', restarting: reconnect in 10 s':'')+'</span>':'<span class="bad">'+d.error+'</span>';
  }catch(e){m.innerHTML='<span class="bad">no answer</span>'}
}
function saveWifi(){post('/api/wifi',{apSsid:$('apSsid').value,apPass:$('apPass').value,staOn:$('staOn').checked?'1':'0',staSsid:$('staSsid').value,staPass:$('staPass').value,host:$('host').value,subnet:$('subnetIn').value.trim(),ipLast:$('ipLast').value},'msgWifi')}
function saveCalib(){post('/api/calib',{sensors:ticked(),ppl:$('ppl').value,pmax:$('pmax').value,pdiv:$('pdiv').value},'msgCalib')}
function saveSpray(){const f={};for(const k of ['rate','nzSp','nzFl','nzBar','pLow','pHigh','fTol'])f[k]=$(k).value.trim();post('/api/spray',f,'msgSpray')}
// the report for an analysis: every answer of the module in one .json file, downloaded by this browser
async function report(){
  const b=$('dumpBtn'),m=$('dumpMsg');b.disabled=true;m.className='note';m.textContent='Gathering…';
  const r={made:new Date().toString(),page:location.href,browser:navigator.userAgent};
  for(const [k,u] of [['status','/api/status'],['settings','/api/settings'],['aow','/aow/status']]){
    try{r[k]=await (await fetch(u,{cache:'no-store'})).json()}catch(e){r[k]={error:String(e)}}}
  const d=new Date(),z=n=>String(n).padStart(2,'0'),fw=(r.status&&r.status.fw)||'Sprayer_ESP32';
  const name=fw+'_report_'+d.getFullYear()+'-'+z(d.getMonth()+1)+'-'+z(d.getDate())+'_'+z(d.getHours())+z(d.getMinutes())+'.json';
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(r,null,1)],{type:'application/json'}));a.download=name;
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),10000);
  m.className='note ok';m.textContent='Downloaded: '+name;b.disabled=false;
}
async function update(){
  const f=$('bin').files[0];if(!f)return;const p=pw();if(p===null)return;
  const m=$('msgProg');m.textContent='sending '+f.name+'…';
  const fd=new FormData();fd.append('bin',f);
  try{const r=await fetch('/api/update?pw='+encodeURIComponent(p),{method:'POST',body:fd});const d=await r.json();
    m.innerHTML=d.ok?'<span class="ok">installed, restarting: reload this page in 15 s</span>':'<span class="bad">'+d.error+'</span>';
  }catch(e){m.innerHTML='<span class="bad">no answer</span>'}
}
refresh();setInterval(refresh,1000);
</script>
</body>
</html>
)SPRAYERPAGE";
