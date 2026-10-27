/* sPage.h - the sprayer module's web page (English).
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Look and feel from the XIAO page of the AiO v4.5 boards (Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32/xPage.h).
   2026-10-26 (Claude): relay cells show the relay number above its function; 16 relays were cramped.
   2026-10-27 (Claude): the divider field is for the pressure signal only (10k / 10k = 0.5).
   2026-10-27 (Claude): the sensors fitted are ticked in Calibration; readings and fields only for those (none: no
   Sprayer card, as on a seed drill).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One self-contained HTML file, no internet needed, made for a phone. Dark, or light when the phone
// asks for it. It asks the module for /api/status once a second.
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
header{display:flex;align-items:center;gap:10px;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line);position:sticky;top:0;z-index:3}
header h1{font-size:16px;margin:0;font-weight:650;white-space:nowrap}
header .sub{color:var(--mute);font-size:13px;flex:1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.dot{width:11px;height:11px;border-radius:50%;background:var(--bad);flex:none}
.dot.on{background:var(--ok)}
main{padding:14px 16px 40px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;min-width:0}
.card h2{font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--mute);margin:0 0 8px;font-weight:650}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:0}
dt{color:var(--mute)}
dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.big{font-size:26px;font-weight:650;font-variant-numeric:tabular-nums}
.big small{font-size:14px;color:var(--mute);font-weight:400;margin-left:4px}
.row3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:8px}
.pill{display:inline-block;padding:1px 9px;border-radius:999px;font-size:13px;font-weight:600;background:var(--soft)}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.secs{display:grid;grid-template-columns:repeat(8,1fr);gap:4px;margin-top:6px}
.secs span{height:26px;border-radius:5px;background:var(--soft);font-size:12px;display:flex;align-items:center;justify-content:center;color:var(--mute)}
.secs span.req{outline:2px solid var(--warn);outline-offset:-2px}
.secs span.on{background:var(--ok);color:var(--accInk);font-weight:700}
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
input[type=text],input[type=password],input[type=number]{font:inherit;padding:9px 10px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);width:100%}
.opt{display:flex;gap:10px;align-items:center;padding:5px 0}
.log{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12.5px;white-space:pre-wrap;overflow-wrap:anywhere;color:var(--mute)}
.msg{font-size:13.5px;margin-top:6px;min-height:1.2em}
[hidden]{display:none!important}
</style>
</head>
<body>
<header><div class="dot" id="dot"></div><h1>Sprayer module</h1><div class="sub" id="sub">connecting…</div></header>
<main>
<div class="grid">

<div class="card" id="sprCard"><h2>Sprayer</h2>
<div class="row3">
<div class="rp"><div class="big" id="bar">–<small>bar</small></div><div class="note">pressure</div></div>
<div class="rf"><div class="big" id="lpm">–<small>L/min</small></div><div class="note">flow</div></div>
<div class="rf"><div class="big" id="total">–<small>L</small></div><div class="note">applied</div></div>
</div>
<dl><dt>Master switch</dt><dd id="master">–</dd><dt class="rp">Sensor voltage</dt><dd class="rp" id="volts">–</dd><dt class="rf">Flow pulses</dt><dd class="rf" id="pulses">–</dd></dl>
<div class="row rf"><button class="sec" onclick="post('/api/total',{},'msgTotal')">Total back to 0</button></div>
<div class="msg" id="msgTotal"></div>
</div>

<div class="card"><h2>Sections</h2>
<div class="secs" id="secs"></div>
<div class="secs" id="secs2"></div>
<p class="note">Green = open. Orange outline = asked for by the program but not open (master switch off, or program silent).</p>
<h2 style="margin-top:12px">Relays</h2>
<div class="secs rl" id="relays"></div>
<p class="note" id="relayNote">What each relay does comes from the program (Tool → Machine → Pin config).</p>
</div>

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

<div class="card"><h2>Wi-Fi</h2>
<dl>
<dt>Own network</dt><dd id="ap">–</dd>
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
<div class="field"><span>Program subnet: the first three numbers of the tablet's IP on the tractor network (the module becomes .123)</span><input type="text" id="subnetIn" maxlength="11" placeholder="192.168.5"></div>
<div class="row"><button class="act" onclick="saveWifi()">Save and restart</button></div>
<div class="msg" id="msgWifi"></div>
</div>

<div class="card"><h2>Calibration</h2>
<div class="opt"><input type="checkbox" id="snP" onchange="showFitted()"><label for="snP">Pressure sensor fitted (G34)</label></div>
<div class="opt"><input type="checkbox" id="snF" onchange="showFitted()"><label for="snF">Flow meter fitted (G35)</label></div>
<p class="note" id="noSensors" hidden>No sensors (a seed drill): the module sends no sprayer readings (PGN 160), so AgOpenWeb and the AiO module page show none.</p>
<div class="field cf"><span>Flow meter pulses per litre (YF-S201: about 450)</span><input type="number" id="ppl" step="0.1"></div>
<div class="field cp"><span>Pressure sensor full scale (bar, at 4.5 V)</span><input type="number" id="pmax" step="0.1"></div>
<div class="field cp"><span>Divider on the pressure signal (10k / (10k + 10k) = 0.5)</span><input type="number" id="pdiv" step="0.001"></div>
<div class="row"><button class="act" onclick="saveCalib()">Save</button></div>
<div class="msg" id="msgCalib"></div>
<p class="note cf">Bucket test: with one nozzle open, fill 10 L and note the litres counted. New pulses per litre = old × (counted ÷ 10).</p>
</div>

<div class="card"><h2>Program</h2>
<dl><dt>Version</dt><dd id="ver">–</dd><dt>Built</dt><dd id="built">–</dd><dt>Up</dt><dd id="up">–</dd><dt>Free memory</dt><dd id="heap">–</dd><dt>Module name</dt><dd id="name">–</dd></dl>
<div class="row"><label class="act">Install a new program (.bin)<input type="file" id="bin" accept=".bin" style="display:none" onchange="update()"></label><button class="danger" onclick="post('/api/restart',{},'msgProg')">Restart</button></div>
<div class="msg" id="msgProg"></div>
<p class="note">Arduino IDE: Sketch → Export compiled binary, then choose the .bin from the sketch folder. The sections close during the update.</p>
</div>

<div class="card"><h2>Events</h2><div class="log" id="log"></div></div>

</div>
</main>
<script>
const $=id=>document.getElementById(id);
const FN={0:'–',17:'Lift ↓',18:'Lift ↑',19:'Tram R',20:'Tram L',21:'Geo'};
function fnName(f){return f>=1&&f<=16?'S'+f:(FN[f]||'?')}
function pw(){return prompt('Password of the own Wi-Fi network')}
function cells(id,n,from){let h='';for(let i=0;i<n;i++)h+='<span id="'+id+(from+i)+'">'+(from+i+1)+'</span>';return h}
$('secs').innerHTML=cells('s',8,0);$('secs2').innerHTML=cells('s',8,8);let nRelays=0;
function ago(s){return s<0?'never':s===0?'now':s+' s ago'}
// show what belongs to the sensors fitted (1 pressure, 2 flow): class p = pressure, f = flow
function fitted(pre,s){for(const e of document.querySelectorAll('.'+pre+'p'))e.hidden=!(s&1);for(const e of document.querySelectorAll('.'+pre+'f'))e.hidden=!(s&2)}
function ticked(){return($('snP').checked?1:0)|($('snF').checked?2:0)}
function showFitted(){const s=ticked();fitted('c',s);$('noSensors').hidden=s!==0}
let settingsLoaded=false;
async function refresh(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});const d=await r.json();
    $('dot').className='dot'+(d.aog.link?' on':'');
    $('sub').textContent=(d.wifi.up?'on '+d.wifi.sta+' as '+d.wifi.ip:'not on the tractor network')+(d.aog.link?' · program talking':' · program silent');
    const sn=d.cal.sensors===undefined?3:d.cal.sensors;$('sprCard').hidden=!sn;fitted('r',sn);
    $('bar').innerHTML=d.spr.bar.toFixed(1)+'<small>bar</small>';
    $('lpm').innerHTML=d.spr.lpm.toFixed(1)+'<small>L/min</small>';
    $('total').innerHTML=d.spr.total.toFixed(1)+'<small>L</small>';
    $('master').innerHTML=d.spr.master?'<span class="ok">on</span>':'<span class="bad">off</span>';
    $('volts').textContent=d.spr.volts.toFixed(2)+' V';
    $('pulses').textContent=d.spr.pulses;
    for(let i=0;i<16;i++){const e=$('s'+i);e.className=(d.out.on>>i&1?'on':'')+(d.aog.req>>i&1&&!(d.out.on>>i&1)?' req':'')}
    if(nRelays!=d.out.fn.length){nRelays=d.out.fn.length;$('relays').innerHTML=cells('r',nRelays,0)}
    let nSec=0;for(let i=0;i<nRelays;i++){const e=$('r'+i),f=d.out.fn[i];if(f>=1&&f<=16)nSec++;e.innerHTML='<small>'+(i+1)+'</small>'+fnName(f);e.className=d.out.bits>>i&1?'on':''}
    $('relayNote').textContent=nRelays+' relays, '+nSec+' of them sections: what each relay does comes from the program (Tool → Machine → Pin config).';
    $('link').innerHTML=d.aog.link?'<span class="ok">talking</span>':'<span class="bad">silent, sections closed</span>';
    $('ago239').textContent=ago(d.aog.ago239);$('agoHello').textContent=ago(d.aog.agoHello);
    $('kmh').textContent=d.aog.kmh.toFixed(1)+' km/h';
    $('hyd').textContent=(d.aog.hydEn?'enabled, ':'disabled, ')+(d.aog.hyd==1?'lowering':d.aog.hyd==2?'raising':'idle')+' (raise '+d.aog.raise+' s, lower '+d.aog.lower+' s)';
    $('tram').textContent=(d.aog.tram&1?'right ':'')+(d.aog.tram&2?'left':'')||'off';
    $('counts').textContent=d.aog.n239+' sections, '+d.aog.nHello+' hellos, '+d.aog.crcErr+' bad checksums';
    $('invert').innerHTML=(d.aog.invert?'<span class="bad">inverted ("relay active high" ticked in the program)</span>':'<span class="ok">normal</span>')+', relay on = pin '+((d.out.onHigh!==false)!=!!d.aog.invert?'high':'low');
    $('ap').textContent=d.wifi.ap+' · '+d.wifi.apCl+' device(s)';
    $('sta').innerHTML=!d.wifi.staOn?'<span class="warn">not set</span>':d.wifi.up?'<span class="ok">'+d.wifi.sta+'</span>':'<span class="bad">'+d.wifi.sta+' (not joined)</span>';
    $('ip').textContent=d.wifi.up?d.wifi.ip:'–';$('rssi').textContent=d.wifi.up?d.wifi.rssi+' dBm':'–';
    $('subnet').textContent=d.wifi.subnet+' → module '+d.wifi.subnet+'.123';
    $('ver').textContent=d.fw+' v'+d.ver;$('built').textContent=d.built;$('up').textContent=Math.floor(d.up/60)+' min';
    $('heap').textContent=Math.round(d.heap/1024)+' kB';$('name').textContent=d.name;
    $('log').textContent=d.log.join('\n');
    if(!settingsLoaded){settingsLoaded=true;loadSettings()}
  }catch(e){$('sub').textContent='no answer from the module';$('dot').className='dot'}
}
async function loadSettings(){
  const d=await (await fetch('/api/settings',{cache:'no-store'})).json();
  $('apSsid').value=d.apSsid;$('staOn').checked=d.staOn;$('staSsid').value=d.staSsid;$('host').value=d.host;$('subnetIn').value=d.subnet;
  $('staPass').placeholder=d.staPassSet?'(saved, leave empty to keep it)':'';
  $('ppl').value=d.ppl;$('pmax').value=d.pmax;$('pdiv').value=d.pdiv;
  const sn=d.sensors===undefined?3:d.sensors;$('snP').checked=!!(sn&1);$('snF').checked=!!(sn&2);showFitted();
}
async function post(url,fields,msgId){
  const p=pw();if(p===null)return;
  const body=new URLSearchParams(Object.assign({pw:p},fields));
  const m=$(msgId);m.textContent='…';
  try{const r=await fetch(url,{method:'POST',body});const d=await r.json();
    m.innerHTML=d.ok?'<span class="ok">done'+(d.restart?', restarting: reconnect in 10 s':'')+'</span>':'<span class="bad">'+d.error+'</span>';
    if(d.ok)settingsLoaded=false;
  }catch(e){m.innerHTML='<span class="bad">no answer</span>'}
}
function saveWifi(){post('/api/wifi',{apSsid:$('apSsid').value,apPass:$('apPass').value,staOn:$('staOn').checked?'1':'0',staSsid:$('staSsid').value,staPass:$('staPass').value,host:$('host').value,subnet:$('subnetIn').value.trim()},'msgWifi')}
function saveCalib(){post('/api/calib',{sensors:ticked(),ppl:$('ppl').value,pmax:$('pmax').value,pdiv:$('pdiv').value},'msgCalib')}
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
