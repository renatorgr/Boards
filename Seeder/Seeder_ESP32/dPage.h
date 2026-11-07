/* dPage.h - the seed drill module's web page (English); d for drill.
   Part of the seed drill section controller (folder Seeder of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr. Made from the page of Sprayer_ESP32 v0.8 (sPage.h).
   Look and feel from the XIAO page of the AiO v4.5 boards (Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32/xPage.h).
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// One self-contained HTML file, no internet needed, made for a phone (and for an AgOpenWeb web card). Dark, or light
// when the phone asks for it. It asks the module for /api/status once a second. The tab is in the address (#settings,
// #module, #firmware), so a reload or a web card keeps it. Only the update check needs internet: the browser asks
// GitHub, never the module.
#pragma once

const char SEEDER_PAGE[] PROGMEM = R"SEEDERPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark light">
<title>Seeder module</title>
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
nav button.new::after{content:"";display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--ok);margin-left:6px;vertical-align:middle}
nav button.on.new::after{background:var(--accInk)}
main{padding:14px 16px 40px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;min-width:0}
.card h2{font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--mute);margin:0 0 8px;font-weight:650}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:0}
dt{color:var(--mute)}
dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.silent{border:2px solid var(--bad);color:var(--bad);border-radius:12px;padding:8px 14px;font-weight:650;margin-bottom:12px}
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
progress{width:100%;height:14px;margin-top:10px;accent-color:var(--acc)}
a{color:var(--acc)}
[hidden]{display:none!important}
</style>
</head>
<body>
<div id="top">
<header><div class="dot" id="dot"></div><h1>Seeder module</h1><div class="sub" id="sub">connecting…</div></header>
<nav id="nav"><button data-t="work" class="on">Sowing</button><button data-t="settings">Settings</button><button data-t="module">Module</button><button data-t="firmware">Firmware</button></nav>
</div>
<main>

<section id="t-work">
<div class="silent" id="silent" hidden>No messages from AgOpenWeb: every section is closed.</div>
<div class="card"><h2>Sections</h2>
<div class="secs work" id="secs"></div>
<p class="note">Green = open. Orange outline = asked for by the program but not open (program silent).</p>
<p class="note" id="detail">–</p>
</div>
</section>

<section id="t-settings" hidden>
<div class="grid">

<div class="card"><h2>Relays</h2>
<div class="secs rl" id="relays"></div>
<p class="note" id="relayNote">What each relay does comes from the program (Tool → Machine → Pin config).</p>
<div class="field"><span>Relay 1 follows section (the next relays the next sections). 0 = what the program's pin config says (sections 1-16 only)</span><input type="number" id="first" min="0" max="64" step="1"></div>
<div class="row"><button class="act" onclick="saveRelays()">Save</button></div>
<div class="msg" id="msgRelays"></div>
<p class="note">A seed drill with more than 16 sections needs the relays in a row from section 1: AgOpenWeb's pin config only knows sections 1-16, and sends sections 17-64 in PGN 229.</p>
</div>

</div>
</section>

<section id="t-module" hidden>
<div class="grid">

<div class="card"><h2>AgOpenWeb</h2>
<dl>
<dt>Program</dt><dd id="link">–</dd>
<dt>Last section message</dt><dd id="ago239">–</dd>
<dt>Sections 17-64 (PGN 229)</dt><dd id="ago229">–</dd>
<dt>Last hello</dt><dd id="agoHello">–</dd>
<dt>Speed</dt><dd id="kmh">–</dd>
<dt>Hydraulic lift</dt><dd id="hyd">–</dd>
<dt>Tramline</dt><dd id="tram">–</dd>
<dt>Messages</dt><dd id="counts">–</dd>
<dt>Output polarity</dt><dd id="invert">–</dd>
</dl>
</div>

<div class="card"><h2>Network</h2>
<dl>
<dt>Own Wi-Fi network</dt><dd id="ap">–</dd>
<dt>Network cable</dt><dd id="eth">–</dd>
<dt>Tractor Wi-Fi</dt><dd id="sta">–</dd>
<dt>Module IP</dt><dd id="ip">–</dd>
<dt>Signal</dt><dd id="rssi">–</dd>
<dt>Program subnet</dt><dd id="subnet">–</dd>
</dl>
<p class="note">The program looks for the machine module at &lt;subnet&gt;.123 on the tractor network. The subnet comes from the program (PGN 201), or is set below.</p>
</div>

<div class="card"><h2>Own Wi-Fi network</h2>
<div class="field"><span>Network name</span><input type="text" id="apSsid" maxlength="32"></div>
<div class="field"><span>New password (only to change it: 8 to 63 characters; blank keeps the current one)</span><input type="password" id="apPass" maxlength="63"></div>
<p class="note">Always on, at 192.168.4.1. Its password is also the one for changing the settings and the firmware.</p>
</div>

<div class="card"><h2>Tractor Wi-Fi network (optional)</h2>
<label class="opt"><input type="checkbox" id="staOn"><span>Use the tractor's Wi-Fi network instead of the network cable<small>The router the tablet uses. The page is then also at http://<b id="hostShow">seeder</b>.local/ or at the module's IP on that network.</small></span></label>
<div class="field"><span>Network name</span><input type="text" id="staSsid" maxlength="32"></div>
<div class="row" style="margin:2px 0"><button class="sec" type="button" id="scan" onclick="scanNets()">Search for networks</button><span id="scanMsg" class="note"></span></div>
<div id="netList" class="nets"></div>
<div class="field"><span>That network's password (blank: keeps the current one; for a new network, blank = open network)</span><input type="password" id="staPass" maxlength="63"></div>
<div class="field"><span>Module name on the tractor network</span><input type="text" id="host" maxlength="24" oninput="$('hostShow').textContent=this.value||'seeder'"></div>
</div>

<div class="card"><h2>Program network</h2>
<div class="field"><span>Program subnet: the first three numbers of the tablet's IP on the tractor network</span><input type="text" id="subnetIn" maxlength="11" placeholder="192.168.5" inputmode="decimal"></div>
<div class="field"><span>Module IP, last number: 123 = the machine module; another number (124, 125…) for a second module. Not 126, the AiO steer module's</span><input type="number" id="ipLast" min="2" max="254" step="1"></div>
</div>

<div class="card"><h2>Save</h2>
<div class="field"><span>Current password of the own Wi-Fi network (to confirm)</span><input type="password" id="pwW" maxlength="63"></div>
<div class="row"><button class="act" type="button" onclick="saveWifi()">Save and restart the module</button></div>
<div class="msg" id="msgWifi"></div>
<p class="note">The module restarts to apply them: every section closes for a few seconds. If you change the name or the password of the own network, connect the phone to the new network again. Forgot the password? In the serial monitor (USB): <span class="mono">wifi-reset</span> (or the BOOT button held 5 s, see the README).</p>
</div>

<div class="card"><h2>Events</h2><div class="log" id="log"></div></div>

</div>
</section>

<section id="t-firmware" hidden>
<div class="grid">

<div class="card"><h2>Program installed</h2>
<dl><dt>Program</dt><dd id="ver">–</dd><dt>Built</dt><dd id="built">–</dd><dt>Board</dt><dd id="board">–</dd><dt>Up</dt><dd id="up">–</dd><dt>Free memory</dt><dd id="heap">–</dd><dt>Name on the network</dt><dd id="name">–</dd></dl>
</div>

<div class="card"><h2>Version on GitHub</h2>
<dl><dt>On GitHub</dt><dd id="ghVer">–</dd></dl>
<div class="row"><button class="act" id="ghInstall" onclick="ghInstall()" disabled>Install from GitHub</button><button class="sec" id="ghCheck" onclick="ghCheck()">Check now</button><a id="ghLink" href="https://github.com/renatorgr/Boards/tree/main/Seeder/Seeder_ESP32" target="_blank" rel="noopener">See on GitHub</a></div>
<p class="note">This phone or tablet asks GitHub, so it needs internet: on the module's own Wi-Fi network alone it usually has none. Open this page over the tractor network (the router with internet). The module's own-network password confirms the installation.</p>
</div>

<div class="card"><h2>Update or restart</h2>
<div class="row" style="margin-top:0"><label class="act">Choose .bin<input type="file" id="bin" accept=".bin" style="display:none" onchange="binChosen()"></label><span id="binName" class="note">no file</span></div>
<div class="row"><button class="act" id="install" onclick="installFile()" disabled>Install</button><button class="danger" onclick="post('/api/restart',{},'msgProg')">Restart</button></div>
<progress id="prog" max="100" value="0" hidden></progress>
<div class="msg" id="msgProg"></div>
<p class="note">Arduino IDE: Sketch → Export compiled binary, then choose Seeder_ESP32.ino.bin in the sketch folder (not the .merged.bin). Every section closes during the update. If it fails, the current program keeps running.</p>
</div>

<div class="card"><h2>Report for an analysis</h2>
<p class="note">Puts the module's state, its settings, the link to AgOpenWeb and the events in one file, to send when something goes wrong. Passwords are not included. The file is downloaded by this phone or computer.</p>
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
// sections come as hex digits, section 1 = the lowest bit ("7ffff" = sections 1-19)
function bit(h,i){const k=h.length-1-(i>>2);return k>=0&&(parseInt(h[k],16)>>(i&3)&1)===1}
function pw(){return prompt('Password of the own Wi-Fi network')}
function cells(id,n,from){let h='';for(let i=0;i<n;i++)h+='<span id="'+id+(from+i)+'">'+(from+i+1)+'</span>';return h}
let nSecs=0,nRelays=0;
function ago(s){return s<0?'never':s===0?'now':s+' s ago'}
const f1=v=>v.toFixed(1);
// tabs: the one in the address (#settings, #module, #firmware), else Sowing
function show(t){
  if(!['work','settings','module','firmware'].includes(t))t='work';
  for(const b of document.querySelectorAll('nav button'))b.classList.toggle('on',b.dataset.t===t);
  for(const s of document.querySelectorAll('main>section'))s.hidden=s.id!=='t-'+t;
  history.replaceState(null,'',t==='work'?location.pathname:'#'+t);
}
$('nav').addEventListener('click',e=>{if(e.target.dataset.t)show(e.target.dataset.t)});
show(location.hash.slice(1));
let settingsLoaded=false;
async function refresh(){
  if(busy&&!waitBoot)return;   // sending a program: the module has no time for anything else
  try{
    const r=await fetch('/api/status',{cache:'no-store'});const d=await r.json();
    $('dot').className='dot'+(d.aog.link?' on':'');
    $('sub').textContent=(d.wifi.up?'on '+(d.wifi.staUp?d.wifi.sta:'the cable')+' as '+d.wifi.ip:'not on the tractor network')+(d.aog.link?' · program talking':' · program silent');
    // Sowing tab: as many sections as the program has asked for (secTop), at least those the relays follow (up to 16)
    $('silent').hidden=d.aog.link;
    let mapped=0,nSec=0;for(const f of d.out.fn)if(f>=1&&f<=64){nSec++;mapped=Math.max(mapped,f)}
    const n=Math.max(8,d.aog.secTop||0,Math.min(16,mapped));
    if(nSecs!==n){nSecs=n;$('secs').innerHTML=cells('s',n,0);$('secs').style.gridTemplateColumns='repeat('+Math.min(n,8)+',1fr)'}
    let open=0;
    for(let i=0;i<nSecs;i++){const e=$('s'+i),on=bit(d.out.on,i);if(on)open++;e.className=(on?'on':'')+(bit(d.aog.req,i)&&!on?' req':'')}
    const lift=d.aog.hyd==1?'lowering':d.aog.hyd==2?'raising':'idle',tram=((d.aog.tram&1?'right ':'')+(d.aog.tram&2?'left':'')).trim();
    $('detail').textContent=open+' of '+nSecs+' open · '+f1(d.aog.kmh)+' km/h · lift '+lift+(tram?' · tramline '+tram:'');
    // Settings tab: the relays
    if(nRelays!=d.out.fn.length){nRelays=d.out.fn.length;$('relays').innerHTML=cells('r',nRelays,0)}
    for(let i=0;i<nRelays;i++){const e=$('r'+i),f=d.out.fn[i];e.innerHTML='<small>'+(i+1)+'</small>'+fnName(f);e.className=d.out.bits>>>i&1?'on':''}
    $('relayNote').textContent=nRelays+' relays, '+nSec+' of them sections: '+(d.out.first?'relay 1 follows section '+d.out.first+', the next relays the next sections (set below).':'what each relay does comes from the program (Tool → Machine → Pin config).');
    // Module tab
    $('link').innerHTML=d.aog.link?'<span class="ok">talking</span>':'<span class="bad">silent, sections closed</span>';
    $('ago239').textContent=ago(d.aog.ago239);$('agoHello').textContent=ago(d.aog.agoHello);
    $('ago229').textContent=d.aog.ago229<0?'never (the tool has 16 sections or fewer)':ago(d.aog.ago229);
    $('kmh').textContent=f1(d.aog.kmh)+' km/h';
    $('hyd').textContent=(d.aog.hydEn?'enabled, ':'disabled, ')+lift+' (raise '+d.aog.raise+' s, lower '+d.aog.lower+' s)';
    $('tram').textContent=tram||'off';
    $('counts').textContent=d.aog.n239+' sections'+(d.aog.n229?' ('+d.aog.n229+' with 17-64)':'')+', '+d.aog.nHello+' hellos, '+d.aog.crcErr+' bad checksums';
    $('invert').innerHTML=(d.aog.invert?'<span class="bad">inverted ("relay active high" ticked in the program)</span>':'<span class="ok">normal</span>')+', relay on = pin '+((d.out.onHigh!==false)!=!!d.aog.invert?'high':'low');
    $('ap').textContent=d.wifi.ap+' · '+d.wifi.apCl+' device(s)';
    $('eth').innerHTML=d.wifi.staOn?'<span class="warn">not used (tractor Wi-Fi instead)</span>':d.wifi.ethUp?'<span class="ok">connected</span>':'<span class="bad">no link</span>';
    $('sta').innerHTML=!d.wifi.staOn?'not used (cable)':d.wifi.staUp?'<span class="ok">'+d.wifi.sta+'</span>':'<span class="bad">'+d.wifi.sta+' (not joined)</span>';
    $('ip').textContent=d.wifi.up?d.wifi.ip:'–';$('rssi').textContent=d.wifi.staUp?d.wifi.rssi+' dBm':'–';
    $('subnet').textContent=d.wifi.subnet+' → module '+d.wifi.subnet+'.'+(d.wifi.ipLast||123);
    $('log').textContent=d.log.join('\n');
    // Firmware tab
    $('ver').textContent=d.fw+' v'+d.ver;$('built').textContent=builtText(d.built)||d.built;$('board').textContent=d.board||'–';
    $('up').textContent=Math.floor(d.up/60)+' min';$('heap').textContent=Math.round(d.heap/1024)+' kB';$('name').textContent=d.name;
    running(d);
    if(!settingsLoaded){settingsLoaded=true;loadSettings()}
  }catch(e){$('sub').textContent='no answer from the module';$('dot').className='dot';if(waitBoot)progMsg('The module is restarting…','warn')}
}
async function loadSettings(){
  const d=await (await fetch('/api/settings',{cache:'no-store'})).json();
  $('apSsid').value=d.apSsid;$('staOn').checked=d.staOn;$('staSsid').value=d.staSsid;$('host').value=d.host;$('subnetIn').value=d.subnet;
  $('hostShow').textContent=d.host;$('ipLast').value=d.ipLast||123;$('first').value=d.first||0;
  $('apPass').value='';$('staPass').value='';$('staPass').placeholder=d.staPassSet?'(saved)':'';
}
async function post(url,fields,msgId,pass){
  const p=pass!==undefined?pass:pw();if(p===null)return;
  const body=new URLSearchParams(Object.assign({pw:p},fields));
  const m=$(msgId);m.textContent='…';
  try{const r=await fetch(url,{method:'POST',body});const d=await r.json();
    // saved: the fields show what the module kept, before "done" (a restart: once it is back)
    if(d.ok&&d.restart)settingsLoaded=false;
    else if(d.ok)try{await loadSettings()}catch(e){}
    m.innerHTML=d.ok?'<span class="ok">done'+(d.restart?', restarting: reconnect in 10 s':'')+'</span>':'<span class="bad">'+d.error+'</span>';
  }catch(e){m.innerHTML='<span class="bad">no answer</span>'}
}
// Wi-Fi settings, as on the XIAO page: the password typed in the Save card, a question before the restart
function saveWifi(){const m=$('msgWifi');
  if($('apPass').value&&$('apPass').value.length<8){m.innerHTML='<span class="bad">The password must have 8 characters or more.</span>';return}
  if(!$('pwW').value){m.innerHTML='<span class="bad">Type the password of the own Wi-Fi network.</span>';$('pwW').focus();return}
  if(!confirm('Save and restart the module? Its Wi-Fi goes off for a few seconds and every section closes.'))return;
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
function saveRelays(){post('/api/relays',{first:$('first').value},'msgRelays')}
// the report for an analysis: every answer of the module in one .json file, downloaded by this browser
async function report(){
  const b=$('dumpBtn'),m=$('dumpMsg');b.disabled=true;m.className='note';m.textContent='Gathering…';
  const r={made:new Date().toString(),page:location.href,browser:navigator.userAgent};
  for(const [k,u] of [['status','/api/status'],['settings','/api/settings']]){
    try{r[k]=await (await fetch(u,{cache:'no-store'})).json()}catch(e){r[k]={error:String(e)}}}
  const d=new Date(),z=n=>String(n).padStart(2,'0'),fw=(r.status&&r.status.fw)||'Seeder_ESP32';
  const name=fw+'_report_'+d.getFullYear()+'-'+z(d.getMonth()+1)+'-'+z(d.getDate())+'_'+z(d.getHours())+z(d.getMinutes())+'.json';
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(r,null,1)],{type:'application/json'}));a.download=name;
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),10000);
  m.className='note ok';m.textContent='Downloaded: '+name;b.disabled=false;
}
// ---- a new program: the one on GitHub, or a chosen .bin; both are checked here before they are sent
// What a .bin is: an ESP32 program (first byte 0xE9, chip 0 = the plain ESP32 of both boards) and, from the text every
// build carries (the start of its /api/status answer, zNet.ino), which program, version, board and build.
async function binInfo(blob,name){
  if(/merged|bootloader|partitions/i.test(name))return {error:'This is the wrong file: use Seeder_ESP32.ino.bin, not the '+name+'.'};
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
let busy=false,waitBoot=false,upBefore=0,fwNow=null;
// what the module runs; after an installation, the restart is seen when it is up again for less time than before
function running(d){
  const k=d.fw+' '+d.ver+' '+d.built+' '+d.board,was=fwNow&&fwNow.key;
  fwNow={fw:d.fw,ver:d.ver,built:d.built,board:d.board,key:k};
  if(waitBoot&&d.up<upBefore){waitBoot=false;busy=false;$('prog').hidden=true;
    progMsg('Back: it runs '+infoText(fwNow)+'.','ok');$('install').disabled=!$('bin').files[0]}
  upBefore=Math.max(upBefore,d.up);
  if(k!==was)renderGh();
  if(!busy&&(ghAt===0||Date.now()-ghAt>6*3600e3))ghCheck();   // GitHub, not waited for
}
function binChosen(){
  const f=$('bin').files[0];$('binName').textContent=f?f.name+' ('+Math.round(f.size/1024)+' kB)':'no file';
  $('install').disabled=!f||busy;progMsg('');
}
async function installFile(){
  const f=$('bin').files[0];if(!f||busy)return;
  if(!fwNow){progMsg('No answer from the module yet.','bad');return}
  const i=await binInfo(f,f.name);
  if(i.error){progMsg(i.error,'bad');return}
  if(!confirm('Install '+infoText(i)+' on this module? It now runs '+infoText(fwNow)+'. Every section closes.'))return;
  const p=pw();if(p===null)return;
  busy=true;$('install').disabled=true;renderGh();const g=$('prog');g.hidden=false;g.value=0;
  sendBin(f,f.name,p,0);
}
// the .bin to the module, progress from p0 to 95 %
function sendBin(blob,name,pass,p0){
  const g=$('prog');progMsg('Sending '+name+'…');
  const fd=new FormData();fd.append('bin',blob,name);
  const x=new XMLHttpRequest();x.open('POST','/api/update?pw='+encodeURIComponent(pass));
  x.upload.onprogress=e=>{if(e.lengthComputable)g.value=p0+Math.round(e.loaded/e.total*(95-p0))};
  const fail=t=>{busy=false;g.hidden=true;progMsg(t,'bad');$('install').disabled=!$('bin').files[0];renderGh()};
  x.onload=()=>{let j={};try{j=JSON.parse(x.responseText)}catch(e){}
    if(x.status===200&&j.ok){g.value=100;waitBoot=true;progMsg('Installed. The module is restarting…','warn')}
    else fail('Not installed: '+(j.error||('error '+x.status))+'. The current program keeps running.')};
  x.onerror=()=>fail('The connection failed. The current program keeps running.');
  x.send(fd);
}

// ---- a newer program on GitHub (renatorgr/Boards, Seeder/Seeder_ESP32/Seeder_ESP32.ino.bin). firmware.json, written
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
    const j=await r.json();gh=j.seeder&&j.seeder.file?j.seeder:null;ghState=gh?'ok':'none'}
  catch(e){ghState='fail'}
  ghAt=Date.now();renderGh();
}
function ghBoardOk(){return !gh||!gh.board||!fwNow||!fwNow.board||gh.board===fwNow.board}
function ghCmp(){return gh&&fwNow?(gh.version?verCmp(gh.version,fwNow.ver):0)||builtCmp(gh.built,fwNow.built):NaN}   // >0: GitHub's is newer
function ghNewer(){return ghCmp()>0&&ghBoardOk()}
function renderGh(){
  const c=ghCmp(),nw=ghNewer(),e=$('ghVer');let t='–',cls='';
  if(ghState==='busy')t='searching…';
  else if(ghState==='fail'){t='no answer from GitHub (does this phone or tablet have internet?)';cls='warn'}
  else if(ghState==='none')t='no seed drill program on GitHub';
  else if(gh){t=verText(gh.version,gh.built)||'a program';
    if(!ghBoardOk()){t+=' — for the '+gh.board+' board, but this module is on the '+fwNow.board+' board';cls='warn'}
    else if(c>0){t+=' — newer than the installed one';cls='ok'}
    else if(c===0)t+=' — the one installed';
    else if(c<0)t+=' — older than the installed one (the new .bin is not on GitHub yet)'}
  e.textContent=t;e.className=cls;
  $('ghInstall').disabled=!nw||busy;
  $('ghLink').href=GH_WEB+(gh?gh.file.replace(/\/[^/]*$/,''):'Seeder/Seeder_ESP32');
  const b=document.querySelector('nav button[data-t="firmware"]');b.classList.toggle('new',nw);b.title=nw?'A newer program is on GitHub':'';
}
async function ghInstall(){
  if(!ghNewer()||busy)return;
  const name=gh.file.split('/').pop(),want=gh.size||0;
  if(!confirm('Install '+(fwNow.fw+' '+verText(gh.version,gh.built))+' from GitHub on this module? It now runs '+infoText(fwNow)+'. Every section closes.'))return;
  const pass=pw();if(pass===null)return;
  busy=true;$('install').disabled=true;renderGh();const g=$('prog');g.hidden=false;g.value=0;
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
  }catch(e){busy=false;g.hidden=true;$('install').disabled=!$('bin').files[0];renderGh();
    progMsg('Not downloaded: '+e.message+'. The current program keeps running.','bad')}
}
refresh();setInterval(refresh,1000);
</script>
</body>
</html>
)SEEDERPAGE";
