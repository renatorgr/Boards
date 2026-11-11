/* xPage.h - the XIAO web page (English).
   Part of the XIAO ESP32C3 / ESP32S3 program that goes with the Teensy firmware AiO_WEB.
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// xPage.h
// The XIAO's web page (served by v4.5_XIAO_ESP32.ino): one self-contained HTML file, no internet
// needed, made for a phone. In English (the Portuguese version and its language button are gone,
// 20 Oct 2026). Dark, or light when the phone asks for it; the header button switches and the phone remembers it.
// The module card names the GPS receiver and the IMU (TM171 or BNO085) the Teensy found. A newer
// version on GitHub (renatorgr/Boards, folder Firmware): the browser asks GitHub (when the phone has
// internet) and can install it through the same update as a chosen file; "latest version installed" when the
// newest version there is the one installed, compiled at the same time.
#pragma once

const char XIAO_PAGE[] PROGMEM = R"XIAOPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark light">
<title>XIAO · tractor module</title>
<style>
/* dark (as before); light when the phone asks for it, or chosen with the header button (data-theme) */
:root{color-scheme:dark;--bg:#121511;--card:#1b1f1a;--ink:#e7ebe3;--mute:#9aa395;--line:#2e352b;--ok:#66bb6a;--warn:#ffb74d;--bad:#ef5350;--acc:#8cc068;--accInk:#10140f;--soft:#232a21;--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
:root[data-theme=light]{color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}
@media (prefers-color-scheme:light){:root:not([data-theme]){color-scheme:light;--bg:#f2f4ef;--card:#ffffff;--ink:#1c2318;--mute:#5f6b5a;--line:#d4dbcf;--ok:#2e7d32;--warn:#b45309;--bad:#c62828;--acc:#4c7d2a;--accInk:#ffffff;--soft:#e9eee5}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);accent-color:var(--acc);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{display:flex;align-items:center;gap:10px;padding:2px 16px;min-height:48px;background:var(--card);border-bottom:1px solid var(--line)}
#top{position:sticky;top:0;z-index:3}   /* header and tabs stay in view while the page scrolls (small windows, the program's web cards) */
header h1{font-size:16px;margin:0;font-weight:650;white-space:nowrap}
header .sub{color:var(--mute);font-size:13px;flex:1;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.dot{width:11px;height:11px;border-radius:50%;background:var(--bad);flex:none}
.dot.on{background:var(--ok)}
nav{display:flex;gap:6px;padding:4px 12px;overflow-x:auto;background:var(--card);border-bottom:1px solid var(--line)}
nav button{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;min-height:44px;padding:7px 14px;font:inherit;font-size:14px;cursor:pointer;white-space:nowrap}
nav button.on{background:var(--acc);border-color:var(--acc);color:var(--accInk)}
main{padding:14px 16px 40px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}
.grid3{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}
@media (max-width:760px){.grid3{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;min-width:0}
.card h2{font-size:12.5px;text-transform:uppercase;letter-spacing:.05em;color:var(--mute);margin:0 0 8px;font-weight:650}
dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:0}
dt{color:var(--mute)}
dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.big{font-size:22px;font-weight:650;font-variant-numeric:tabular-nums}
.pill{display:inline-block;padding:1px 9px;border-radius:999px;font-size:13px;font-weight:600;background:var(--soft)}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.pill.ok{background:color-mix(in srgb,var(--ok) 16%,transparent)}
.pill.warn{background:color-mix(in srgb,var(--warn) 18%,transparent)}
.pill.bad{background:color-mix(in srgb,var(--bad) 16%,transparent)}
.mono{font-family:var(--mono);font-size:12.5px;white-space:pre-wrap;overflow-wrap:anywhere}
.note{color:var(--mute);font-size:13.5px;margin:6px 0 0}
.note.bad{color:var(--bad);font-weight:600}.note.warn{color:var(--warn)}.note.ok{color:var(--ok)}   /* a refusal red (and bold), a wait orange, a success green: .note alone painted them all grey */
.age{color:var(--mute);font-size:12.5px;font-weight:400;text-transform:none;letter-spacing:0;float:right}
.secs{display:grid;grid-template-columns:repeat(8,1fr);gap:4px;margin-top:8px}
.secs span{height:22px;border-radius:5px;background:var(--soft);font-size:11px;display:flex;align-items:center;justify-content:center;color:var(--mute)}
.secs span.on{background:var(--ok);color:var(--accInk);font-weight:700}
.log{max-height:55vh;overflow:auto;background:var(--card);border:1px solid var(--line);border-radius:12px;padding:8px 10px}
.ll{display:flex;gap:10px;padding:2px 0;border-bottom:1px dashed var(--line)}
.ll .t{color:var(--mute);flex:none;font-variant-numeric:tabular-nums;width:78px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:10px 0}
button.act,label.act{border:0;background:var(--acc);color:var(--accInk);border-radius:8px;padding:9px 16px;font:inherit;font-weight:600;cursor:pointer}
button.sec{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button.danger{border:1px solid var(--bad);background:transparent;color:var(--bad);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button:disabled{opacity:.45;cursor:default}
.field{display:flex;flex-direction:column;gap:3px;margin:8px 0}
.field span{color:var(--mute);font-size:13.5px}
input[type=text],input[type=password],select{font:inherit;padding:9px 10px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);width:100%}
.opt{display:flex;gap:10px;align-items:flex-start;padding:5px 0}
.opt input{margin-top:4px}
.opt small{display:block;color:var(--mute)}
progress{width:100%;height:14px}
h3{font-size:14px;margin:14px 0 4px}
.hidden{display:none!important}
.nets{display:flex;flex-direction:column;gap:4px;margin:2px 0 6px}
.nets button{display:flex;justify-content:space-between;align-items:center;gap:10px;width:100%;text-align:left;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:8px;padding:8px 10px;font:inherit;cursor:pointer}
.nets button.sel{border-color:var(--acc);box-shadow:inset 0 0 0 1px var(--acc)}
.nets .n{overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.nets .r{color:var(--mute);font-size:13px;font-variant-numeric:tabular-nums;white-space:nowrap}
.info{background:color-mix(in srgb,var(--acc) 16%,transparent);border:1px solid var(--acc);border-radius:12px;padding:9px 12px;margin:0 0 12px;font-weight:600}
.info.warn{background:color-mix(in srgb,var(--warn) 16%,transparent);border-color:var(--warn)}
a{color:var(--acc)}
.tbtn{margin-left:auto;flex:none;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:8px;min-width:44px;min-height:44px;padding:0;font:inherit;font-size:18px;font-weight:650;cursor:pointer}
@media (max-width:520px){header .sub{display:none}}
/* the page's own question box (ask / askText in the script): the browser's confirm() and prompt() show nothing inside an
   AgOpenWeb web card (its frame has no allow-modals), so the page never uses them; buttons 44 px high, as the cards ask */
.ask{position:fixed;inset:0;z-index:9;background:rgba(0,0,0,.55);display:flex;align-items:center;justify-content:center;padding:12px}
.ask[hidden],.ask [hidden]{display:none!important}
.askBox{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px;width:100%;max-width:440px;max-height:100%;overflow:auto;box-shadow:0 8px 30px rgba(0,0,0,.35)}
.askBox p{margin:0 0 12px;white-space:pre-line}
.askBox input{display:block;width:100%;font:inherit;padding:9px 10px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);margin:0 0 12px}
.askBtns{display:flex;gap:10px;justify-content:flex-end}
.askBtns button{min-height:44px;min-width:96px}
</style>
</head>
<body>
<div id="top">
<header><span class="dot" id="dot" title="link to the XIAO"></span><h1>XIAO · module</h1><span class="sub" id="hdr">connecting…</span><button class="tbtn" id="theme" type="button" title="Light / dark">☀</button></header>
<nav id="nav"><button data-t="status" class="on">Status</button><button data-t="log">Messages</button><button data-t="wifi">WiFi</button><button data-t="prog">Firmware</button></nav>
</div>
<main>

<section id="t-status">
<div id="verNote" class="info warn hidden"></div>
<div id="ghNote" class="info hidden"></div>
<div id="stale" class="info hidden">This page is older than the XIAO program running now. Reload it: Ctrl+F5 on a computer; on a phone or in a web card, close the page and open it again.</div>
<div class="grid3">
 <div class="card"><h2>Module (Teensy)<span class="age" id="mAge"></span></h2>
  <div class="big" id="mFix">–</div>
  <dl style="margin-top:6px">
   <dt>Satellites</dt><dd id="mSats">–</dd><dt>HDOP</dt><dd id="mHdop">–</dd><dt>Correction age</dt><dd id="mAge2">–</dd>
   <dt>Speed</dt><dd id="mKmh">–</dd><dt>Antenna heading</dt><dd id="mDual">–</dd><dt>IMU</dt><dd id="mTm">–</dd><dt>GPS receiver</dt><dd id="mRx">–</dd>
   <dt>Wheel angle (WAS)</dt><dd id="mWas">–</dd><dt>Autosteer</dt><dd id="mAct">–</dd>
   <dt>Ethernet cable</dt><dd id="mLink">–</dd><dt>Program</dt><dd id="mAog">–</dd><dt>RTCM corrections</dt><dd id="mRtcm">–</dd>
   <dt>Firmware</dt><dd id="mFw" class="mono">–</dd><dt>Processor</dt><dd id="mCpu">–</dd></dl>
  <p class="note">The module's full page opens only over the Ethernet cable, on the module's network: <a class="mono" id="mPage" href="http://192.168.5.126/">http://192.168.5.126/</a>. Another network: WiFi tab, «Module network».</p></div>
 <div class="card"><h2><span id="aName">Program</span> → module<span class="age" id="aAge"></span></h2>
  <div class="big" id="aKmh">–</div>
  <dl style="margin-top:6px"><dt>Autosteer requested</dt><dd id="aSt">–</dd><dt>Requested angle</dt><dd id="aAng">–</dd>
   <dt>Steering data (PGN 254)</dt><dd id="aRate">–</dd><dt>Hydraulic lift</dt><dd id="aHyd">–</dd><dt>Tram line</dt><dd id="aTram">–</dd>
   <dt>U-turn</dt><dd id="aUt">–</dd><dt>Program hello</dt><dd id="aHello">–</dd></dl>
  <p class="note" style="margin-top:8px" id="aSecT">Sections 1–16</p><div class="secs" id="aSecs"></div></div>
 <div class="card"><h2>XIAO</h2>
  <dl><dt>Up for</dt><dd id="xUp">–</dd><dt>Start-up reason</dt><dd id="xRst">–</dd>
   <dt>XIAO WiFi</dt><dd id="xAp">–</dd><dt>Devices connected</dt><dd id="xCl">–</dd><dt>Other network</dt><dd id="xSta">–</dd><dt>IP on the other network</dt><dd id="xIp">–</dd>
   <dt>Link to the Teensy</dt><dd id="xLink">–</dd><dt>Received / sent</dt><dd id="xKB">–</dd><dt>Checksum errors</dt><dd id="xCrc">–</dd>
   <dt>Free memory</dt><dd id="xHeap">–</dd><dt>Temperature</dt><dd id="xTemp">–</dd></dl></div>
</div>
</section>

<section id="t-log" class="hidden">
<h3>Module (Teensy)</h3>
<div id="logT" class="log mono"></div>
<h3>XIAO</h3>
<div id="logX" class="log mono" style="max-height:30vh"></div>
<p class="note">Times are counted from when the Teensy (or the XIAO) was switched on.</p>
</section>

<section id="t-wifi" class="hidden">
<form id="fs" class="grid3" autocomplete="off">
<div class="card"><h2>XIAO WiFi network</h2>
 <label class="field"><span>Network name</span><input type="text" name="apSsid" maxlength="32"></label>
 <label class="field"><span>New password (only to change it: 8 to 63 characters; blank keeps the current one)</span><input type="password" name="apPass" maxlength="63"></label>
 <p class="note">Always on, at 192.168.4.1. Its password is also the one for changing the settings and the firmware.</p>
</div>
<div class="card"><h2>Other WiFi network (optional)</h2>
 <label class="opt"><input type="checkbox" name="staOn" value="1"><span>Also connect the XIAO to another network<small>The farm network or the phone's hotspot. The page is then also at http://<b id="hostShow">xiao</b>.local/ or at the IP the XIAO gets.</small></span></label>
 <label class="field"><span>Network name</span><input type="text" name="staSsid" maxlength="32"></label>
 <div class="row" style="margin:2px 0"><button class="sec" type="button" id="scan">Search for networks</button><span id="scanMsg" class="note"></span></div>
 <div id="netList" class="nets"></div>
 <label class="field"><span>That network's password (blank: keeps the current one; for a new network, blank = open network)</span><input type="password" name="staPass" maxlength="63"></label>
 <label class="field"><span>XIAO name on that network</span><input type="text" name="host" maxlength="24"></label>
</div>
<div class="card"><h2>Confirm and save</h2>
 <label class="field"><span>Current password of the XIAO network (to confirm)</span><input type="password" name="pw" id="pwW" maxlength="63"></label>
 <div class="row"><button class="act" type="submit">Save and restart the XIAO</button><span id="setMsg" class="note"></span></div>
 <p class="note">The XIAO restarts to apply the new settings. If you change the name or the password of the XIAO network, connect the phone to the new network again. Forgot the password? Through the USB port on the panel, in the serial monitor: <span class="mono">wifi-reset</span>.</p>
</div>
</form>
<div class="grid3" style="margin-top:12px">
<div class="card"><h2>Module network (Ethernet)</h2>
 <dl><dt>Module page</dt><dd><a id="nIp" class="mono" href="http://192.168.5.126/">–</a></dd></dl>
 <label class="field"><span>Subnet: the first three numbers of the module's IP (the last stays .126, or .120 without autosteer)</span><input type="text" id="subnet" maxlength="11" placeholder="192.168.5" inputmode="decimal"></label>
 <label class="field"><span>Password of the XIAO network (to confirm)</span><input type="password" id="pwN" maxlength="63"></label>
 <div class="row"><button class="act" type="button" id="subnetBtn">Change and restart the module</button><span id="subnetMsg" class="note"></span></div>
 <p class="note">For a module on a network that is not 192.168.5.x: set that network's first three numbers. The Teensy saves it and restarts; its page is then at the new IP, and AgOpenWeb / AgOpenGPS must use the same subnet (they also set it, with PGN 201). Only with autosteer off.</p></div>
</div>
</section>

<section id="t-prog" class="hidden">
<div id="fw" class="grid3">
<div class="card"><h2>XIAO firmware</h2>
 <dl><dt>Version</dt><dd id="pFw" class="mono">–</dd><dt>Built</dt><dd id="pBuilt">–</dd><dt>Size</dt><dd id="pSize">–</dd></dl></div>
<div class="card"><h2>Version on GitHub</h2>
 <dl><dt>On GitHub</dt><dd id="ghVer">–</dd></dl>
 <div class="row"><button class="act" id="ghInstall" disabled>Install from GitHub</button><button class="sec" id="ghCheck">Check now</button><a id="ghLink" href="https://github.com/renatorgr/Boards/tree/main/Firmware" target="_blank" rel="noopener">See on GitHub</a></div>
 <p class="note">It uses the password of the «Update or restart» card. The phone needs internet: on the XIAO's own network alone it usually has none. Connect the XIAO to the farm network or to the phone's hotspot and open the page over that network.</p></div>
<div class="card"><h2>Update or restart</h2>
 <div class="row" style="margin-top:0"><label class="act">Choose .bin<input type="file" id="binFile" accept=".bin" class="hidden"></label><span id="binName" class="note">no file</span></div>
 <label class="field"><span>XIAO network password (to confirm)</span><input type="password" id="pwP" maxlength="63"></label>
 <div class="row"><button class="act" id="install" disabled>Install</button><button class="danger" id="reboot">Restart</button></div>
 <progress id="prog" max="100" value="0" class="hidden"></progress>
 <p id="fwMsg" class="note"></p>
 <p class="note">If the installation fails, the current program keeps running.</p></div>
<div class="card"><h2>Module (Teensy) firmware</h2>
 <dl><dt>Running</dt><dd id="tFw" class="mono">–</dd><dt>Built</dt><dd id="tBuilt">–</dd><dt>On GitHub</dt><dd id="tGh">–</dd><dt>Previous firmware</dt><dd id="tPrev">–</dd></dl>
 <div class="row"><button class="act" id="tGhInstall" disabled>Install from GitHub</button><button class="sec" id="tRevert" disabled>Go back to the previous firmware</button></div>
 <div class="row"><label class="act">Choose .hex<input type="file" id="tHexFile" accept=".hex" class="hidden"></label><span id="tHexName" class="note">no file</span></div>
 <p id="tHexInfo" class="note hidden"></p>
 <label class="opt"><input type="checkbox" id="tForce"><span>Other firmware<small>Installs it even without the module's update page inside (the next update will then have to be by USB).</small></span></label>
 <div class="row"><button class="act" id="tInstall" disabled>Install on the module</button></div>
 <progress id="tProg" max="100" value="0" class="hidden"></progress>
 <p id="tMsg" class="note"></p>
 <p class="note">For when the module's own page does not open: the file goes to the Teensy over this XIAO's link (about a minute) and the Teensy checks it as one from its own page. The previous firmware is the one that ran before the module's last update, kept in the Teensy's flash: no file or internet needed. Only with autosteer off and the tractor stopped. Uses the password of the «Update or restart» card.</p></div>
<div class="card"><h2>Report for an analysis</h2>
 <p class="note">Puts the XIAO's state and settings, the module status it receives and both message lists in one file, to send when something goes wrong. Passwords are not included. The file is downloaded by this phone or computer.</p>
 <div class="row"><button class="sec" type="button" id="dumpBtn">Download the report</button><span id="dumpMsg" class="note"></span></div></div>
</div>
</section>
</main>
<div id="ask" class="ask" hidden><div class="askBox" role="dialog" aria-modal="true" aria-labelledby="askText"><p id="askText"></p><input id="askIn" type="password" autocomplete="current-password" hidden><div class="askBtns"><button type="button" class="sec" id="askNo">Cancel</button><button type="button" class="act" id="askYes">OK</button></div></div></div>
<script>
"use strict";
const $=id=>document.getElementById(id);
// ---- the page's own question box, instead of the browser's confirm() and prompt(): inside an AgOpenWeb web card the frame has
// no allow-modals, so those return false / null without showing anything (AgOpenWeb Docs/ADD_ON_CARDS.md). ask(text) answers
// true for OK and false for Cancel; askText(text) the text typed, or null for Cancel. A new question closes the one still open.
let askDone=null;
function askShow(text,withInput){
 if(askDone)askEnd(false);
 $("askText").textContent=text;const i=$("askIn");i.hidden=!withInput;i.value="";$("ask").hidden=false;
 (withInput?i:$("askYes")).focus();
 return new Promise(r=>{askDone=r;});}
function askEnd(v){const r=askDone;askDone=null;$("ask").hidden=true;if(r)r(v);}
const ask=text=>askShow(text,false);
const askText=text=>askShow(text,true).then(ok=>ok?$("askIn").value:null);
$("askYes").addEventListener("click",()=>askEnd(true));$("askNo").addEventListener("click",()=>askEnd(false));
$("ask").addEventListener("click",e=>{if(e.target===$("ask"))askEnd(false);});
$("ask").addEventListener("keydown",e=>{if(e.key==="Escape")askEnd(false);else if(e.key==="Enter"&&e.target===$("askIn"))askEnd(true);});
// ---- light / dark: the phone's setting until the header button is used, then this phone's choice
function themeNow(){return document.documentElement.dataset.theme||(matchMedia("(prefers-color-scheme: light)").matches?"light":"dark");}
function themeShow(){$("theme").textContent=themeNow()==="light"?"☾":"☀";}
try{const t=localStorage.getItem("theme");if(t==="light"||t==="dark")document.documentElement.dataset.theme=t;}catch(e){}
$("theme").addEventListener("click",()=>{const t=themeNow()==="light"?"dark":"light";document.documentElement.dataset.theme=t;try{localStorage.setItem("theme",t);}catch(e){}themeShow();});
themeShow();
const PAGE_BUILT=")XIAOPAGE" __DATE__ " " __TIME__ R"XIAOPAGE(";
let lastState=null,lastTeensy=null,lastNets=null,tab="status",chipId=-1,chipName="",setLoaded=false,online=false,logAfter=0,xAfter=0,epoch=0,busy=false,waitBoot=false,upBefore=0,lastUp=0;
const fmt=(v,d,u)=>(v===null||v===undefined||Number.isNaN(v))?"–":Number(v).toFixed(d)+(u||"");
function set(id,txt,cls){const e=$(id);if(!e)return;e.textContent=txt;e.classList.remove("ok","warn","bad");if(cls)e.classList.add(cls);}
function pill(id,txt,cls){const e=$(id);e.textContent="";const s=document.createElement("span");s.className="pill "+(cls||"");s.textContent=txt;e.appendChild(s);}
async function getJ(u,o){const r=await fetch(u,Object.assign({cache:"no-store"},o||{}));let j={};try{j=await r.json();}catch(e){}if(!r.ok&&!j.error)j.error="HTTP "+r.status;return j;}
function dur(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?h+" h "+m+" min":m?m+" min "+(s%60)+" s":s+" s";}
function ago(s){return s<0?"never":s<2?"now":(dur(s)+" ago");}
const WHY={1:"no HPR",2:"HPR late",3:"QF too low",4:"HPR empty"};

function renderModule(t){
 lastTeensy=t;renderTeensyFw();
 if(!t||!t.s){set("mAge","no data from the module","warn");set("mFix","–");return;}
 const s=t.s,g=s.gnss||{},d=s.dual||{},m=s.tm||{},st=s.steer||{},n=s.net||{};
 set("mAge",ago(t.ago>3?t.ago:0),t.ago>3?"warn":"");
 const fixCls=g.gga&&g.fix>0?"ok":"bad";   // green with a position (the name says how good), red without one
 set("mFix",g.gga?(g.fixName):"no data from the receiver",fixCls);
 set("mSats",g.sats||"–");set("mHdop",g.hdop||"–");set("mAge2",g.age?g.age+" s":"–");
 set("mKmh",fmt(g.kmh,1," km/h"));
 if(d.ok)set("mDual",Number(d.heading).toFixed(2)+"° (QF "+d.qf+")","ok");else set("mDual",d.hpr?"no heading"+" ("+(WHY[d.why]?(WHY[d.why]):"?")+")":"no HPR",d.hpr?"bad":"warn");
 set("mTm",(m.type?m.type+" · ":"")+(m.ok?"roll "+fmt(m.roll,1,"°")+" · "+fmt(m.out,1,"°"):(m.seen?"no data":"not detected")),m.ok?"":"bad");
 set("mRx",g.model||"–");   // the receiver as it answered the Teensy
 set("mWas",fmt(st.was,1,"°"));
 pill("mAct",(st.active?"steering":"off"),st.active?"ok":"");
 pill("mLink",n.link?"connected":"unplugged",n.link?"ok":"bad");
 const hn=n.host==="AgOpenWeb"||n.host==="AgOpenGPS"?n.host:"";   // the program, from its hello (Teensy)
 pill("mAog",!n.aog?"no communication":n.hostBoth?"two programs":hn?hn+" · "+"talking":"talking",!n.aog||n.hostBoth?"warn":"ok");
 $("aName").textContent=hn||"Program";
 pill("mRtcm",(n.rtcm?"arriving":"stopped"),n.rtcm?"ok":"warn");
 const fw=s.fw||"–",sp=fw.lastIndexOf(" ");set("mFw",(sp>0?fw.slice(0,sp):fw).replace(/_/g,"_\u200b"));   // "name version": the name has the version already; may break after "_"
 set("mCpu",s.cpuMhz?s.cpuMhz+" MHz · "+Number(s.cpuTemp).toFixed(1)+" °C":"–",s.cpuHot?"warn":"");   // the Teensy's clock and temperature (4 Nov 2026)
 if(s.ip){for(const id of["mPage","nIp"]){const a=$(id);a.href="http://"+s.ip+"/";a.textContent="http://"+s.ip+"/";}   // the module's page, at its real IP (Status note, WiFi tab)
  const si=$("subnet");if(!si.value&&document.activeElement!==si)si.value=s.ip.split(".").slice(0,3).join(".");}
 renderVer();renderTeensyFw();
}
// The XIAO and the Teensy must run the same version (the link changed between versions)
function renderVer(){
 const tv=lastTeensy&&lastTeensy.s&&lastTeensy.s.fw?lastTeensy.s.fw.slice(lastTeensy.s.fw.lastIndexOf(" ")+1):"",xv=lastState?lastState.ver:"";
 const bad=!!(tv&&xv&&tv!==xv),n=$("verNote");
 n.textContent=bad?"The Teensy runs version "+tv+" and this XIAO version "+xv+": they must be the same. Install the same version on both.":"";
 n.classList.toggle("hidden",!bad);
}
function renderState(x){
 lastState=x;lastUp=x.up;if(x.chipId!==undefined){chipId=x.chipId;chipName=x.chip;}
 $("hdr").textContent=x.fw+" · "+(x.wifi.st==="ok"?x.wifi.ip:"192.168.4.1");
 const a=x.aog;
 set("aAge",a.ago254<0?"nothing received":ago(a.ago254>2?a.ago254:0),a.ago254<0||a.ago254>2?"warn":"");
 set("aKmh",a.ago254<0?"–":fmt(a.kmh,1," km/h"));
 pill("aSt",a.ago254<0?"–":(a.status===1?"on":"off"),a.status===1?"ok":"");
 set("aAng",a.ago254<0?"–":fmt(a.angle,1,"°"));
 set("aRate",a.ago254<0?"–":a.rate254+"/s");
 set("aHyd",a.ago239<0?"–":(a.hyd===2?"up":a.hyd===1?"down":"off"));
 set("aTram",a.ago239<0?"–":a.tram?String(a.tram):"no");
 set("aUt",a.ago239<0?"–":a.uturn?String(a.uturn):"no");
 set("aHello",ago(a.helloAgo));
 // one square per section: 16 at least, up to 64 (PGN 229), as many as the highest section asked for since the XIAO started
 const box=$("aSecs"),hx=String(a.sec64||(a.sec>>>0).toString(16)),ns=Math.max(16,a.secN||0);
 if(box.childElementCount!==ns){box.textContent="";for(let i=1;i<=ns;i++){const s=document.createElement("span");s.textContent=i;box.appendChild(s);}set("aSecT","Sections 1–"+ns);}
 for(let i=0;i<ns;i++){const d=hx.length-1-(i>>2);box.children[i].classList.toggle("on",a.ago254>=0&&d>=0&&!!((parseInt(hx[d],16)>>(i&3))&1));}
 set("xUp",dur(x.up));set("xRst",(x.reset),x.resetBad?"warn":"");
 set("xAp",x.wifi.ap+" · "+"channel"+" "+x.wifi.ch);set("xCl",String(x.wifi.apCl));
 const w=x.wifi;
 set("xSta",!w.staOn?"off":w.st==="ok"?w.sta+" · "+w.rssi+" dBm":w.sta+" · "+"connecting"+"…",w.staOn&&w.st!=="ok"?"warn":"");
 set("xIp",w.staOn&&w.st==="ok"?w.ip:"–");
 pill("xLink",(x.link.talking?"talking":"no data"),x.link.talking?"ok":"bad");
 set("xKB",x.link.rxKB.toFixed(1)+" / "+x.link.txKB.toFixed(1)+" kB");
 set("xCrc",String(x.link.crcErr),x.link.crcErr?"warn":"");
 set("xHeap",Math.round(x.heap/1024)+" kB");set("xTemp",x.temp!==undefined?Number(x.temp).toFixed(1)+" °C":"–");
 renderGh();
 renderVer();
 set("pFw",x.fw);set("pBuilt",x.built);set("pSize",Math.round(x.sketch/1024)+" kB ("+"max. "+Math.round(x.sketchFree/1024)+" kB)");
}
function setOnline(v){online=v;$("dot").classList.toggle("on",v);}
// ---- the report for an analysis: every answer of the XIAO in one .json file, downloaded by this browser
$("dumpBtn").addEventListener("click",async()=>{
 const b=$("dumpBtn"),m=$("dumpMsg");b.disabled=true;m.className="note";m.textContent="Gathering…";
 const r={made:new Date().toString(),page:location.href,browser:navigator.userAgent};
 for(const [k,u] of [["state","/api/state"],["teensy","/api/teensy"],["settings","/api/settings"]]){try{r[k]=await getJ(u);}catch(e){r[k]={error:String(e)};}}
 const lt=[],lx=[];let a=0,xa=0;
 for(let i=0;i<40;i++){let j;try{j=await getJ("/api/log?after="+a+"&xafter="+xa);}catch(e){break;}
  if(j.error||!j.t)break;
  for(const l of j.t)lt.push(dur(Math.floor(l.t/1000))+"  "+l.x);
  for(const l of j.x)lx.push(dur(Math.floor(l.t/1000))+"  "+l.x);
  if(j.t.length)a=j.t[j.t.length-1].s;if(j.x.length)xa=j.x[j.x.length-1].s;
  if(!j.more)break;}
 r.teensyLog=lt;r.xiaoLog=lx;
 const d=new Date(),z=n=>String(n).padStart(2,"0"),fw=((r.state&&r.state.fw)||"xiao").split(" ")[0];
 const name=fw+"_report_"+d.getFullYear()+"-"+z(d.getMonth()+1)+"-"+z(d.getDate())+"_"+z(d.getHours())+z(d.getMinutes())+".json";
 const el=document.createElement("a");el.href=URL.createObjectURL(new Blob([JSON.stringify(r,null,1)],{type:"application/json"}));el.download=name;
 document.body.appendChild(el);el.click();el.remove();setTimeout(()=>URL.revokeObjectURL(el.href),10000);
 m.className="note ok";m.textContent="Downloaded: "+name;b.disabled=false;});
// The XIAO restarted (its time on went back, or another build runs): the whole page again from the XIAO,
// on the same tab - a new program brings its own page, and every card and list starts afresh.
function reloadAfterRestart(msg,b){
 try{sessionStorage.setItem("tab",tab);if(msg)sessionStorage.setItem("fwMsg",msg);}catch(e){}
 if(PAGE_BUILT&&b&&b!==PAGE_BUILT)location.replace(location.pathname+"?b="+encodeURIComponent(b));   // another build: its page, never a kept copy
 else location.reload();
}
// PAGE_BUILT is when this page's own program was compiled (the compiler puts it in; empty in the simulator). When the
// XIAO reports another build, this page is an old one: one a browser kept, or one that missed the restart. It is
// asked again at an address the browser has never seen (?b=<build>), so no kept copy can answer; if the page that
// comes is still the old one, a notice says how to reload it by hand.
// After an update or a restart asked from this page: has the module restarted? Its time on went back, or it is
// behind where it would be had it kept running (a restart asked seconds after start-up leaves it no room to go back).
let bootAt=0;
function restarted(up){return up<upBefore||up+2<upBefore+(Date.now()-bootAt)/1000;}
function pageStale(b){
 if(!PAGE_BUILT||!b||b===PAGE_BUILT)return false;
 const q="?b="+encodeURIComponent(b);
 if(location.search===q){$("stale").classList.remove("hidden");return false;}
 try{sessionStorage.setItem("tab",tab);}catch(e){}
 location.replace(location.pathname+q);return true;
}
async function pollState(){
 try{const x=await getJ("/api/state");if(x.error)throw 0;
  if(lastState&&(x.up+3<lastState.up||x.built!==lastState.built)){reloadAfterRestart(waitBoot?"Updated. Running: "+x.fw+", built "+x.built+".":"",x.built);return;}
  if(pageStale(x.built))return;
  if(waitBoot&&restarted(x.up)){waitBoot=false;busy=false;$("prog").classList.add("hidden");fwMsg(("Updated. Running: "+x.fw+", built "+x.built+"."),"ok");}
  setOnline(true);renderState(x);
  if(tab==="status"||tab==="prog"||tWait){const t=await getJ("/api/teensy");renderModule(t);tWaitCheck(t);}
 }catch(e){setOnline(false);}
}
function addLines(box,lines){
 const near=box.scrollTop+box.clientHeight>=box.scrollHeight-30;
 for(const l of lines){const d=document.createElement("div");d.className="ll";
  const t=document.createElement("span");t.className="t";t.textContent=dur(Math.floor(l.t/1000));d.appendChild(t);
  const x=document.createElement("span");x.textContent=l.x;
  if(/NO DATA|REFUSED|no dual|NOT detected|bad checksum|late|unplugged|silent|stopped|no GGA|dropped|lost/.test(l.x))x.className="warn";
  d.appendChild(x);box.appendChild(d);}
 while(box.childElementCount>300)box.removeChild(box.firstChild);
 if(near)box.scrollTop=box.scrollHeight;
}
async function pollLog(){
 try{let more=true,n=0;
  while(more&&n++<6){const j=await getJ("/api/log?after="+logAfter+"&xafter="+xAfter);if(j.error)return;
   if(j.epoch!==epoch){epoch=j.epoch;logAfter=0;$("logT").textContent="";continue;}
   if(j.xlast<xAfter){xAfter=0;$("logX").textContent="";}
   addLines($("logT"),j.t);addLines($("logX"),j.x);
   if(j.t.length)logAfter=j.t[j.t.length-1].s;if(j.x.length)xAfter=j.x[j.x.length-1].s;
   more=!!j.more;}
 }catch(e){}
}
function show(t){
 tab=t;for(const b of document.querySelectorAll("nav button"))b.classList.toggle("on",b.dataset.t===t);
 for(const s of document.querySelectorAll("main>section"))s.classList.toggle("hidden",s.id!=="t-"+t);
 if(t==="wifi"&&!setLoaded)loadSettings();     // once: going to another tab and back keeps what was typed
 if(t==="log")pollLog();
}
$("nav").addEventListener("click",e=>{if(e.target.dataset.t)show(e.target.dataset.t);});

// ---- WiFi settings
async function loadSettings(){try{const s=await getJ("/api/settings");if(s.error)return;const f=$("fs");setLoaded=true;
 f.apSsid.value=s.apSsid;f.staOn.checked=s.staOn;f.staSsid.value=s.staSsid;f.host.value=s.host;$("hostShow").textContent=s.host;
 f.apPass.value="";f.staPass.value="";f.staPass.placeholder=s.staPassSet?"(saved)":"";}catch(e){}}
$("fs").addEventListener("submit",async e=>{e.preventDefault();const f=$("fs"),m=$("setMsg");
 if(f.apPass.value&&f.apPass.value.length<8){m.textContent="The password must have 8 characters or more.";m.className="note bad";return;}
 if(!await ask("Save and restart the XIAO? The XIAO WiFi goes off for a few seconds."))return;
 const p=new URLSearchParams();for(const k of ["apSsid","apPass","staSsid","staPass","host","pw"])p.set(k,f[k].value);p.set("staOn",f.staOn.checked?"1":"0");
 try{const j=await getJ("/api/settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:p.toString()});
  if(j.ok){setLoaded=false;m.textContent="Saved. The XIAO is restarting.";m.className="note ok";}else{m.textContent="Error: "+j.error;m.className="note bad";}
 }catch(e){m.textContent="No connection to the XIAO.";m.className="note bad";}});
// The module's subnet (WiFi tab, "Module network"): POST /api/subnet; the XIAO passes it to the Teensy (PGN 144),
// which saves it and restarts with the new IP
$("subnetBtn").addEventListener("click",async()=>{
 const v=$("subnet").value.trim(),m=$("subnetMsg"),ok=/^(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/.exec(v);
 if(!ok||ok.slice(1).some(x=>+x>255)){m.textContent="Three numbers, like 192.168.5.";m.className="note bad";return;}
 const cur=lastTeensy&&lastTeensy.s&&lastTeensy.s.ip?lastTeensy.s.ip:"",last=cur?cur.split(".")[3]:"126";
 if(cur&&cur.split(".").slice(0,3).join(".")===v){m.textContent="The module is on that subnet already.";m.className="note";return;}
 if(!await ask("Change the module's subnet to "+v+" and restart it?\n\nThe module will be at "+v+"."+last+"; its page opens at that IP from a device on that network, and AgOpenWeb / AgOpenGPS must use subnet "+v+"."))return;
 m.textContent="Sending…";m.className="note";
 try{const j=await getJ("/api/subnet",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"subnet="+encodeURIComponent(v)+"&pw="+encodeURIComponent($("pwN").value)});
  if(j.ok){m.textContent="Sent. The module saves it and restarts as "+v+"."+last+" (see Messages; the IP above changes when it is back).";m.className="note ok";}
  else{m.textContent="Error: "+j.error;m.className="note bad";}
 }catch(e){m.textContent="No connection to the XIAO.";m.className="note bad";}});
// the networks found, one per line, strongest first; a tap chooses one
function sig(r){return (r>=-60?"strong":r>=-72?"medium":"weak");}
function showNets(list){lastNets=list;const box=$("netList"),f=$("fs");box.textContent="";
 for(const n of list){const b=document.createElement("button");b.type="button";b.classList.toggle("sel",n.s===f.staSsid.value);
  const a=document.createElement("span");a.className="n";a.textContent=n.s;b.appendChild(a);
  const r=document.createElement("span");r.className="r";r.textContent=n.r+" dBm · "+sig(n.r)+(n.open?" · "+"open":"");b.appendChild(r);
  b.addEventListener("click",()=>{const same=f.staSsid.value===n.s;f.staSsid.value=n.s;f.staOn.checked=true;
   for(const o of box.children)o.classList.toggle("sel",o===b);
   if(!same){f.staPass.value="";f.staPass.placeholder=n.open?"(open network)":"";if(!n.open)f.staPass.focus();}});
  box.appendChild(b);}}
function netsMsg(list){const n=list.length;$("scanMsg").textContent=n?(n===1?"1 network":n+" networks")+" · "+"tap one to choose it":"no network found";}
$("scan").addEventListener("click",async()=>{const m=$("scanMsg");m.textContent="searching…";m.className="note";$("netList").textContent="";
 for(let i=0;i<20;i++){try{const j=await getJ("/api/scan");if(j.list){j.list.sort((a,b)=>b.r-a.r);showNets(j.list);
   netsMsg(j.list);return;}}catch(e){}
  await new Promise(r=>setTimeout(r,800));}
 m.textContent="no answer";});
// the password typed once is used in both tabs (only in this page, never saved)
for(const id of ["pwW","pwP"])$(id).addEventListener("input",e=>{for(const o of ["pwW","pwP"])if(o!==id)$(o).value=e.target.value;});

// ---- program
function fwMsg(t,c){const m=$("fwMsg");m.textContent=t;m.className="note "+(c||"");}
// the program header says which chip it is for (byte 12-13): the other XIAO's program is refused here already
const CHIPS={5:"ESP32-C3",9:"ESP32-S3"};
$("binFile").addEventListener("change",async()=>{const f=$("binFile").files[0];$("binName").textContent=f?f.name+" ("+Math.round(f.size/1024)+" kB)":"no file";$("install").disabled=!f||busy;fwMsg("");
 if(!f||chipId<0)return;
 try{const h=new Uint8Array(await f.slice(0,16).arrayBuffer());
  if(h[0]!==0xE9){fwMsg("This file is not a program for the XIAO (wrong .bin).","bad");$("install").disabled=true;return;}
  const id=h[12]|h[13]<<8;
  if(id!==chipId){fwMsg(("This file is for "+(CHIPS[id]?"the XIAO "+CHIPS[id]:"another kind of XIAO")+"; this XIAO is "+chipName+"."),"bad");$("install").disabled=true;}
 }catch(e){}});
$("install").addEventListener("click",async()=>{
 const f=$("binFile").files[0];if(!f||busy)return;
 if(!f.name.toLowerCase().endsWith(".bin")){fwMsg("Choose a .bin file.","bad");return;}
 if(/merged|bootloader|partitions/i.test(f.name)){fwMsg("This is the wrong file: use the ….ino.bin.","bad");return;}
 if(!await ask(("Install "+f.name+" on the XIAO?")))return;
 busy=true;$("install").disabled=true;$("ghInstall").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;
 sendBin(f,f.name,0);});
// The .bin to the XIAO (a chosen file, or one downloaded from GitHub); progress from p0 to 95 %
function sendBin(blob,name,p0){
 const p=$("prog");fwMsg("Sending…");
 const fd=new FormData();fd.append("file",blob,name);
 const x=new XMLHttpRequest();x.open("POST","/update?pw="+encodeURIComponent($("pwP").value));
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=p0+Math.round(e.loaded/e.total*(95-p0));};
 const fail=t=>{busy=false;$("install").disabled=!$("binFile").files[0];renderGh();p.classList.add("hidden");fwMsg(t,"bad");};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;upBefore=lastUp;bootAt=Date.now();waitBoot=true;fwMsg("Installed. The XIAO is restarting…","warn");}
  else fail("Not installed: "+(j.error||("error "+x.status))+". The current program keeps running.");};
 x.onerror=()=>fail("The connection to the XIAO dropped during the upload (weak Wi-Fi?). The current program keeps running. Try again closer to the XIAO, or from its own network (http://192.168.4.1/) with a downloaded .bin.");
 x.send(fd);}

// ---- a newer version on GitHub (renatorgr/Boards, folder Firmware, one folder per version:
// vX.Y_AiO_WEB, with vX.Y_XIAO_ESP32_bin/vX.Y_XIAO_ESP32C3.bin and ..S3.bin in it).
// Asked by this browser when the page opens, every 6 hours, and with "Check now".
// firmware.json (written by Firmware/simulator/build.py on every change on main, published with the simulator)
// says when each XIAO .bin there was compiled: a new build of the same version shows as new. Without it, the
// folder names alone (the version number only).
const GH_API="https://api.github.com/repos/renatorgr/Boards/contents/Firmware",GH_WEB="https://github.com/renatorgr/Boards/tree/main/Firmware";
const GH_PUB="https://renatorgr.github.io/Boards/firmware.json";
const GH_RX=/^v(\d+(?:\.\d+)*)_AiO_WEB$/,MON="JanFebMarAprMayJunJulAugSepOctNovDec";
let gh=null,ghState="",ghAt=0;
function verCmp(a,b){const x=String(a).split(".").map(Number),y=String(b).split(".").map(Number);for(let i=0;i<Math.max(x.length,y.length);i++){const d=(x[i]||0)-(y[i]||0);if(d)return d;}return 0;}
function builtKey(b){const m=/^([A-Z][a-z]{2}) +(\d{1,2}) (\d{4}) (\d\d):(\d\d):(\d\d)$/.exec(String(b||"").trim()),i=m?MON.indexOf(m[1]):-1;   // "Oct 31 2026 19:27:45" -> "20261031192745"
 return i<0||i%3?"":m[3]+String(i/3+1).padStart(2,"0")+m[2].padStart(2,"0")+m[4]+m[5]+m[6];}
function builtCmp(a,b){a=builtKey(a);b=builtKey(b);return a&&b?(a>b)-(a<b):0;}
function builtText(b){const k=builtKey(b);return k?(+k.slice(6,8))+" "+MON.substr(k.slice(4,6)*3-3,3)+" "+k.slice(0,4)+" "+k.slice(8,10)+":"+k.slice(10,12):"";}   // "31 Oct 2026 19:27"
function verText(v,b){const t=builtText(b);return v+(t?", built "+t:"");}
function runsText(){const t=lastState&&builtText(lastState.built);return t?"the build of "+t:"version "+runningVer();}   // what this XIAO runs
function runningVer(){const m=lastState&&/^v(\d+(?:\.\d+)*)_/.exec(lastState.fw);return m?m[1]:"";}
function myChip(){return chipName==="ESP32-S3"?"S3":chipName==="ESP32-C3"?"C3":"";}
function ghBuilt(){const x=gh&&gh.xiao&&gh.xiao[myChip()];return x&&x.built||"";}   // when the .bin for this XIAO's chip was compiled
async function ghList(path){const r=await fetch(GH_API+(path?"/"+path:"")+"?ref=main");/* a simple request: no CORS preflight */if(!r.ok)throw new Error("HTTP "+r.status);return r.json();}
async function ghCheck(){
 if(ghState==="busy")return;
 ghState="busy";renderGh();
 let best=null;
 try{const r=await fetch(GH_PUB,{cache:"no-store"}),j=r.ok?await r.json():null,m=j&&GH_RX.exec(j.folder||"");
  if(m)best={ver:m[1],dir:j.folder,xiao:j.xiao||{},teensy:j.teensy||null};}catch(e){}
 try{if(!best){const l=await ghList("");
   for(const e of l)if(e.type==="dir"){const m=GH_RX.exec(e.name);if(m&&(!best||verCmp(m[1],best.ver)>0))best={ver:m[1],dir:e.name};}}
  gh=best;ghState=best?"ok":"none";}
 catch(e){ghState="fail";}
 ghAt=Date.now();renderGh();
}
function ghCmp(){const r=runningVer();return gh&&r?verCmp(gh.ver,r)||builtCmp(ghBuilt(),lastState.built):NaN;}   // >0: GitHub's is newer
function ghNewer(){return ghCmp()>0;}
function renderGh(){
 const c=ghCmp(),nw=c>0;
 set("ghVer",ghState==="busy"?"searching…":ghState==="fail"?"no answer from GitHub (does this device have internet?)":ghState==="none"?"no version on GitHub":
  !gh?"–":verText(gh.ver,ghBuilt())+(nw?" — newer than the installed one":c===0?" — the one installed":c<0?" — older than the installed one (the new .bin is not on GitHub yet)":""),
  nw?"ok":ghState==="fail"?"warn":"");
 $("ghInstall").disabled=!nw||busy;
 $("ghLink").href=gh?GH_WEB+"/"+gh.dir:GH_WEB;
 const n=$("ghNote");n.textContent=nw?("New XIAO program on GitHub: "+verText(gh.ver,ghBuilt())+". This XIAO runs "+runsText()+". Install it in the Firmware tab."):"";
 n.classList.toggle("hidden",!nw);
 renderTeensyFw();
}
async function ghInstall(){
 if(!ghNewer()||busy)return;
 const chip=myChip();
 if(!chip){fwMsg("It is not known whether this XIAO is a C3 or an S3.","bad");return;}
 if(!$("pwP").value){fwMsg("Type the XIAO network password.","bad");$("pwP").focus();return;}
 const bdir="v"+gh.ver+"_XIAO_ESP32_bin",name="v"+gh.ver+"_XIAO_ESP32"+chip+".bin";
 if(!await ask(("Install "+verText(gh.ver,ghBuilt())+" from GitHub ("+name+") on the XIAO?")))return;
 busy=true;$("install").disabled=true;$("ghInstall").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;
 fwMsg("Downloading from GitHub…");
 try{
  const f=(await ghList(gh.dir+"/"+bdir)).find(e=>e.type==="file"&&e.name===name);
  if(!f)throw new Error(("the file "+name+" is not on GitHub"));
  const res=await fetch(f.download_url);if(!res.ok)throw new Error("GitHub: HTTP "+res.status);
  const rd=res.body.getReader(),parts=[];let got=0;
  for(;;){const {done,value}=await rd.read();if(done)break;parts.push(value);got+=value.length;p.value=Math.round(got/f.size*45);}
  const blob=new Blob(parts);
  if(blob.size!==f.size)throw new Error(("file incomplete ("+blob.size+" of "+f.size+" bytes)"));
  const h=new Uint8Array(await blob.slice(0,16).arrayBuffer());
  if(h[0]!==0xE9||(h[12]|h[13]<<8)!==chipId)throw new Error("This file is not a program for the XIAO (wrong .bin).");
  sendBin(blob,name,45);
 }catch(e){busy=false;$("install").disabled=!$("binFile").files[0];p.classList.add("hidden");renderGh();
  fwMsg("Not downloaded: "+e.message+". The current program keeps running.","bad");}
}
$("ghCheck").addEventListener("click",ghCheck);
$("ghInstall").addEventListener("click",ghInstall);

// ---- the module's (Teensy's) firmware, from this page over the link (2 Nov 2026): for a module whose own page cannot be
// reached. A .hex (chosen here, or the one on GitHub) goes to the XIAO as our own update does (POST /api/teensy-update)
// and on to the Teensy block by block; "Previous firmware" is the copy the Teensy keeps of the firmware that ran before
// its last update (POST /api/teensy-revert). The module's answer and its restart are watched through /api/teensy.
let tBusy=false,tWait=null,tHex=null;
function tMsg(t,c){const m=$("tMsg");m.textContent=t;m.className="note "+(c||"");}
// What is inside a chosen .hex, read in this browser (as the Teensy page does): the "[boot] vX.Y_AiO_WEB, built ..." line the
// firmware prints at start-up, "fw_teensy41" (a Teensy 4.1 firmware) and the family marker (it has the update page inside)
function hexRead(text){
 let s="";
 for(const ln of text.split("\n")){if(ln.charCodeAt(0)!==58||ln.substr(7,2)!=="00")continue;const n=parseInt(ln.substr(1,2),16);let o="";for(let i=0;i<n;i++)o+=String.fromCharCode(parseInt(ln.substr(9+2*i,2),16));s+=o;}
 const m=/\[boot\] (v[\w.]+_AiO_WEB), built ([A-Z][a-z]{2} [ \d]\d \d{4} \d\d:\d\d:\d\d)/.exec(s);
 return {fw:m?m[1]:"",built:m?m[2].replace("  "," "):"",t41:s.includes("fw_teensy41"),marker:s.includes("AiO-WEB-OTA-1")||s.includes("AOG-UM982-TM171-WEB-OTA-1")};
}
function tName(fw){return String(fw||"").split(" ")[0];}
function renderTeensyFw(){
 const s=lastTeensy&&lastTeensy.s,talk=!!(s&&lastState&&lastState.link&&lastState.link.talking);
 set("tFw",s?tName(s.fw):"–");set("tBuilt",s?s.built:"–");
 const p=s&&s.prev;
 set("tPrev",!s?"–":p?tName(p.fw)+", built "+(builtText(p.built)||p.built):"none kept yet (kept from the module's next update on)");
 $("tRevert").disabled=!talk||!p||tBusy;
 const g=gh?gh.teensy:null,mv=s?(/^v(\d+(?:\.\d+)*)_/.exec(s.fw)||[])[1]||"":"";
 const c=g&&mv?(verCmp(gh.ver,mv)||(g.built?builtCmp(g.built,s.built):NaN)):NaN;
 set("tGh",ghState==="busy"?"searching…":ghState==="fail"?"no answer from GitHub (does this device have internet?)":!gh?"–":!g?"no .hex on GitHub":
  verText(gh.ver,g.built)+(c>0?" — newer than the module's":c===0?" — the one the module runs":c<0?" — older than the module's":g.built?"":" (build date unknown)"),c>0?"ok":ghState==="fail"?"warn":"");
 $("tGhInstall").disabled=!g||!talk||tBusy||c===0;
 $("tInstall").disabled=!$("tHexFile").files[0]||!talk||tBusy;
}
$("tHexFile").addEventListener("change",()=>{const f=$("tHexFile").files[0];tHex=null;
 $("tHexName").textContent=f?f.name+" ("+Math.round(f.size/1024)+" kB)":"no file";tMsg("");renderTeensyFw();
 const inf=$("tHexInfo");inf.classList.toggle("hidden",!f);if(!f)return;
 inf.textContent="Reading the file…";inf.className="note";
 const r=new FileReader();
 r.onload=()=>{const b=tHex=hexRead(r.result),s=lastTeensy&&lastTeensy.s;let t,c="";
  if(!b.t41){t="Not a Teensy 4.1 firmware (no fw_teensy41 inside): the module refuses it.";c="bad";}
  else if(!b.fw){t="No AiO_WEB build data inside: another firmware. It needs «Other firmware» to install"+(b.marker?".":", and it has no update page: the next update would be by USB.");c="warn";}
  else{const same=!!s&&s.built===b.built;t=b.fw+", built "+b.built+(same?" (the firmware the module runs now)":"")+(b.marker?"":"; no update page inside: needs «Other firmware»");c=same?"":"ok";}
  inf.textContent=t;inf.className="note "+c;};
 r.onerror=()=>{inf.textContent="The file could not be read.";inf.className="note bad";};
 r.readAsText(f);
});
function tPassword(){if($("pwP").value)return true;tMsg("Type the XIAO network password (Update or restart card).","bad");$("pwP").focus();return false;}
$("tInstall").addEventListener("click",async()=>{
 const f=$("tHexFile").files[0];if(!f||tBusy)return;
 if(!f.name.toLowerCase().endsWith(".hex")){tMsg("Choose a .hex file.","bad");return;}
 if(!tPassword())return;
 const b=tHex&&tHex.fw?" ("+tHex.fw+", built "+tHex.built+")":"";
 if(!await ask(("Install "+f.name+b+" on the module (Teensy) over the link?\n\nAutosteer must be off and the tractor stopped. Autosteer is off during the update, about a minute.")))return;
 sendTeensyHex(f,f.name,$("tForce").checked,0);
});
// The .hex to the XIAO and on to the Teensy; progress from p0 to 95 %, then the Teensy's answer
function sendTeensyHex(blob,name,force,p0){
 tBusy=true;busy=true;renderTeensyFw();const p=$("tProg");p.classList.remove("hidden");p.value=p0;
 tMsg("Sending the file to the module over the link (about a minute)…");
 const fd=new FormData();fd.append("file",blob,name);
 const x=new XMLHttpRequest();
 x.open("POST","/api/teensy-update?pw="+encodeURIComponent($("pwP").value)+"&force="+(force?"1":"0")+"&size="+blob.size);
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=p0+Math.round(e.loaded/e.total*(95-p0));};
 x.upload.onload=()=>tMsg("File sent; the module is checking and writing it…");
 const fail=t=>{tBusy=false;busy=false;p.classList.add("hidden");tMsg(t,"bad");renderTeensyFw();};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;busy=false;tWaitStart("Checked ("+Math.round(j.size/1024)+" kB). The module is installing and restarting — do not switch the power off…");}
  else fail("Not installed: "+(j.error||("error "+x.status))+". The module's current firmware keeps running.");};
 x.onerror=()=>fail("The connection to the XIAO dropped during the upload (weak Wi-Fi?). The module's current firmware keeps running. Try again closer to the XIAO, or from its own network (http://192.168.4.1/).");
 x.send(fd);
}
// After the module said yes: wait for it to come back (its time on goes back, or another build runs), through /api/teensy
function tWaitStart(msg){tMsg(msg,"warn");const s=lastTeensy&&lastTeensy.s;tWait={up:s?s.up:0,built:s?s.built:"",at:Date.now()};}
function tWaitCheck(t){
 if(!tWait)return;
 const s=t&&t.s,done=()=>{tWait=null;tBusy=false;$("tProg").classList.add("hidden");renderTeensyFw();};
 if(t&&t.ago>=0&&t.ago<=2&&s&&(s.up<tWait.up||s.built!==tWait.built)){done();tMsg("Done. The module runs "+tName(s.fw)+", built "+s.built+".","ok");}
 else if(Date.now()-tWait.at>150e3){done();tMsg("No news from the module yet. Look at the Messages tab, or at the module's own page.","warn");}
}
async function tGhInstall(){
 const g=gh&&gh.teensy;if(!g||tBusy)return;
 if(!tPassword())return;
 const names=[g.file,gh.dir+".hex",gh.dir+".ino.hex"].filter(Boolean);   // the Arduino IDE exports vX.Y_AiO_WEB.ino.hex
 if(!await ask(("Install "+verText(gh.ver,g.built)+" from GitHub on the module (Teensy) over the link?\n\nAutosteer must be off and the tractor stopped. Autosteer is off during the update, about a minute.")))return;
 tBusy=true;busy=true;renderTeensyFw();const p=$("tProg");p.classList.remove("hidden");p.value=0;tMsg("Downloading from GitHub…");
 try{
  const l=await ghList(gh.dir),f=names.map(n=>l.find(e=>e.type==="file"&&e.name===n)).find(Boolean);
  if(!f)throw new Error(("the file "+names[0]+" is not on GitHub"));
  const res=await fetch(f.download_url);if(!res.ok)throw new Error("GitHub: HTTP "+res.status);
  const rd=res.body.getReader(),parts=[];let got=0;
  for(;;){const {done,value}=await rd.read();if(done)break;parts.push(value);got+=value.length;p.value=Math.round(got/f.size*45);}
  const blob=new Blob(parts);
  if(blob.size!==f.size)throw new Error(("file incomplete ("+blob.size+" of "+f.size+" bytes)"));
  sendTeensyHex(blob,f.name,false,45);
 }catch(e){tBusy=false;busy=false;p.classList.add("hidden");renderTeensyFw();tMsg("Not downloaded: "+e.message+". The module's current firmware keeps running.","bad");}
}
$("tGhInstall").addEventListener("click",tGhInstall);
$("tRevert").addEventListener("click",async()=>{
 const s=lastTeensy&&lastTeensy.s,p=s&&s.prev;if(!p||tBusy)return;
 if(!tPassword())return;
 if(!await ask(("Put the module (Teensy) back to its previous firmware, "+tName(p.fw)+" built "+(builtText(p.built)||p.built)+"?\n\nAutosteer must be off and the tractor stopped. The firmware running now is replaced; the settings are kept.")))return;
 tBusy=true;renderTeensyFw();tMsg("Asking the module…");
 try{const j=await getJ("/api/teensy-revert",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"pw="+encodeURIComponent($("pwP").value)});
  if(j.ok)tWaitStart("The module is going back to its previous firmware and restarting — do not switch the power off…");
  else{tBusy=false;renderTeensyFw();tMsg("Not done: "+j.error+". The module's current firmware keeps running.","bad");}
 }catch(e){tBusy=false;renderTeensyFw();tMsg("No connection to the XIAO.","bad");}
});
$("reboot").addEventListener("click",async()=>{if(busy||!await ask("Restart the XIAO?"))return;
 try{const j=await getJ("/api/restart",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"pw="+encodeURIComponent($("pwP").value)});
  fwMsg(j.ok?"Restarting…":"Error: "+j.error,j.ok?"warn":"bad");}catch(e){fwMsg("No connection to the XIAO.","bad");}});

// ---- one request at a time, every second
let tickN=0;
async function tick(){tickN++;if(busy&&!waitBoot)return;await pollState();if(!online)return;
 if(lastState&&!busy&&(ghAt===0||Date.now()-ghAt>6*3600e3))ghCheck();   // GitHub, not waited for
 if(tab==="log"||tickN%5===0)await pollLog();}
// after a reload for a restart: the same tab, and the update's result
try{const t=sessionStorage.getItem("tab"),m=sessionStorage.getItem("fwMsg");sessionStorage.removeItem("tab");sessionStorage.removeItem("fwMsg");
 if(t&&$("t-"+t))show(t);if(m)fwMsg(m,"ok");}catch(e){}
(async function loop(){await tick();setTimeout(loop,1000);})();
</script>
</body>
</html>
)XIAOPAGE";
