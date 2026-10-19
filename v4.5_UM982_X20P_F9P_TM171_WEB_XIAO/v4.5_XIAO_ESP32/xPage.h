/* xPage.h - the XIAO web page (Portuguese / English).
   Part of the XIAO ESP32C3 / ESP32S3 program that goes with the Teensy firmware
   UM982_X20P_F9P_TM171_WEB_XIAO (renatorgr).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// xPage.h
// The XIAO's web page (served by v4.5_XIAO_ESP32.ino): one self-contained HTML file, no internet
// needed, made for a phone. Text in Portuguese, and in English with the EN button (the module's
// messages stay in English). The English is the EN table in the script.
// The module card names the GPS receiver and the IMU (TM171 or BNO085) the Teensy found. A newer
// version on GitHub (renatorgr/Boards, folder Firmware): the browser asks GitHub (when the phone has
// internet) and can install it through the same update as a chosen file; "up to date" when the
// newest version there is the one installed.
#pragma once

const char XIAO_PAGE[] PROGMEM = R"XIAOPAGE(<!doctype html>
<html lang="pt">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark">
<title>XIAO · módulo do trator</title>
<style>
:root{color-scheme:dark;--bg:#121511;--card:#1b1f1a;--ink:#e7ebe3;--mute:#9aa395;--line:#2e352b;--ok:#66bb6a;--warn:#ffb74d;--bad:#ef5350;--acc:#8cc068;--accInk:#10140f;--soft:#232a21;--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
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
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}
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
#fw .card,#fs .card{margin:0 0 12px}
.nets{display:flex;flex-direction:column;gap:4px;margin:2px 0 6px}
.nets button{display:flex;justify-content:space-between;align-items:center;gap:10px;width:100%;text-align:left;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:8px;padding:8px 10px;font:inherit;cursor:pointer}
.nets button.sel{border-color:var(--acc);box-shadow:inset 0 0 0 1px var(--acc)}
.nets .n{overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.nets .r{color:var(--mute);font-size:13px;font-variant-numeric:tabular-nums;white-space:nowrap}
.info{background:color-mix(in srgb,var(--acc) 16%,transparent);border:1px solid var(--acc);border-radius:12px;padding:9px 12px;margin:0 0 12px;font-weight:600}
a{color:var(--acc)}
.lang{margin-left:auto;flex:none;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:999px;padding:3px 11px;font:inherit;font-size:13px;font-weight:650;cursor:pointer}
@media (max-width:520px){header .sub{display:none}}
</style>
</head>
<body>
<header><span class="dot" id="dot" title="ligação ao XIAO"></span><h1>XIAO · módulo</h1><span class="sub" id="hdr">a ligar…</span><button class="lang" id="lang" type="button" title="Português / English">EN</button></header>
<nav id="nav"><button data-t="status" class="on">Estado</button><button data-t="log">Mensagens</button><button data-t="wifi">WiFi</button><button data-t="prog">Programa</button></nav>
<main>

<section id="t-status">
<div id="ghNote" class="info hidden"></div>
<div class="grid">
 <div class="card"><h2>Módulo (Teensy)<span class="age" id="mAge"></span></h2>
  <div class="big" id="mFix">–</div>
  <dl style="margin-top:6px">
   <dt>Satélites</dt><dd id="mSats">–</dd><dt>HDOP</dt><dd id="mHdop">–</dd><dt>Idade das correções</dt><dd id="mAge2">–</dd>
   <dt>Velocidade</dt><dd id="mKmh">–</dd><dt>Heading das antenas</dt><dd id="mDual">–</dd><dt>IMU</dt><dd id="mTm">–</dd><dt>Recetor GPS</dt><dd id="mRx">–</dd>
   <dt>Ângulo da roda (WAS)</dt><dd id="mWas">–</dd><dt>Ângulo pedido</dt><dd id="mSet">–</dd><dt>Autosteer</dt><dd id="mAct">–</dd>
   <dt>Cabo Ethernet</dt><dd id="mLink">–</dd><dt>Programa</dt><dd id="mAog">–</dd><dt>Correções RTCM</dt><dd id="mRtcm">–</dd>
   <dt>Firmware</dt><dd id="mFw" class="mono">–</dd></dl>
  <p class="note">A página completa do módulo abre só pelo cabo Ethernet: <span class="mono">http://192.168.5.126/</span></p></div>
 <div class="card"><h2><span id="aName">Programa</span> → módulo<span class="age" id="aAge"></span></h2>
  <div class="big" id="aKmh">–</div>
  <dl style="margin-top:6px"><dt>Autosteer pedido</dt><dd id="aSt">–</dd><dt>Ângulo pedido</dt><dd id="aAng">–</dd>
   <dt>Dados de direção (PGN 254)</dt><dd id="aRate">–</dd><dt>Elevador hidráulico</dt><dd id="aHyd">–</dd><dt>Tram line</dt><dd id="aTram">–</dd>
   <dt>U-turn</dt><dd id="aUt">–</dd><dt>Hello do programa</dt><dd id="aHello">–</dd></dl>
  <p class="note" style="margin-top:8px">Secções 1–16</p><div class="secs" id="aSecs"></div></div>
 <div class="card"><h2>XIAO</h2>
  <dl><dt>Versão</dt><dd id="xFw" class="mono">–</dd><dt>Ligado há</dt><dd id="xUp">–</dd><dt>Motivo do arranque</dt><dd id="xRst">–</dd>
   <dt>WiFi do XIAO</dt><dd id="xAp">–</dd><dt>Dispositivos ligados</dt><dd id="xCl">–</dd><dt>Outra rede</dt><dd id="xSta">–</dd><dt>IP na outra rede</dt><dd id="xIp">–</dd>
   <dt>Ligação ao Teensy</dt><dd id="xLink">–</dd><dt>Recebido / enviado</dt><dd id="xKB">–</dd><dt>Erros de checksum</dt><dd id="xCrc">–</dd>
   <dt>Memória livre</dt><dd id="xHeap">–</dd></dl></div>
</div>
</section>

<section id="t-log" class="hidden">
<h3>Módulo (Teensy)</h3>
<div id="logT" class="log mono"></div>
<h3>XIAO</h3>
<div id="logX" class="log mono" style="max-height:30vh"></div>
<p class="note">As horas são o tempo desde que o Teensy (ou o XIAO) ligou.</p>
</section>

<section id="t-wifi" class="hidden">
<form id="fs" autocomplete="off">
<div class="card"><h2>Rede WiFi do XIAO</h2>
 <label class="field"><span>Nome da rede</span><input type="text" name="apSsid" maxlength="32"></label>
 <label class="field"><span>Nova palavra-passe (só para a mudar: 8 a 63 caracteres; em branco fica a atual)</span><input type="password" name="apPass" maxlength="63"></label>
 <p class="note">Está sempre ligada, em 192.168.4.1. É também a palavra-passe para alterar definições e o programa.</p>
</div>
<div class="card"><h2>Outra rede WiFi (opcional)</h2>
 <label class="opt"><input type="checkbox" name="staOn" value="1"><span>Ligar o XIAO também a outra rede<small>Rede da exploração ou o hotspot do telemóvel. A página fica também em http://<b id="hostShow">xiao</b>.local/ ou no IP que o XIAO receber.</small></span></label>
 <label class="field"><span>Nome da rede</span><input type="text" name="staSsid" maxlength="32"></label>
 <div class="row" style="margin:2px 0"><button class="sec" type="button" id="scan">Procurar redes</button><span id="scanMsg" class="note"></span></div>
 <div id="netList" class="nets"></div>
 <label class="field"><span>Palavra-passe dessa rede (em branco: fica a atual; para uma rede nova, em branco = rede aberta)</span><input type="password" name="staPass" maxlength="63"></label>
 <label class="field"><span>Nome do XIAO nessa rede</span><input type="text" name="host" maxlength="24"></label>
</div>
<div class="card"><h2>Guardar</h2>
 <label class="field"><span>Palavra-passe atual da rede do XIAO (para confirmar)</span><input type="password" name="pw" id="pwW" maxlength="63"></label>
 <div class="row"><button class="act" type="submit">Guardar e reiniciar o XIAO</button><span id="setMsg" class="note"></span></div>
 <p class="note">O XIAO reinicia para aplicar. Se mudar o nome ou a palavra-passe da rede do XIAO, volte a ligar o telemóvel à rede nova. Esqueceu a palavra-passe? Pela porta USB do painel, no monitor série: <span class="mono">wifi-reset</span>.</p>
</div>
</form>
</section>

<section id="t-prog" class="hidden">
<div id="fw">
<div class="card"><h2>Programa do XIAO</h2>
 <dl><dt>Versão</dt><dd id="pFw" class="mono">–</dd><dt>Compilado</dt><dd id="pBuilt">–</dd><dt>Tamanho</dt><dd id="pSize">–</dd></dl></div>
<div class="card"><h2>Versão no GitHub</h2>
 <dl><dt>No GitHub</dt><dd id="ghVer">–</dd></dl>
 <div class="row"><button class="act" id="ghInstall" disabled>Instalar a partir do GitHub</button><button class="sec" id="ghCheck">Procurar agora</button><a id="ghLink" href="https://github.com/renatorgr/Boards/tree/main/Firmware" target="_blank" rel="noopener">Ver no GitHub</a></div>
 <p class="note">Usa a palavra-passe do cartão de baixo. O telemóvel precisa de internet: com o telemóvel só na rede do XIAO, normalmente não tem. Ligue o XIAO à rede da exploração ou ao hotspot do telemóvel e abra a página por essa rede.</p></div>
<div class="card"><h2>Atualizar ou reiniciar</h2>
 <div class="row" style="margin-top:0"><label class="act">Escolher .bin<input type="file" id="binFile" accept=".bin" class="hidden"></label><span id="binName" class="note">nenhum ficheiro</span></div>
 <label class="field"><span>Palavra-passe da rede do XIAO (para confirmar)</span><input type="password" id="pwP" maxlength="63"></label>
 <div class="row"><button class="act" id="install" disabled>Instalar</button><button class="danger" id="reboot">Reiniciar</button></div>
 <progress id="prog" max="100" value="0" class="hidden"></progress>
 <p id="fwMsg" class="note"></p>
 <p class="note">Se a instalação falhar, o programa atual continua.</p></div>
</div>
</section>
</main>
<script>
"use strict";
const $=id=>document.getElementById(id);
let lastState=null,lastTeensy=null,lastNets=null,tab="status",chipId=-1,chipName="",setLoaded=false,online=false,logAfter=0,xAfter=0,epoch=0,busy=false,waitBoot=false,upBefore=0,lastUp=0;
const fmt=(v,d,u)=>(v===null||v===undefined||Number.isNaN(v))?"–":Number(v).toFixed(d)+(u||"");
function set(id,txt,cls){const e=$(id);if(!e)return;e.textContent=txt;e.classList.remove("ok","warn","bad");if(cls)e.classList.add(cls);}
function pill(id,txt,cls){const e=$(id);e.textContent="";const s=document.createElement("span");s.className="pill "+(cls||"");s.textContent=txt;e.appendChild(s);}
async function getJ(u,o){const r=await fetch(u,Object.assign({cache:"no-store"},o||{}));let j={};try{j=await r.json();}catch(e){}if(!r.ok&&!j.error)j.error="HTTP "+r.status;return j;}
function dur(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?h+" h "+m+" min":m?m+" min "+(s%60)+" s":s+" s";}
// ---- language: Portuguese (the page as written) or English (this table). The choice stays in this
// browser; without one, the phone's language decides. The messages are always in English.
const EN={"Recetor GPS":"GPS receiver","ligação ao XIAO":"link to the XIAO","XIAO · módulo":"XIAO · module","a ligar…":"connecting…","Estado":"Status","Mensagens":"Messages","Programa":"Program","Módulo (Teensy)":"Module (Teensy)","Satélites":"Satellites","Idade das correções":"Correction age","Velocidade":"Speed","Heading das antenas":"Antenna heading","Ângulo da roda (WAS)":"Wheel angle (WAS)","Ângulo pedido":"Requested angle","Cabo Ethernet":"Ethernet cable","Correções RTCM":"RTCM corrections","A página completa do módulo abre só pelo cabo Ethernet:":"The module's full page opens only over the Ethernet cable:","→ módulo":"→ module","Autosteer pedido":"Autosteer requested","Dados de direção (PGN 254)":"Steering data (PGN 254)","Elevador hidráulico":"Hydraulic lift","Hello do programa":"Program hello","Secções 1–16":"Sections 1–16","Versão":"Version","Ligado há":"Up for","Motivo do arranque":"Start-up reason","WiFi do XIAO":"XIAO WiFi","Dispositivos ligados":"Devices connected","Outra rede":"Other network","IP na outra rede":"IP on the other network","Ligação ao Teensy":"Link to the Teensy","Recebido / enviado":"Received / sent","Erros de checksum":"Checksum errors","Memória livre":"Free memory","As horas são o tempo desde que o Teensy (ou o XIAO) ligou.":"Times are counted from when the Teensy (or the XIAO) was switched on.","Rede WiFi do XIAO":"XIAO WiFi network","Nome da rede":"Network name","Nova palavra-passe (só para a mudar: 8 a 63 caracteres; em branco fica a atual)":"New password (only to change it: 8 to 63 characters; blank keeps the current one)","Está sempre ligada, em 192.168.4.1. É também a palavra-passe para alterar definições e o programa.":"Always on, at 192.168.4.1. Its password is also the one for changing the settings and the program.","Outra rede WiFi (opcional)":"Other WiFi network (optional)","Ligar o XIAO também a outra rede":"Also connect the XIAO to another network","Rede da exploração ou o hotspot do telemóvel. A página fica também em http://":"The farm network or the phone's hotspot. The page is then also at http://",".local/ ou no IP que o XIAO receber.":".local/ or at the IP the XIAO gets.","Procurar redes":"Search for networks","Palavra-passe dessa rede (em branco: fica a atual; para uma rede nova, em branco = rede aberta)":"That network's password (blank: keeps the current one; for a new network, blank = open network)","Nome do XIAO nessa rede":"XIAO name on that network","Guardar":"Save","Palavra-passe atual da rede do XIAO (para confirmar)":"Current password of the XIAO network (to confirm)","Guardar e reiniciar o XIAO":"Save and restart the XIAO","O XIAO reinicia para aplicar. Se mudar o nome ou a palavra-passe da rede do XIAO, volte a ligar o telemóvel à rede nova. Esqueceu a palavra-passe? Pela porta USB do painel, no monitor série:":"The XIAO restarts to apply them. If you change the name or the password of the XIAO network, connect the phone to the new network again. Forgot the password? Through the USB port on the panel, in the serial monitor:","Programa do XIAO":"XIAO program","Compilado":"Built","Tamanho":"Size","Atualizar ou reiniciar":"Update or restart","Escolher .bin":"Choose .bin","nenhum ficheiro":"no file","Palavra-passe da rede do XIAO (para confirmar)":"XIAO network password (to confirm)","Instalar":"Install","Reiniciar":"Restart","Se a instalação falhar, o programa atual continua.":"If the installation fails, the current program keeps running.","nunca":"never","agora":"now","sem dados do módulo":"no data from the module","sem dados do recetor":"no data from the receiver","sem HPR":"no HPR","HPR atrasado":"HPR late","UM982 sem solução":"UM982 without a solution","HPR vazio":"HPR empty","sem heading":"no heading","sem dados":"no data","não detetado":"not detected","a conduzir":"steering","desligado":"off","ligado":"on","sem comunicação":"no communication","dois programas":"two programs","a comunicar":"talking","a chegar":"arriving","paradas":"stopped","nada recebido":"nothing received","subir":"up","descer":"down","não":"no","canal":"channel","desligada":"off","a ligar":"connecting","corrente ligada":"power on","reinício pedido":"restart requested","erro do programa":"program error","programa parado (watchdog)":"program stopped (watchdog)","falha de tensão":"low voltage (brownout)","botão RESET":"RESET button","outro":"other","(guardada)":"(saved)","A palavra-passe tem de ter 8 caracteres ou mais.":"The password must have 8 characters or more.","Guardar e reiniciar o XIAO? O WiFi do XIAO desliga-se uns segundos.":"Save and restart the XIAO? The XIAO WiFi goes off for a few seconds.","Guardado. O XIAO está a reiniciar.":"Saved. The XIAO is restarting.","Erro":"Error","erro":"error","Sem ligação ao XIAO.":"No connection to the XIAO.","forte":"strong","média":"medium","fraca":"weak","aberta":"open","(rede aberta)":"(open network)","a procurar…":"searching…","toque numa para a escolher":"tap one to choose it","nenhuma rede encontrada":"no network found","sem resposta":"no answer","Este ficheiro não é um programa para o XIAO (.bin errado).":"This file is not a program for the XIAO (wrong .bin).","Escolha um ficheiro .bin.":"Choose a .bin file.","Este é o ficheiro errado: use o ….ino.bin.":"This is the wrong file: use the ….ino.bin.","A enviar…":"Sending…","Instalado. O XIAO está a reiniciar…":"Installed. The XIAO is restarting…","A ligação falhou. O programa atual continua.":"The connection failed. The current program keeps running.","Reiniciar o XIAO?":"Restart the XIAO?","A reiniciar…":"Restarting…","Versão no GitHub":"Version on GitHub","No GitHub":"On GitHub","Instalar a partir do GitHub":"Install from GitHub","Procurar agora":"Check now","Ver no GitHub":"See on GitHub","Usa a palavra-passe do cartão de baixo. O telemóvel precisa de internet: com o telemóvel só na rede do XIAO, normalmente não tem. Ligue o XIAO à rede da exploração ou ao hotspot do telemóvel e abra a página por essa rede.":"It uses the password of the card below. The phone needs internet: on the XIAO's own network alone it usually has none. Connect the XIAO to the farm network or to the phone's hotspot and open the page over that network.","sem resposta do GitHub (este aparelho tem internet?)":"no answer from GitHub (does this device have internet?)","nenhuma versão no GitHub":"no version on GitHub","A descarregar do GitHub…":"Downloading from GitHub…","Escreva a palavra-passe da rede do XIAO.":"Type the XIAO network password.","Português / English":"Português / English"};
const REV={};for(const k in EN)REV[EN[k]]=k;
let LANG="";try{LANG=localStorage.getItem("lang")||"";}catch(e){}
if(LANG!=="pt"&&LANG!=="en")LANG=/^pt/i.test(navigator.language||"")?"pt":"en";
const T=s=>LANG==="en"&&EN[s]!==undefined?EN[s]:s;          // a fixed text
const L=(pt,en)=>LANG==="en"?en:pt;                           // a text with numbers or names in it
const VPT={"no fix":"sem fix","RTK fixed":"RTK fixo","other":"outro","no data":"sem dados"};   // words the Teensy sends in English
const V=v=>LANG==="pt"&&VPT[v]?VPT[v]:v;
function ptOf(k){return EN[k]!==undefined?k:REV[k];}
function applyLang(){
 document.documentElement.lang=LANG;document.title=L("XIAO · módulo do trator","XIAO · tractor module");
 const w=document.createTreeWalker(document.body,NodeFilter.SHOW_TEXT,{acceptNode:n=>n.parentElement.closest("script,style,.mono")?NodeFilter.FILTER_REJECT:NodeFilter.FILTER_ACCEPT});
 // each text remembers its Portuguese the first time it is seen (two Portuguese texts can have the same English)
 for(let n=w.nextNode();n;n=w.nextNode()){if(n.k===undefined){n.raw=n.nodeValue;n.was=n.raw.trim();n.k=n.was&&ptOf(n.was)||"";}
  if(n.k)n.nodeValue=n.raw.replace(n.was,T(n.k));}
 for(const e of document.querySelectorAll("[title],[placeholder]"))for(const a of ["title","placeholder"]){const v=e.getAttribute(a),pt=v&&ptOf(v);if(pt)e.setAttribute(a,T(pt));}
 $("lang").textContent=LANG==="en"?"PT":"EN";
 if(lastState)renderState(lastState);
 if(lastTeensy)renderModule(lastTeensy);
 if(lastNets){showNets(lastNets);netsMsg(lastNets);}
}
function ago(s){return s<0?T("nunca"):s<2?T("agora"):L("há "+dur(s),dur(s)+" ago");}
const PT={"wrong password":"palavra-passe errada","network name":"nome da rede: 1 a 32 caracteres","password: 8":"palavra-passe: 8 a 63 caracteres",
 "other network password":"palavra-passe da outra rede: 8 a 63 caracteres","other network name":"nome da outra rede: 1 a 32 caracteres",
 "other network: no name":"falta o nome da outra rede","name on the network":"nome na rede: letras, algarismos e '-', até 24","no file":"nenhum ficheiro",
 "upload aborted":"o envio foi interrompido","other chip ESP32-C3":"este ficheiro é para o XIAO ESP32-C3","other chip ESP32-S3":"este ficheiro é para o XIAO ESP32-S3","other chip":"este ficheiro é para outro tipo de ESP32","Magic":"não é um programa para o ESP32 (.bin errado)","Bad Size":"ficheiro com tamanho errado"};
function ptErr(e){e=String(e||"");if(LANG==="en")return e;for(const k in PT)if(e.indexOf(k)>=0)return PT[k];return e;}
const WHY={1:"sem HPR",2:"HPR atrasado",3:"UM982 sem solução",4:"HPR vazio"};

function renderModule(t){
 lastTeensy=t;
 if(!t||!t.s){set("mAge",T("sem dados do módulo"),"warn");set("mFix","–");return;}
 const s=t.s,g=s.gnss||{},d=s.dual||{},m=s.tm||{},st=s.steer||{},n=s.net||{};
 set("mAge",ago(t.ago>3?t.ago:0),t.ago>3?"warn":"");
 const fixCls=g.fix===4?"ok":(g.fix===5||g.fix===2)?"warn":"bad";
 set("mFix",g.gga?V(g.fixName):T("sem dados do recetor"),fixCls);
 set("mSats",g.sats||"–");set("mHdop",g.hdop||"–");set("mAge2",g.age?g.age+" s":"–");
 set("mKmh",fmt(g.kmh,1," km/h"));
 if(d.ok)set("mDual",Number(d.heading).toFixed(2)+"° (QF "+d.qf+")","ok");else set("mDual",d.hpr?T("sem heading")+" ("+(WHY[d.why]?T(WHY[d.why]):"?")+")":T("sem HPR"),d.hpr?"bad":"warn");
 set("mTm",(m.type?m.type+" · ":"")+(m.ok?"roll "+fmt(m.roll,1,"°")+" · "+fmt(m.out,1,"°"):T(m.seen?"sem dados":"não detetado")),m.ok?"":"bad");
 set("mRx",g.model||"–");   // the receiver as it answered the Teensy
 set("mWas",fmt(st.was,1,"°"));set("mSet",fmt(st.set,1,"°"));
 pill("mAct",T(st.active?"a conduzir":"desligado"),st.active?"ok":"");
 pill("mLink",n.link?L("ligado","connected"):L("desligado","unplugged"),n.link?"ok":"bad");
 const hn=n.host==="AgOpenWeb"||n.host==="AgOpenGPS"?n.host:"";   // the program, from its hello (Teensy)
 pill("mAog",!n.aog?T("sem comunicação"):n.hostBoth?T("dois programas"):hn?hn+" · "+T("a comunicar"):T("a comunicar"),!n.aog||n.hostBoth?"warn":"ok");
 $("aName").textContent=hn||T("Programa");
 pill("mRtcm",T(n.rtcm?"a chegar":"paradas"),n.rtcm?"ok":"warn");
 const fw=s.fw||"–",sp=fw.lastIndexOf(" ");set("mFw",(sp>0?fw.slice(0,sp):fw).replace(/_/g,"_\u200b"));   // "name version": the name has the version already; may break after "_"
}
function renderState(x){
 lastState=x;lastUp=x.up;if(x.chipId!==undefined){chipId=x.chipId;chipName=x.chip;}
 $("hdr").textContent=x.fw+" · "+(x.wifi.st==="ok"?x.wifi.ip:"192.168.4.1");
 const a=x.aog;
 set("aAge",a.ago254<0?T("nada recebido"):ago(a.ago254>2?a.ago254:0),a.ago254<0||a.ago254>2?"warn":"");
 set("aKmh",a.ago254<0?"–":fmt(a.kmh,1," km/h"));
 pill("aSt",a.ago254<0?"–":T(a.status===1?"ligado":"desligado"),a.status===1?"ok":"");
 set("aAng",a.ago254<0?"–":fmt(a.angle,1,"°"));
 set("aRate",a.ago254<0?"–":a.rate254+"/s");
 set("aHyd",a.ago239<0?"–":T(a.hyd===2?"subir":a.hyd===1?"descer":"desligado"));
 set("aTram",a.ago239<0?"–":a.tram?String(a.tram):T("não"));
 set("aUt",a.ago239<0?"–":a.uturn?String(a.uturn):T("não"));
 set("aHello",ago(a.helloAgo));
 const box=$("aSecs");if(!box.childElementCount)for(let i=1;i<=16;i++){const s=document.createElement("span");s.textContent=i;box.appendChild(s);}
 for(let i=0;i<16;i++)box.children[i].classList.toggle("on",a.ago254>=0&&!!(a.sec&(1<<i)));
 set("xFw",x.fw);set("xUp",dur(x.up));set("xRst",T(x.reset),x.resetBad?"warn":"");
 set("xAp",x.wifi.ap+" · "+T("canal")+" "+x.wifi.ch);set("xCl",String(x.wifi.apCl));
 const w=x.wifi;
 set("xSta",!w.staOn?T("desligada"):w.st==="ok"?w.sta+" · "+w.rssi+" dBm":w.sta+" · "+T("a ligar")+"…",w.staOn&&w.st!=="ok"?"warn":"");
 set("xIp",w.staOn&&w.st==="ok"?w.ip:"–");
 pill("xLink",T(x.link.talking?"a comunicar":"sem dados"),x.link.talking?"ok":"bad");
 set("xKB",x.link.rxKB.toFixed(1)+" / "+x.link.txKB.toFixed(1)+" kB");
 set("xCrc",String(x.link.crcErr),x.link.crcErr?"warn":"");
 set("xHeap",Math.round(x.heap/1024)+" kB");
 renderGh();
 set("pFw",x.fw);set("pBuilt",x.built);set("pSize",Math.round(x.sketch/1024)+" kB ("+L("máx. ","max. ")+Math.round(x.sketchFree/1024)+" kB)");
}
function setOnline(v){online=v;$("dot").classList.toggle("on",v);}
async function pollState(){
 try{const x=await getJ("/api/state");if(x.error)throw 0;
  if(waitBoot&&x.up<upBefore){waitBoot=false;busy=false;$("prog").classList.add("hidden");fwMsg(L("Atualizado. A correr: "+x.fw+", compilado "+x.built+".","Updated. Running: "+x.fw+", built "+x.built+"."),"ok");}
  setOnline(true);renderState(x);
  if(tab==="status"){const t=await getJ("/api/teensy");renderModule(t);}
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
 f.apPass.value="";f.staPass.value="";f.staPass.placeholder=s.staPassSet?T("(guardada)"):"";}catch(e){}}
$("fs").addEventListener("submit",async e=>{e.preventDefault();const f=$("fs"),m=$("setMsg");
 if(f.apPass.value&&f.apPass.value.length<8){m.textContent=T("A palavra-passe tem de ter 8 caracteres ou mais.");m.className="note bad";return;}
 if(!confirm(T("Guardar e reiniciar o XIAO? O WiFi do XIAO desliga-se uns segundos.")))return;
 const p=new URLSearchParams();for(const k of ["apSsid","apPass","staSsid","staPass","host","pw"])p.set(k,f[k].value);p.set("staOn",f.staOn.checked?"1":"0");
 try{const j=await getJ("/api/settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:p.toString()});
  if(j.ok){setLoaded=false;m.textContent=T("Guardado. O XIAO está a reiniciar.");m.className="note ok";}else{m.textContent=T("Erro")+": "+ptErr(j.error);m.className="note bad";}
 }catch(e){m.textContent=T("Sem ligação ao XIAO.");m.className="note bad";}});
// the networks found, one per line, strongest first; a tap chooses one
function sig(r){return T(r>=-60?"forte":r>=-72?"média":"fraca");}
function showNets(list){lastNets=list;const box=$("netList"),f=$("fs");box.textContent="";
 for(const n of list){const b=document.createElement("button");b.type="button";b.classList.toggle("sel",n.s===f.staSsid.value);
  const a=document.createElement("span");a.className="n";a.textContent=n.s;b.appendChild(a);
  const r=document.createElement("span");r.className="r";r.textContent=n.r+" dBm · "+sig(n.r)+(n.open?" · "+T("aberta"):"");b.appendChild(r);
  b.addEventListener("click",()=>{const same=f.staSsid.value===n.s;f.staSsid.value=n.s;f.staOn.checked=true;
   for(const o of box.children)o.classList.toggle("sel",o===b);
   if(!same){f.staPass.value="";f.staPass.placeholder=n.open?T("(rede aberta)"):"";if(!n.open)f.staPass.focus();}});
  box.appendChild(b);}}
function netsMsg(list){const n=list.length;$("scanMsg").textContent=n?L(n===1?"1 rede":n+" redes",n===1?"1 network":n+" networks")+" · "+T("toque numa para a escolher"):T("nenhuma rede encontrada");}
$("scan").addEventListener("click",async()=>{const m=$("scanMsg");m.textContent=T("a procurar…");m.className="note";$("netList").textContent="";
 for(let i=0;i<20;i++){try{const j=await getJ("/api/scan");if(j.list){j.list.sort((a,b)=>b.r-a.r);showNets(j.list);
   netsMsg(j.list);return;}}catch(e){}
  await new Promise(r=>setTimeout(r,800));}
 m.textContent=T("sem resposta");});
// the password typed once is used in both tabs (only in this page, never saved)
for(const id of ["pwW","pwP"])$(id).addEventListener("input",e=>{for(const o of ["pwW","pwP"])if(o!==id)$(o).value=e.target.value;});

// ---- program
function fwMsg(t,c){const m=$("fwMsg");m.textContent=t;m.className="note "+(c||"");}
// the program header says which chip it is for (byte 12-13): the other XIAO's program is refused here already
const CHIPS={5:"ESP32-C3",9:"ESP32-S3"};
$("binFile").addEventListener("change",async()=>{const f=$("binFile").files[0];$("binName").textContent=f?f.name+" ("+Math.round(f.size/1024)+" kB)":T("nenhum ficheiro");$("install").disabled=!f||busy;fwMsg("");
 if(!f||chipId<0)return;
 try{const h=new Uint8Array(await f.slice(0,16).arrayBuffer());
  if(h[0]!==0xE9){fwMsg(T("Este ficheiro não é um programa para o XIAO (.bin errado)."),"bad");$("install").disabled=true;return;}
  const id=h[12]|h[13]<<8;
  if(id!==chipId){fwMsg(L("Este ficheiro é para o XIAO "+(CHIPS[id]||"de outro tipo")+"; este XIAO é "+chipName+".","This file is for "+(CHIPS[id]?"the XIAO "+CHIPS[id]:"another kind of XIAO")+"; this XIAO is "+chipName+"."),"bad");$("install").disabled=true;}
 }catch(e){}});
$("install").addEventListener("click",()=>{
 const f=$("binFile").files[0];if(!f||busy)return;
 if(!f.name.toLowerCase().endsWith(".bin")){fwMsg(T("Escolha um ficheiro .bin."),"bad");return;}
 if(/merged|bootloader|partitions/i.test(f.name)){fwMsg(T("Este é o ficheiro errado: use o ….ino.bin."),"bad");return;}
 if(!confirm(L("Instalar "+f.name+" no XIAO?","Install "+f.name+" on the XIAO?")))return;
 busy=true;$("install").disabled=true;$("ghInstall").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;
 sendBin(f,f.name,0);});
// The .bin to the XIAO (a chosen file, or one downloaded from GitHub); progress from p0 to 95 %
function sendBin(blob,name,p0){
 const p=$("prog");fwMsg(T("A enviar…"));
 const fd=new FormData();fd.append("file",blob,name);
 const x=new XMLHttpRequest();x.open("POST","/update?pw="+encodeURIComponent($("pwP").value));
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=p0+Math.round(e.loaded/e.total*(95-p0));};
 const fail=t=>{busy=false;$("install").disabled=!$("binFile").files[0];renderGh();p.classList.add("hidden");fwMsg(t,"bad");};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;upBefore=lastUp;waitBoot=true;fwMsg(T("Instalado. O XIAO está a reiniciar…"),"warn");}
  else fail(L("Não instalado: ","Not installed: ")+ptErr(j.error||(T("erro")+" "+x.status))+L(". O programa atual continua.",". The current program keeps running."));};
 x.onerror=()=>fail(T("A ligação falhou. O programa atual continua."));
 x.send(fd);}

// ---- a newer version on GitHub (renatorgr/Boards, folder Firmware, one folder per version:
// vX.Y_UM982_X20P_F9P_TM171_WEB_XIAO, with vX.Y_XIAO_ESP32_bin/vX.Y_XIAO_ESP32C3.bin and ..S3.bin in it).
// Asked by this browser when the page opens, every 6 hours, and with "Procurar agora".
const GH_API="https://api.github.com/repos/renatorgr/Boards/contents/Firmware",GH_WEB="https://github.com/renatorgr/Boards/tree/main/Firmware";
const GH_RX=/^v(\d+(?:\.\d+)*)_UM982_X20P_F9P_TM171_WEB_XIAO$/;
let gh=null,ghState="",ghAt=0;
function verCmp(a,b){const x=String(a).split(".").map(Number),y=String(b).split(".").map(Number);for(let i=0;i<Math.max(x.length,y.length);i++){const d=(x[i]||0)-(y[i]||0);if(d)return d;}return 0;}
function runningVer(){const m=lastState&&/^v(\d+(?:\.\d+)*)_/.exec(lastState.fw);return m?m[1]:"";}
async function ghList(path){const r=await fetch(GH_API+(path?"/"+path:"")+"?ref=main");/* a simple request: no CORS preflight */if(!r.ok)throw new Error("HTTP "+r.status);return r.json();}
async function ghCheck(){
 if(ghState==="busy")return;
 ghState="busy";renderGh();
 try{const l=await ghList("");let best=null;
  for(const e of l)if(e.type==="dir"){const m=GH_RX.exec(e.name);if(m&&(!best||verCmp(m[1],best.ver)>0))best={ver:m[1],dir:e.name};}
  gh=best;ghState=best?"ok":"none";}
 catch(e){ghState="fail";}
 ghAt=Date.now();renderGh();
}
function ghNewer(){const r=runningVer();return !!(gh&&r&&verCmp(gh.ver,r)>0);}
function renderGh(){
 const r=runningVer(),nw=ghNewer();
 set("ghVer",ghState==="busy"?T("a procurar…"):ghState==="fail"?T("sem resposta do GitHub (este aparelho tem internet?)"):ghState==="none"?T("nenhuma versão no GitHub"):
  !gh?"–":gh.ver+(nw?L(" — versão nova (instalada: "+r+")"," — new version (installed: "+r+")"):verCmp(gh.ver,r)===0?L(" — atualizado, é a versão instalada"," — up to date, latest version installed"):L(" — a instalada ("+r+") é mais recente"," — the installed version ("+r+") is newer")),
  nw?"ok":ghState==="fail"?"warn":"");
 $("ghInstall").disabled=!nw||busy;
 $("ghLink").href=gh?GH_WEB+"/"+gh.dir:GH_WEB;
 const n=$("ghNote");n.textContent=nw?L("Há uma versão nova no GitHub: "+gh.ver+" (instalada: "+r+"). Instale-a no separador Programa.","There is a new version on GitHub: "+gh.ver+" (installed: "+r+"). Install it in the Program tab."):"";
 n.classList.toggle("hidden",!nw);
}
async function ghInstall(){
 if(!ghNewer()||busy)return;
 const chip=chipName==="ESP32-S3"?"S3":chipName==="ESP32-C3"?"C3":"";
 if(!chip){fwMsg(L("Não se sabe se este XIAO é C3 ou S3.","It is not known whether this XIAO is a C3 or an S3."),"bad");return;}
 if(!$("pwP").value){fwMsg(T("Escreva a palavra-passe da rede do XIAO."),"bad");$("pwP").focus();return;}
 const bdir="v"+gh.ver+"_XIAO_ESP32_bin",name="v"+gh.ver+"_XIAO_ESP32"+chip+".bin";
 if(!confirm(L("Instalar a versão "+gh.ver+" do GitHub ("+name+") no XIAO?","Install version "+gh.ver+" from GitHub ("+name+") on the XIAO?")))return;
 busy=true;$("install").disabled=true;$("ghInstall").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;
 fwMsg(T("A descarregar do GitHub…"));
 try{
  const f=(await ghList(gh.dir+"/"+bdir)).find(e=>e.type==="file"&&e.name===name);
  if(!f)throw new Error(L("o ficheiro "+name+" não está no GitHub","the file "+name+" is not on GitHub"));
  const res=await fetch(f.download_url);if(!res.ok)throw new Error("GitHub: HTTP "+res.status);
  const rd=res.body.getReader(),parts=[];let got=0;
  for(;;){const {done,value}=await rd.read();if(done)break;parts.push(value);got+=value.length;p.value=Math.round(got/f.size*45);}
  const blob=new Blob(parts);
  if(blob.size!==f.size)throw new Error(L("ficheiro incompleto ("+blob.size+" de "+f.size+" bytes)","file incomplete ("+blob.size+" of "+f.size+" bytes)"));
  const h=new Uint8Array(await blob.slice(0,16).arrayBuffer());
  if(h[0]!==0xE9||(h[12]|h[13]<<8)!==chipId)throw new Error(T("Este ficheiro não é um programa para o XIAO (.bin errado)."));
  sendBin(blob,name,45);
 }catch(e){busy=false;$("install").disabled=!$("binFile").files[0];p.classList.add("hidden");renderGh();
  fwMsg(L("Não descarregado: ","Not downloaded: ")+e.message+L(". O programa atual continua.",". The current program keeps running."),"bad");}
}
$("ghCheck").addEventListener("click",ghCheck);
$("ghInstall").addEventListener("click",ghInstall);
$("reboot").addEventListener("click",async()=>{if(busy||!confirm(T("Reiniciar o XIAO?")))return;
 try{const j=await getJ("/api/restart",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"pw="+encodeURIComponent($("pwP").value)});
  fwMsg(j.ok?T("A reiniciar…"):T("Erro")+": "+ptErr(j.error),j.ok?"warn":"bad");}catch(e){fwMsg(T("Sem ligação ao XIAO."),"bad");}});

// ---- one request at a time, every second
let tickN=0;
async function tick(){tickN++;if(busy&&!waitBoot)return;await pollState();if(!online)return;
 if(lastState&&!busy&&(ghAt===0||Date.now()-ghAt>6*3600e3))ghCheck();   // GitHub, not waited for
 if(tab==="log"||tickN%5===0)await pollLog();}
$("lang").addEventListener("click",()=>{LANG=LANG==="en"?"pt":"en";try{localStorage.setItem("lang",LANG);}catch(e){}applyLang();});
applyLang();
(async function loop(){await tick();setTimeout(loop,1000);})();
</script>
</body>
</html>
)XIAOPAGE";
