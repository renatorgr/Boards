/* zWebPage.h - the module's web page (Portuguese / English).
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 Micro board
   (renatorgr, github.com/renatorgr/Boards).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zWebPage.h
// The module's web page (served by zWeb.ino): one self-contained HTML file, no internet needed.
// Kept in flash (PROGMEM). Text in Portuguese, and in English with the EN button (v7.4; the
// [status] messages stay in English). The English is the EN table in the script.
#ifndef ZWEBPAGE_H
#define ZWEBPAGE_H

const char WEB_PAGE[] PROGMEM = R"WEBPAGE(<!doctype html>
<html lang="pt">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark">
<title>UM982 · TM171</title>
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
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:12px}
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
.box{background:var(--soft);border-radius:8px;padding:8px 10px;margin-top:8px}
#log,#umOut{height:62vh;overflow:auto;background:var(--card);border:1px solid var(--line);border-radius:12px;padding:8px 10px}
#umOut{height:40vh}
.ll{display:flex;gap:10px;padding:2px 0;border-bottom:1px dashed var(--line)}
.ll .t{color:var(--mute);flex:none;font-variant-numeric:tabular-nums;width:86px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}
button.act,label.act{border:0;background:var(--acc);color:var(--accInk);border-radius:8px;padding:9px 16px;font:inherit;font-weight:600;cursor:pointer}
button.sec{border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button.danger{border:1px solid var(--bad);background:transparent;color:var(--bad);border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}
button:disabled{opacity:.45;cursor:default}
input[type=text]{font:inherit;font-family:var(--mono);padding:9px 10px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--ink);flex:1;min-width:180px}
#fs .card{margin:0 0 12px}
.opt{display:flex;gap:10px;align-items:flex-start;padding:5px 0}
.opt input{margin-top:4px}
.opt small{display:block;color:var(--mute)}
select{font:inherit;padding:6px 8px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink)}
progress{width:100%;height:14px}
.hidden{display:none!important}
.lang{margin-left:auto;flex:none;border:1px solid var(--line);background:var(--soft);color:var(--ink);border-radius:999px;padding:3px 11px;font:inherit;font-size:13px;font-weight:650;cursor:pointer}
@media (max-width:520px){header .sub{display:none}}
</style>
</head>
<body>
<header><span class="dot" id="dot" title="ligação ao módulo"></span><h1>UM982 · TM171</h1><span class="sub" id="hdr">a ligar…</span><button class="lang" id="lang" type="button" title="Português / English">EN</button></header>
<nav id="nav"><button data-t="status" class="on">Estado</button><button data-t="log">Mensagens</button><button data-t="settings">Definições</button><button data-t="um982">UM982</button><button data-t="firmware">Firmware</button></nav>
<main>

<section id="t-status">
<div class="grid">
 <div class="card"><h2>GNSS</h2>
  <div class="big" id="fixName">–</div>
  <dl style="margin-top:6px">
   <dt>Satélites</dt><dd id="sats">–</dd><dt>HDOP</dt><dd id="hdop">–</dd>
   <dt>Idade das correções</dt><dd id="age">–</dd><dt>Velocidade</dt><dd id="kmh">–</dd>
   <dt>Latitude</dt><dd id="lat" class="nw">–</dd><dt>Longitude</dt><dd id="lon" class="nw">–</dd><dt>Altitude</dt><dd id="alt" class="nw">–</dd><dt>Erros de checksum</dt><dd id="crc">–</dd>
  </dl></div>
 <div class="card"><h2>Heading das duas antenas</h2>
  <div class="big" id="dualHdg">–</div>
  <dl style="margin-top:6px"><dt>Estado</dt><dd id="dualState">–</dd><dt>Qualidade (QF)</dt><dd id="qf">–</dd><dt>Roll das antenas</dt><dd id="antRoll">–</dd></dl>
  <div class="box mono" id="hprRaw">–</div></div>
 <div class="card"><h2>TM171</h2>
  <div class="big" id="tmHdg">–</div>
  <dl style="margin-top:6px"><dt>Estado</dt><dd id="tmState">–</dd><dt>Roll</dt><dd id="roll">–</dd><dt>Pitch</dt><dd id="pitch">–</dd>
   <dt>Yaw em bruto</dt><dd id="yaw">–</dd><dt>Alinhamento</dt><dd id="align">–</dd><dt>Sentido do yaw</dt><dd id="sign">–</dd></dl></div>
 <div class="card"><h2>Direção</h2>
  <div class="big" id="was">–</div>
  <dl style="margin-top:6px"><dt>Ângulo pedido</dt><dd id="setp">–</dd><dt>PWM</dt><dd id="pwm">–</dd><dt>Autosteer</dt><dd id="active">–</dd>
   <dt>Interruptor</dt><dd id="sw">–</dd><dt>Motor</dt><dd id="motor">–</dd><dt>Velocidade (programa)</dt><dd id="aogKmh">–</dd></dl></div>
 <div class="card"><h2>Ligações</h2>
  <dl><dt>Cabo Ethernet</dt><dd id="link" class="nw">–</dd><dt>Programa</dt><dd id="aog" class="nw">–</dd><dt>IP do programa</dt><dd id="aogIp" class="nw">–</dd>
   <dt>Correções RTCM</dt><dd id="rtcm" class="nw">–</dd><dt>RTCM recebido</dt><dd id="rtcmKB" class="nw">–</dd>
   <dt>IP do módulo</dt><dd id="ip">–</dd><dt>Ligado há</dt><dd id="up">–</dd></dl>
  <p class="note">O recetor envia (últimos 5 s):</p><div class="box mono" id="rx">–</div></div>
 <div class="card"><h2>Frases enviadas</h2>
  <dl><dt>Modo</dt><dd id="mode" class="nw">–</dd><dt>Escolha</dt><dd id="modeSel">–</dd><dt>Heading enviado</dt><dd id="cSrc">–</dd><dt>No AgOpenGPS</dt><dd id="aogSrc">–</dd><dt>Épocas enviadas</dt><dd id="epochs">–</dd></dl>
  <div class="box mono" id="sent">–</div></div>
 <div class="card"><h2>XIAO ESP32 (Serial2)</h2>
  <div class="big" id="xState">–</div>
  <dl style="margin-top:6px"><dt>Último hello</dt><dd id="xHello" class="nw">–</dd>
   <dt>Pacotes para o XIAO</dt><dd id="xTo">–</dd><dt>PGN do XIAO</dt><dd id="xFrom">–</dd><dt>Último PGN do XIAO</dt><dd id="xLast">–</dd>
   <dt>Erros de checksum</dt><dd id="xCrc">–</dd><dt>Perdidos</dt><dd id="xDrop">–</dd><dt>Recebido / enviado</dt><dd id="xKB" class="nw">–</dd>
   <dt>Versão do XIAO</dt><dd id="xProg">–</dd><dt>WiFi do XIAO</dt><dd id="xAp">–</dd><dt>Rede WiFi</dt><dd id="xSta">–</dd><dt>IP na rede WiFi</dt><dd id="xIp">–</dd></dl>
  </div>
</div>
</section>

<section id="t-log" class="hidden">
<div class="row"><button class="sec" id="logPause">Pausar</button><button class="sec" id="logClear">Limpar ecrã</button><span class="note">Mensagens [status] do firmware, só quando algo muda. O módulo guarda as últimas 60.</span></div>
<div id="log" class="mono"></div>
</section>

<section id="t-settings" class="hidden">
<form id="fs">
<div class="card"><h2>Modo de funcionamento</h2>
 <label class="opt"><input type="radio" name="outputMode" value="2"><span>Automático<small>Vê qual programa está ligado, pelo «hello» que ele envia a cada segundo, e envia as frases desse programa. Até lá, as do AgOpenGPS, que o AgOpenWeb também lê.</small></span></label>
 <label class="opt"><input type="radio" name="outputMode" value="0"><span>AgOpenWeb<small>$GNGGA, $GNVTG, $GNTHS (heading das antenas) e $INHPR (TM171) em cada época.</small></span></label>
 <label class="opt"><input type="radio" name="outputMode" value="1"><span>AgOpenGPS<small>$PAOGI (UM982) ou $PANDA (X20P / F9P) em cada época.</small></span></label>
</div>
<div class="card"><h2>Heading das duas antenas (UM982)</h2>
 <label class="opt"><input type="checkbox" name="hprAcceptFloat" value="1"><span>Aceitar heading em float (QF 5)<small>Desligado: só com QF 4 (fixo).</small></span></label>
 <div class="opt"><span style="flex:1">Espera máxima pelo VTG e HPR depois do GGA<small>Predefinição 60 ms. Mais tempo atrasa a época; menos pode perder o HPR.</small></span><select name="epochWaitMs"><option>20</option><option>30</option><option>40</option><option>50</option><option>60</option><option>70</option><option>80</option><option>100</option></select></div>
</div>
<div class="card"><h2>TM171</h2>
 <label class="opt"><input type="checkbox" name="tm171SwapRollPitch" value="1"><span>Trocar roll e pitch<small>TM171 rodado 90° na placa. O "Use Y axis" do AgOpenGPS também troca.</small></span></label>
 <div class="opt"><span style="flex:1">Sentido do yaw<small>Automático: aprende nas curvas, com o heading das antenas ou, sem ele, com o rumo do GPS.</small></span><select name="tm171YawSign"><option value="0">automático</option><option value="1">+1</option><option value="-1">−1</option></select></div>
</div>
<div class="card"><h2>XIAO ESP32 (encaixe GPS2)</h2>
 <label class="opt"><input type="checkbox" name="xiaoOn" value="1"><span>Ponte AgOpenWeb ↔ XIAO ligada<small>Quando o XIAO diz «ESP32-hello», os pacotes do AgOpenWeb (UDP 8888) seguem para ele e os PGN dele seguem para o AgOpenWeb (UDP 9999), como no firmware v26. Desligada: o Teensy ignora o XIAO. Sem XIAO no encaixe, ligada ou desligada é igual.</small></span></label>
</div>
<div class="row"><button class="act" type="submit">Guardar</button><button class="sec" type="button" id="defaults">Repor predefinições</button><span id="setMsg" class="note"></span></div>
<p class="note">Ficam guardadas no Teensy (EEPROM) e aplicam-se logo, sem reiniciar. As definições de direção continuam a vir do AgOpenWeb / AgOpenGPS.</p>
</form>
</section>

<section id="t-um982" class="hidden">
<div class="card">
 <h2>Comando para o UM982</h2>
 <div class="row"><input type="text" id="umCmd" placeholder="ex.: UNILOGLIST" autocomplete="off" autocapitalize="characters" spellcheck="false"><button class="act" id="umSend">Enviar</button></div>
 <div class="row"><span class="note">Preencher:</span><button class="sec" data-c="CONFIG">CONFIG</button><button class="sec" data-c="UNILOGLIST">UNILOGLIST</button><button class="sec" data-c="GPHPR 0.1">GPHPR 0.1</button><button class="sec" data-c="SAVECONFIG">SAVECONFIG</button></div>
 <p class="note">O comando vai para a porta do UM982 ligada ao Teensy. <b>CONFIG</b> mostra a configuração e <b>UNILOGLIST</b> as mensagens ativas. Uma alteração só fica guardada depois de <b>SAVECONFIG</b>. Cuidado com FRESET (volta às definições de fábrica, incluindo a velocidade da porta) e com mudar a velocidade da porta: o Teensy deixa de ouvir o recetor.</p>
</div>
<p class="note" style="margin-top:12px">Respostas do UM982 (5 s depois de cada comando):</p>
<div id="umOut" class="mono"></div>
</section>

<section id="t-firmware" class="hidden">
<div class="grid">
<div class="card"><h2>Firmware a correr</h2>
 <dl><dt>Firmware</dt><dd id="fw" class="mono">–</dd><dt>Versão</dt><dd id="fwVer">–</dd><dt>Compilado</dt><dd id="built">–</dd><dt>Placa</dt><dd id="target">–</dd></dl>
 <div class="row"><button class="danger" id="reboot">Reiniciar o Teensy</button></div></div>
<div class="card"><h2>Atualizar o firmware</h2>
 <div class="row"><label class="act">Escolher .hex<input type="file" id="hexFile" accept=".hex" class="hidden"></label><span id="hexName" class="note">nenhum ficheiro</span></div>
 <label class="opt"><input type="checkbox" id="force"><span>Outro firmware<small>Instala mesmo sem esta página de atualização lá dentro (a próxima atualização terá de ser por USB).</small></span></label>
 <div class="row"><button class="act" id="install" disabled>Instalar</button></div>
 <progress id="prog" max="100" value="0" class="hidden"></progress>
 <p id="fwMsg" class="note"></p>
 <p class="note">Só com o autosteer desligado e o trator parado. O ficheiro é verificado inteiro antes de substituir o firmware atual; se algo falhar, o atual continua. Nos segundos finais da instalação <b>não desligue a corrente</b>. As definições guardadas mantêm-se.</p></div>
</div>
</section>
</main>
<script>
"use strict";
const $=id=>document.getElementById(id);
let lastS=null,tab="status",online=false,logAfter=0,logPaused=false,umAfter=0,umUntil=0,lastUp=0,upBefore=0,waitingBoot=false,busy=false;
const fmt=(v,d,u)=>(v===null||v===undefined||Number.isNaN(v))?"–":Number(v).toFixed(d)+(u||"");
function set(id,txt,cls){const e=$(id);if(!e)return;e.textContent=txt;e.classList.remove("ok","warn","bad");if(cls)e.classList.add(cls);}
function pill(id,txt,cls){const e=$(id);e.textContent="";const s=document.createElement("span");s.className="pill "+(cls||"");s.textContent=txt;e.appendChild(s);}
async function getJ(u,o){const r=await fetch(u,Object.assign({cache:"no-store"},o||{}));let j={};try{j=await r.json();}catch(e){}if(!r.ok&&!j.error)j.error="HTTP "+r.status;return j;}
function setOnline(v){online=v;$("dot").classList.toggle("on",v);}
function dur(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?h+" h "+m+" min":m?m+" min "+(s%60)+" s":s+" s";}
const WHY={1:"sem HPR do recetor (uma antena, ou HPR desligado no UM982)",2:"o HPR não chega com o GGA",3:"o UM982 não tem solução de heading das duas antenas",4:"heading vazio no HPR"};

// ---- language: Portuguese (the page as written) or English (this table). The choice stays in this
// browser; without one, the browser's language decides. The [status] messages are always in English.
const EN={"ligação ao módulo":"link to the module","a ligar…":"connecting…","Estado":"Status","Mensagens":"Messages","Definições":"Settings","Satélites":"Satellites","Idade das correções":"Correction age","Velocidade":"Speed","Erros de checksum":"Checksum errors","Heading das duas antenas":"Dual-antenna heading","Qualidade (QF)":"Quality (QF)","Roll das antenas":"Antenna roll","Yaw em bruto":"Raw yaw","Alinhamento":"Alignment","Sentido do yaw":"Yaw direction","Direção":"Steering","Ângulo pedido":"Requested angle","Interruptor":"Switch","Velocidade (programa)":"Speed (program)","Ligações":"Connections","Cabo Ethernet":"Ethernet cable","Programa":"Program","IP do programa":"Program IP","Correções RTCM":"RTCM corrections","RTCM recebido":"RTCM received","IP do módulo":"Module IP","Ligado há":"Up for","O recetor envia (últimos 5 s):":"The receiver sends (last 5 s):","Frases enviadas":"Sentences sent","Modo":"Mode","Escolha":"Choice","Heading enviado":"Heading sent","No AgOpenGPS":"In AgOpenGPS","Épocas enviadas":"Epochs sent","Último hello":"Last hello","Pacotes para o XIAO":"Packets to the XIAO","PGN do XIAO":"PGNs from the XIAO","Último PGN do XIAO":"Last PGN from the XIAO","Perdidos":"Lost","Recebido / enviado":"Received / sent","Versão do XIAO":"XIAO version","WiFi do XIAO":"XIAO WiFi","Rede WiFi":"WiFi network","IP na rede WiFi":"IP on the WiFi network","Pausar":"Pause","Continuar":"Resume","Limpar ecrã":"Clear screen","Mensagens [status] do firmware, só quando algo muda. O módulo guarda as últimas 60.":"[status] messages from the firmware, only when something changes. The module keeps the last 60.","Modo de funcionamento":"Operating mode","Automático":"Automatic","Vê qual programa está ligado, pelo «hello» que ele envia a cada segundo, e envia as frases desse programa. Até lá, as do AgOpenGPS, que o AgOpenWeb também lê.":"Finds out which program is connected, from the «hello» it sends every second, and sends that program's sentences. Until then, the AgOpenGPS ones, which AgOpenWeb reads too.","$GNGGA, $GNVTG, $GNTHS (heading das antenas) e $INHPR (TM171) em cada época.":"$GNGGA, $GNVTG, $GNTHS (antenna heading) and $INHPR (TM171) every epoch.","$PAOGI (UM982) ou $PANDA (X20P / F9P) em cada época.":"$PAOGI (UM982) or $PANDA (X20P / F9P) every epoch.","Heading das duas antenas (UM982)":"Dual-antenna heading (UM982)","Aceitar heading em float (QF 5)":"Accept a float heading (QF 5)","Desligado: só com QF 4 (fixo).":"Off: only with QF 4 (fixed).","Espera máxima pelo VTG e HPR depois do GGA":"Longest wait for the VTG and HPR after the GGA","Predefinição 60 ms. Mais tempo atrasa a época; menos pode perder o HPR.":"Default 60 ms. Longer delays the epoch; shorter may miss the HPR.","Trocar roll e pitch":"Swap roll and pitch","TM171 rodado 90° na placa. O \"Use Y axis\" do AgOpenGPS também troca.":"TM171 turned 90° on the board. The \"Use Y axis\" of AgOpenGPS swaps them too.","Automático: aprende nas curvas, com o heading das antenas ou, sem ele, com o rumo do GPS.":"Automatic: learned in turns, from the antenna heading or, without it, from the GPS course.","automático":"automatic","XIAO ESP32 (encaixe GPS2)":"XIAO ESP32 (GPS2 socket)","Ponte AgOpenWeb ↔ XIAO ligada":"AgOpenWeb ↔ XIAO bridge on","Quando o XIAO diz «ESP32-hello», os pacotes do AgOpenWeb (UDP 8888) seguem para ele e os PGN dele seguem para o AgOpenWeb (UDP 9999), como no firmware v26. Desligada: o Teensy ignora o XIAO. Sem XIAO no encaixe, ligada ou desligada é igual.":"When the XIAO says «ESP32-hello», the AgOpenWeb packets (UDP 8888) go on to it and its PGNs go to AgOpenWeb (UDP 9999), as in firmware v26. Off: the Teensy ignores the XIAO. With no XIAO in the socket, on or off makes no difference.","Guardar":"Save","Repor predefinições":"Restore defaults","Ficam guardadas no Teensy (EEPROM) e aplicam-se logo, sem reiniciar. As definições de direção continuam a vir do AgOpenWeb / AgOpenGPS.":"Saved in the Teensy (EEPROM) and applied at once, without a restart. The steering settings still come from AgOpenWeb / AgOpenGPS.","Comando para o UM982":"Command for the UM982","ex.: UNILOGLIST":"e.g. UNILOGLIST","Enviar":"Send","Preencher:":"Fill in:","O comando vai para a porta do UM982 ligada ao Teensy.":"The command goes to the UM982 port wired to the Teensy.","mostra a configuração e":"shows the configuration and","as mensagens ativas. Uma alteração só fica guardada depois de":"the active messages. A change is kept only after",". Cuidado com FRESET (volta às definições de fábrica, incluindo a velocidade da porta) e com mudar a velocidade da porta: o Teensy deixa de ouvir o recetor.":". Careful with FRESET (back to the factory settings, port speed included) and with changing the port speed: the Teensy stops hearing the receiver.","Respostas do UM982 (5 s depois de cada comando):":"UM982 replies (for 5 s after each command):","Firmware a correr":"Running firmware","Versão":"Version","Compilado":"Built","Placa":"Board","Reiniciar o Teensy":"Restart the Teensy","Atualizar o firmware":"Update the firmware","Escolher .hex":"Choose .hex","nenhum ficheiro":"no file","Outro firmware":"Other firmware","Instala mesmo sem esta página de atualização lá dentro (a próxima atualização terá de ser por USB).":"Installs it even without this update page inside (the next update will then have to be by USB).","Instalar":"Install","Só com o autosteer desligado e o trator parado. O ficheiro é verificado inteiro antes de substituir o firmware atual; se algo falhar, o atual continua. Nos segundos finais da instalação":"Only with autosteer off and the tractor stopped. The whole file is checked before it replaces the current firmware; if anything fails, the current one keeps running. In the last seconds of the installation","não desligue a corrente":"do not switch the power off",". As definições guardadas mantêm-se.":". The saved settings are kept.","sem dados do recetor":"no data from the receiver","OK (fixo)":"OK (fixed)","sem HPR":"no HPR","sem heading":"no heading","sem HPR do recetor (uma antena, ou HPR desligado no UM982)":"no HPR from the receiver (single antenna, or HPR off in the UM982)","o HPR não chega com o GGA":"the HPR does not arrive with the GGA","o UM982 não tem solução de heading das duas antenas":"the UM982 has no heading solution from its two antennas","heading vazio no HPR":"empty heading in the HPR","nenhum HPR recebido":"no HPR received","sem dados":"no data","não detetado":"not detected","com as antenas":"with the antennas","não alinhado":"not aligned","fixo":"fixed","a aprender":"learning","a conduzir":"steering","desligado":"off","ligado":"on","sem comunicação":"no communication","dois programas":"two programs","a comunicar":"talking","a chegar":"arriving","paradas":"stopped","automática":"automatic","automática, à espera do hello":"automatic, waiting for the hello","manual: o AgOpenGPS não lê estas frases":"manual: AgOpenGPS does not read these sentences","TM171 (o AgOpenGPS faz a fusão)":"TM171 (AgOpenGPS does the fusion)","antenas":"antennas","TM171 (sem antenas)":"TM171 (no antennas)","rumo GPS (sem antenas nem TM171)":"GPS course (no antennas, no TM171)","mantido (parado / TM171 por alinhar)":"held (stopped / TM171 not aligned yet)","fonte do heading":"heading source","ponte desligada":"bridge off","sem hello":"no hello","nunca":"never","cliente":"client","clientes":"clients","não configurada":"not set","sem ligação":"not connected","(predefinições: ainda nada guardado)":"(defaults: nothing saved yet)","Guardado.":"Saved.","Erro":"Error","erro":"error","Sem ligação ao módulo.":"No connection to the module.","sem ligação ao módulo":"no connection to the module","Repor as predefinições desta página?":"Restore the defaults of this page?","Este comando pode cortar a ligação entre o UM982 e o Teensy. Enviar mesmo assim?":"This command can cut the link between the UM982 and the Teensy. Send it anyway?","Escolha um ficheiro .hex.":"Choose a .hex file.","A enviar o ficheiro…":"Sending the file…","Ficheiro enviado; o Teensy está a verificar e a gravar…":"File sent; the Teensy is checking and writing it…","A ligação falhou. O firmware atual continua.":"The connection failed. The current firmware keeps running.","Reiniciar o Teensy? A direção e o GPS param uns segundos.":"Restart the Teensy? Steering and GPS stop for a few seconds.","A reiniciar…":"Restarting…","Português / English":"Português / English"};
const REV={};for(const k in EN)REV[EN[k]]=k;
let LANG="";try{LANG=localStorage.getItem("lang")||"";}catch(e){}
if(LANG!=="pt"&&LANG!=="en")LANG=/^pt/i.test(navigator.language||"")?"pt":"en";
const T=s=>LANG==="en"&&EN[s]!==undefined?EN[s]:s;          // a fixed text
const L=(pt,en)=>LANG==="en"?en:pt;                           // a text with numbers or names in it
const ago=s=>L("há "+dur(s),dur(s)+" ago");
const VPT={"no fix":"sem fix","RTK fixed":"RTK fixo","other":"outro","positions":"posições"};   // words the module sends in English
const V=v=>LANG==="pt"&&VPT[v]?VPT[v]:v;
function ptOf(k){return EN[k]!==undefined?k:REV[k];}            // a text in either language -> its Portuguese
function applyLang(){
 document.documentElement.lang=LANG;
 const w=document.createTreeWalker(document.body,NodeFilter.SHOW_TEXT,{acceptNode:n=>n.parentElement.closest("script,style,.mono")?NodeFilter.FILTER_REJECT:NodeFilter.FILTER_ACCEPT});
 // each text remembers its Portuguese the first time it is seen (two Portuguese texts can have the same English)
 for(let n=w.nextNode();n;n=w.nextNode()){if(n.k===undefined){n.raw=n.nodeValue;n.was=n.raw.trim();n.k=n.was&&ptOf(n.was)||"";}
  if(n.k)n.nodeValue=n.raw.replace(n.was,T(n.k));}
 for(const e of document.querySelectorAll("[title],[placeholder]"))for(const a of ["title","placeholder"]){const v=e.getAttribute(a),pt=v&&ptOf(v);if(pt)e.setAttribute(a,T(pt));}
 $("lang").textContent=LANG==="en"?"PT":"EN";
 $("logPause").textContent=T(logPaused?"Continuar":"Pausar");
 if(lastS)render(lastS);
}
$("lang").addEventListener("click",()=>{LANG=LANG==="en"?"pt":"en";try{localStorage.setItem("lang",LANG);}catch(e){}applyLang();});

function render(s){
 lastS=s;lastUp=s.up;
 $("hdr").textContent=s.fw+" · "+s.ip;
 const g=s.gnss,fixCls=g.fix===4?"ok":(g.fix===5||g.fix===2)?"warn":"bad";
 set("fixName",g.gga?V(g.fixName):T("sem dados do recetor"),fixCls);
 set("sats",g.sats||"–");set("hdop",g.hdop||"–");set("age",g.age?g.age+" s":"–");
 set("kmh",fmt(g.kmh,1," km/h")+(g.speedSrc!=="-"?" ("+V(g.speedSrc)+")":""));
 set("lat",g.lat!==null?g.lat.toFixed(7)+"°":"–");set("lon",g.lon!==null?g.lon.toFixed(7)+"°":"–");
 set("alt",(g.lat!==null&&g.alt)?Number(g.alt).toFixed(2)+" m":"–");
 set("crc",String(g.crcErr),g.crcErr?"warn":"");
 const d=s.dual;
 set("dualHdg",d.ok?Number(d.heading).toFixed(2)+"°":"–",d.ok?"ok":"");
 if(d.ok)pill("dualState",d.qf===4?T("OK (fixo)"):"OK (float)","ok");
 else if(!d.hpr)pill("dualState",T("sem HPR"),"warn");
 else pill("dualState",T("sem heading"),"bad");
 set("qf",d.hpr?String(d.lastQf):"–");set("antRoll",d.hpr?fmt(d.antRoll,2,"°"):"–");
 $("hprRaw").textContent=d.ok?(d.raw||"–"):(WHY[d.why]?T(WHY[d.why])+"\n":"")+(d.raw||T("nenhum HPR recebido"));
 const t=s.tm;
 set("tmHdg",t.ok?fmt(t.out,2,"°"):"–",t.ok?"":"bad");
 pill("tmState",t.ok?"OK":t.seen?T("sem dados"):T("não detetado"),t.ok?"ok":"bad");
 set("roll",t.ok?fmt(t.roll,2,"°"):"–");set("pitch",t.ok?fmt(t.pitch,2,"°"):"–");set("yaw",t.ok?fmt(t.yaw,2,"°"):"–");
 set("align",t.aligned?T("com as antenas")+" ("+fmt(t.offset,1,"°")+")":T("não alinhado"));
 set("sign",(t.sign>0?"+1":"−1")+" ("+T(t.locked?"fixo":"a aprender")+")");
 const st=s.steer;
 set("was",fmt(st.was,1,"°"));set("setp",fmt(st.set,1,"°"));set("pwm",String(st.pwm));
 pill("active",T(st.active?"a conduzir":"desligado"),st.active?"ok":"");
 set("sw",T(st.switchOn?"ligado":"desligado"));set("motor",st.motor);set("aogKmh",fmt(st.aogKmh,1," km/h"));
 const n=s.net;
 pill("link",n.link?L("ligado","connected"):L("desligado","unplugged"),n.link?"ok":"bad");
 const hn=n.host==="AgOpenWeb"||n.host==="AgOpenGPS"?n.host:"";
 pill("aog",!n.aog?T("sem comunicação"):n.hostBoth?T("dois programas"):hn?hn+" · "+T("a comunicar"):T("a comunicar"),!n.aog||n.hostBoth?"warn":"ok");set("aogIp",n.aog?n.aogIp:"–");
 pill("rtcm",T(n.rtcm?"a chegar":"paradas"),n.rtcm?"ok":"warn");set("rtcmKB",n.rtcmKB.toFixed(0)+" kB");
 set("ip",s.ip);set("up",dur(s.up));
 $("rx").textContent=g.rx?g.rx.split(", ").join("\n"):"–";
 const cl=s.out.mode==="classic";
 set("mode",cl?(s.out.kind===3?"AgOpenGPS ($PANDA)":"AgOpenGPS ($PAOGI)"):"AgOpenWeb");
 set("modeSel",T(s.out.auto?(n.host?"automática":"automática, à espera do hello"):!cl&&n.host==="AgOpenGPS"?"manual: o AgOpenGPS não lê estas frases":"manual"),!s.out.auto&&!cl&&n.host==="AgOpenGPS"?"warn":"");
 set("cSrc",s.out.mode!=="classic"?"–":T(s.out.kind===3?"TM171 (o AgOpenGPS faz a fusão)":(["–","antenas","TM171 (sem antenas)","rumo GPS (sem antenas nem TM171)","mantido (parado / TM171 por alinhar)"][s.out.classicSrc]||"–")),s.out.classicSrc>=2?"warn":"");
 set("aogSrc",s.out.aogSrc?T("fonte do heading")+": "+s.out.aogSrc:"–");
 set("epochs",String(s.out.epochs));
 $("sent").textContent=s.out.lines.length?s.out.lines.join("\n"):"–";
 const x=s.xiao;
 if(x){
  const xs=!x.on?[T("ponte desligada"),""]:x.det?[L("ligado","connected"),"ok"]:x.hellos?[T("sem hello"),"bad"]:[T("não detetado"),"warn"];
  set("xState",xs[0],xs[1]);
  set("xHello",x.helloAgo<0?T("nunca"):ago(x.helloAgo));
  set("xTo",String(x.to));set("xFrom",String(x.from));set("xLast",x.lastPgn<0?"–":String(x.lastPgn));
  set("xCrc",String(x.crcErr),x.crcErr?"warn":"");set("xDrop",String(x.drop+x.cut),(x.drop+x.cut)?"warn":"");
  set("xKB",x.rxKB.toFixed(1)+" / "+x.txKB.toFixed(1)+" kB");
  let xi=null;try{xi=x.info?JSON.parse(x.info):null;}catch(e){}
  if(xi){set("xProg",(xi.v||"?")+(xi.c?" · "+xi.c:"")+(x.infoAgo>15?" ("+ago(x.infoAgo)+")":""));
   set("xAp",(xi.ap||"–")+(xi.cl!==undefined?" · "+xi.cl+" "+T(xi.cl==1?"cliente":"clientes"):""));
   set("xSta",!xi.sta?T("não configurada"):xi.st==="ok"?xi.sta+" · "+xi.rssi+" dBm":xi.sta+" · "+T("sem ligação"),xi.sta&&xi.st!=="ok"?"warn":"");
   set("xIp",xi.sta&&xi.st==="ok"?xi.ip:"–");}
  else{set("xProg","–");set("xAp","–");set("xSta","–");set("xIp","–");}
 }
 const sp=s.fw.lastIndexOf(" ");set("fw",sp>0?s.fw.slice(0,sp):s.fw);set("fwVer",sp>0?s.fw.slice(sp+1):"–");set("built",s.built);set("target",s.target);
}

async function pollStatus(){
 try{const s=await getJ("/api/status");if(s.error)throw 0;
  if(waitingBoot&&s.up<upBefore){waitingBoot=false;busy=false;$("prog").classList.add("hidden");$("install").disabled=!$("hexFile").files[0];
   fwMsg(L("Atualizado. A correr: "+s.fw+", compilado "+s.built+".","Updated. Running: "+s.fw+", built "+s.built+"."),"ok");}
  setOnline(true);render(s);
 }catch(e){setOnline(false);}
}
function addLines(box,lines,withTime){
 const near=box.scrollTop+box.clientHeight>=box.scrollHeight-30;
 for(const l of lines){const d=document.createElement("div");d.className="ll";
  if(withTime){const t=document.createElement("span");t.className="t";t.textContent=dur(Math.floor(l.t/1000));d.appendChild(t);}
  const x=document.createElement("span");x.textContent=withTime?l.x:l;
  if(withTime&&/NO DATA|REFUSED|no dual|NOT detected|bad checksum|late|unplugged|silent|stopped|no GGA|dropped/.test(l.x))x.className="warn";
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
async function pollUm(){
 try{const j=await getJ("/api/um982?after="+umAfter);if(j.error)return;
  if(j.next<=umAfter){umAfter=0;return;}
  addLines($("umOut"),j.lines,false);umAfter=j.next-1;}catch(e){}
}
function show(t){
 tab=t;for(const b of document.querySelectorAll("nav button"))b.classList.toggle("on",b.dataset.t===t);
 for(const s of document.querySelectorAll("main>section"))s.classList.toggle("hidden",s.id!=="t-"+t);
 if(t==="settings")loadSettings();
 if(t==="log"){const b=$("log");b.scrollTop=b.scrollHeight;}
}
$("nav").addEventListener("click",e=>{if(e.target.dataset.t)show(e.target.dataset.t);});

// ---- settings
function fillSettings(s){
 const f=$("fs");
 f.querySelector('input[name=outputMode][value="'+s.outputMode+'"]').checked=true;
 f.hprAcceptFloat.checked=s.hprAcceptFloat==1;f.tm171SwapRollPitch.checked=s.tm171SwapRollPitch==1;
 f.epochWaitMs.value=String(s.epochWaitMs);
 if(f.epochWaitMs.value!==String(s.epochWaitMs)){const o=document.createElement("option");o.textContent=s.epochWaitMs;f.epochWaitMs.appendChild(o);f.epochWaitMs.value=String(s.epochWaitMs);}
 f.tm171YawSign.value=String(s.tm171YawSign);f.xiaoOn.checked=s.xiaoOff!=1;
}
async function loadSettings(){try{const j=await getJ("/api/settings");if(j.ok){fillSettings(j.settings);const m=$("setMsg");m.className="note";m.textContent=j.saved?"":T("(predefinições: ainda nada guardado)");}}catch(e){}}
async function postSettings(body){
 const m=$("setMsg");
 try{const j=await getJ("/api/settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:body});
  if(j.ok){fillSettings(j.settings);m.textContent=T("Guardado.");m.className="note ok";}else{m.textContent=T("Erro")+": "+ptErr(j.error);m.className="note bad";}
 }catch(e){m.textContent=T("Sem ligação ao módulo.");m.className="note bad";}
}
$("fs").addEventListener("submit",e=>{e.preventDefault();const f=$("fs");
 const p=new URLSearchParams();p.set("outputMode",f.querySelector("input[name=outputMode]:checked").value);
 p.set("hprAcceptFloat",f.hprAcceptFloat.checked?"1":"0");p.set("epochWaitMs",f.epochWaitMs.value);
 p.set("tm171SwapRollPitch",f.tm171SwapRollPitch.checked?"1":"0");p.set("tm171YawSign",f.tm171YawSign.value);
 p.set("xiaoOff",f.xiaoOn.checked?"0":"1");
 postSettings(p.toString());});
$("defaults").addEventListener("click",()=>{if(confirm(T("Repor as predefinições desta página?")))postSettings("defaults=1");});

// ---- UM982
for(const b of document.querySelectorAll("[data-c]"))b.addEventListener("click",()=>{$("umCmd").value=b.dataset.c;$("umCmd").focus();});
async function umSend(){
 const c=$("umCmd").value.trim();if(!c)return;
 if(/^(FRESET|RESET|UNLOG\b)/i.test(c)||/^CONFIG\s+COM\d\s+\d+/i.test(c)){if(!confirm(T("Este comando pode cortar a ligação entre o UM982 e o Teensy. Enviar mesmo assim?")))return;}
 addLines($("umOut"),["> "+c],false);
 try{const j=await getJ("/api/um982",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"cmd="+encodeURIComponent(c)});
  if(!j.ok)addLines($("umOut"),[T("erro")+": "+ptErr(j.error)],false);}catch(e){addLines($("umOut"),[T("sem ligação ao módulo")],false);}
 umUntil=Date.now()+6000;$("umCmd").select();
}
$("umSend").addEventListener("click",umSend);
$("umCmd").addEventListener("keydown",e=>{if(e.key==="Enter")umSend();});

// ---- firmware
const PT=[["autosteer is engaged","o autosteer está ligado — desligue-o primeiro"],["is moving","o trator está em movimento — pare primeiro"],
 ["size not plausible","ficheiro vazio ou com tamanho impossível"],["not an Intel HEX","não é um ficheiro .hex (Intel HEX)"],
 ["checksum error","ficheiro .hex danificado (erro de checksum)"],["bad hex","ficheiro .hex danificado (linha inválida)"],
 ["out of order","ficheiro .hex com endereços fora de ordem"],["outside the Teensy","endereços fora da memória do Teensy 4.1"],
 ["incomplete","ficheiro incompleto (cortado)"],["too small","ficheiro demasiado pequeno para ser um firmware"],
 ["does not start","o firmware não começa no início da memória do Teensy 4.1"],["fw_teensy41 missing","não foi compilado para Teensy 4.1, ou não tem atualização pela rede"],
 ["not this firmware","não é este firmware (não tem a página de atualização) — marque «Outro firmware» para o instalar na mesma"],
 ["flash erase","erro a apagar a memória do Teensy"],["flash verify","erro ao verificar a memória do Teensy"],
 ["connection lost","a ligação caiu durante o envio"],["timeout","o envio parou (tempo esgotado)"],
 ["too large for the update","o firmware atual é demasiado grande para a atualização"],["unsupported","ficheiro .hex com formato não suportado"],
 ["invalid value","valor inválido"],["hex line too long","ficheiro .hex com uma linha demasiado comprida"],["too long","comando vazio ou demasiado comprido"],["invalid character","carácter inválido no comando"]];
function ptErr(e){e=String(e||"");if(LANG==="en")return e;for(const [k,v] of PT)if(e.indexOf(k)>=0)return v;return e;}
function fwMsg(t,c){const m=$("fwMsg");m.textContent=t;m.className="note "+(c||"");}
$("hexFile").addEventListener("change",()=>{const f=$("hexFile").files[0];$("hexName").textContent=f?f.name+" ("+Math.round(f.size/1024)+" kB)":T("nenhum ficheiro");$("install").disabled=!f||busy;fwMsg("");});
$("install").addEventListener("click",()=>{
 const f=$("hexFile").files[0];if(!f||busy)return;
 if(!f.name.toLowerCase().endsWith(".hex")){fwMsg(T("Escolha um ficheiro .hex."),"bad");return;}
 if(!confirm(L("Instalar "+f.name+"?\n\nO autosteer tem de estar desligado e o trator parado. A direção fica desligada durante a atualização.","Install "+f.name+"?\n\nAutosteer must be off and the tractor stopped. Steering is off during the update.")))return;
 busy=true;$("install").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;
 fwMsg(T("A enviar o ficheiro…"));
 const x=new XMLHttpRequest();
 x.open("POST","/api/ota"+($("force").checked?"?force=1":""));
 x.setRequestHeader("Content-Type","text/plain");
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=Math.round(e.loaded/e.total*95);};
 x.upload.onload=()=>fwMsg(T("Ficheiro enviado; o Teensy está a verificar e a gravar…"));
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;upBefore=lastUp;waitingBoot=true;setOnline(false);
   fwMsg(L("Verificado ("+Math.round(j.size/1024)+" kB). A instalar e a reiniciar — não desligue a corrente…","Checked ("+Math.round(j.size/1024)+" kB). Installing and restarting — do not switch the power off…"),"warn");}
  else{busy=false;$("install").disabled=false;p.classList.add("hidden");fwMsg(L("Não instalado: ","Not installed: ")+ptErr(j.error||(T("erro")+" "+x.status))+L(". O firmware atual continua.",". The current firmware keeps running."),"bad");}};
 x.onerror=()=>{busy=false;$("install").disabled=false;p.classList.add("hidden");fwMsg(T("A ligação falhou. O firmware atual continua."),"bad");};
 x.send(f);
});
$("reboot").addEventListener("click",async()=>{if(!confirm(T("Reiniciar o Teensy? A direção e o GPS param uns segundos.")))return;
 try{await getJ("/api/reboot",{method:"POST"});}catch(e){}fwMsg(T("A reiniciar…"),"warn");});
$("logPause").addEventListener("click",()=>{logPaused=!logPaused;$("logPause").textContent=T(logPaused?"Continuar":"Pausar");});
$("logClear").addEventListener("click",()=>{$("log").textContent="";});

// ---- one request at a time to the module, every second
let tickN=0;
async function tick(){
 tickN++;
 if(busy&&!waitingBoot)return;                        // the upload has the module to itself
 await pollStatus();
 if(!online)return;
 if(tab==="log"||tickN%5===0)await pollLog();
 if(tab==="um982"&&Date.now()<umUntil)await pollUm();
}
applyLang();
(async function loop(){await tick();setTimeout(loop,1000);})();
</script>
</body>
</html>
)WEBPAGE";

#endif
