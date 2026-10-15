// xPage.h
// The XIAO's web page (served by v7.2_XIAO_ESP32C3.ino): one self-contained HTML file, no internet
// needed, made for a phone. Text in Portuguese.
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
@media (max-width:520px){header .sub{display:none}}
</style>
</head>
<body>
<header><span class="dot" id="dot" title="ligação ao XIAO"></span><h1>XIAO · módulo</h1><span class="sub" id="hdr">a ligar…</span></header>
<nav id="nav"><button data-t="status" class="on">Estado</button><button data-t="log">Mensagens</button><button data-t="wifi">WiFi</button><button data-t="prog">Programa</button></nav>
<main>

<section id="t-status">
<div class="grid">
 <div class="card"><h2>Módulo (Teensy)<span class="age" id="mAge"></span></h2>
  <div class="big" id="mFix">–</div>
  <dl style="margin-top:6px">
   <dt>Satélites</dt><dd id="mSats">–</dd><dt>HDOP</dt><dd id="mHdop">–</dd><dt>Idade das correções</dt><dd id="mAge2">–</dd>
   <dt>Velocidade</dt><dd id="mKmh">–</dd><dt>Heading das antenas</dt><dd id="mDual">–</dd><dt>TM171</dt><dd id="mTm">–</dd>
   <dt>Ângulo da roda (WAS)</dt><dd id="mWas">–</dd><dt>Ângulo pedido</dt><dd id="mSet">–</dd><dt>Autosteer</dt><dd id="mAct">–</dd>
   <dt>Cabo Ethernet</dt><dd id="mLink">–</dd><dt>AgOpenWeb / AOG</dt><dd id="mAog">–</dd><dt>Correções RTCM</dt><dd id="mRtcm">–</dd>
   <dt>Firmware</dt><dd id="mFw" class="mono">–</dd></dl>
  <p class="note">A página completa do módulo abre só pelo cabo Ethernet: <span class="mono">http://192.168.5.126/</span></p></div>
 <div class="card"><h2>AgOpenGPS → módulo<span class="age" id="aAge"></span></h2>
  <div class="big" id="aKmh">–</div>
  <dl style="margin-top:6px"><dt>Autosteer pedido</dt><dd id="aSt">–</dd><dt>Ângulo pedido</dt><dd id="aAng">–</dd>
   <dt>Dados de direção (PGN 254)</dt><dd id="aRate">–</dd><dt>Elevador hidráulico</dt><dd id="aHyd">–</dd><dt>Tram line</dt><dd id="aTram">–</dd>
   <dt>U-turn</dt><dd id="aUt">–</dd><dt>Hello do AgIO / AgOpenWeb</dt><dd id="aHello">–</dd></dl>
  <p class="note" style="margin-top:8px">Secções 1–16</p><div class="secs" id="aSecs"></div></div>
 <div class="card"><h2>XIAO</h2>
  <dl><dt>Programa</dt><dd id="xFw" class="mono">–</dd><dt>Ligado há</dt><dd id="xUp">–</dd><dt>Último arranque</dt><dd id="xRst">–</dd>
   <dt>WiFi do XIAO</dt><dd id="xAp">–</dd><dt>Dispositivos ligados</dt><dd id="xCl">–</dd><dt>Outra rede</dt><dd id="xSta">–</dd>
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
 <label class="field"><span>Nova palavra-passe (8 a 63 caracteres; em branco fica a atual)</span><input type="password" name="apPass" maxlength="63"></label>
 <p class="note">Está sempre ligada, em 192.168.4.1. É também a palavra-passe para alterar definições e o programa.</p>
</div>
<div class="card"><h2>Outra rede WiFi (opcional)</h2>
 <label class="opt"><input type="checkbox" name="staOn" value="1"><span>Ligar o XIAO também a outra rede<small>Rede da exploração ou o hotspot do telemóvel. A página fica também em http://<b id="hostShow">xiao</b>.local/ ou no IP que o XIAO receber.</small></span></label>
 <label class="field"><span>Nome da rede</span><input type="text" name="staSsid" maxlength="32" list="nets"><datalist id="nets"></datalist></label>
 <div class="row" style="margin:2px 0"><button class="sec" type="button" id="scan">Procurar redes</button><span id="scanMsg" class="note"></span></div>
 <label class="field"><span>Palavra-passe dessa rede (em branco: fica a atual; para uma rede nova, em branco = rede aberta)</span><input type="password" name="staPass" maxlength="63"></label>
 <label class="field"><span>Nome do XIAO nessa rede</span><input type="text" name="host" maxlength="24"></label>
</div>
<div class="card"><h2>Guardar</h2>
 <label class="field"><span>Palavra-passe atual da rede do XIAO</span><input type="password" name="pw" maxlength="63"></label>
 <div class="row"><button class="act" type="submit">Guardar e reiniciar o XIAO</button><span id="setMsg" class="note"></span></div>
 <p class="note">O XIAO reinicia para aplicar. Se mudar o nome ou a palavra-passe da rede do XIAO, volte a ligar o telemóvel à rede nova. Esqueceu a palavra-passe? Pela porta USB do painel, no monitor série: <span class="mono">wifi-reset</span>.</p>
</div>
</form>
</section>

<section id="t-prog" class="hidden">
<div id="fw">
<div class="card"><h2>Programa do XIAO</h2>
 <dl><dt>Programa</dt><dd id="pFw" class="mono">–</dd><dt>Compilado</dt><dd id="pBuilt">–</dd><dt>Tamanho</dt><dd id="pSize">–</dd></dl></div>
<div class="card"><h2>Atualizar o programa</h2>
 <p class="note" style="margin-top:0">No Arduino IDE: <b>Sketch → Export Compiled Binary</b>. Usar o ficheiro <span class="mono">….ino.bin</span> (não o <span class="mono">merged</span> nem o <span class="mono">bootloader</span>).</p>
 <div class="row"><label class="act">Escolher .bin<input type="file" id="binFile" accept=".bin" class="hidden"></label><span id="binName" class="note">nenhum ficheiro</span></div>
 <label class="field"><span>Palavra-passe da rede do XIAO</span><input type="password" id="pw2" maxlength="63"></label>
 <div class="row"><button class="act" id="install" disabled>Instalar</button></div>
 <progress id="prog" max="100" value="0" class="hidden"></progress>
 <p id="fwMsg" class="note"></p>
 <p class="note">O ficheiro é verificado antes de substituir o programa atual; se algo falhar, o atual continua. Se o programa novo não arrancar bem, programa-se pela porta USB.</p></div>
<div class="card"><h2>Reiniciar</h2>
 <label class="field"><span>Palavra-passe da rede do XIAO</span><input type="password" id="pw3" maxlength="63"></label>
 <div class="row"><button class="danger" id="reboot">Reiniciar o XIAO</button><span id="rbMsg" class="note"></span></div></div>
</div>
</section>
</main>
<script>
"use strict";
const $=id=>document.getElementById(id);
let tab="status",online=false,logAfter=0,xAfter=0,epoch=0,busy=false,waitBoot=false,upBefore=0,lastUp=0;
const fmt=(v,d,u)=>(v===null||v===undefined||Number.isNaN(v))?"–":Number(v).toFixed(d)+(u||"");
function set(id,txt,cls){const e=$(id);if(!e)return;e.textContent=txt;e.classList.remove("ok","warn","bad");if(cls)e.classList.add(cls);}
function pill(id,txt,cls){const e=$(id);e.textContent="";const s=document.createElement("span");s.className="pill "+(cls||"");s.textContent=txt;e.appendChild(s);}
async function getJ(u,o){const r=await fetch(u,Object.assign({cache:"no-store"},o||{}));let j={};try{j=await r.json();}catch(e){}if(!r.ok&&!j.error)j.error="HTTP "+r.status;return j;}
function dur(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?h+" h "+m+" min":m?m+" min "+(s%60)+" s":s+" s";}
function ago(s){return s<0?"nunca":s<2?"agora":"há "+dur(s);}
const PT={"wrong password":"palavra-passe errada","network name":"nome da rede: 1 a 32 caracteres","password: 8":"palavra-passe: 8 a 63 caracteres",
 "other network password":"palavra-passe da outra rede: 8 a 63 caracteres","other network name":"nome da outra rede: 1 a 32 caracteres",
 "other network: no name":"falta o nome da outra rede","name on the network":"nome na rede: letras, algarismos e '-', até 24","no file":"nenhum ficheiro",
 "upload aborted":"o envio foi interrompido","Magic":"não é um programa para o ESP32 (.bin errado)","Bad Size":"ficheiro com tamanho errado"};
function ptErr(e){e=String(e||"");for(const k in PT)if(e.indexOf(k)>=0)return PT[k];return e;}
const WHY={1:"sem HPR",2:"HPR atrasado",3:"UM982 sem solução",4:"HPR vazio"};

function renderModule(t){
 if(!t||!t.s){set("mAge","sem dados do módulo","warn");set("mFix","–");return;}
 const s=t.s,g=s.gnss||{},d=s.dual||{},m=s.tm||{},st=s.steer||{},n=s.net||{};
 set("mAge",t.ago>3?"há "+dur(t.ago):"agora",t.ago>3?"warn":"");
 const fixCls=g.fix===4?"ok":(g.fix===5||g.fix===2)?"warn":"bad";
 set("mFix",g.gga?g.fixName:"sem dados do recetor",fixCls);
 set("mSats",g.sats||"–");set("mHdop",g.hdop||"–");set("mAge2",g.age?g.age+" s":"–");
 set("mKmh",fmt(g.kmh,1," km/h"));
 if(d.ok)set("mDual",Number(d.heading).toFixed(2)+"° (QF "+d.qf+")","ok");else set("mDual",d.hpr?"sem heading ("+(WHY[d.why]||"?")+")":"sem HPR",d.hpr?"bad":"warn");
 set("mTm",m.ok?"roll "+fmt(m.roll,1,"°")+" · "+fmt(m.out,1,"°"):m.seen?"sem dados":"não detetado",m.ok?"":"bad");
 set("mWas",fmt(st.was,1,"°"));set("mSet",fmt(st.set,1,"°"));
 pill("mAct",st.active?"a conduzir":"desligado",st.active?"ok":"");
 pill("mLink",n.link?"ligado":"desligado",n.link?"ok":"bad");
 pill("mAog",n.aog?"a comunicar":"sem comunicação",n.aog?"ok":"warn");
 pill("mRtcm",n.rtcm?"a chegar":"paradas",n.rtcm?"ok":"warn");
 set("mFw",s.fw||"–");
}
function renderState(x){
 lastUp=x.up;
 $("hdr").textContent=x.fw+" · "+(x.wifi.st==="ok"?x.wifi.ip:"192.168.4.1");
 const a=x.aog;
 set("aAge",a.ago254<0?"nada recebido":a.ago254>2?"há "+dur(a.ago254):"agora",a.ago254<0||a.ago254>2?"warn":"");
 set("aKmh",a.ago254<0?"–":fmt(a.kmh,1," km/h"));
 pill("aSt",a.ago254<0?"–":a.status===1?"ligado":"desligado",a.status===1?"ok":"");
 set("aAng",a.ago254<0?"–":fmt(a.angle,1,"°"));
 set("aRate",a.ago254<0?"–":a.rate254+"/s");
 set("aHyd",a.ago239<0?"–":a.hyd===2?"subir":a.hyd===1?"descer":"desligado");
 set("aTram",a.ago239<0?"–":a.tram?String(a.tram):"não");
 set("aUt",a.ago239<0?"–":a.uturn?String(a.uturn):"não");
 set("aHello",ago(a.helloAgo));
 const box=$("aSecs");if(!box.childElementCount)for(let i=1;i<=16;i++){const s=document.createElement("span");s.textContent=i;box.appendChild(s);}
 for(let i=0;i<16;i++)box.children[i].classList.toggle("on",a.ago254>=0&&!!(a.sec&(1<<i)));
 set("xFw",x.fw);set("xUp",dur(x.up));set("xRst",x.reset);
 set("xAp",x.wifi.ap+" · canal "+x.wifi.ch);set("xCl",String(x.wifi.apCl));
 set("xSta",!x.wifi.staOn?"desligada":x.wifi.st==="ok"?x.wifi.sta+" · "+x.wifi.ip+" · "+x.wifi.rssi+" dBm":x.wifi.sta+" · a ligar…",x.wifi.staOn&&x.wifi.st!=="ok"?"warn":"");
 pill("xLink",x.link.talking?"a comunicar":"sem dados",x.link.talking?"ok":"bad");
 set("xKB",x.link.rxKB.toFixed(1)+" / "+x.link.txKB.toFixed(1)+" kB");
 set("xCrc",String(x.link.crcErr),x.link.crcErr?"warn":"");
 set("xHeap",Math.round(x.heap/1024)+" kB");
 set("pFw",x.fw);set("pBuilt",x.built);set("pSize",Math.round(x.sketch/1024)+" kB (máx. "+Math.round(x.sketchFree/1024)+" kB)");
}
function setOnline(v){online=v;$("dot").classList.toggle("on",v);}
async function pollState(){
 try{const x=await getJ("/api/state");if(x.error)throw 0;
  if(waitBoot&&x.up<upBefore){waitBoot=false;busy=false;$("prog").classList.add("hidden");fwMsg("Atualizado. A correr: "+x.fw+", compilado "+x.built+".","ok");}
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
 if(t==="wifi")loadSettings();
 if(t==="log")pollLog();
}
$("nav").addEventListener("click",e=>{if(e.target.dataset.t)show(e.target.dataset.t);});

// ---- WiFi settings
async function loadSettings(){try{const s=await getJ("/api/settings");if(s.error)return;const f=$("fs");
 f.apSsid.value=s.apSsid;f.staOn.checked=s.staOn;f.staSsid.value=s.staSsid;f.host.value=s.host;$("hostShow").textContent=s.host;
 f.apPass.value="";f.staPass.value="";f.staPass.placeholder=s.staPassSet?"(guardada)":"";}catch(e){}}
$("fs").addEventListener("submit",async e=>{e.preventDefault();const f=$("fs"),m=$("setMsg");
 if(f.apPass.value&&f.apPass.value.length<8){m.textContent="A palavra-passe tem de ter 8 caracteres ou mais.";m.className="note bad";return;}
 if(!confirm("Guardar e reiniciar o XIAO? O WiFi do XIAO desliga-se uns segundos."))return;
 const p=new URLSearchParams();for(const k of ["apSsid","apPass","staSsid","staPass","host","pw"])p.set(k,f[k].value);p.set("staOn",f.staOn.checked?"1":"0");
 try{const j=await getJ("/api/settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:p.toString()});
  if(j.ok){m.textContent="Guardado. O XIAO está a reiniciar.";m.className="note ok";}else{m.textContent="Erro: "+ptErr(j.error);m.className="note bad";}
 }catch(e){m.textContent="Sem ligação ao XIAO.";m.className="note bad";}});
$("scan").addEventListener("click",async()=>{const m=$("scanMsg");m.textContent="a procurar…";m.className="note";
 for(let i=0;i<20;i++){try{const j=await getJ("/api/scan");if(j.list){const dl=$("nets");dl.textContent="";
   for(const n of j.list){const o=document.createElement("option");o.value=n.s;o.label=n.r+" dBm"+(n.open?" · aberta":"");dl.appendChild(o);}
   m.textContent=j.list.length+" redes: "+j.list.slice(0,6).map(n=>n.s+" ("+n.r+" dBm)").join(", ");return;}}catch(e){}
  await new Promise(r=>setTimeout(r,800));}
 m.textContent="sem resposta";});

// ---- program
function fwMsg(t,c){const m=$("fwMsg");m.textContent=t;m.className="note "+(c||"");}
$("binFile").addEventListener("change",()=>{const f=$("binFile").files[0];$("binName").textContent=f?f.name+" ("+Math.round(f.size/1024)+" kB)":"nenhum ficheiro";$("install").disabled=!f||busy;fwMsg("");});
$("install").addEventListener("click",()=>{
 const f=$("binFile").files[0];if(!f||busy)return;
 if(!f.name.toLowerCase().endsWith(".bin")){fwMsg("Escolha um ficheiro .bin.","bad");return;}
 if(/merged|bootloader|partitions/i.test(f.name)){fwMsg("Este é o ficheiro errado: use o ….ino.bin.","bad");return;}
 if(!confirm("Instalar "+f.name+" no XIAO?"))return;
 busy=true;$("install").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;fwMsg("A enviar…");
 const fd=new FormData();fd.append("file",f,f.name);
 const x=new XMLHttpRequest();x.open("POST","/update?pw="+encodeURIComponent($("pw2").value));
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=Math.round(e.loaded/e.total*95);};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;upBefore=lastUp;waitBoot=true;fwMsg("Instalado. O XIAO está a reiniciar…","warn");}
  else{busy=false;$("install").disabled=false;p.classList.add("hidden");fwMsg("Não instalado: "+ptErr(j.error||("erro "+x.status))+". O programa atual continua.","bad");}};
 x.onerror=()=>{busy=false;$("install").disabled=false;p.classList.add("hidden");fwMsg("A ligação falhou. O programa atual continua.","bad");};
 x.send(fd);});
$("reboot").addEventListener("click",async()=>{const m=$("rbMsg");if(!confirm("Reiniciar o XIAO?"))return;
 try{const j=await getJ("/api/restart",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"pw="+encodeURIComponent($("pw3").value)});
  m.textContent=j.ok?"A reiniciar…":"Erro: "+ptErr(j.error);m.className="note "+(j.ok?"warn":"bad");}catch(e){}});

// ---- one request at a time, every second
let tickN=0;
async function tick(){tickN++;if(busy&&!waitBoot)return;await pollState();if(!online)return;if(tab==="log"||tickN%5===0)await pollLog();}
(async function loop(){await tick();setTimeout(loop,1000);})();
</script>
</body>
</html>
)XIAOPAGE";
