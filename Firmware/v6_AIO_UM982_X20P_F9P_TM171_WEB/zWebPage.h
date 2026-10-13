// zWebPage.h
// The module's web page (served by zWeb.ino): one self-contained HTML file, no internet needed.
// Kept in flash (PROGMEM). Text in Portuguese.
#ifndef ZWEBPAGE_H
#define ZWEBPAGE_H

const char WEB_PAGE[] PROGMEM = R"WEBPAGE(<!doctype html>
<html lang="pt">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark">
<title>AiO UM982 TM171</title>
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
@media (max-width:520px){header .sub{display:none}}
</style>
</head>
<body>
<header><span class="dot" id="dot" title="ligação ao módulo"></span><h1>AiO UM982 · TM171</h1><span class="sub" id="hdr">a ligar…</span></header>
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
   <dt>Interruptor</dt><dd id="sw">–</dd><dt>Motor</dt><dd id="motor">–</dd><dt>Velocidade (AgOpenWeb)</dt><dd id="aogKmh">–</dd></dl></div>
 <div class="card"><h2>Ligações</h2>
  <dl><dt>Cabo Ethernet</dt><dd id="link" class="nw">–</dd><dt>AgOpenWeb / AOG</dt><dd id="aog" class="nw">–</dd><dt>IP do AgOpenWeb</dt><dd id="aogIp" class="nw">–</dd>
   <dt>Correções RTCM</dt><dd id="rtcm" class="nw">–</dd><dt>RTCM recebido</dt><dd id="rtcmKB" class="nw">–</dd>
   <dt>IP do módulo</dt><dd id="ip">–</dd><dt>Ligado há</dt><dd id="up">–</dd></dl>
  <p class="note">O recetor envia (últimos 5 s):</p><div class="box mono" id="rx">–</div></div>
 <div class="card"><h2>Frases enviadas</h2>
  <dl><dt>Modo</dt><dd id="mode" class="nw">–</dd><dt>Épocas enviadas</dt><dd id="epochs">–</dd></dl>
  <div class="box mono" id="sent">–</div></div>
</div>
</section>

<section id="t-log" class="hidden">
<div class="row"><button class="sec" id="logPause">Pausar</button><button class="sec" id="logClear">Limpar ecrã</button><span class="note">Mensagens [status] do firmware, só quando algo muda. O módulo guarda as últimas 60.</span></div>
<div id="log" class="mono"></div>
</section>

<section id="t-settings" class="hidden">
<form id="fs">
<div class="card"><h2>Frases enviadas</h2>
 <label class="opt"><input type="radio" name="outputMode" value="0"><span>Conjunto padrão — AgOpenWeb<small>$GNGGA, $GNVTG, $GNTHS (heading das antenas) e $INHPR (TM171) em cada época.</small></span></label>
 <label class="opt"><input type="radio" name="outputMode" value="1"><span>Clássico — AgOpenGPS / AgIO<small>Uma frase por época: $PAOGI com o heading das antenas e o roll do TM171; sem heading das antenas, $PANDA com o heading do TM171. O AgOpenWeb também lê estas frases.</small></span></label>
 <div id="noDualBox" style="margin:4px 0 0 26px">
  <label class="opt"><input type="radio" name="classicNoDual" value="0"><span>Sem heading das antenas: $PANDA<small>Como o firmware oficial. Recomendado.</small></span></label>
  <label class="opt"><input type="radio" name="classicNoDual" value="1"><span>Sem heading das antenas: $PAOGI com o heading do TM171<small>Só depois de o TM171 ter sido alinhado com as antenas desde o arranque; até lá, $PANDA.</small></span></label>
 </div>
</div>
<div class="card"><h2>Heading das duas antenas (UM982)</h2>
 <label class="opt"><input type="checkbox" name="hprAcceptFloat" value="1"><span>Aceitar heading em float (QF 5)<small>Desligado: só com QF 4 (fixo).</small></span></label>
 <div class="opt"><span style="flex:1">Espera máxima pelo VTG e HPR depois do GGA<small>Predefinição 60 ms. Mais tempo atrasa a época; menos pode perder o HPR.</small></span><select name="epochWaitMs"><option>20</option><option>30</option><option>40</option><option>50</option><option>60</option><option>70</option><option>80</option><option>100</option></select></div>
</div>
<div class="card"><h2>TM171</h2>
 <label class="opt"><input type="checkbox" name="tm171SwapRollPitch" value="1"><span>Trocar roll e pitch<small>TM171 rodado 90° na placa. O "Use Y axis" do AgOpenGPS também troca.</small></span></label>
 <div class="opt"><span style="flex:1">Sentido do yaw<small>Automático: aprende com o heading das antenas nas curvas. Com uma antena só fica +1.</small></span><select name="tm171YawSign"><option value="0">automático</option><option value="1">+1</option><option value="-1">−1</option></select></div>
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
 <dl><dt>Firmware</dt><dd id="fw" class="mono nw">–</dd><dt>Versão</dt><dd id="fwVer">–</dd><dt>Compilado</dt><dd id="built">–</dd><dt>Placa</dt><dd id="target">–</dd></dl>
 <div class="row"><button class="danger" id="reboot">Reiniciar o Teensy</button></div></div>
<div class="card"><h2>Atualizar o firmware</h2>
 <p class="note" style="margin-top:0">No Arduino IDE: <b>Sketch → Export Compiled Binary</b>. O ficheiro <span class="mono">.hex</span> fica na pasta do sketch (ou na subpasta <span class="mono">build</span>).</p>
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
let tab="status",online=false,logAfter=0,logPaused=false,umAfter=0,umUntil=0,lastUp=0,upBefore=0,waitingBoot=false,busy=false;
const fmt=(v,d,u)=>(v===null||v===undefined||Number.isNaN(v))?"–":Number(v).toFixed(d)+(u||"");
function set(id,txt,cls){const e=$(id);if(!e)return;e.textContent=txt;e.classList.remove("ok","warn","bad");if(cls)e.classList.add(cls);}
function pill(id,txt,cls){const e=$(id);e.textContent="";const s=document.createElement("span");s.className="pill "+(cls||"");s.textContent=txt;e.appendChild(s);}
async function getJ(u,o){const r=await fetch(u,Object.assign({cache:"no-store"},o||{}));let j={};try{j=await r.json();}catch(e){}if(!r.ok&&!j.error)j.error="HTTP "+r.status;return j;}
function setOnline(v){online=v;$("dot").classList.toggle("on",v);}
function dur(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h?h+" h "+m+" min":m?m+" min "+(s%60)+" s":s+" s";}
const WHY={1:"sem HPR do recetor (uma antena, ou HPR desligado no UM982)",2:"o HPR não chega com o GGA",3:"o UM982 não tem solução de heading das duas antenas",4:"heading vazio no HPR"};

function render(s){
 lastUp=s.up;
 $("hdr").textContent=s.fw+" · "+s.ip;
 const g=s.gnss,fixCls=g.fix===4?"ok":(g.fix===5||g.fix===2)?"warn":"bad";
 set("fixName",g.gga?g.fixName:"sem dados do recetor",fixCls);
 set("sats",g.sats||"–");set("hdop",g.hdop||"–");set("age",g.age?g.age+" s":"–");
 set("kmh",fmt(g.kmh,1," km/h")+(g.speedSrc!=="-"?" ("+g.speedSrc+")":""));
 set("lat",g.lat!==null?g.lat.toFixed(7)+"°":"–");set("lon",g.lon!==null?g.lon.toFixed(7)+"°":"–");
 set("alt",(g.lat!==null&&g.alt)?Number(g.alt).toFixed(2)+" m":"–");
 set("crc",String(g.crcErr),g.crcErr?"warn":"");
 const d=s.dual;
 set("dualHdg",d.ok?Number(d.heading).toFixed(2)+"°":"–",d.ok?"ok":"");
 if(d.ok)pill("dualState",d.qf===4?"OK (fixo)":"OK (float)","ok");
 else if(!d.hpr)pill("dualState","sem HPR","warn");
 else pill("dualState","sem heading","bad");
 set("qf",d.hpr?String(d.lastQf):"–");set("antRoll",d.hpr?fmt(d.antRoll,2,"°"):"–");
 $("hprRaw").textContent=d.ok?(d.raw||"–"):(WHY[d.why]?WHY[d.why]+"\n":"")+(d.raw||"nenhum HPR recebido");
 const t=s.tm;
 set("tmHdg",t.ok?fmt(t.out,2,"°"):"–",t.ok?"":"bad");
 pill("tmState",t.ok?"OK":t.seen?"sem dados":"não detetado",t.ok?"ok":"bad");
 set("roll",t.ok?fmt(t.roll,2,"°"):"–");set("pitch",t.ok?fmt(t.pitch,2,"°"):"–");set("yaw",t.ok?fmt(t.yaw,2,"°"):"–");
 set("align",t.aligned?"com as antenas ("+fmt(t.offset,1,"°")+")":"não alinhado");
 set("sign",(t.sign>0?"+1":"−1")+(t.locked?" (fixo)":" (a aprender)"));
 const st=s.steer;
 set("was",fmt(st.was,1,"°"));set("setp",fmt(st.set,1,"°"));set("pwm",String(st.pwm));
 pill("active",st.active?"a conduzir":"desligado",st.active?"ok":"");
 set("sw",st.switchOn?"ligado":"desligado");set("motor",st.motor);set("aogKmh",fmt(st.aogKmh,1," km/h"));
 const n=s.net;
 pill("link",n.link?"ligado":"desligado",n.link?"ok":"bad");
 pill("aog",n.aog?"a comunicar":"sem comunicação",n.aog?"ok":"warn");set("aogIp",n.aog?n.aogIp:"–");
 pill("rtcm",n.rtcm?"a chegar":"paradas",n.rtcm?"ok":"warn");set("rtcmKB",n.rtcmKB.toFixed(0)+" kB");
 set("ip",s.ip);set("up",dur(s.up));
 $("rx").textContent=g.rx?g.rx.split(", ").join("\n"):"–";
 set("mode",s.out.mode==="classic"?"clássico (PAOGI / PANDA)":"padrão (AgOpenWeb)");
 set("epochs",String(s.out.epochs));
 $("sent").textContent=s.out.lines.length?s.out.lines.join("\n"):"–";
 const sp=s.fw.lastIndexOf(" ");set("fw",sp>0?s.fw.slice(0,sp):s.fw);set("fwVer",sp>0?s.fw.slice(sp+1):"–");set("built",s.built);set("target",s.target);
}

async function pollStatus(){
 try{const s=await getJ("/api/status");if(s.error)throw 0;
  if(waitingBoot&&s.up<upBefore){waitingBoot=false;busy=false;$("prog").classList.add("hidden");$("install").disabled=!$("hexFile").files[0];
   fwMsg("Atualizado. A correr: "+s.fw+", compilado "+s.built+".","ok");}
  setOnline(true);render(s);
 }catch(e){setOnline(false);}
}
function addLines(box,lines,withTime){
 const near=box.scrollTop+box.clientHeight>=box.scrollHeight-30;
 for(const l of lines){const d=document.createElement("div");d.className="ll";
  if(withTime){const t=document.createElement("span");t.className="t";t.textContent=dur(Math.floor(l.t/1000));d.appendChild(t);}
  const x=document.createElement("span");x.textContent=withTime?l.x:l;
  if(withTime&&/NO DATA|REFUSED|no dual|NOT detected|bad checksum|late|unplugged|silent|stopped|no GGA/.test(l.x))x.className="warn";
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
 f.querySelector('input[name=classicNoDual][value="'+s.classicNoDual+'"]').checked=true;
 f.hprAcceptFloat.checked=s.hprAcceptFloat==1;f.tm171SwapRollPitch.checked=s.tm171SwapRollPitch==1;
 f.epochWaitMs.value=String(s.epochWaitMs);
 if(f.epochWaitMs.value!==String(s.epochWaitMs)){const o=document.createElement("option");o.textContent=s.epochWaitMs;f.epochWaitMs.appendChild(o);f.epochWaitMs.value=String(s.epochWaitMs);}
 f.tm171YawSign.value=String(s.tm171YawSign);noDualVis();
}
function noDualVis(){$("noDualBox").style.opacity=$("fs").querySelector('input[name=outputMode][value="1"]').checked?1:.45;}
$("fs").addEventListener("change",noDualVis);
async function loadSettings(){try{const j=await getJ("/api/settings");if(j.ok){fillSettings(j.settings);const m=$("setMsg");m.className="note";m.textContent=j.saved?"":"(predefinições: ainda nada guardado)";}}catch(e){}}
async function postSettings(body){
 const m=$("setMsg");
 try{const j=await getJ("/api/settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:body});
  if(j.ok){fillSettings(j.settings);m.textContent="Guardado.";m.className="note ok";}else{m.textContent="Erro: "+ptErr(j.error);m.className="note bad";}
 }catch(e){m.textContent="Sem ligação ao módulo.";m.className="note bad";}
}
$("fs").addEventListener("submit",e=>{e.preventDefault();const f=$("fs");
 const p=new URLSearchParams();p.set("outputMode",f.querySelector("input[name=outputMode]:checked").value);
 p.set("classicNoDual",f.querySelector("input[name=classicNoDual]:checked").value);
 p.set("hprAcceptFloat",f.hprAcceptFloat.checked?"1":"0");p.set("epochWaitMs",f.epochWaitMs.value);
 p.set("tm171SwapRollPitch",f.tm171SwapRollPitch.checked?"1":"0");p.set("tm171YawSign",f.tm171YawSign.value);
 postSettings(p.toString());});
$("defaults").addEventListener("click",()=>{if(confirm("Repor as predefinições desta página?"))postSettings("defaults=1");});

// ---- UM982
for(const b of document.querySelectorAll("[data-c]"))b.addEventListener("click",()=>{$("umCmd").value=b.dataset.c;$("umCmd").focus();});
async function umSend(){
 const c=$("umCmd").value.trim();if(!c)return;
 if(/^(FRESET|RESET|UNLOG\b)/i.test(c)||/^CONFIG\s+COM\d\s+\d+/i.test(c)){if(!confirm("Este comando pode cortar a ligação entre o UM982 e o Teensy. Enviar mesmo assim?"))return;}
 addLines($("umOut"),["> "+c],false);
 try{const j=await getJ("/api/um982",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"cmd="+encodeURIComponent(c)});
  if(!j.ok)addLines($("umOut"),["erro: "+ptErr(j.error)],false);}catch(e){addLines($("umOut"),["sem ligação ao módulo"],false);}
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
function ptErr(e){e=String(e||"");for(const [k,v] of PT)if(e.indexOf(k)>=0)return v;return e;}
function fwMsg(t,c){const m=$("fwMsg");m.textContent=t;m.className="note "+(c||"");}
$("hexFile").addEventListener("change",()=>{const f=$("hexFile").files[0];$("hexName").textContent=f?f.name+" ("+Math.round(f.size/1024)+" kB)":"nenhum ficheiro";$("install").disabled=!f||busy;fwMsg("");});
$("install").addEventListener("click",()=>{
 const f=$("hexFile").files[0];if(!f||busy)return;
 if(!f.name.toLowerCase().endsWith(".hex")){fwMsg("Escolha um ficheiro .hex.","bad");return;}
 if(!confirm("Instalar "+f.name+"?\n\nO autosteer tem de estar desligado e o trator parado. A direção fica desligada durante a atualização."))return;
 busy=true;$("install").disabled=true;const p=$("prog");p.classList.remove("hidden");p.value=0;
 fwMsg("A enviar o ficheiro…");
 const x=new XMLHttpRequest();
 x.open("POST","/api/ota"+($("force").checked?"?force=1":""));
 x.setRequestHeader("Content-Type","text/plain");
 x.upload.onprogress=e=>{if(e.lengthComputable)p.value=Math.round(e.loaded/e.total*95);};
 x.upload.onload=()=>fwMsg("Ficheiro enviado; o Teensy está a verificar e a gravar…");
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText);}catch(e){}
  if(x.status===200&&j.ok){p.value=100;upBefore=lastUp;waitingBoot=true;setOnline(false);
   fwMsg("Verificado ("+Math.round(j.size/1024)+" kB). A instalar e a reiniciar — não desligue a corrente…","warn");}
  else{busy=false;$("install").disabled=false;p.classList.add("hidden");fwMsg("Não instalado: "+ptErr(j.error||("erro "+x.status))+". O firmware atual continua.","bad");}};
 x.onerror=()=>{busy=false;$("install").disabled=false;p.classList.add("hidden");fwMsg("A ligação falhou. O firmware atual continua.","bad");};
 x.send(f);
});
$("reboot").addEventListener("click",async()=>{if(!confirm("Reiniciar o Teensy? A direção e o GPS param uns segundos."))return;
 try{await getJ("/api/reboot",{method:"POST"});}catch(e){}fwMsg("A reiniciar…","warn");});
$("logPause").addEventListener("click",()=>{logPaused=!logPaused;$("logPause").textContent=logPaused?"Continuar":"Pausar";});
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
(async function loop(){await tick();setTimeout(loop,1000);})();
</script>
</body>
</html>
)WEBPAGE";

#endif
