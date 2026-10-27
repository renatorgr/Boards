/* sim-common.js - the web page simulator: a pretend AiO v4.5 module behind the real pages.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
   Written 2026 by Claude (Anthropic) for renatorgr.
   2026-10-27 (Claude): the seed drill's sprayer module sends no PGN 160, as Sprayer_ESP32 v0.7.2 with no sensors ticked.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// What the Teensy page (sim-teensy.js) and the XIAO page (sim-xiao.js) share. build.py puts these scripts in
// the <head> of the pages taken from the firmware (zWebPage.h, xPage.h), so they run before the page's own
// script and answer its requests in the browser:
// - hook() / hookXhr(): the page's fetch() and XMLHttpRequest calls to its module go to a handler here;
//   anything else (the page's check for a newer version on GitHub) goes out as usual.
// - bar(): the strip at the top that says it is a simulator, with the choices (receiver, tractor driving or
//   parked, machine module). The choices are in the page's address, so a link opens the same.
// - Module(): the Teensy, with made-up values in the same JSON as the firmware (zWeb.ino statusJson and
//   apiLog / apiSettings*, zReceiver.ino receiverJson, zRxCheck.ino, TM171.ino, zMachine.ino). A tractor drives
//   up and down a field with autosteer on, or stands parked. Buttons do what the module does, more or less:
//   they refuse while autosteer is on, take a few seconds, then show a result.
// When the firmware's JSON changes (a new field, a new answer), change it here too: the page is always the
// newest one from the firmware, these answers are not.
"use strict";
window.AioSim = (function () {
const BUILD = window.SIM_BUILD || {};
const VERSION = BUILD.version || "4.5";
const FW_NAME = "v" + VERSION + "_AiO_WEB";
const BUILT = BUILD.built || "Oct 25 2026 09:00:00";
const Q = new URLSearchParams(location.search);

// ---- the choices (bar at the top): what is connected to the module, and what the tractor does
const CHOICES = {
 gps:   {label: "GPS", dflt: "um982", options: [["um982", "UM982 (two antennas)"], ["x20p", "X20P"], ["f9p", "F9P"], ["dualf9p", "Dual F9P"]]},
 imu:   {label: "IMU", dflt: "tm171", options: [["tm171", "TM171"], ["bno", "BNO085 (I2C)"], ["rvc", "BNO085 (RVC)"], ["none", "none"]]},
 motor: {label: "Steering", dflt: "cytron", options: [["cytron", "Cytron"], ["ibt2", "IBT2"], ["danfoss", "Danfoss"], ["keya", "Keya motor"]]},
 prog:  {label: "Program", dflt: "aow", options: [["aow", "AgOpenWeb"], ["aog", "AgOpenGPS"]]},
 xiao:  {label: "XIAO", check: true, dflt: "1"},
 mach:  {label: "Machine module", dflt: "nano", options: [["nano", "sprayer module (ESP32 / Nano)"], ["aog", "official Machine_UDP_v5"], ["0", "none"]]},
 impl:  {label: "Implement", dflt: "seeder", options: [["seeder", "seed drill, 16 rows 25 cm apart"], ["sprayer", "sprayer, 8 m, 5 sections"]]},
 park:  {label: "Tractor", dflt: "0", options: [["0", "driving, autosteer on"], ["1", "parked"]]},
 chip:  {label: "XIAO chip", dflt: "c3", options: [["c3", "ESP32-C3"], ["s3", "ESP32-S3"]]},
 wifi:  {label: "Wi-Fi", dflt: "sta", options: [["sta", "on the tractor network"], ["lost", "tractor network lost"], ["ap", "tractor network not set"]]},
};
const opt = {};
for (const [k, c] of Object.entries(CHOICES)) {
 const v = Q.get(k);
 opt[k] = v !== null && (c.check ? v === "0" || v === "1" : c.options.some(o => o[0] === v)) ? v : c.dflt;
}
if (opt.gps === "dualf9p") opt.xiao = "0";             // one thing on the GPS2 port: the second F9P
const OPT_KEY = Object.keys(CHOICES).map(k => opt[k]).join(",");
const STATE_V = 2;

function load(key, fresh) {
 try { const s = JSON.parse(sessionStorage.getItem(key)); if (s && s.v === STATE_V && s.opt === OPT_KEY) return s; } catch (e) {}
 const s = fresh(); s.v = STATE_V; s.opt = OPT_KEY; return s;
}
function save(key, s) { try { sessionStorage.setItem(key, JSON.stringify(s)); } catch (e) {} }
function forget(key) { try { sessionStorage.removeItem(key); } catch (e) {} }

// ---- small helpers
const sleep = ms => new Promise(r => setTimeout(r, ms));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const wrap360 = h => ((h % 360) + 360) % 360;
const r2 = (v, d) => Math.round(v * Math.pow(10, d)) / Math.pow(10, d);
const wob = (t, a, f) => a * Math.sin(t * f) * Math.cos(t * f * 0.37 + 1.1);   // a slow wobble, the same for the same t
const OFFLINE = {};
// the implement the tractor pulls: a section of the machine module per row or per boom section, the width of a pass
const IMPLEMENTS = {
 seeder:  {name: "seed drill", sections: 16, width: 16 * 0.25, nozzles: null},   // 16 rows 25 cm apart (4 m), a relay a row, no sprayer sensors
 sprayer: {name: "sprayer", sections: 5, width: 8, nozzles: [4, 2, 4, 2, 4]},     // 8 m boom, 5 sections with 4 2 4 2 4 nozzles
};
const IMPL = IMPLEMENTS[opt.impl], ALL_SECTIONS = (1 << IMPL.sections) - 1;
const nozzles = bits => IMPL.nozzles ? IMPL.nozzles.reduce((a, n, i) => a + ((bits >> i) & 1) * n, 0) : 0;   // nozzles open
function form(body) {
 const p = new URLSearchParams(typeof body === "string" ? body : ""), o = {};
 for (const [k, v] of p) o[k] = v;
 return o;
}
const ok = j => ({code: 200, json: j || {ok: true}});
const err = (code, e) => ({code: code, json: {ok: false, error: e}});
function nmea(body) { let c = 0; for (let i = 0; i < body.length; i++) c ^= body.charCodeAt(i); return "$" + body + "*" + c.toString(16).toUpperCase().padStart(2, "0"); }
function utc() { const d = new Date(Math.floor(Date.now() / 100) * 100), z = n => String(n).padStart(2, "0"); return z(d.getUTCHours()) + z(d.getUTCMinutes()) + z(d.getUTCSeconds()) + "." + z(Math.floor(d.getUTCMilliseconds() / 10)); }
function ddmm(deg, w) { const a = Math.abs(deg), d = Math.floor(a); return String(d).padStart(w, "0") + ((a - d) * 60).toFixed(7).padStart(10, "0"); }

// ---- the page's requests to its module
// route(method, path, query, body) answers {code, json} or {code, text}, OFFLINE (the module is restarting:
// no connection), or null (not for the module: sent as usual)
function simPath(u) {
 if (u.origin !== location.origin) return null;
 const i = u.pathname.indexOf("/api/");
 return i >= 0 ? u.pathname.slice(i) : /\/update$/.test(u.pathname) ? "/update" : null;
}
function hook(route) {
 const real = window.fetch.bind(window);
 window.fetch = async function (input, init) {
  const u = new URL(typeof input === "string" ? input : input.url, location.href), path = simPath(u);
  if (!path) return real(input, init);
  const method = String((init && init.method) || (typeof input !== "string" && input.method) || "GET").toUpperCase();
  let body = init && init.body !== undefined ? init.body : "";
  if (body instanceof URLSearchParams) body = body.toString();
  await sleep(20 + Math.random() * 40);
  const r = await route(method, path, u.searchParams, body);
  if (r === OFFLINE) throw new TypeError("Failed to fetch");
  if (r === null) return new Response("Not found", {status: 404});
  const text = r.text !== undefined ? r.text : JSON.stringify(r.json);
  return new Response(text, {status: r.code || 200, headers: {"Content-Type": r.text !== undefined ? "text/plain" : "application/json"}});
 };
}
// Uploads (the firmware file): progress at about 300 kB/s, then upload(path, query, body) answers as route() does
function hookXhr(upload) {
 const P = XMLHttpRequest.prototype, open = P.open, send = P.send, header = P.setRequestHeader;
 P.open = function (method, url) {
  const u = new URL(url, location.href), path = simPath(u);
  this._sim = path ? {method: String(method).toUpperCase(), path: path, query: u.searchParams} : null;
  if (!this._sim) return open.apply(this, arguments);
 };
 P.setRequestHeader = function () { if (!this._sim) return header.apply(this, arguments); };
 P.send = function (body) {
  if (!this._sim) return send.apply(this, arguments);
  const x = this, s = this._sim, f = body instanceof FormData ? body.get("file") : body;
  const total = f && f.size !== undefined ? f.size : String(body || "").length;
  const steps = Math.max(10, Math.round(total / 30000)), up = x.upload;
  let n = 0;
  const done = (code, text) => {
   Object.defineProperty(x, "status", {value: code, configurable: true});
   Object.defineProperty(x, "responseText", {value: text, configurable: true});
   Object.defineProperty(x, "readyState", {value: 4, configurable: true});
   if (x.onload) x.onload();
  };
  const step = async () => {
   n++;
   if (up.onprogress) up.onprogress({lengthComputable: true, loaded: Math.min(total, Math.round(total * n / steps)), total: total});
   if (n < steps) { setTimeout(step, 100); return; }
   if (up.onload) up.onload();
   await sleep(1200);                                   // the module checks the file
   const r = await upload(s.path, s.query, body);
   if (r === OFFLINE) { if (x.onerror) x.onerror(); return; }
   done(r.code || 200, JSON.stringify(r.json));
  };
  setTimeout(step, 100);
 };
}

// ---- the strip at the top: "this is a simulator", and the choices (keys of CHOICES, skip: {key: [values not
// offered]}); a change opens the page again with the new choices, and the pretend module starts afresh
// (stateKeys: its sessionStorage keys)
function bar(note, keys, stateKeys, skip) {
 const css = "#simBar{display:flex;flex-wrap:wrap;gap:6px 14px;align-items:center;padding:7px 16px;" +
  "background:color-mix(in srgb,var(--warn) 16%,var(--card));border-bottom:1px solid var(--line);font-size:13.5px}" +
  "#simBar b{color:var(--warn);letter-spacing:.03em}#simBar .n{color:var(--mute);flex:1 1 260px}" +
  "#simBar label{display:flex;gap:6px;align-items:center;white-space:nowrap}#simBar select{padding:3px 6px;font-size:13px}";
 const build = () => {
  const st = document.createElement("style");st.textContent = css;document.head.appendChild(st);
  const b = document.createElement("div");b.id = "simBar";
  const t = document.createElement("b");t.textContent = "SIMULATOR";b.appendChild(t);
  const n = document.createElement("span");n.className = "n";n.textContent = note;b.appendChild(n);
  for (const key of keys) {
   const it = CHOICES[key], l = document.createElement("label"), txt = document.createTextNode(it.label);
   let e;
   if (it.check) {
    e = document.createElement("input");e.type = "checkbox";e.checked = opt[key] === "1";
    if (key === "xiao" && opt.gps === "dualf9p") { e.disabled = true;l.title = "The GPS2 port has the second F9P"; }
    l.appendChild(e);l.appendChild(txt);
   } else {
    e = document.createElement("select");
    for (const [v, t] of it.options) {
     if (skip && skip[key] && skip[key].includes(v)) continue;
     const o = document.createElement("option");o.value = v;o.textContent = t;e.appendChild(o);
    }
    e.value = opt[key];
    l.appendChild(txt);l.appendChild(e);
   }
   e.addEventListener("change", () => {
    const q = new URLSearchParams(location.search), v = it.check ? (e.checked ? "1" : "0") : e.value;
    if (v === it.dflt) q.delete(key); else q.set(key, v);
    for (const k of stateKeys) forget(k);
    location.replace(location.pathname + (q.toString() ? "?" + q.toString() : ""));
   });
   b.appendChild(l);
  }
  const a = document.createElement("a");a.href = "../";a.textContent = "About";b.appendChild(a);
  document.body.insertBefore(b, document.body.firstChild);
 };
 if (document.body) build(); else document.addEventListener("DOMContentLoaded", build);
}

// ---- the AgOpenGPS settings, as the firmware has them (zRxConfig.h; build.py reads them from there)
const UBX_SETS = BUILD.ubxSets || [
 {name: "Single", file: "Single.txt", fw: "HPG 1.13", count: 19, keys: 1212},
 {name: "Dual - Right - Position", file: "DualPosition.txt", fw: "HPG 1.13", count: 19, keys: 1212},
 {name: "Dual - Left - RelPos", file: "DualRelPos.txt", fw: "HPG 1.13", count: 19, keys: 1212},
 {name: "X20P", file: "X20P.txt", fw: "HPG 2.11", count: 2, keys: 32},
];
const UM982D = BUILD.um982d || ["CONFIG HEADING TRACTOR", "CONFIG HEADING VARIABLELENGTH", "CONFIG COM1 460800", "MODE ROVER SURVEY", "GPGGA 0.1", "GPVTG 0.1", "GPHPR 0.1"];
const UM982S = BUILD.um982s || ["CONFIG HEADING VARIABLELENGTH", "CONFIG COM1 460800", "MODE ROVER SURVEY", "GPGGA 0.1", "GPVTG 0.1", "GPHPR 0.1"];
// what the Teensy needs from a u-blox receiver (zRxCheck.ino CK_ESS): id, roles (S single, D dual position,
// R dual relpos, X X20P), the AgOpenGPS value
const CK_ESS = [["ubxIn", "SDRX", 1], ["rtcmIn", "SDX", 1], ["ubxOut", "SDRX", 1], ["nmeaOut", "SDX", 1], ["gga", "SDX", 1], ["vtg", "SDX", 1],
 ["rate", "SDRX", 100], ["navRate", "SDRX", 1], ["baud1", "SDRX", 460800], ["rtcmOut2", "D", 1], ["baud2", "DR", 460800],
 ["r1005", "D", 1], ["r1074", "D", 1], ["r1084", "D", 1], ["r1094", "D", 1], ["r1230", "D", 1], ["r4072", "D", 1], ["relpos", "R", 1], ["rtcmIn2", "R", 1]];
const ROLE_CH = "SDRX";

// ---- the receivers
const RX = {
 um982:   {kind: "um982", model: "UM982", fw: "R4.10Build13495", prot: "", hw: "", role: -1, dual: "um982", maker: "Unicore"},
 x20p:    {kind: "ublox", model: "ZED-X20P", fw: "HPG 2.11", prot: "50.10", hw: "000A0000", role: 3, dual: "", maker: "u-blox"},
 f9p:     {kind: "ublox", model: "ZED-F9P", fw: "HPG 1.13", prot: "27.31", hw: "00190000", role: 0, dual: "", maker: "u-blox"},
 dualf9p: {kind: "ublox", model: "ZED-F9P", fw: "HPG 1.13", prot: "27.31", hw: "00190000", role: 1, dual: "f9p", maker: "u-blox"},
};
const UBX_V = {rate: 100, navRate: 1, dyn: 4, minElev: 10, hp: 1, gga: 1, vtg: 1, rmc: 0, gsa: 0, gsv: 0, gll: 0, gst: 0, zda: 0,
 gps: 1, glo: 1, gal: 1, bds: 1, qzss: 0, sbas: 0, inRtcm: 1};
const UM_V = {rate: 100, mode: 2, elev: 10, rtkTo: 600, sbas: 0, hdMode: 1, hdLen: 0, hdTol: 3, hdOff: 90, ptOff: 0,
 gga: 100, vtg: 100, hpr: 100, rmc: 0, gsa: 0, gsv: 0, gst: 0, zda: 0, ths: 0, ksxt: 0, gps: 1, bds: 1, glo: 1, gal: 1, qzss: 1};
const UBX_NAMES = Object.keys(UBX_V), UM_NAMES = Object.keys(UM_V);

// ---- the field and the tractor
// Up and down a 200 m field, the passes an implement's width apart (4 m seed drill, 8 m sprayer), a U-turn at each end;
// parked: standing at the start.
const FIELD = {lat: 38.5712345, lon: -7.9134567, alt: 212.43, len: 200, width: IMPL.width};
const START = 15, PASS = 90, TURN = 12;     // s: moves off after power-on, one pass, one U-turn
const RUNNING = 30;                         // s: a page opened the first time finds the module on for this long, the tractor on its way
const V = FIELD.len / PASS, RT = FIELD.width / 2;
function tractor(t, parked) {
 if (parked || t < START) return {x: 0, y: 0, hdg: 0, kmh: Math.abs(wob(t, 0.04, 0.9)), steer: wob(t, 0.15, 0.3), turn: 0, straight: false, moving: false, r: 0, k: 0, s: 0};
 const s = t - START, k = Math.floor(s / (PASS + TURN)), r = s - k * (PASS + TURN), up = k % 2 === 0, x0 = k * FIELD.width;
 if (r < PASS) {
  const a = V * r, e = 0.03 * Math.sin(r * 0.9);
  return {x: x0 + e, y: up ? a : FIELD.len - a, hdg: wrap360((up ? 0 : 180) + 0.25 * Math.cos(r * 0.9)), kmh: V * 3.6 + wob(t, 0.05, 1.3),
   steer: 0.9 * Math.sin(r * 0.9) + 0.25 * Math.sin(r * 2.3), turn: 0, straight: true, moving: true, r: r, k: k, s: s};
 }
 const ph = (r - PASS) / TURN * Math.PI, cx = x0 + RT, ramp = Math.min(1, (r - PASS) / 1.5, (PASS + TURN - r) / 1.5);
 return {x: cx - RT * Math.cos(ph), y: up ? FIELD.len + RT * Math.sin(ph) : -RT * Math.sin(ph), hdg: wrap360(up ? ph * 180 / Math.PI : 180 - ph * 180 / Math.PI),
  kmh: Math.PI * RT / TURN * 3.6, steer: (up ? 1 : -1) * 33 * ramp, turn: up ? 1 : -1, straight: false, moving: true, r: r, k: k, s: s};
}
function latLon(x, y) { return [FIELD.lat + y / 111320, FIELD.lon + x / (111320 * Math.cos(FIELD.lat * Math.PI / 180))]; }

// ---- the module (the Teensy)
// key: where its state is kept in this browser tab (sessionStorage), so a reload of the page carries on.
function Module(key) {
 // what is connected (the choices): GPS, IMU, steering, program, XIAO on the GPS2 port, machine module
 const h = {rx: opt.gps, imu: {tm171: "TM171", bno: "BNO085", rvc: "BNO085", none: ""}[opt.imu],
  imuPort: {tm171: "Serial5", bno: "I2C 0x4A", rvc: "Serial5 (RVC)", none: ""}[opt.imu],
  host: opt.prog === "aog" ? "AgOpenGPS" : "AgOpenWeb", motor: {cytron: "Cytron", ibt2: "IBT2", danfoss: "Danfoss", keya: "Keya"}[opt.motor],
  gps2: 0, xiao: opt.xiao === "1", mach: opt.mach !== "0", nano: opt.mach === "nano"};
 const rx = RX[h.rx], parked = opt.park === "1";
 const fresh = () => ({
  t0: Date.now() - RUNNING * 1000, boot: "power", bootN: 1, net: "192.168.5",
  cfg: {outputMode: 2, classicNoDual: 0, hprAcceptFloat: 1, epochWaitMs: 60, tm171SwapRollPitch: 0, tm171YawSign: 0, xiaoOff: h.gps2,
   wasCheckOff: 0, wdtOff: 0, umSingle: 0, simMode: 0, steerMinKmhX10: 2, steerMaxKmh: 25, softStartX10ms: 0, currentKickX10ms: 50},
  wdt: true, saved: true, fwOver: null,
  log: [], logNext: 1, tlAt: -1,
  simSince: -1, heldBase: 0,
  id: {st: "done", at: 4.5},
  ubx: {op: "", end: 0, seq: 1, res: rx.kind === "ublox" ? "read" : "", at: rx.kind === "ublox" ? 5 : -1, v: Object.assign({}, UBX_V), pend: null},
  um: {op: "", end: 0, seq: 1, res: rx.kind === "um982" ? "read" : "", at: rx.kind === "um982" ? 5 : -1, v: Object.assign({}, UM_V), cmds: 0, okN: 0, pend: null},
  umLines: [], umNext: 1,
  ck: {op: "", end: 0, port: 0, t0: 0, auto: false, ports: [ckPortFresh(0), ckPortFresh(1)]},
  tm: {calib: "", kind: "gyro", end: 0, doneAt: -1, n: 0, sw: 0x0008083B, boot: 0, accGain: 350, magGain: 100, inhibit: 10, odr: 25,
   readAt: 4.6, ackAt: -1, pendEnd: 0, pend: null, fixAt: -1},
 });
 function ckPortFresh(p) {
  const role = p === 0 ? rx.role : 2;
  return {chk: p === 0 || rx.dual === "f9p" ? "checked" : "", essAt: role >= 0 ? 6 : -1, cmpAt: -1, load: "", loadAt: -1, loadOk: 0, loadTotal: 0,
   scan: "", found: 0, baud: "", umListed: p === 0 && rx.kind === "um982"};
 }
 const st = load(key, fresh);
 const T = () => (Date.now() - st.t0) / 1000;
 const cfg = st.cfg;
 const isTm = h.imu === "TM171", hasImu = !!h.imu;
 const keya = h.motor === "Keya";
 // the second F9P is on the GPS2 port (dual F9P), a XIAO otherwise; the GPS2 port is automatic (finds it) or off
 const f9p2Here = () => rx.dual === "f9p";
 const f9pOn = () => f9p2Here() && cfg.xiaoOff !== 1;
 const gps2Name = () => cfg.xiaoOff === 1 ? "off" : f9pOn() ? "auto: F9P" : h.xiao ? "auto: XIAO" : "auto";
 const dualRx = () => rx.dual === "um982" ? !cfg.umSingle : f9pOn();
 const hostName = h.host;
 const ip = () => st.net + ".126";
 const fwName = () => st.fwOver ? st.fwOver.fw : FW_NAME;
 const fwVer = () => fwName().replace(/^v/, "").replace(/_AiO_WEB$/, "");
 const built = () => st.fwOver ? st.fwOver.built : BUILT;

 function line(x, t) {
  st.log.push({s: st.logNext++, t: Math.max(0, Math.round((t === undefined ? T() : t) * 1000)), x: x});
  if (st.log.length > 60) st.log.shift();              // LOG_LINES
 }

 // ---- what the receiver gives
 function fixAt(t) {
  if (t < 4) return {gga: false, q: -1};
  if (t < 9) return {gga: true, q: 1, sats: "17", hdop: "1.1", age: ""};
  const w = (t - 9) % 900;                               // a short float once every 15 minutes
  return {gga: true, q: t < 22 || (w > 600 && w < 609) ? 5 : 4, sats: String(26 + Math.round(wob(t, 2, 0.05))),
   hdop: (0.6 + Math.abs(wob(t, 0.12, 0.03))).toFixed(1), age: (0.6 + Math.floor(t * 10 % 10) / 10).toFixed(1)};
 }
 const autosteerOn = t => !parked && t > START + 3;
 const imuOn = t => hasImu && t > 1.6;
 // the dual heading: UM982 HPR (QF 4 fixed, 5 float) from 12 s; second F9P RELPOSNED fixed only
 function dualAt(t, f) {
  if (!dualRx() || t < 4) return {hpr: rx.dual === "um982" && t >= 4 && f.gga, ok: false, qf: 0, why: 1};
  const qf = t < 12 ? 0 : f.q === 4 ? 4 : 5;
  if (rx.dual === "f9p") return {hpr: true, ok: qf === 4, qf: qf, why: qf === 4 ? 0 : 3};
  return {hpr: true, ok: qf === 4 || (qf === 5 && !!cfg.hprAcceptFloat), qf: qf, why: qf ? 0 : 3};
 }

 // ---- the timeline of messages: start-up, then the receiver and the program coming in
 function timeline() {
  const L = [], dual = dualRx(), um = rx.kind === "um982";
  const out = cfg.outputMode === 2 ? "automatic (by the program's hello; classic until then)" : cfg.outputMode === 1 ? "classic $PAOGI/$PANDA" : "standard set";
  const gps2 = cfg.xiaoOff === 1 ? "off" : "auto";
  L.push([0.3, "[boot] " + fwName() + ", built " + built() + " with Teensyduino 1.58"]);
  L.push([0.3, st.boot === "restart" ? "[boot] Start: restart by software (web page, firmware update, or the program's subnet change)" : "[boot] Start: power on (or the Teensy's button, or a USB upload)"]);
  L.push([0.4, "[boot] Settings from the EEPROM: output " + out + ", GPS2 port " + gps2]);
  L.push([0.4, "[boot] Steering: minimum " + (cfg.steerMinKmhX10 / 10).toFixed(1) + " km/h, maximum " + cfg.steerMaxKmh + " km/h, soft start " + cfg.softStartX10ms * 10 + " ms (0 = off), current kickout delay " + cfg.currentKickX10ms * 10 + " ms"]);
  if (cfg.wasCheckOff) L.push([0.4, "[boot] WAS check OFF (bench setting): a broken WAS wire does not stop autosteer"]);
  if (cfg.simMode) L.push([0.4, "[boot] Simulator mode ON (for the program's simulator only): GPS data is not sent to the program"]);
  L.push([0.9, "[boot] Autosteer: ADS1115 (WAS) found, waiting for the program"]);
  L.push([1.1, "[boot] Ethernet: UDP ports 5120 GNSS out, 2233 NTRIP in, 8888 steering in / out, 9999 machine module in"]);
  L.push([1.6, {tm171: "[boot] IMU: TM171 on Serial5", bno: "[boot] IMU: BNO085 on I2C 0x4A (pins 18 / 19)", rvc: "[boot] IMU: BNO085 on Serial5 (RVC mode)",
   none: "[boot] IMU NOT detected: no TM171 or BNO085 on Serial5, no BNO085 on I2C 0x4A / 0x4B - no roll, no IMU heading (still listening on Serial5)"}[opt.imu]]);
  if (!keya) L.push([3.7, "[boot] No Keya heartbeat: steering by the valve / motor driver set in the program (Cytron, IBT2, Danfoss); a Keya found later will be used"]);
  else L.push([2.1, "[status] Keya motor found on the CAN bus (heartbeat): steering by the Keya"]);
  L.push([3.8, "[boot] Setup done in 3.8 s"]);
  if (st.wdt) L.push([3.8, "[boot] Watchdog on: the Teensy restarts if the firmware stops for 2 s"]);
  L.push([4.1, "[status] Ethernet: link up, module IP " + ip() + " (web page http://" + ip() + "/), sending to " + st.net + ".255:9999"]);
  L.push([4.5, "[status] GPS receiver: " + rx.maker + " " + rx.model + ", firmware " + rx.fw]);
  if (isTm) L.push([4.6, "[status] TM171: settings read from ID 1 (switches 0x" + (st.tm.sw >>> 0).toString(16).toUpperCase().padStart(8, "0") + ", UART 115200 bps, inhibit " + st.tm.inhibit + " ms, " + st.tm.odr + " Hz; bytes " + tmRaw() + ")"]);
  L.push([5.0, "[status] GPS: GPS (fix 1, 17 satellites)"]);
  const p0 = latLon(0, 0);
  L.push([5.0, "[status] GPS: first position " + p0[0].toFixed(7) + ", " + p0[1].toFixed(7) + ", altitude " + FIELD.alt.toFixed(2) + " m"]);
  L.push([5.0, "[status] GPS: VTG arriving, speed from the receiver"]);
  if (h.xiao && cfg.xiaoOff !== 1) L.push([5.2, "[xiao] XIAO said hello on Serial2: AgOpenWeb packets (UDP 8888) now go to it, its PGNs go to UDP 9999"]);
  if (f9pOn()) L.push([1.6, "[status] GPS2 port: second F9P (dual F9P) - a GNSS receiver sends valid data on it"]);
  if (f9pOn()) L.push([5.4, "[status] Second F9P (GPS2 port): RELPOSNED arriving, baseline 121.4 cm"]);
  if (h.mach) L.push([5.5, "[machine] machine module " + st.net + ".123 heard on UDP 9999 (web page: Machine tab)"]);
  const classic = cfg.outputMode === 1 || (cfg.outputMode === 2 && hostName === "AgOpenGPS");
  L.push([6.2, cfg.outputMode === 2 ? "[status] Program: " + hostName + " (" + (hostName === "AgOpenGPS" ? "AgIO hello" : "hello") + " from " + st.net + ".10). Automatic mode: " +
   (classic ? "classic sentences ($PAOGI with the UM982, $PANDA with F9P / X20P)" : "standard sentences ($GNGGA $GNVTG $GNTHS $INHPR) from now on")
   : "[status] Program: " + hostName + " (hello from " + st.net + ".10), output mode " + (cfg.outputMode === 1 ? "AgOpenGPS (classic)" : "AgOpenWeb (standard)") + " (set on the web page)"]);
  L.push([6.3, "[status] " + hostName + " connected, IP " + st.net + ".10"]);
  L.push([7.5, "[status] RTCM corrections arriving from " + hostName + " (NTRIP)"]);
  L.push([9.0, "[status] GPS: RTK float (fix 5, 25 satellites, correction age 1.0 s)"]);
  L.push([9.5, "[status] GPS: the receiver sends (last 5 s): " + rxTypes()]);
  if (hasImu) L.push([10.0, "[status] " + h.imu + ": data OK (roll, and heading without dual antenna)"]);
  if (dual && rx.dual === "um982" && cfg.hprAcceptFloat) L.push([12.0, "[status] Dual: dual-antenna heading OK (HPR QF 5 float), sent as " + (classic ? "$PAOGI" : "$GNTHS")]);
  L.push([22.0, "[status] GPS: RTK fixed (fix 4, 26 satellites, correction age 0.8 s)"]);
  if (dual) L.push([22.0, "[status] Dual: dual-antenna heading OK (" + (rx.dual === "f9p" ? "second F9P, RELPOSNED, RTK" : "HPR QF 4") + " fixed), sent as " + (classic ? "$PAOGI" : "$GNTHS")]);
  if (!parked && hasImu) {
   if (!dual) L.push([START + 6, "[status] " + h.imu + ": heading aligned to the GPS course (no dual heading yet; forward travel assumed)"]);
   L.push([START + PASS + TURN + 1, "[status] " + h.imu + ": yaw direction learned from the " + (dual ? "dual heading" : "GPS course") + ": +1 (it can be set on the Settings page to skip learning)"]);
  }
  return L;
 }
 // the messages from tA (exclusive) to tB: the timeline, then a float and back to fixed every 15 minutes
 function advanceLog(tB) {
  const tA = st.tlAt;
  for (const [t, x] of timeline()) if (t > tA && t <= tB) line(x, t);
  for (let n = Math.max(0, Math.floor((tA - 609) / 900)); 609 + 900 * n <= tB + 900; n++) {
   const a = 9 + 900 * n + 601, b = 9 + 900 * n + 609;
   if (a > tA && a <= tB) line("[status] GPS: RTK float (fix 5, 25 satellites, correction age 1.2 s)", a);
   if (b > tA && b <= tB) line("[status] GPS: RTK fixed (fix 4, 26 satellites, correction age 0.9 s)", b);
  }
  st.tlAt = tB;
 }
 function rxTypes() {
  if (rx.kind === "um982") return "GNGGA 10.0 Hz, GNHPR 10.0 Hz, GNVTG 10.0 Hz";
  return "GNGGA 10.0 Hz, GNVTG 10.0 Hz";
 }

 // ---- things that take a while (receiver reads and writes, checks, calibrations): finish them in time
 function progress() {
  const t = T();
  if (t < 0) return;
  advanceLog(t);
  if (st.id.st === "ask" && t >= st.id.at + 1.2) { st.id = {st: "done", at: t}; line("[status] GPS receiver: " + rx.maker + " " + rx.model + ", firmware " + rx.fw); }
  const u = st.ubx;
  if (u.op && t >= u.end) {
   if (u.op === "write") { Object.assign(u.v, u.pend); line("[status] GPS receiver: settings written to RAM, BBR and flash"); u.res = "write"; }
   else if (u.op === "read") u.res = "read";
   else if (u.op === "ubx-on") { line("[status] GPS receiver: UBX output on UART1 switched on"); u.res = ""; }
   u.op = ""; u.seq++; u.at = t; u.pend = null;
  }
  const m = st.um;
  if (m.op && t >= m.end) {
   if (m.op === "write") { Object.assign(m.v, m.pend); m.res = m.cmds ? "write" : "write-none"; line("[status] GPS receiver: UM982 settings written (" + m.okN + " of " + m.cmds + " commands), SAVECONFIG done"); }
   else m.res = "read";
   m.op = ""; m.seq++; m.at = t; m.pend = null;
   st.ck.ports[0].umListed = true;
  }
  const c = st.ck;
  if (c.op && t >= c.end) {
   const p = c.ports[c.port];
   if (c.op === "check") { p.chk = "checked"; p.essAt = t; if (rx.kind === "ublox" || c.port === 1) p.cmpAt = t; }
   else if (c.op === "scan") { p.scan = "found"; p.found = 460800; }
   else if (c.op === "baud") p.baud = "baud-ok";
   else if (c.op === "load") {
    p.load = "loaded"; p.loadAt = t; p.loadTotal = p.loadOk = c.msgs; p.chk = "checked"; p.essAt = t; p.cmpAt = t;
    line("[status] " + (c.port ? "Second F9P" : "GPS") + ": AgOpenGPS settings loaded (" + c.file + "): " + c.msgs + " of " + c.msgs + " taken");
   }
   c.op = "";
  }
  const tm = st.tm;
  if (tm.calib === "starting" && t >= tm.end - 20) tm.calib = "running";
  if (tm.calib === "running" && t >= tm.end) { tm.calib = tm.kind === "install" ? "ok" : "finishing"; if (tm.kind === "install") { tm.doneAt = t; tm.n++; line("[status] TM171: new install calibration saved to flash; switch the whole module off and on"); } }
  if (tm.calib === "finishing" && t >= tm.end + 1) { tm.calib = "ok"; tm.doneAt = t; tm.n++; line("[status] TM171: gyro calibration done (kept until the TM171 is powered off)"); }
  if (tm.pend && t >= tm.pendEnd) {
   if (tm.pend.write) {
    Object.assign(tm, tm.pend.v); tm.ackAt = t;
    line("[status] TM171: settings written to ID 1 (switches 0x" + (tm.sw >>> 0).toString(16).toUpperCase().padStart(8, "0") + ", gains " + tm.accGain + "/" + tm.magGain + ", " + tm.odr + " Hz" + (tm.pend.save ? ", saved to flash" : "") + ")");
   }
   if (tm.pend.fix) tm.fixAt = t;
   tm.readAt = t; tm.pend = null;
  }
 }

 // ---- the TM171 settings object, as bytes (TM171.ino tm171SettingHex)
 function tmRaw() {
  const tm = st.tm, b = [];
  const u16 = v => { b.push(v & 255, (v >> 8) & 255); }, u32 = v => { u16(v & 0xFFFF); u16((v >>> 16) & 0xFFFF); };
  u32(tm.sw); u16(0); u16(1152); u16(0); u16(tm.accGain); u16(tm.magGain); u16(tm.inhibit); u16(0); u16(tm.odr);
  return b.map(x => x.toString(16).toUpperCase().padStart(2, "0")).join(" ");
 }
 function tmSetJson(t) {
  const tm = st.tm, w = tm.sw >>> 0, bit = n => !!((w >>> n) & 1);
  return {valid: true, odd: false, age: Math.max(0, Math.floor(t - tm.readAt)), pending: !!tm.pend, ackAge: tm.ackAt >= 0 ? Math.floor(t - tm.ackAt) : -1,
   noAnswer: 0, rate: tm.odr, baudNow: 115200, id: 1, noWire: false, setupWhy: "", fixAgo: tm.fixAt >= 0 ? Math.floor(t - tm.fixAt) : -1,
   sw: w, gyro: bit(0), acc: bit(1), mag: bit(2), gyroErr: bit(3), selfAdapt: bit(4), status: bit(5), raw: bit(6), quat: bit(8),
   euler: bit(10), rpy: bit(11), gravity: bit(12), uart: bit(19), can: bit(21), boot: (w >>> 24) & 3, baud: 115200,
   accGain: tm.accGain, magGain: tm.magGain, inhibit: tm.inhibit, odr: tm.odr, w4: 0, w8: 0, w16: 0, raw: tmRaw()};
 }

 // ---- the receiver check's summary for a port (zRxCheck.ino ckPortJson / rxCkStatusJson)
 function ckRole(p) { return p === 0 ? rx.role : 2; }
 function ckPortOn(p) { return p === 0 || f9pOn(); }
 function ckHealth(p) {
  if (p === 1 && !f9pOn()) return "wait";
  if (p === 1 && !f9p2Here()) return "none";          // GPS2 set to a second F9P, but a XIAO is there
  return T() < 3 ? "wait" : "ok";
 }
 function ckPort(p, t) {
  const k = st.ck.ports[p], um = p === 0 && rx.kind === "um982", role = ckRole(p), set = um ? null : UBX_SETS[role];
  const health = ckHealth(p), live = health === "ok";
  const ess = [];
  if (!um && k.essAt >= 0 && live) for (const [id, roles, want] of CK_ESS) if (roles.includes(ROLE_CH[role])) ess.push({id: id, st: 1, v: want, want: want});
  const hz = p === 0 ? (live ? (um ? [100, 100, 100] : [100, 100, 0]) : [0, 0, 0]) : (live ? [100, 0] : [0, 0]);
  return {on: ckPortOn(p), health: health, bytes: live ? 5 * (p === 0 ? 7200 : 1700) : 0, good: live ? (p === 0 ? (um ? 150 : 100) : 50) : 0, hz: hz, at: 460800,
   model: live ? (p === 0 ? rx.model : "ZED-F9P") : "", fw: live ? (p === 0 ? rx.fw : "HPG 1.13") : "", fwSt: !um && live ? "same" : "",
   set: !um && !!set, role: set ? set.name : "", file: set ? set.file : "", setFw: set ? set.fw : "", keys: set ? set.keys : 0,
   essAgo: !um && k.essAt >= 0 && live ? Math.floor(t - k.essAt) : -1, ess: ess,
   cmpAgo: !um && k.cmpAt >= 0 ? Math.floor(t - k.cmpAt) : -1, same: !um && k.cmpAt >= 0 ? set.keys : 0, diff: 0, unk: 0,
   umListed: um && k.umListed && st.um.at >= 0, umD: um ? umMask(UM982D) : 0, umS: um ? umMask(UM982S) : 0,
   boot: live ? "ok" : "wait", scan: k.scan, found: k.found, foundKind: k.found ? (um ? "unicore" : "ublox") : "", baud: k.baud, chk: k.chk,
   load: k.load, loadFile: set ? set.file : cfg.umSingle ? "ConfigUM982S.txt" : "ConfigUM982D.txt",
   loadOk: k.loadOk, loadRam: 0, loadNak: 0, loadNone: 0, loadTotal: k.loadTotal, loadAgo: k.loadAt >= 0 ? Math.floor(t - k.loadAt) : -1,
   dump: "", dumpN: 0, dumpRead: 0, dumpFull: false, dumpAgo: -1};
 }
 // the UM982 has every AgOpenGPS command of the two-antenna file (and the one-antenna file's, but its smoother heading)
 function umMask(list) { let m = 0; list.forEach((c, i) => { if (!/SMOOTH HEADING 10/.test(c)) m |= 1 << i; }); return m >>> 0; }
 function ckStatus(p, t) {
  const k = ckPort(p, t), c = st.ck, here = !!c.op && c.port === p, um = p === 0 && rx.kind === "um982";
  return {um: um, file: um ? (cfg.umSingle ? "ConfigUM982S.txt" : "ConfigUM982D.txt") : k.file, health: k.health, model: k.model, fw: k.fw, fwSt: k.fwSt,
   found: k.scan === "found" ? k.found : 0, notFound: false, noAnswer: false, checked: k.health === "ok" && (um ? k.chk === "checked" : k.essAgo >= 0),
   miss: "", at: 460800, op: here ? c.op : "", auto: false, autoDid: "", baud: k.baud, load: k.load, loadOk: k.loadOk, loadTotal: k.loadTotal, loadAgo: k.loadAgo};
 }

 // ---- /api/status (zWeb.ino statusJson); full = the web page's copy, not = the XIAO's (zXiao.ino)
 function status(full) {
  const t = Math.max(0, T()), up = Math.floor(t), f = fixAt(t), tr = tractor(t, parked), d = dualAt(t, f), um = rx.kind === "um982";
  const [lat, lon] = latLon(tr.x, tr.y), alt = FIELD.alt + wob(t, 0.02, 0.4);
  const imuOk = imuOn(t), aligned = imuOk && (dualRx() ? t > 12 : !parked && t > START + 6), locked = cfg.tm171YawSign !== 0 || (!parked && t > START + PASS + TURN + 1);
  const IMU_OFF = 47.3;
  let roll = 1.4 * Math.sin(t / 9) + 0.35 * Math.sin(t * 1.7) + tr.turn * 0.8, pitch = -0.9 + 0.5 * Math.sin(t / 13);
  if (cfg.tm171SwapRollPitch) [roll, pitch] = [pitch, roll];
  const yaw = wrap360(tr.hdg - IMU_OFF + wob(t, 0.3, 0.2)), outH = aligned ? wrap360(tr.hdg + wob(t, 0.2, 0.25)) : yaw;
  const engaged = autosteerOn(t), sp = engaged ? tractor(t + 0.25, parked).steer : 0, was = tr.steer + wob(t, 0.08, 2.1);
  const sc = steerCfg(), host = t >= 6 ? hostName : "";
  const classic = cfg.outputMode === 1 || (cfg.outputMode === 2 && host === "AgOpenGPS"), gga = f.gga;
  const hdgStr = d.ok ? wrap360(tr.hdg + wob(t, 0.15, 0.6)).toFixed(4).padStart(9, "0") : "";
  const sim = !!cfg.simMode;
  const held = sim ? Math.floor(st.heldBase + (t - st.simSince) * 10) : Math.floor(st.heldBase);
  const epochs = gga ? Math.max(0, Math.floor((t - 4) * 10) - held) : 0;
  const rtcm = t >= 7.5 && t > 0, rtcmT = Math.max(0, t - 7.5);
  const wasRaw = Math.round(6805 + was * sc.counts / 2), wasV = wasRaw / 2668;
  const kick = sc.pressure || sc.current ? sc.kickMax : 0;
  const j = {fw: fwName() + " " + fwVer(), built: built(), target: "fw_teensy41", marker: "AiO-WEB-OTA-1", up: up, ip: ip()};
  j.net = {link: true, aog: t >= 6, aogIp: t >= 6 ? st.net + ".10" : "0.0.0.0", host: host, hostBoth: false, helloAgo: t >= 6 ? 0 : -1,
   rtcm: rtcm, rtcmKB: r2(rtcmT * 1.31, 1), rtcmBps: rtcm ? Math.round(1310 + wob(t, 60, 0.2)) : 0, rtcmOk: Math.floor(rtcmT * 4.6), rtcmBad: t > 60 ? 1 : 0,
   rtcmTypes: rtcm ? (um ? "1006 1008 1033 1075 1085 1095 1125 1230" : "1005 1074 1084 1094 1124 1230") : "",
   radio: false, radioKB: 0, radioBps: 0, radioOk: 0, radioBad: 0, radioTypes: ""};
  j.gnss = {gga: gga, fix: gga ? f.q : -1, fixName: gga ? ["no fix", "GPS", "DGPS", "other", "RTK fixed", "RTK float"][f.q] : "no data", sats: gga ? f.sats : "", hdop: gga ? f.hdop : "",
   age: gga ? f.age : "", lat: gga ? r2(lat, 7) : null, lon: gga ? r2(lon, 7) : null, alt: gga ? alt.toFixed(2) : "", kmh: gga ? r2(tr.kmh, 2) : 0, speedSrc: gga ? "VTG" : "-"};
  if (full) j.gnss.rx = t >= 5 ? rxTypes() : "";
  j.gnss.model = t >= 4.5 ? rx.model : "";
  if (full) {
   j.gnss.rxKind = t >= 4.5 ? rx.kind : ""; j.gnss.rxFw = t >= 4.5 ? rx.fw : "";
   j.gnss.ckH = ckHealth(0); j.gnss.ck2H = ckHealth(1);
   j.gnss.single = !dualRx(); j.gnss.ck = ckStatus(0, t);
  }
  j.gnss.crcErr = 0;
  j.dual = {hpr: d.hpr, ok: d.ok, qf: d.ok ? d.qf : 0, lastQf: d.qf, heading: hdgStr, why: d.ok ? 0 : d.hpr ? (d.why || 3) : 1, antRoll: d.hpr ? r2(roll + wob(t, 0.1, 0.7), 2) : 0};
  if (full) {
   if (rx.dual === "um982" && d.hpr) j.dual.raw = nmea("GNHPR," + utc() + "," + (hdgStr || "000.0000") + "," + (pitch).toFixed(4) + "," + (roll).toFixed(4) + "," + d.qf + "," + f.sats + "," + (f.age || "0.0") + ",0000");
   else if (rx.dual === "f9p" && d.hpr) j.dual.raw = "RELPOSNED iTOW " + Math.floor(Date.now() % 604800000) + ", heading " + wrap360(tr.hdg + 90).toFixed(2) + ", baseline 121.4 cm, down " + (roll * 2.1).toFixed(1) + " cm, carrSoln " + (d.ok ? 2 : 1) + ", flags 0x" + (d.ok ? "13F" : "0BF");
   else j.dual.raw = "";
  }
  j.dual.late = 0;
  j.dual.src = f9pOn() ? "f9p" : rx.dual === "um982" ? "um982" : "";
  if (full) {
   const on = f9pOn() && t > 1.6, here = on && t > 5;
   j.dual.f9p2 = {on: on, ago: here ? 0 : -1, frames: here ? Math.floor((t - 5) * 10) : 0, bad: 0, bytes: here ? Math.floor((t - 5) * 720) : 0, nmea: 0, nmeaAgo: -1,
    port: cfg.xiaoOff === 1 ? "off" : on ? "F9P" : h.xiao && t > 5.2 ? "XIAO" : "auto", seen: cfg.xiaoOff === 1 && f9p2Here() && t > 5,
    heading: here ? r2(wrap360(tr.hdg + 90), 2) : 0, baseline: here ? 121.4 : 0, roll: here ? r2(roll, 2) : 0, carr: here ? (d.ok ? 2 : 1) : 0, flags: here ? (d.ok ? 0x13F : 0x0BF) : 0};
   j.dual.ck2 = ckStatus(1, t);
  }
  j.tm = {type: imuOk ? h.imu : "", port: imuOk ? h.imuPort : "", ok: imuOk, seen: imuOk,
   roll: imuOk ? r2(roll, 2) : 0, pitch: imuOk ? r2(pitch, 2) : 0, yaw: imuOk ? r2(yaw, 2) : 0, out: imuOk ? r2(outH, 2) : 0,
   sign: cfg.tm171YawSign || 1, locked: locked, aligned: aligned, offset: aligned ? IMU_OFF : 0, crcErr: 0};
  const tm = st.tm, busy = ["starting", "running", "finishing"].includes(tm.calib);
  Object.assign(j.tm, {calib: tm.calib, calibKind: tm.kind, calibLeft: tm.calib === "running" ? Math.max(0, Math.ceil(tm.end - t)) : -1,
   calibAgo: tm.doneAt >= 0 && !busy ? Math.floor(t - tm.doneAt) : -1, calibN: tm.n});
  if (isTm) j.tm.set = tmSetJson(t);
  j.steer = {was: r2(was, 2), set: r2(sp, 2), pwm: engaged ? Math.round(clamp((sp - was) * sc.kp * 3 + Math.sign(sp - was) * sc.minPwm, -sc.hiPwm, sc.hiPwm)) : 0,
   switchOn: engaged, active: engaged, moving: !parked && tr.moving && !sim, motor: h.motor, aogKmh: t >= 6 ? r2(tr.kmh, 1) : 0, guidance: engaged ? 1 : 0,
   adc: true, wasRaw: wasRaw, wasV: r2(wasV, 2), wasOut: false, wasBlock: false, wasZero: false, wasCheck: !cfg.wasCheckOff,
   sensor: sc.pressure ? "pressure" : sc.current ? "current" : "", sensorV: kick ? Math.round(engaged ? 14 + 6 * Math.abs(Math.sin(t / 3)) : 2) : 0,
   sensorMax: kick ? 31 : 0, kick: kick, enc: -1, off: "", offAgo: -1, offVal: 0, wasless: false, keyaLost: false, keyaBlock: false};
  if (full) j.steer.keya = keya ? {found: true, lost: false, block: false, lostCount: 0, ago: 12, pos: Math.round(was * 25), speed: engaged ? Math.round((sp - was) * 20) : 0,
   set: engaged ? Math.round((sp - was) * 22) : 0, current: engaged ? Math.round(4 + Math.abs(sp - was) * 3) : 1, slip: 0, faults: "",
   gpsWas: r2(tr.steer + wob(t, 0.4, 0.5), 1), gpsWasOk: tr.kmh > 1}
   : {found: false, lost: false, block: false, lostCount: 0, ago: -1, pos: 0, speed: 0, set: 0, current: 0, slip: 0, faults: "", gpsWas: 0, gpsWasOk: false};
  if (full) {
   const got = t >= 6.5 ? Math.floor(t - 6.5) : -1;
   j.steerCfg = Object.assign({got252: got, got251: got, got247: hostName === "AgOpenWeb" ? got : -1, wheelbase: hostName === "AgOpenWeb" && got >= 0 ? 2.85 : 3.2}, sc);
   j.boot = {why: st.boot, srsr: st.boot === "restart" ? "0x10" : "0x01", addr: "", step: "", wdt: st.wdt, wdtOff: !!cfg.wdtOff, hwMsgs: 0, hwMsg: ""};
  }
  const dualNow = dualRx();
  j.out = {mode: classic ? "classic" : "standard", auto: cfg.outputMode === 2, kind: !gga ? 0 : classic ? (dualNow ? 2 : 3) : 1,
   aogSrc: !classic ? "" : dualNow ? "Dual" : "Fix", classicSrc: classic && dualNow ? (d.ok ? 1 : imuOk ? 2 : 4) : 0,
   epochs: epochs, sim: sim, simHeld: sim, held: held};
  if (full) j.out.lines = gga && !sim ? sentences(t, f, tr, d, hdgStr, lat, lon, alt, roll, pitch, outH, imuOk, classic, dualNow) : [];
  const xHere = h.xiao && cfg.xiaoOff !== 1 && t > 5.2, xt = Math.max(0, t - 5.2);
  j.xiao = {on: cfg.xiaoOff !== 1 && !f9pOn(), port: cfg.xiaoOff === 1 ? 1 : f9pOn() ? 2 : 0, det: xHere, helloAgo: xHere ? up % 2 : -1, hellos: xHere ? Math.floor(xt / 2) + 1 : 0,
   to: xHere ? Math.floor(xt * 13) : 0, from: xHere ? Math.floor(xt / 5) + 3 : 0, lastPgn: xHere ? 143 : -1, crcErr: 0, cut: 0, drop: 0,
   rxKB: r2(xHere ? xt * 0.05 : 0, 1), txKB: r2(xHere ? xt * 1.9 : 0, 1), text: 0};
  if (full) j.xiao.info = xHere ? JSON.stringify(xiaoInfo(up)) : "";
  j.xiao.infoAgo = xHere ? up % 5 : -1;
  if (full && h.mach && t > 5.5) j.mach = machine(t, tr, engaged);
  return j;
 }
 function xiaoInfo(up) {
  return {v: VERSION, c: opt.chip === "s3" ? "ESP32-S3" : "ESP32-C3", ap: "XIAO-3F2A", apIp: "192.168.4.1", cl: 1, sta: "Farm WiFi", st: "ok", ip: "192.168.1.57", rssi: -61, up: up};
 }
 function steerCfg() {
  const base = {kp: 12, hiPwm: 235, minPwm: 9, lowPwm: 28, counts: 134, offset: -36, ackerman: 97, invertWas: false, singleWas: true, relayHigh: false,
   motorDir: false, cytron: true, danfoss: false, sw: false, btn: true, encoder: false, pressure: true, current: false, kickMax: 92, yAxis: false};
  // a valve (Cytron, Danfoss) with a pressure sensor; a motor (IBT2, Keya) with the current sensor (the Keya: its slip)
  if (h.motor === "Danfoss") Object.assign(base, {cytron: false, danfoss: true, sw: true, btn: false, counts: 118, offset: 12});
  if (h.motor === "IBT2") Object.assign(base, {cytron: false, pressure: false, current: true, kickMax: 70, sw: true, btn: false, counts: 118, offset: 12});
  if (h.motor === "Keya") Object.assign(base, {kp: 40, minPwm: 20, lowPwm: 30, counts: 1, offset: 0, cytron: false, pressure: false, current: true, kickMax: 60});
  return base;
 }
 // the sentences sent to the program (zHandlers.ino sendStandard / sendClassic)
 function sentences(t, f, tr, d, hdgStr, lat, lon, alt, roll, pitch, outH, imuOk, classic, dualNow) {
  const time = utc(), la = ddmm(lat, 2), lo = ddmm(lon, 3), ns = lat >= 0 ? "N" : "S", ew = lon >= 0 ? "E" : "W";
  const kmh = tr.kmh.toFixed(3), kn = (tr.kmh / 1.852).toFixed(3), track = tr.hdg.toFixed(3);
  if (!classic) return [
   nmea("GNGGA," + time + "," + la + "," + ns + "," + lo + "," + ew + "," + f.q + "," + f.sats + "," + f.hdop + "," + alt.toFixed(3) + ",M,52.104,M," + f.age + ",0000"),
   nmea("GNVTG," + track + ",T,,M," + kn + ",N," + kmh + ",K,A"),
   nmea(d.ok ? "GNTHS," + hdgStr + ",A" : "GNTHS,,V"),
   nmea(imuOk ? "INHPR," + time + "," + outH.toFixed(2) + "," + roll.toFixed(2) + "," + pitch.toFixed(2) + ",4,,," : "INHPR," + time + ",,,,0,,,")];
  const common = time + "," + la + "," + ns + "," + lo + "," + ew + "," + f.q + "," + f.sats + "," + f.hdop + "," + alt.toFixed(3) + "," + f.age + "," + kn + ",";
  if (dualNow) return [nmea("PAOGI," + common + (d.ok ? hdgStr : outH.toFixed(2)) + "," + roll.toFixed(2) + "," + pitch.toFixed(2) + ",0")];
  return [nmea("PANDA," + common + (imuOk ? String(Math.round(outH * 10) % 3600) : "65535") + "," + Math.round(roll * 10) + "," + Math.round(pitch * 10) + ",0")];
 }
 // the sprayer at time t: the sections the program asks for (PGN 254 / 239: on while the tractor sprays a pass), the ones
 // open a moment later (sec: the module lags 0.6 s): every row of the seed drill, or every section of the sprayer's boom,
 // then its pressure and flow (0.8 L/min a nozzle at 3 bar) and the litres since the module started; the seed drill has no
 // such sensors on the module. Shared with the sprayer module's own page (sim-sprayer.js).
 function sprayer(t, tr, engaged) {
  const cmdAt = x => { const q = tractor(x, parked); return engaged && q.straight && q.r > 1 && q.r < PASS - 1 ? ALL_SECTIONS : 0; };
  const cmd = cmdAt(t), sec = cmdAt(t - 0.6);
  if (!IMPL.nozzles) return {cmd: cmd, sec: sec, bar: 0, lmin: 0, total: 0, main: engaged};
  const bar = sec ? 3.0 + wob(t, 0.08, 0.7) : engaged ? 3.3 : 0.1, lmin = nozzles(sec) * 0.8 * Math.sqrt(bar / 3);
  const passes = tr.moving ? Math.floor(tr.s / (PASS + TURN)) : 0, spray = tr.moving ? clamp(tr.r - 1.6, 0, PASS - 2) : 0;
  return {cmd: cmd, sec: sec, bar: bar, lmin: lmin, total: (passes * (PASS - 2) + spray) * nozzles(ALL_SECTIONS) * 0.8 / 60, main: engaged};
 }
 // the machine module (zMachine.ino machineJson): the sprayer module (ESP32, or the Nano: hello reply, PGN 237 and PGN 160 with
 // the readings) or the official Machine_UDP_v5 (hello reply without a checksum, PGN 237 five times a second, no PGN 160);
 // the seed drill's ESP32 is set to no sensors on its page, so it sends no PGN 160 either (sections from the hello reply);
 // secN = the highest section asked for or reported since the module (here: the page) started, as mach.secTop
 let machTop = 0;
 function machine(t, tr, engaged) {
  const sp = sprayer(t, tr, engaged), hellos = Math.floor(t - 5.5), used = sp.cmd | sp.sec;
  for (let k = 16; k > machTop; k--) if ((used >> (k - 1)) & 1) { machTop = k; break; }
  const p160 = h.nano && !!IMPL.nozzles;
  const j = {ip: st.net + ".123", here: true, ago: 0, pkts: hellos * (p160 ? 7 : 6), bad: 0, lastPgn: p160 ? 160 : 237, cmd: sp.cmd,
   helloAgo: 0, helloN: hellos, helloNoCrc: !h.nano, secAgo: 0, sec: sp.sec, secN: machTop, secFrom160: p160, sprayAgo: -1};
  if (!p160) return j;
  return Object.assign(j, {sprayAgo: 0, sprayN: hellos, bar: r2(sp.bar, 2), lmin: r2(sp.lmin, 2), total: r2(sp.total, 1), main: sp.main});
 }

 // ---- /api/gps (zReceiver.ino receiverJson, zRxCheck.ino rxCkJson)
 function gps() {
  const t = Math.max(0, T()), u = st.ubx, m = st.um, c = st.ck, um = rx.kind === "um982", ub = rx.kind === "ublox";
  const ubxV = ub && u.at >= 0 ? u.v : {};
  const umV = um && m.at >= 0 ? m.v : {};
  const ckOp = c.op ? c.op : "";
  const msg = c.op === "load" || c.op === "check" ? Math.min(c.msgs - 1, Math.floor((t - c.t0) / (c.end - c.t0) * c.msgs)) : 0;
  return {ok: true, kind: t >= 4.5 ? rx.kind : "", model: t >= 4.5 ? rx.model : "", fw: t >= 4.5 ? rx.fw : "", prot: ub ? rx.prot : "", hw: ub ? rx.hw : "",
   id: st.id.st, idAgo: st.id.st === "done" ? Math.floor(t - st.id.at) : -1, talks: t >= 4, hpr: um,
   ubx: {op: u.op, seq: u.seq, res: ub ? u.res : "", readAgo: ub && u.at >= 0 ? Math.floor(t - u.at) : -1, frames: ub ? Math.floor(t * 2) : 0, bad: 0,
    v: ubxV, na: []},
   um: {op: m.op, seq: m.seq, res: um ? m.res : "", readAgo: um && m.at >= 0 ? Math.floor(t - m.at) : -1, port: um ? 1 : 0,
    baud: um ? {COM1: 460800, COM2: 460800, COM3: 460800} : {}, nmea: um ? "V410" : "", v: umV, cmds: m.cmds, okN: m.okN, fail: []},
   ck: {op: ckOp, port: c.port, dumpRead: 0, dumpN: 0, baudTry: c.op === "scan" ? [9600, 19200, 38400, 57600, 115200, 230400, 460800][Math.min(6, Math.floor((t - c.t0) / 0.6))] : 0,
    msg: c.op === "load" || (c.op === "check" && ub) ? msg : 0, msgs: c.op === "load" || (c.op === "check" && ub) ? c.msgs : 0, umSingle: !!cfg.umSingle,
    um982d: UM982D, um982s: UM982S, ports: [ckPort(0, t), ckPort(1, t)]}};
 }
 const steering = () => autosteerOn(Math.max(0, T()));
 const ENGAGED = "autosteer is engaged - disengage it first";
 function gpsPost(b) {
  const t = T(), a = b.act || "";
  if (!a) return err(400, "no action");
  if (a.startsWith("ck")) {
   const port = b.port === "1" ? 1 : 0, what = a.slice(2);
   if (a !== "ckcheck" && a !== "ckdump" && steering()) return err(409, ENGAGED);
   if (st.ck.op) return err(409, "busy");
   if (!["check", "scan", "baud", "load"].includes(what)) return err(400, "unknown action");
   if (port === 1 && !f9pOn()) return err(400, "no second F9P on the GPS2 port");
   const um = port === 0 && rx.kind === "um982", set = um ? null : UBX_SETS[ckRole(port)];
   const n = what === "load" ? (um ? (b.um === "s" ? UM982S.length : UM982D.length) : set.count) : what === "check" ? (um ? 1 : set.count) : 1;
   const dur = {check: um ? 2 : 4, scan: 4.5, baud: 2.5, load: um ? n * 0.25 : n * 0.35}[what];
   Object.assign(st.ck, {op: what, port: port, t0: t, end: t + dur, msgs: n, file: um ? (b.um === "s" ? "ConfigUM982S.txt" : "ConfigUM982D.txt") : set.file});
   if (what === "load" && um) cfg.umSingle = b.um === "s" ? 1 : 0;
   return ok();
  }
  if (st.ck.op && st.ck.port === 0) return err(409, "busy");
  const u = st.ubx, m = st.um;
  if (a === "identify") { st.id = {st: "ask", at: t}; line("[web] GPS receiver: asked again what it is"); return ok(); }
  if (a === "read" || a === "ubxon" || a === "write") {
   if (rx.kind !== "ublox") return err(400, "not a u-blox receiver");
   if (u.op) return err(409, "busy");
   if (a === "write") {
    if (steering()) return err(409, ENGAGED);
    const v = {}, list = [];
    for (const n of UBX_NAMES) if (b[n] !== undefined) { const x = Number(b[n]); if (!Number.isInteger(x)) return err(400, "invalid value"); v[n] = x; list.push(n + " " + x); }
    u.pend = v; line("[web] GPS receiver settings: " + list.join(", "));
   }
   u.op = a === "ubxon" ? "ubx-on" : a; u.end = t + (a === "write" ? 2.2 : 1.2);
   return ok();
  }
  if (a === "umread" || a === "umwrite") {
   if (rx.kind !== "um982") return err(400, "not a UM982");
   if (m.op) return err(409, "busy");
   if (a === "umwrite") {
    if (steering()) return err(409, ENGAGED);
    const v = {};
    for (const n of UM_NAMES) if (b[n] !== undefined) { const x = Number(b[n]); if (!Number.isFinite(x)) return err(400, "invalid value"); v[n] = x; }
    m.pend = v; m.cmds = m.okN = Object.keys(v).length;
    line("[web] UM982 settings: " + Object.keys(v).length + " to write, then SAVECONFIG");
   }
   m.op = a === "umread" ? "read" : "write"; m.end = t + (a === "umread" ? 2 : 1 + 0.4 * (m.cmds || 1));
   return ok();
  }
  return err(400, "unknown action");
 }
 // the UM982 console (zWeb.ino apiUm982Post / apiUm982Get): an answer per command
 function umPost(b) {
  const c = String(b.cmd || "").trim();
  if (!c) return err(400, "no command");
  if (c.length > 120) return err(400, "empty or too long command");
  if (rx.kind !== "um982") return err(409, "no UM982");
  if (st.um.op) return err(409, "busy: the page is reading or writing the UM982 settings");
  line("[web] UM982 <- " + c);
  const up = c.toUpperCase(), add = x => st.umLines.push({s: st.umNext++, x: x});
  if (/^VERSIONA?$/.test(up)) add("#VERSIONA,97,GPS,FINE,2338,295000000,0,0,18,511;\"UM982\",\"R4.10Build13495\",\"HRPT00-S10C-P\",\"2310415000001-LR23A0225104238\",\"ff3bd496fd91c71f\",\"2023/06/14\"*3c14e0ba");
  else if (/^CONFIG$/.test(up)) for (const x of UM982D.filter(x => /^CONFIG/.test(x))) add("$CONFIG,COM1," + x + "*00");
  else add("$command," + c + ",response: OK*" + (c.length * 7 % 256).toString(16).toUpperCase().padStart(2, "0"));
  while (st.umLines.length > 60) st.umLines.shift();
  return ok();
 }
 function umGet(after) {
  return {next: st.umNext, lines: st.umLines.filter(l => l.s > after).map(l => l.x)};
 }

 // ---- /api/imu (zWeb.ino apiImuPost, TM171.ino)
 function imuPost(b) {
  const t = T(), a = b.act || "", tm = st.tm;
  if (!a) return err(400, "no act");
  if (!isTm) return err(409, "no TM171 found");
  if (a === "calib" || a === "install") {
   if (steering()) return err(409, ENGAGED);
   if (!parked && !cfg.simMode && tractor(t, parked).moving) return err(409, "the tractor is moving - stop first (on a bench, switch simulator mode on)");
   if (["starting", "running", "finishing"].includes(tm.calib)) return err(409, "a calibration is running");
   Object.assign(tm, {calib: a === "install" ? "running" : "starting", kind: a === "install" ? "install" : "gyro", end: t + (a === "install" ? 20 : 21)});
   line(a === "install" ? "[status] TM171: new install calibration started (about 20 s; keep the tractor parked, do not cut power)"
    : "[status] TM171: gyro calibration started (20 s; keep the tractor parked, the engine can stay running)");
   return ok();
  }
  if (a === "calibstop") {
   if (tm.kind === "install" && tm.calib === "running") { line("[status] TM171: new install calibration cannot be stopped (it has started), waiting for the result"); return ok(); }
   if (["starting", "running", "finishing"].includes(tm.calib)) { tm.calib = "aborted"; tm.doneAt = t; line("[status] TM171: gyro calibration stopped (web page)"); }
   return ok();
  }
  if (a === "setread") { tm.pend = {}; tm.pendEnd = t + 0.8; return ok(); }
  if (a === "setwrite" || a === "fixsetup") {
   if (steering()) return err(409, ENGAGED);
   if (a === "fixsetup") { tm.pend = {write: true, fix: true, save: true, v: {sw: 0x0008083B, boot: 0, accGain: 350, magGain: 100, odr: 25}}; tm.pendEnd = t + 2; return ok(); }
   const flag = n => b[n] === "1", num = (n, d) => b[n] !== undefined ? parseInt(b[n], 10) : d;
   const boot = num("boot", 0), acc = num("accGain", 210), mg = num("magGain", 100), odr = num("odr", 25);
   if (!(boot >= 0 && boot <= 2 && acc >= 0 && acc <= 65535 && mg >= 0 && mg <= 65535 && odr >= 0 && odr <= 65535)) return err(400, "value out of range");
   let sw = 0x0008080B & ~4 | (flag("mag") ? 4 : 0);
   sw = (sw & ~(8 | 16 | 32)) | (flag("gyroErr") ? 8 : 0) | (flag("selfAdapt") ? 16 : 0) | (flag("status") ? 32 : 0) | (boot << 24);
   tm.pend = {write: true, save: flag("save"), v: {sw: sw >>> 0, accGain: acc, magGain: mg, odr: odr}}; tm.pendEnd = t + 1.2;
   return ok();
  }
  return err(400, "unknown act");
 }

 // ---- /api/settings (zWeb.ino apiSettingsGet / apiSettingsPost)
 const FIELDS = [["outputMode", 0, 2], ["classicNoDual", 0, 1], ["hprAcceptFloat", 0, 1], ["epochWaitMs", 20, 100], ["tm171SwapRollPitch", 0, 1],
  ["tm171YawSign", -1, 1], ["xiaoOff", 0, 1], ["wasCheckOff", 0, 1], ["wdtOff", 0, 1], ["umSingle", 0, 1], ["simMode", 0, 1],
  ["steerMinKmhX10", 1, 30], ["steerMaxKmh", 5, 40], ["softStartX10ms", 0, 100], ["currentKickX10ms", 0, 200]];
 const DEFAULTS = {outputMode: 2, classicNoDual: 0, hprAcceptFloat: 1, epochWaitMs: 60, tm171SwapRollPitch: 0, tm171YawSign: 0, xiaoOff: 0,
  wasCheckOff: 0, wdtOff: 0, umSingle: 0, simMode: 0, steerMinKmhX10: 2, steerMaxKmh: 25, softStartX10ms: 0, currentKickX10ms: 50};
 const settingsJson = () => ({ok: true, settings: Object.assign({}, cfg), saved: st.saved, wdt: st.wdt});
 function settingsPost(b) {
  const s = b.defaults === "1" ? Object.assign({}, DEFAULTS) : Object.assign({}, cfg);
  if (b.defaults !== "1") for (const [n, lo, hi] of FIELDS) {
   if (b[n] === undefined) continue;
   const v = /^-?\d+$/.test(b[n]) ? parseInt(b[n], 10) : NaN;
   if (!(v >= lo && v <= hi)) return err(400, "invalid value for " + n);
   s[n] = v;
  }
  const t = T(), was = Object.assign({}, cfg);
  if (s.simMode !== was.simMode) {
   if (s.simMode) st.simSince = t; else st.heldBase += (t - st.simSince) * 10;
  }
  Object.assign(cfg, s); st.saved = true;
  if (cfg.wasCheckOff !== was.wasCheckOff) line(cfg.wasCheckOff ? "[web] WAS check OFF (bench setting): a broken WAS wire will NOT stop autosteer" : "[web] WAS check on");
  if (cfg.wdtOff !== was.wdtOff) { if (!cfg.wdtOff) st.wdt = true; else line("[web] Watchdog switched off: it stops at the next start of the Teensy"); }
  const outT = cfg.outputMode === 2 ? "automatic" + (hostName === "AgOpenGPS" ? " (now classic)" : " (now standard, AgOpenWeb)") : cfg.outputMode === 1 ? "classic ($PAOGI with UM982, $PANDA with F9P / X20P)" : "standard";
  line("[web] settings saved: output " + outT + ", float heading " + (cfg.hprAcceptFloat ? "yes" : "no") + ", epoch wait " + cfg.epochWaitMs + " ms, IMU roll/pitch swap " +
   (cfg.tm171SwapRollPitch ? "yes" : "no") + ", yaw " + (cfg.tm171YawSign === 0 ? "auto" : cfg.tm171YawSign > 0 ? "+1" : "-1") + ", GPS2 port " +
   gps2Name() + ", WAS check " + (cfg.wasCheckOff ? "OFF" : "on") + ", watchdog " + (cfg.wdtOff ? "off" : "on"));
  if (cfg.umSingle !== was.umSingle) line(cfg.umSingle ? "[web] UM982 with one antenna: no dual-antenna heading on the page, AgOpenGPS settings ConfigUM982S.txt" : "[web] UM982 with two antennas: AgOpenGPS settings ConfigUM982D.txt");
  if (cfg.simMode !== was.simMode) line(cfg.simMode ? "[web] Simulator mode ON (for the program's simulator only): the GPS is not sent to the program" : "[web] Simulator mode off: the GPS goes to the program again");
  if (["steerMinKmhX10", "steerMaxKmh", "softStartX10ms", "currentKickX10ms"].some(n => cfg[n] !== was[n]))
   line("[web] Steering: minimum " + (cfg.steerMinKmhX10 / 10).toFixed(1) + " km/h, maximum " + cfg.steerMaxKmh + " km/h, soft start " + cfg.softStartX10ms * 10 + " ms (0 = off), current kickout delay " + cfg.currentKickX10ms * 10 + " ms");
  return ok(settingsJson());
 }

 // ---- /api/log (zWeb.ino apiLog)
 function log(after) {
  const lines = st.log.filter(l => l.s > after);
  return {next: st.logNext, now: Math.round(Math.max(0, T()) * 1000), lines: lines};
 }
 // ---- /api/diag (zWeb.ino apiDiag)
 function diag() {
  const t = Math.max(0, T());
  return {fw: fwName() + " " + fwVer(), built: built(), teensyduino: 158, upMs: Math.round(t * 1000), ram: {heapFree: 412640, stackFree: 238112},
   loop: {n: Math.floor(t * 41000), maxUs: 2950, max5sUs: 610}, web: {req: Math.floor(t * 1.2), err: 0, drop: 0, lastErr: ""}, logLines: st.logNext - 1};
 }

 // ---- the firmware update (zWeb.ino apiOta, zOTA.ino): the file is read here as the Teensy checks it
 async function ota(query, blob) {
  progress();
  const force = query.get("force") === "1";
  if (steering()) { line("[web] firmware update REFUSED: " + ENGAGED + " (old firmware kept)"); return err(409, ENGAGED); }
  const size = blob && blob.size !== undefined ? blob.size : 0;
  if (size < 1000 || size > 8000000) { line("[web] firmware update REFUSED: no file, or file size not plausible (old firmware kept)"); return err(400, "no file, or file size not plausible"); }
  const text = await blob.text();
  let data = "", e = "", n = 0;
  if (text[0] !== ":") e = "not an Intel HEX file";
  else {
   for (const l of text.split("\n")) {
    if (l.charCodeAt(0) !== 58 || l.substr(7, 2) !== "00") continue;
    const k = parseInt(l.substr(1, 2), 16); n += k;
    for (let i = 0; i < k; i++) data += String.fromCharCode(parseInt(l.substr(9 + 2 * i, 2), 16));
   }
   if (!/:00000001FF/.test(text)) e = "file incomplete (no end record)";
   else if (n < 16384) e = "file too small to be a Teensy firmware";
   else if (!data.includes("fw_teensy41")) e = "not built for a Teensy 4.1 (fw_teensy41 missing)";
   else if (!force && !data.includes("AiO-WEB-OTA-1") && !data.includes("AOG-UM982-TM171-WEB-OTA-1")) e = "not this firmware (no web update in it) - tick 'other firmware' to install it anyway";
  }
  if (e) { line("[web] firmware update REFUSED: " + e + " (old firmware kept)"); return err(400, e); }
  line("[web] firmware update: " + size + " bytes received, steering off");
  line("[web] firmware update: " + n + " bytes checked, installing and restarting");
  // the module then runs the version in the file (another firmware: the simulator keeps showing this one)
  const m = /\[boot\] (v[\w.]+_AiO_WEB), built ([A-Z][a-z]{2} [ \d]\d \d{4} \d\d:\d\d:\d\d)/.exec(data);
  st.fwOver = m ? {fw: m[1], built: m[2].replace("  ", " ")} : null;
  restart(5000);
  return ok({ok: true, size: n});
 }
 // the Teensy restarts: offMs without an answer, then up again from 0 with a new log
 function restart(offMs) {
  setTimeout(() => {
   st.t0 = Date.now() + offMs; st.boot = "restart"; st.bootN++; st.wdt = !cfg.wdtOff;
   st.log = []; st.logNext = 1; st.tlAt = -1; st.simSince = cfg.simMode ? 0 : -1; st.heldBase = 0;
   st.id = {st: "done", at: 4.5}; st.ubx.op = st.um.op = st.ck.op = ""; st.ubx.at = rx.kind === "ublox" ? 5 : -1; st.um.at = rx.kind === "um982" ? 5 : -1;
   st.tm.calib = ""; st.tm.doneAt = -1; st.tm.readAt = 4.6; st.tm.ackAt = -1; st.tm.pend = null; st.umLines = []; st.umNext = 1;
   save(key, st);
  }, 1200);
 }
 function sprayerAt(tt) { const t = tt === undefined ? Math.max(0, T()) : tt; return sprayer(t, tractor(t, parked), autosteerOn(t)); }
 function setSubnet(net) { line("[status] subnet " + st.net + " -> " + net + " from the XIAO: saved, restarting as " + net + ".126"); st.net = net; restart(4000); }

 return {
  st: st, T: T, line: line, progress: progress, save: () => save(key, st), offline: () => T() < 0,
  status: status, log: log, settingsJson: settingsJson, settingsPost: settingsPost, gps: gps, gpsPost: gpsPost, umPost: umPost, umGet: umGet,
  imuPost: imuPost, diag: diag, ota: ota, restart: restart, setSubnet: setSubnet, xiaoInfo: xiaoInfo,
  tractor: t => tractor(t, parked), engaged: steering, machineCmd: () => sprayerAt().cmd, sprayerAt: sprayerAt,
 };
}

return {opt: opt, CHOICES: CHOICES, BUILD: BUILD, VERSION: VERSION, RUNNING: RUNNING, IMPL: IMPL, hook: hook, hookXhr: hookXhr, bar: bar, Module: Module, OFFLINE: OFFLINE,
 form: form, ok: ok, err: err, load: load, save: save, forget: forget, wob: wob, r2: r2, sleep: sleep, nozzles: nozzles};
})();
