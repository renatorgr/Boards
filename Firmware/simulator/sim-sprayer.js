/* sim-sprayer.js - the web page simulator: the sprayer module's page (Sprayer/Sprayer_ESP32/sPage.h) with a pretend ESP32 behind it.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
   Written 2026 by Claude (Anthropic) for renatorgr.
   2026-10-27 (Claude): default pressure divider 0.5, as the module (10k / 10k).
   2026-10-27 (Claude): the sensors fitted (v0.7.2): the sprayer has both, ticked on the page.
   2026-10-27 (Claude): v0.8: sections as hex digits, the first section the relays follow, the module IP's last number.
   2026-10-27 (Claude): v0.9: the sprayer on the ESP32_Relay_X8 board.
   2026-11-02 (Claude): v0.10.0: the rate (L/ha), the pressure for the target and the alarms (zRate.ino), the spraying
   settings (POST /api/spray), the section widths from the program (PGN 235) and /aow/status.
   2026-11-02 (Claude): the Firmware tab: the pretend module runs the .bin on main (its build date from firmware.json);
   the update comes as an upload with progress, and after the restart the module runs what was sent (the program, version
   and build read from the .bin, as the page does).
   2026-11-02 (Claude): v0.10.1: "Search for networks" on the Module tab (GET /api/scan).
   2026-11-07 (Claude): v0.10.2: the module's name on the network is its own Wi-Fi name (no 16-digit name, no /aow/status).
   2026-11-07 (Claude): v0.10.3: "kind":"Sprayer" in /api/status.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The sprayer module's answers (zNet.ino: apiStatus, apiSettingsGet, apiWifiPost, apiScan, apiCalibPost, apiSprayPost, apiTotalPost,
// apiRestartPost, the program update on /api/update). The sections the program asks for and the sprayer's readings come
// from the pretend tractor of sim-common.js, the same the Teensy page's Machine tab shows: the 8 m sprayer's 8 sections
// on the ESP32_Relay_X8 board (8 relays from the program's pin config, both sensors, on the tractor's Wi-Fi).
// The own-network password is the module's default, agopengps.
"use strict";
(function () {
const S = window.AioSim, KEY = "aiosim.sprayer", MKEY = "aiosim.sprayer.module", M = S.Module(MKEY);
const B = S.BUILD.sprayer || {}, FW = B.name || "Sprayer_ESP32", VER = B.version || "0.5";
const IMPL = S.IMPL, RELAYS = 8;
const AP = "Sprayer-4E7C";                                      // from the ESP32's factory MAC, as zSettings.ino makes it
// sta: on the tractor's Wi-Fi; lost: was, until LOST_AT s; ap: the tractor's Wi-Fi not set
const NET = S.opt.wifi, LOST_AT = 24;
// the spraying settings (Settings tab): the pretend nozzles give 0.8 L/min at 3 bar (sim-common.js), and the target
// is what the pretend tractor sprays at 8 km/h; the rest are the module's defaults
const SPRAY = {rate: 120, nzSp: 0.5, nzFl: 0.8, nzBar: 3, pLow: 1, pHigh: 6, fTol: 20};
const TOOL_CM = Array.from({length: IMPL.sections}, () => Math.round(IMPL.width / IMPL.sections * 100));   // PGN 235, kept in flash
const X = S.load(KEY, () => ({t0: Date.now() - S.RUNNING * 1000, ap: AP, pass: "agopengps", staOn: NET !== "ap", sta: NET === "ap" ? "" : "Farm WiFi",
 staPassSet: NET !== "ap", host: "sprayer", subnet: "192.168.5", ipLast: 123, sensors: 3,
 ppl: 450, pmax: 10, pdiv: 0.5, totalBase: 0, secTop: 0, ev: [], evAt: -1, ...SPRAY}));
if (X.sensors === undefined) X.sensors = 3;                         // kept from a page opened before v0.7.2
if (X.ipLast === undefined) Object.assign(X, {ipLast: 123, secTop: 0});   // and before v0.8
if (X.rate === undefined) Object.assign(X, SPRAY);                  // and before v0.10
const T = () => (Date.now() - X.t0) / 1000;
S.bar("No board behind this page: the values are made up. The own-network password is agopengps. Park the tractor to see the sections closed.",
 ["wifi", "park"], [KEY, MKEY]);

// on the tractor's Wi-Fi from 3.2 s; the program talks (PGN 239 ten times a second) from 5.5 s
const cut = () => NET === "ap" || (NET === "lost" && T() > LOST_AT);
const staUp = () => X.staOn && !!X.sta && T() > 3.2 && !cut();
const linkOk = () => staUp() && T() > 5.5;
const ip = () => X.subnet + "." + X.ipLast;
const reach = () => NET !== "ap" && X.staOn && !!X.sta;            // set up to be on the tractor network
const talking = () => !reach() ? 0 : Math.max(0, Math.min(T(), NET === "lost" ? LOST_AT : Infinity) - 5.5);   // s the program has talked

// the module's own messages: m:ss and the text, the last 12, as logEvent() keeps them
function event(x, t) {
 const s = Math.max(0, Math.floor(t === undefined ? T() : t));
 X.ev.push(Math.floor(s / 60) + ":" + String(s % 60).padStart(2, "0") + " " + x);
 if (X.ev.length > 12) X.ev.shift();
}
function events() {
 const sta = X.staOn && !!X.sta;
 const L = [[0.2, "own network '" + X.ap + "' at 192.168.4.1" + (sta ? ", joining the tractor network" : ", tractor network not set")],
  [0.3, "ready: module IP " + ip() + " on the tractor network, page at 192.168.4.1"]];
 if (sta && NET !== "ap") {
  L.push([3.2, "joined '" + X.sta + "', IP " + ip() + ", -58 dBm"]);
  L.push([5.6, "machine config from the program: lift off, raise 2 s, lower 4 s, outputs normal"]);
  // the module logs the first eight; AgOpenWeb's pin config has sections 1-8 here
  L.push([5.7, "output functions from the program: relays 1 2 3 4 5 6 7 8"]);
  if (NET === "lost") L.push([LOST_AT, "lost the tractor network '" + X.sta + "'"]);
 }
 const tB = T();
 for (const [t, x] of L) if (t > X.evAt && t <= tB) event(x, t);
 X.evAt = tB;
}
// the module restarts: offMs without an answer, then up again from 0
function restart(offMs) {
 setTimeout(() => { X.t0 = Date.now() + offMs; X.ev = []; X.evAt = -1; X.secTop = 0; S.save(KEY, X); }, 500);
}

// what the sprayer is doing: the program's request, the relays (they follow at once, applyOutputs), the readings
function sprayer() {
 const link = linkOk(), sp = M.sprayerAt(link ? undefined : NET === "lost" ? LOST_AT : 0);
 const req = link || NET === "lost" ? sp.cmd : 0, on = link ? sp.cmd : 0;     // the request stays as last heard while the program is silent
 const bar = !(X.sensors & 1) ? 0 : on ? sp.bar : link && sp.main ? 3.3 : NET === "lost" ? 3.3 : 0.1;  // pump pressure with the valves closed
 const flow = X.sensors & 2, lpm = flow && on ? S.nozzles(on) * 0.8 * Math.sqrt(sp.bar / 3) : 0;
 return {req: req, on: on, bar: bar, lpm: lpm, total: flow ? Math.max(0, sp.total - X.totalBase) : 0, kmh: sp.main ? M.tractor(Math.max(0, M.T())).kmh : 0, raw: sp.total};
}

// zRate.ino rateTick(): open width and nozzles, L/ha, the pressure for the target and the speed hint, the alarms
// (worked out once a second, as the module does)
const held = [0, 0, 0, 0, 0];
let lastOpen = -1, since = 0, alarms = 0, lastRateT = -1;
function rate(sp, link) {
 let width = 0, noz = 0;
 TOOL_CM.forEach((cm, i) => { if ((sp.on >> i) & 1) { width += cm / 100; if (X.nzSp > 0) noz += Math.round(cm / 100 / X.nzSp); } });
 const p = X.sensors & 1 ? sp.bar : 0, chart = X.nzFl > 0 && X.nzBar > 0;
 const expected = chart && X.sensors & 1 && noz ? noz * X.nzFl * Math.sqrt(p / X.nzBar) : -1;
 const kmh = link ? Math.round(sp.kmh * 10) / 10 : 0, moving = kmh >= 1 && width > 0;   // PGN 239 has the speed in 0.1 km/h
 const lha = moving && X.sensors & 2 ? sp.lpm * 600 / (kmh * width) : -1;
 let need = -1, hint = -1;
 if (moving && X.rate > 0) {
  const needL = X.rate * kmh * width / 600;
  if (chart && noz) need = X.nzBar * (needL / (noz * X.nzFl)) ** 2;
  else if (X.sensors === 3 && sp.lpm > 0.5 && p > 0) need = p * (needL / sp.lpm) ** 2;
  if (need > 0 && X.pHigh > 0 && need > X.pHigh) hint = kmh * Math.sqrt(X.pHigh / need);
  else if (need > 0 && X.pLow > 0 && need < X.pLow) hint = kmh * Math.sqrt(X.pLow / need);
 }
 const sec = Math.floor(T());
 if (sec !== lastRateT) {
  lastRateT = sec;
  since = sp.on !== lastOpen ? 0 : since + 1; lastOpen = sp.on;
  const settled = since >= 3, spraying = sp.on !== 0, tol = Math.max(1, X.fTol) / 100;
  let raw = 0;
  if (X.sensors & 1 && spraying && X.pLow > 0 && p < X.pLow) raw |= 1;
  if (X.sensors & 1 && X.pHigh > 0 && p > X.pHigh) raw |= 2;
  if (X.sensors & 2 && spraying && settled && expected > 0.5) raw |= sp.lpm < expected * (1 - tol) ? 4 : sp.lpm > expected * (1 + tol) ? 8 : 0;
  if (X.sensors & 2 && !spraying && settled && sp.lpm > 0.5) raw |= 16;
  alarms = 0;
  for (let b = 0; b < 5; b++) { held[b] = raw & (1 << b) ? held[b] + 1 : 0; if (held[b] >= 3) alarms |= 1 << b; }
 }
 return {lha: lha < 0 ? -1 : S.r2(lha, 1), target: X.rate, needBar: need > 0 ? S.r2(need, 2) : -1, hintKmh: hint > 0 ? S.r2(hint, 1) : -1,
  width: S.r2(width, 2), nozzles: noz, expected: expected < 0 ? -1 : S.r2(expected, 2), alarms: alarms, minBar: X.pLow, maxBar: X.pHigh};
}

// what relay i does (relayFn): the program's pin config, AgOpenWeb's default here (sections 1-8)
const relayFn = i => i < 8 ? i + 1 : 0;
const hex = v => v.toString(16);                                    // sections as the module sends them (jsonHex)

function status() {
 const t = T(), up = staUp(), link = linkOk(), sp = sprayer(), tr = M.tractor(Math.max(0, M.T()));
 const silent = NET === "lost" && t > LOST_AT ? Math.floor(t - LOST_AT) : -1;     // s since the program was last heard
 const many = IMPL.sections > 16, fn = Array.from({length: RELAYS}, (_, i) => relayFn(i));
 let bits = 0, on = 0;                                              // the relays on, and the sections they open
 fn.forEach((f, i) => { if (f >= 1 && f <= 31 && (sp.on >> (f - 1)) & 1) { bits += 2 ** i; on |= 1 << (f - 1); } });
 for (let k = 31; k > X.secTop; k--) if ((sp.req >> (k - 1)) & 1) { X.secTop = k; break; }
 return {fw: FW, ver: X.ver || VER, built: X.built || B.built || S.BUILD.built || "Oct 25 2026 09:00:00", board: "ESP32_Relay_X8", kind: "Sprayer", up: Math.floor(t), heap: 231000 + Math.round(S.wob(t, 1500, 0.2)), name: X.ap,
  wifi: {ap: X.ap, apCl: 1, staOn: X.staOn, sta: X.sta, up: up, ip: up ? ip() : "", ipLast: X.ipLast,
   rssi: up ? -58 + Math.round(S.wob(t, 3, 0.1)) : 0, host: X.host, subnet: X.subnet},
  aog: {link: link, ago239: link ? 0 : silent, agoHello: link ? 0 : silent, kmh: S.r2(sp.kmh, 1), hyd: link ? (tr.moving && !tr.straight ? 2 : 1) : 0,
   tram: 0, geo: 0, uturn: link && tr.moving && !tr.straight ? 1 : 0, req: hex(sp.req), secTop: X.secTop, ago229: !many ? -1 : link ? 0 : silent,
   n239: Math.floor(talking() * 10), n229: many ? Math.floor(talking() * 10) : 0, nHello: Math.floor(talking()),
   crcErr: 0, invert: 0, hydEn: 0, raise: 2, lower: 4},
  out: {bits: bits, on: hex(on), onHigh: true, fn: fn},
  spr: {bar: S.r2(sp.bar, 2), lpm: S.r2(sp.lpm, 2), total: S.r2(sp.total, 1), master: true, volts: X.sensors & 1 ? S.r2(0.5 + sp.bar / X.pmax * 4, 2) : 0,
   pulses: Math.round(sp.total * X.ppl)},
  cal: {sensors: X.sensors, ppl: X.ppl, pmax: X.pmax, pdiv: X.pdiv},
  rate: rate({on: on, bar: sp.bar, lpm: sp.lpm, kmh: sp.kmh}, link),
  tool: {n: TOOL_CM.length, n235: link ? 1 : 0, cm: TOOL_CM},
  log: X.ev.slice()};
}
const settings = () => ({apSsid: X.ap, staOn: X.staOn, staSsid: X.sta, staPassSet: X.staPassSet, host: X.host, sensors: X.sensors, ppl: X.ppl,
 ipLast: X.ipLast, subnet: X.subnet, pmax: X.pmax, pdiv: X.pdiv, rate: X.rate, nzSp: X.nzSp, nzFl: X.nzFl, nzBar: X.nzBar, pLow: X.pLow,
 pHigh: X.pHigh, fTol: X.fTol});
const number = (v, lo, hi) => /^\d{1,5}$/.test(String(v)) && +v >= lo && +v <= hi;   // argNumber

const printable = (v, lo, hi) => typeof v === "string" && v.length >= lo && v.length <= hi && /^[\x20-\x7E]*$/.test(v);
// "192.168.1" -> "192.168.1" with the numbers tidied, or "" when it is not three numbers 0-255 (parseSubnet)
function subnet(text) {
 const m = /^(\d+)\.(\d+)\.(\d+)$/.exec(String(text || "").trim());
 if (!m) return "";
 const n = m.slice(1).map(Number);
 return n.every(v => v <= 255) ? n.join(".") : "";
}
function wifiPost(b) {
 if (b.pw !== X.pass) return S.err(403, "wrong password");
 if (!printable(b.apSsid || "", 1, 32)) return S.err(400, "own network name: 1 to 32 characters");
 if (b.apPass && !printable(b.apPass, 8, 63)) return S.err(400, "own network password: 8 to 63 characters");
 const staOn = b.staOn === "1", staSsid = b.staSsid || "";
 if (staOn && !printable(staSsid, 1, 32)) return S.err(400, "tractor network name: 1 to 32 characters");
 if (b.staPass && !printable(b.staPass, 8, 63)) return S.err(400, "tractor network password: 8 to 63 characters");
 if (!printable(b.host || "", 1, 24)) return S.err(400, "name: 1 to 24 characters");
 const net = subnet(b.subnet);
 if (!net) return S.err(400, "subnet: three numbers, like 192.168.1");
 if (b.ipLast !== undefined && !number(b.ipLast, 2, 254)) return S.err(400, "module IP last number: 2 to 254");
 if (b.staPass || staSsid !== X.sta) X.staPassSet = !!b.staPass;   // a new network without a password: an open one
 Object.assign(X, {ap: b.apSsid, staOn: staOn, sta: staSsid, host: b.host, subnet: net, ipLast: b.ipLast === undefined ? X.ipLast : +b.ipLast});
 if (b.apPass) X.pass = b.apPass;
 event("Wi-Fi settings saved from the page, restarting");
 restart(3000);
 return S.ok({ok: true, restart: true});
}
// "Search for networks" (GET /api/scan, zNet.ino apiScan): busy for 2 s, then the networks the pretend module hears
let scanAt = -1;
function scan() {
 const t = T();
 if (scanAt < 0 || t - scanAt > 10) { scanAt = t; return S.ok({busy: true}); }
 if (t - scanAt < 2) return S.ok({busy: true});
 scanAt = -1;
 return S.ok({busy: false, list: [{s: "Farm WiFi", r: -58, c: 6, open: false}, {s: "Phone hotspot", r: -66, c: 1, open: false},
  {s: "Workshop", r: -77, c: 11, open: false}, {s: "Guest", r: -84, c: 6, open: true}]});
}
function calibPost(b) {
 if (b.pw !== X.pass) return S.err(403, "wrong password");
 const ppl = parseFloat(b.ppl) || 0, pmax = parseFloat(b.pmax) || 0, pdiv = parseFloat(b.pdiv) || 0;
 const sensors = b.sensors === undefined ? X.sensors : parseInt(b.sensors, 10);
 if (!(ppl > 1 && ppl < 100000)) return S.err(400, "pulses per litre: 1 to 100000");
 if (!(pmax > 0.5 && pmax < 100)) return S.err(400, "sensor range: 0.5 to 100 bar");
 if (!(pdiv > 0.1 && pdiv <= 1)) return S.err(400, "divider: 0.1 to 1");
 if (!(sensors >= 0 && sensors <= 3)) return S.err(400, "sensors: 0 to 3");
 Object.assign(X, {sensors: sensors, ppl: ppl, pmax: pmax, pdiv: pdiv});
 event("saved: " + ["no sensors", "pressure sensor", "flow meter", "pressure sensor + flow meter"][sensors] + ", " + ppl.toFixed(1) + " pulses/L, " + pmax.toFixed(1) + " bar sensor, divider " + pdiv.toFixed(3));
 return S.ok();
}
// apiSprayPost: target rate, nozzles, pressure limits, flow tolerance (argFloat: digits and one dot)
function sprayPost(b) {
 if (b.pw !== X.pass) return S.err(403, "wrong password");
 const num = (k, lo, hi) => { const v = String(b[k] === undefined ? "" : b[k]); return /^\d*\.?\d*$/.test(v) && v.length >= 1 && v.length <= 8 && v !== "." && +v >= lo && +v <= hi ? +v : null; };
 const v = {rate: num("rate", 0, 2000), nzSp: num("nzSp", 0, 5), nzFl: num("nzFl", 0, 50), nzBar: num("nzBar", 0.1, 50), pLow: num("pLow", 0, 50), pHigh: num("pHigh", 0, 50), fTol: num("fTol", 1, 100)};
 if (v.rate === null) return S.err(400, "target rate: 0 to 2000 L/ha");
 if (v.nzSp === null) return S.err(400, "nozzle spacing: 0 to 5 m");
 if (v.nzFl === null) return S.err(400, "nozzle flow: 0 to 50 L/min");
 if (v.nzBar === null) return S.err(400, "chart pressure: 0.1 to 50 bar");
 if (v.pLow === null) return S.err(400, "minimum pressure: 0 to 50 bar");
 if (v.pHigh === null) return S.err(400, "maximum pressure: 0 to 50 bar");
 if (v.pHigh > 0 && v.pLow >= v.pHigh) return S.err(400, "the minimum pressure must be below the maximum");
 if (v.fTol === null) return S.err(400, "flow tolerance: 1 to 100 %");
 v.fTol = Math.round(v.fTol);
 Object.assign(X, v);
 event("saved: target " + v.rate.toFixed(0) + " L/ha, nozzles every " + v.nzSp.toFixed(2) + " m, " + v.nzFl.toFixed(2) + " L/min at " + v.nzBar.toFixed(1) +
  " bar, " + v.pLow.toFixed(1) + "-" + v.pHigh.toFixed(1) + " bar, flow +-" + v.fTol + " %");
 return S.ok();
}
function totalPost(b) {
 if (b.pw !== X.pass) return S.err(403, "wrong password");
 X.totalBase = sprayer().raw;
 event("total set to 0 from the page");
 return S.ok();
}
function restartPost(b) {
 if (b.pw !== X.pass) return S.err(403, "wrong password");
 event("restart from the page");
 restart(3000);
 return S.ok({ok: true, restart: true});
}
// a new program (.bin in a form, the password in the address): the Update library takes only an ESP32 image (first byte
// 0xE9); after the restart the module runs it (which program, version and build: the text its /api/status starts with)
async function updatePost(q, body) {
 if (q.get("pw") !== X.pass) return S.err(403, "wrong password");
 const f = body instanceof FormData ? body.get("bin") : null;
 event("program update: " + ((f && f.name) || "file"));
 const h = f && f.size ? new Uint8Array(await f.slice(0, 1).arrayBuffer()) : [];
 if (h[0] !== 0xE9) { event("update: failed"); return S.err(500, "update failed, the old program stays"); }
 const m = /\{"fw":"([^"]+)","ver":"([^"]*)","built":"([^"]*)"/.exec(new TextDecoder("latin1").decode(await f.arrayBuffer()));
 event("update: " + f.size + " bytes written, restarting");
 if (m && m[1] === FW) { X.ver = m[2]; X.built = m[3]; }
 restart(4000);
 return S.ok({ok: true, restart: true});
}

S.hook(async (method, path, q, body) => {
 if (T() < 0) return S.OFFLINE;
 events();
 const b = S.form(body), get = method === "GET", post = method === "POST";
 let r = null;
 if (get && path === "/api/status") r = S.ok(status());
 else if (get && path === "/api/settings") r = S.ok(settings());
 else if (post && path === "/api/wifi") r = wifiPost(b);
 else if (get && path === "/api/scan") r = scan();
 else if (post && path === "/api/calib") r = calibPost(b);
 else if (post && path === "/api/spray") r = sprayPost(b);
 else if (post && path === "/api/total") r = totalPost(b);
 else if (post && path === "/api/restart") r = restartPost(b);
 S.save(KEY, X);
 return r;
});
// the program update comes as an upload with progress (XMLHttpRequest)
S.hookXhr(async (path, q, body) => {
 if (T() < 0) return S.OFFLINE;
 if (path !== "/api/update") return S.err(404, "Not found");
 const r = await updatePost(q, body);
 S.save(KEY, X);
 return r;
});
})();
