/* sim-sprayer.js - the web page simulator: the sprayer module's page (Sprayer/Sprayer_ESP32/sPage.h) with a pretend ESP32 behind it.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
   Written 2026 by Claude (Anthropic) for renatorgr.
   2026-10-27 (Claude): default pressure divider 0.5, as the module (10k / 10k).
   2026-10-27 (Claude): the sensors fitted (v0.7.2): the sprayer has both, the seed drill none; ticked on the page.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The sprayer module's answers (zNet.ino: apiStatus, apiSettingsGet, apiWifiPost, apiCalibPost, apiTotalPost,
// apiRestartPost, the program update on /api/update). The sections the program asks for and the sprayer's
// readings come from the pretend tractor of sim-common.js, the same the Teensy page's Machine tab shows: the seed
// drill's 16 rows on a separate ESP32 with 16 relays (no pressure or flow sensor: set to none, so no Sprayer card), or
// the sprayer's 5 sections on the ESP32_Relay_X8 board (both sensors).
// The own-network password is the module's default, agopengps.
"use strict";
(function () {
const S = window.AioSim, KEY = "aiosim.sprayer", MKEY = "aiosim.sprayer.module", M = S.Module(MKEY);
const B = S.BUILD.sprayer || {}, FW = B.name || "Sprayer_ESP32", VER = B.version || "0.5";
const NAME = "00007C4EA4CF12B3", AP = "Sprayer-4E7C";      // from the ESP32's factory MAC, as zSettings.ino makes them
const NET = S.opt.wifi, LOST_AT = 24;                       // sta: on the tractor network; lost: was, until LOST_AT s; ap: not set
const IMPL = S.IMPL, X8 = !!IMPL.nozzles, RELAYS = X8 ? 8 : 16;   // the sprayer: the X8 board (8 relays on HIGH); the seed drill: a dev module with two relay modules (16, on LOW)
const X = S.load(KEY, () => ({t0: Date.now() - S.RUNNING * 1000, ap: AP, pass: "agopengps", staOn: NET !== "ap", sta: NET === "ap" ? "" : "Farm WiFi",
 staPassSet: NET !== "ap", host: "sprayer", subnet: "192.168.5", sensors: X8 ? 3 : 0, ppl: 450, pmax: 10, pdiv: 0.5, totalBase: 0, ev: [], evAt: -1}));
if (X.sensors === undefined) X.sensors = X8 ? 3 : 0;                // kept from a page opened before v0.7.2
const T = () => (Date.now() - X.t0) / 1000;
S.bar("No board behind this page: the values are made up. The own-network password is agopengps. Park the tractor to see the sections closed." +
 (X8 ? "" : " The seed drill has no pressure or flow sensor: none is ticked in Calibration, so the module sends no PGN 160."), ["impl", "wifi", "park"], [KEY, MKEY]);

// on the tractor network from 3.2 s; the program talks (PGN 239 ten times a second) from 5.5 s
const staUp = () => X.staOn && !!X.sta && T() > 3.2 && !(NET === "lost" && T() > LOST_AT);
const linkOk = () => staUp() && T() > 5.5;
const talking = () => Math.max(0, Math.min(T(), NET === "lost" ? LOST_AT : Infinity) - 5.5);   // s the program has talked

// the module's own messages: m:ss and the text, the last 12, as logEvent() keeps them
function event(x, t) {
 const s = Math.max(0, Math.floor(t === undefined ? T() : t));
 X.ev.push(Math.floor(s / 60) + ":" + String(s % 60).padStart(2, "0") + " " + x);
 if (X.ev.length > 12) X.ev.shift();
}
function events() {
 const sta = X.staOn && !!X.sta;
 const L = [[0.2, "own network '" + X.ap + "' at 192.168.4.1" + (sta ? ", joining the tractor network" : ", tractor network not set")],
  [0.3, "ready: module IP " + X.subnet + ".123 on the tractor network, page at 192.168.4.1"]];
 if (sta) {
  L.push([3.2, "joined '" + X.sta + "', IP " + X.subnet + ".123, -58 dBm"]);
  L.push([5.6, "machine config from the program: lift off, raise 2 s, lower 4 s, outputs normal"]);
  L.push([5.7, "output functions from the program: relays 1 2 3 4 5 6 7 8"]);      // the module logs the first eight
  if (NET === "lost") L.push([LOST_AT, "lost the tractor network '" + X.sta + "'"]);
 }
 const tB = T();
 for (const [t, x] of L) if (t > X.evAt && t <= tB) event(x, t);
 X.evAt = tB;
}
// the module restarts: offMs without an answer, then up again from 0
function restart(offMs) {
 setTimeout(() => { X.t0 = Date.now() + offMs; X.ev = []; X.evAt = -1; S.save(KEY, X); }, 500);
}

// what the sprayer is doing: the program's request, the relays (they follow at once, applyOutputs), the readings
function sprayer() {
 const link = linkOk(), sp = M.sprayerAt(link ? undefined : NET === "lost" ? LOST_AT : 0);
 const req = link || NET === "lost" ? sp.cmd : 0, on = link ? sp.cmd : 0;     // the request stays as last heard while the program is silent
 const bar = !X8 || !(X.sensors & 1) ? 0 : on ? sp.bar : link && sp.main ? 3.3 : NET === "lost" ? 3.3 : 0.1;  // pump pressure with the valves closed
 const flow = X8 && (X.sensors & 2), lpm = flow && on ? S.nozzles(on) * 0.8 * Math.sqrt(sp.bar / 3) : 0;
 return {req: req, on: on, bar: bar, lpm: lpm, total: flow ? Math.max(0, sp.total - X.totalBase) : 0, kmh: sp.main ? M.tractor(Math.max(0, M.T())).kmh : 0, raw: sp.total};
}

function status() {
 const t = T(), up = staUp(), link = linkOk(), sp = sprayer(), tr = M.tractor(Math.max(0, M.T()));
 const silent = NET === "lost" && t > LOST_AT ? Math.floor(t - LOST_AT) : -1;     // s since the program was last heard
 return {fw: FW, ver: VER, built: S.BUILD.built || "Oct 25 2026 09:00:00", up: Math.floor(t), heap: 231000 + Math.round(S.wob(t, 1500, 0.2)), name: NAME,
  wifi: {ap: X.ap, apCl: 1, staOn: X.staOn, sta: X.sta, up: up, ip: up ? X.subnet + ".123" : "", rssi: up ? -58 + Math.round(S.wob(t, 3, 0.1)) : 0,
   host: X.host, subnet: X.subnet},
  aog: {link: link, ago239: link ? 0 : silent, agoHello: link ? 0 : silent, kmh: S.r2(sp.kmh, 1), hyd: link ? (tr.moving && !tr.straight ? 2 : 1) : 0,
   tram: 0, geo: 0, uturn: link && tr.moving && !tr.straight ? 1 : 0, req: sp.req, n239: Math.floor(talking() * 10), nHello: Math.floor(talking()),
   crcErr: 0, invert: 0, hydEn: 0, raise: 2, lower: 4},
  // the relays are sections 1-8 (the v0.6 default) or, from the program's pin config, 1-16 (the AgOpenGPS default)
  out: {bits: sp.on & 0xFFFF, on: sp.on, onHigh: X8, fn: Array.from({length: RELAYS}, (_, i) => i + 1)},
  spr: {bar: S.r2(sp.bar, 2), lpm: S.r2(sp.lpm, 2), total: S.r2(sp.total, 1), master: true, volts: X8 && (X.sensors & 1) ? S.r2(0.5 + sp.bar / X.pmax * 4, 2) : 0,
   pulses: Math.round(sp.total * X.ppl)},
  cal: {sensors: X.sensors, ppl: X.ppl, pmax: X.pmax, pdiv: X.pdiv},
  log: X.ev.slice()};
}
const settings = () => ({apSsid: X.ap, staOn: X.staOn, staSsid: X.sta, staPassSet: X.staPassSet, host: X.host, sensors: X.sensors, ppl: X.ppl, subnet: X.subnet, pmax: X.pmax, pdiv: X.pdiv});

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
 if (b.staPass || staSsid !== X.sta) X.staPassSet = !!b.staPass;   // a new network without a password: an open one
 Object.assign(X, {ap: b.apSsid, staOn: staOn, sta: staSsid, host: b.host, subnet: net});
 if (b.apPass) X.pass = b.apPass;
 event("Wi-Fi settings saved from the page, restarting");
 restart(3000);
 return S.ok({ok: true, restart: true});
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
// a new program (.bin in a form, the password in the address): the Update library takes only an ESP32 image (first byte 0xE9)
async function updatePost(q, body) {
 if (q.get("pw") !== X.pass) return S.err(403, "wrong password");
 const f = body instanceof FormData ? body.get("bin") : null;
 event("program update: " + ((f && f.name) || "file"));
 if (f && f.size) await S.sleep(Math.min(4000, f.size / 300));         // about 300 kB/s over Wi-Fi
 const h = f && f.size ? new Uint8Array(await f.slice(0, 1).arrayBuffer()) : [];
 if (h[0] !== 0xE9) { event("update: failed"); return S.err(500, "update failed, the old program stays"); }
 event("update: " + f.size + " bytes written, restarting");
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
 else if (post && path === "/api/calib") r = calibPost(b);
 else if (post && path === "/api/total") r = totalPost(b);
 else if (post && path === "/api/restart") r = restartPost(b);
 else if (post && path === "/api/update") r = await updatePost(q, body);
 S.save(KEY, X);
 return r;
});
})();
