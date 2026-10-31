/* sim-xiao.js - the web page simulator: the XIAO page (xPage.h) with a pretend XIAO and module behind it.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
   Written 2026 by Claude (Anthropic) for renatorgr.
   2026-10-31 (Claude): the pretend XIAO was compiled when the .bin for its chip on main was (firmware.json).
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The XIAO's answers (v4.5_XIAO_ESP32.ino apiState, apiTeensy, apiLog, apiSettings*, apiScan, apiSubnet,
// apiRestart, the program update on /update), with the module's status from the pretend Teensy of
// sim-common.js (its short copy, as the Teensy sends it to the XIAO once a second). The XIAO network password
// is the XIAO's default, agopengps.
"use strict";
(function () {
const S = window.AioSim, KEY = "aiosim.xiao", MKEY = "aiosim.xiao.module";
if (S.opt.gps === "dualf9p") S.opt.gps = "f9p";          // this page is the XIAO's: the GPS2 port has the XIAO
S.opt.xiao = "1";
const M = S.Module(MKEY);
const C3 = S.opt.chip !== "s3", CHIP = C3 ? "ESP32-C3" : "ESP32-S3", CHIP_ID = C3 ? 5 : 9;
const BIN = (S.BUILD.xiaoBin || {})[C3 ? "c3" : "s3"] || (C3 ? 1102560 : 990640);
const X = S.load(KEY, () => ({t0: Date.now() - S.RUNNING * 1000, reset: "power on", ap: "XIAO-3F2A", pass: "agopengps", staOn: true, sta: "Farm WiFi", staPassSet: true,
 host: "xiao", ev: [], evNext: 1, evAt: -1, scanAt: -1}));
const T = () => (Date.now() - X.t0) / 1000;
S.bar("No XIAO and no module behind this page: the values are made up. The XIAO password is agopengps.",
 ["gps", "imu", "motor", "prog", "chip", "park"], [KEY, MKEY], {gps: ["dualf9p"]});

function event(x, t) {
 X.ev.push({s: X.evNext++, t: Math.max(0, Math.round((t === undefined ? T() : t) * 1000)), x: x});
 if (X.ev.length > 40) X.ev.shift();
}
// the XIAO's own messages at start-up
function events() {
 const L = [[0.3, "v" + S.VERSION + "_XIAO_" + (C3 ? "ESP32C3" : "ESP32S3") + " started (" + X.reset + ")"],
  [0.5, "WiFi: own network '" + X.ap + "' at 192.168.4.1" + (X.staOn ? ", joining the other network" : "")]];
 if (X.staOn) L.push([3.2, "WiFi: joined '" + X.sta + "', IP 192.168.1.57, -61 dBm, channel 6"]);
 L.push([8.0, "WiFi: 1 device(s) on the XIAO network"]);
 const tB = T();
 for (const [t, x] of L) if (t > X.evAt && t <= tB) event(x, t);
 X.evAt = tB;
}
// the XIAO restarts: offMs without an answer, then up again from 0
function restart(why, offMs) {
 setTimeout(() => { X.t0 = Date.now() + offMs; X.reset = why; X.ev = []; X.evNext = 1; X.evAt = -1; S.save(KEY, X); }, 1500);
}
const staSt = () => !X.staOn || !X.sta ? "off" : T() > 3.2 ? "ok" : "con";
let lastS = null, lostAt = 0;

function state() {
 const t = T(), up = Math.floor(t), mod = !M.offline(), mt = Math.max(0, M.T());
 const tr = M.tractor(mt), eng = mod && M.engaged(), prog = mod && mt >= 6, n = Math.max(0, mt - 6);
 const w = {ap: X.ap, apCl: t > 8 ? 1 : 0, ch: 6, host: X.host, staOn: X.staOn && !!X.sta, sta: X.sta, st: staSt()};
 if (w.st === "ok") { w.ip = "192.168.1.57"; w.rssi = -61 + Math.round(S.wob(t, 3, 0.1)); }
 return {fw: "v" + S.VERSION + "_XIAO_" + (C3 ? "ESP32C3" : "ESP32S3"), ver: S.VERSION, chip: CHIP, chipId: CHIP_ID, built: (S.BUILD.xiaoBuilt || {})[C3 ? "c3" : "s3"] || S.BUILD.built || "Oct 25 2026 09:00:00",
  up: up, reset: X.reset, resetBad: false, heap: (C3 ? 187000 : 251000) + Math.round(S.wob(t, 900, 0.3)), heapMin: C3 ? 171036 : 236412,
  sketch: BIN, sketchFree: C3 ? 1966080 : 3342336, wifi: w,
  link: {talking: mod && mt > 5.2, lastAgo: mod ? 0 : Math.floor((Date.now() - lostAt) / 1000), frames: Math.floor(mt * 12), crcErr: 0, cut: 0,
   rxKB: S.r2(mt * 1.9, 1), txKB: S.r2(mt * 0.05, 1), hellos: Math.floor(mt / 2), sets: Math.floor(mt), broken: 0},
  aog: {helloAgo: prog ? 0 : -1, rate254: prog ? 10 : 0, ago254: prog ? 0 : -1, kmh: prog ? S.r2(tr.kmh, 1) : 0, angle: eng ? S.r2(M.tractor(mt + 0.25).steer, 2) : 0,
   status: eng ? 1 : 0, sec: prog ? M.machineCmd() : 0, ago239: prog ? 0 : -1, uturn: 0, hyd: !prog ? 0 : tr.moving && !tr.straight ? 2 : 1, tram: 0, geo: 0,
   pgns: prog ? [[200, Math.floor(n)], [239, Math.floor(n * 10)], [251, 1], [252, 1], [254, Math.floor(n * 10)]] : []}};
}
function teensy() {
 if (M.offline()) { if (!lostAt) lostAt = Date.now(); return {ago: lastS ? Math.floor((Date.now() - lostAt) / 1000) : -1, s: lastS || undefined}; }
 lostAt = 0;
 const s = M.status(false);
 lastS = s;
 return {ago: 0, s: s};
}
function log(after, xafter) {
 const mod = M.offline() ? [] : M.st.log.filter(l => l.s > after), x = X.ev.filter(l => l.s > xafter);
 return {epoch: M.st.bootN, last: M.st.logNext - 1, now: Math.round(Math.max(0, T()) * 1000), t: mod.slice(0, 20), xlast: X.evNext - 1, x: x.slice(0, 10),
  more: mod.length > 20 || x.length > 10};
}
const printable = (v, lo, hi) => typeof v === "string" && v.length >= lo && v.length <= hi && /^[\x20-\x7E]*$/.test(v);
function settingsPost(b) {
 if (b.pw !== X.pass) return S.err(403, "wrong password");
 if (!printable(b.apSsid || "", 1, 32)) return S.err(400, "network name: 1 to 32 characters");
 if (b.apPass && !printable(b.apPass, 8, 63)) return S.err(400, "password: 8 to 63 characters");
 if (b.staSsid && !printable(b.staSsid, 1, 32)) return S.err(400, "other network name: 1 to 32 characters");
 if (b.staPass && !printable(b.staPass, 8, 63)) return S.err(400, "other network password: 8 to 63 characters");
 const staOn = b.staOn === "1";
 if (staOn && !b.staSsid) return S.err(400, "other network: no name");
 const host = String(b.host || "").toLowerCase();
 if (!/^[a-z0-9-]{1,24}$/.test(host)) return S.err(400, "name on the network: letters, digits and '-', up to 24");
 if (b.staPass) X.staPassSet = true;                    // another network without a password: an open one
 else if ((b.staSsid || "") !== X.sta) X.staPassSet = false;
 Object.assign(X, {ap: b.apSsid, staOn: staOn, sta: b.staSsid || "", host: host});
 if (b.apPass) X.pass = b.apPass;
 event("Settings saved: own network '" + X.ap + "', other network " + (X.staOn ? X.sta : "off") + ", name " + X.host + ".local - restarting");
 restart("restart requested", 3000);
 return S.ok();
}
function scan() {
 const t = T();
 if (X.scanAt < 0 || t - X.scanAt > 10) { X.scanAt = t; return S.ok({busy: true}); }
 if (t - X.scanAt < 2) return S.ok({busy: true});
 X.scanAt = -1;
 return S.ok({busy: false, list: [{s: "Farm WiFi", r: -61, c: 6, open: false}, {s: "Workshop", r: -74, c: 11, open: false},
  {s: "Phone hotspot", r: -55, c: 1, open: false}, {s: "Guest", r: -83, c: 6, open: true}]});
}
function subnet(b) {
 if (b.pw !== X.pass) return S.err(403, "wrong password");
 const m = /^(\d+)\.(\d+)\.(\d+)$/.exec(String(b.subnet || "")), n = m ? m.slice(1).map(Number) : [];
 if (!m || n[0] === 0 || n[0] === 127 || n[0] > 223 || n[1] > 255 || n[2] > 255)
  return S.err(400, "subnet: three numbers, like 192.168.5 (the first 1 to 223, not 127)");
 if (M.offline()) return S.err(409, "no data from the module: is the Teensy on and the link working?");
 event("Module subnet " + n.join(".") + " sent to the Teensy from the page: it saves it and restarts");
 M.setSubnet(n.join("."));
 return S.ok();
}

S.hook((method, path, q, body) => {
 if (T() < 0) return S.OFFLINE;
 events(); if (!M.offline()) M.progress();
 const b = S.form(body), get = method === "GET", post = method === "POST";
 let r = null;
 if (get && path === "/api/state") r = S.ok(state());
 else if (get && path === "/api/teensy") r = S.ok(teensy());
 else if (get && path === "/api/log") r = S.ok(log(parseInt(q.get("after") || "0", 10) || 0, parseInt(q.get("xafter") || "0", 10) || 0));
 else if (get && path === "/api/settings") r = S.ok({apSsid: X.ap, staOn: X.staOn, staSsid: X.sta, staPassSet: X.staPassSet, host: X.host});
 else if (post && path === "/api/settings") r = settingsPost(b);
 else if (get && path === "/api/scan") r = scan();
 else if (post && path === "/api/subnet") r = subnet(b);
 else if (post && path === "/api/restart") {
  if (b.pw !== X.pass) r = S.err(403, "wrong password");
  else { event("Restart requested from the page"); restart("restart requested", 3000); r = S.ok(); }
 }
 S.save(KEY, X); M.save();
 return r;
});
// the program update (POST /update?pw=, the file in a form): the password, and the chip it is made for
S.hookXhr(async (path, q, body) => {
 if (T() < 0) return S.OFFLINE;
 if (path !== "/update") return S.err(404, "Not found");
 const f = body instanceof FormData ? body.get("file") : null;
 let e = "";
 if (q.get("pw") !== X.pass) e = "wrong password";
 else if (!f || !f.size) e = "no file";
 else {
  const h = new Uint8Array(await f.slice(0, 16).arrayBuffer()), id = h[12] | h[13] << 8;
  if (h[0] === 0xE9 && id !== CHIP_ID) e = id === 5 ? "other chip ESP32-C3" : id === 9 ? "other chip ESP32-S3" : "other chip";
  else if (h[0] !== 0xE9) e = "Magic byte is wrong, not 0xE9";
 }
 if (!e) event("Program update: receiving " + (f.name || "file"));
 if (e) { event("Program update REFUSED: " + e); S.save(KEY, X); return S.err(e === "wrong password" ? 403 : 400, e); }
 event("Program update: " + f.size + " bytes checked, restarting");
 restart("restart requested", 4000);
 S.save(KEY, X);
 return S.ok();
});
})();
