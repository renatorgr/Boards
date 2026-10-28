/* sim-seeder.js - the web page simulator: the seed drill module's page (Seeder/Seeder_ESP32/dPage.h) with a pretend Relay24_ESP32 behind it.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
   Written 2026 by Claude (Anthropic) for renatorgr.
   2026-10-27 (Claude): made from sim-sprayer.js (Sprayer_ESP32 v0.8) when the seed drill got its own program, Seeder_ESP32 v0.1.
   2026-10-28 (Claude): Seeder_ESP32 v0.2: the Relay24_ESP32 board (24 relays) instead of the KC868-A32 (32).
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The seed drill module's answers (zNet.ino: apiStatus, apiSettingsGet, apiWifiPost, apiRelaysPost, apiRestartPost, the
// program update on /api/update). The sections the program asks for come from the pretend tractor of sim-common.js, the
// same the Teensy page's Machine tab shows: Renato's no-till seed drill, 19 sections on the Relay24_ESP32 board (24 relays,
// relay 1 follows section 1 and so on), on the network cable. No pressure or flow sensor.
// The own-network password is the module's default, agopengps.
"use strict";
(function () {
const S = window.AioSim, KEY = "aiosim.seeder", MKEY = "aiosim.seeder.module", M = S.Module(MKEY);
const B = S.BUILD.seeder || {}, FW = B.name || "Seeder_ESP32", VER = B.version || "0.1";
const IMPL = S.IMPL, RELAYS = 24, BOARD = "Relay24_ESP32";
const NAME = "00007C4EA4CF12B3", AP = "Seeder-4E7C";           // from the ESP32's factory MAC, as zSettings.ino makes them
// sta: on the tractor network (the cable); lost: was, until LOST_AT s (the cable unplugged); ap: the cable not plugged in
const NET = S.opt.wifi, LOST_AT = 24;
const X = S.load(KEY, () => ({t0: Date.now() - S.RUNNING * 1000, ap: AP, pass: "agopengps", staOn: false, sta: "", staPassSet: false,
 host: "seeder", subnet: "192.168.5", ipLast: 123, first: 1, secTop: 0, ev: [], evAt: -1}));
const T = () => (Date.now() - X.t0) / 1000;
S.bar("No board behind this page: the values are made up. The own-network password is agopengps. Park the tractor to see the sections closed." +
 " The Relay24_ESP32 board is on the network cable: \"tractor network lost\" = the cable unplugged.",
 ["wifi", "park"], [KEY, MKEY]);

// on the tractor network from 2.5 s on the cable (3.2 s on the tractor's Wi-Fi, when chosen on the page instead);
// the program talks (PGN 239 ten times a second, and 229 with it on the 19 sections) from 5.5 s
const cut = () => NET === "ap" || (NET === "lost" && T() > LOST_AT);
const staUp = () => X.staOn && !!X.sta && T() > 3.2 && !cut();
const ethUp = () => !X.staOn && T() > 2.5 && !cut();
const netUp = () => staUp() || ethUp();
const linkOk = () => netUp() && T() > 5.5;
const ip = () => X.subnet + "." + X.ipLast;
const reach = () => NET !== "ap" && (!X.staOn || !!X.sta);          // set up to be on the tractor network
const talking = () => !reach() ? 0 : Math.max(0, Math.min(T(), NET === "lost" ? LOST_AT : Infinity) - 5.5);   // s the program has talked

// the module's own messages: m:ss and the text, the last 12, as logEvent() keeps them
function event(x, t) {
 const s = Math.max(0, Math.floor(t === undefined ? T() : t));
 X.ev.push(Math.floor(s / 60) + ":" + String(s % 60).padStart(2, "0") + " " + x);
 if (X.ev.length > 12) X.ev.shift();
}
function events() {
 const sta = X.staOn && !!X.sta, eth = !X.staOn;
 const L = [[0.2, "own network '" + X.ap + "' at 192.168.4.1" + (X.staOn ? ", joining the tractor's Wi-Fi" : ", tractor network on the cable")],
  [0.3, "ready: module IP " + ip() + " on the tractor network, page at 192.168.4.1"]];
 if (sta && NET !== "ap") L.push([3.2, "joined '" + X.sta + "', IP " + ip() + ", -58 dBm"]);
 if (eth && NET !== "ap") L.push([2.5, "network cable connected, IP " + ip() + ", 100 Mbit/s"]);
 if ((sta || eth) && NET !== "ap") {
  L.push([5.6, "machine config from the program: lift off, raise 2 s, lower 4 s, outputs normal"]);
  // the module logs the first eight; AgOpenWeb's pin config has sections 1-8 here
  L.push([5.7, "output functions from the program: relays 1 2 3 4 5 6 7 8" + (X.first ? " (not used: the relays follow the sections in a row)" : "")]);
  if (NET === "lost") L.push([LOST_AT, sta ? "lost the tractor network '" + X.sta + "'" : "network cable: no link (unplugged, or the switch is off)"]);
 }
 const tB = T();
 for (const [t, x] of L) if (t > X.evAt && t <= tB) event(x, t);
 X.evAt = tB;
}
// the module restarts: offMs without an answer, then up again from 0
function restart(offMs) {
 setTimeout(() => { X.t0 = Date.now() + offMs; X.ev = []; X.evAt = -1; X.secTop = 0; S.save(KEY, X); }, 500);
}

// what the seed drill is doing: the program's request and the relays (they follow at once, applyOutputs)
function seeder() {
 const link = linkOk(), sp = M.sprayerAt(link ? undefined : NET === "lost" ? LOST_AT : 0);
 const req = link || NET === "lost" ? sp.cmd : 0, on = link ? sp.cmd : 0;     // the request stays as last heard while the program is silent
 return {req: req, on: on, kmh: sp.main ? M.tractor(Math.max(0, M.T())).kmh : 0};
}

// what relay i does (relayFn): with X.first, sections in a row from that one; else the program's pin config (sections 1-8)
const relayFn = i => X.first ? (X.first + i <= 64 ? X.first + i : 0) : i < 8 ? i + 1 : 0;
const hex = v => v.toString(16);                                    // sections as the module sends them (jsonHex)

function status() {
 const t = T(), up = netUp(), link = linkOk(), sd = seeder(), tr = M.tractor(Math.max(0, M.T()));
 const silent = NET === "lost" && t > LOST_AT ? Math.floor(t - LOST_AT) : -1;     // s since the program was last heard
 const many = IMPL.sections > 16, fn = Array.from({length: RELAYS}, (_, i) => relayFn(i));
 let bits = 0, on = 0;                                              // the relays on, and the sections they open
 fn.forEach((f, i) => { if (f >= 1 && f <= 31 && (sd.on >> (f - 1)) & 1) { bits += 2 ** i; on |= 1 << (f - 1); } });
 for (let k = 31; k > X.secTop; k--) if ((sd.req >> (k - 1)) & 1) { X.secTop = k; break; }
 return {fw: FW, ver: VER, built: S.BUILD.built || "Oct 25 2026 09:00:00", kind: "Seeder", board: BOARD, up: Math.floor(t),
  heap: 231000 + Math.round(S.wob(t, 1500, 0.2)), name: NAME,
  wifi: {ap: X.ap, apCl: 1, staOn: X.staOn, sta: X.sta, up: up, staUp: staUp(), eth: true, ethUp: ethUp(), ip: up ? ip() : "", ipLast: X.ipLast,
   rssi: staUp() ? -58 + Math.round(S.wob(t, 3, 0.1)) : 0, host: X.host, subnet: X.subnet},
  aog: {link: link, ago239: link ? 0 : silent, agoHello: link ? 0 : silent, kmh: S.r2(sd.kmh, 1), hyd: link ? (tr.moving && !tr.straight ? 2 : 1) : 0,
   tram: 0, geo: 0, uturn: link && tr.moving && !tr.straight ? 1 : 0, req: hex(sd.req), secTop: X.secTop, ago229: !many ? -1 : link ? 0 : silent,
   n239: Math.floor(talking() * 10), n229: many ? Math.floor(talking() * 10) : 0, nHello: Math.floor(talking()),
   crcErr: 0, invert: 0, hydEn: 0, raise: 2, lower: 4},
  out: {bits: bits, on: hex(on), first: X.first, onHigh: true, fn: fn},
  log: X.ev.slice()};
}
const settings = () => ({apSsid: X.ap, staOn: X.staOn, staSsid: X.sta, staPassSet: X.staPassSet, host: X.host, eth: true, ipLast: X.ipLast,
 first: X.first, subnet: X.subnet});
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
 event("network settings saved from the page, restarting");
 restart(3000);
 return S.ok({ok: true, restart: true});
}
function relaysPost(b) {
 if (b.pw !== X.pass) return S.err(403, "wrong password");
 if (!number(b.first, 0, 64)) return S.err(400, "first section: 0 to 64");
 X.first = +b.first;
 event(X.first ? "relays 1-" + RELAYS + " follow sections " + X.first + "-" + Math.min(64, X.first + RELAYS - 1) : "relays follow the program's pin config");
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
 if (f && f.size) await S.sleep(Math.min(4000, f.size / 1000));        // about 1 MB/s on the cable
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
 else if (post && path === "/api/relays") r = relaysPost(b);
 else if (post && path === "/api/restart") r = restartPost(b);
 else if (post && path === "/api/update") r = await updatePost(q, body);
 S.save(KEY, X);
 return r;
});
})();
