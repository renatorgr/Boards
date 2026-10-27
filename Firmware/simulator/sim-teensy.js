/* sim-teensy.js - the web page simulator: the Teensy page (zWebPage.h) with a pretend module behind it.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
   Written 2026 by Claude (Anthropic) for renatorgr.
   2026-10-27 (Claude): the Machine tab links to the seed drill module's page or the sprayer module's page, by the implement.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// The page's requests, as zWeb.ino webDispatch() takes them, answered by the pretend module of sim-common.js;
// the firmware update (POST /api/ota) as an upload with its progress; the strip at the top with the choices.
"use strict";
(function () {
const S = window.AioSim, KEY = "aiosim.teensy", M = S.Module(KEY);
S.bar("No module behind this page: the values are made up. Park the tractor to try the update and the calibrations.",
 ["gps", "imu", "motor", "prog", "xiao", "mach", "impl", "park"], [KEY]);
// Our machine modules have a page of their own (the seed drill's, the sprayer's): a link to the one of the chosen
// implement at the end of the Machine tab's card, the way its IP on the Machine tab would open it on the tractor.
// Not with the official module (no page).
if (S.opt.mach === "nano" || S.opt.mach === "two") {
 const link = () => {
  const card = document.querySelector("#t-machine .card");
  if (!card) return;
  const p = document.createElement("p"), a = document.createElement("a"), seeder = S.opt.impl === "seeder";
  p.className = "note";p.id = "simModuleLink";
  const q = new URLSearchParams();
  if (S.opt.park !== S.CHOICES.park.dflt) q.set("park", S.opt.park);
  a.href = (seeder ? "../seeder/" : "../sprayer/") + (q.toString() ? "?" + q.toString() : "");
  a.textContent = seeder ? "the seed drill module's own page" : "the sprayer module's own page";
  p.append("In the simulator: open ", a, ". On the tractor it is at http://" + M.st.net + ".123/ (its own Wi-Fi network: http://192.168.4.1/).");
  card.appendChild(p);
 };
 if (document.body) link(); else document.addEventListener("DOMContentLoaded", link);
}
S.hook((method, path, q, body) => {
 if (M.offline()) return S.OFFLINE;
 M.progress();
 const b = S.form(body), after = parseInt(q.get("after") || "0", 10) || 0, get = method === "GET", post = method === "POST";
 let r = null;
 if (get && path === "/api/status") r = S.ok(M.status(true));
 else if (get && path === "/api/log") r = S.ok(M.log(after));
 else if (get && path === "/api/settings") r = S.ok(M.settingsJson());
 else if (post && path === "/api/settings") r = M.settingsPost(b);
 else if (get && path === "/api/diag") r = S.ok(M.diag());
 else if (get && path === "/api/gps") r = S.ok(M.gps());
 else if (post && path === "/api/gps") r = M.gpsPost(b);
 else if (get && path === "/api/um982") r = S.ok(M.umGet(after));
 else if (post && path === "/api/um982") r = M.umPost(b);
 else if (post && path === "/api/imu") r = M.imuPost(b);
 else if (get && path === "/api/rxdump") r = {code: 404, text: "No settings read yet"};
 else if (post && path === "/api/reboot") { M.line("[web] restart requested"); M.restart(3000); r = S.ok(); }
 M.save();
 return r;
});
S.hookXhr(async (path, q, body) => {
 if (M.offline()) return S.OFFLINE;
 if (path !== "/api/ota") return S.err(404, "Not found");
 const r = await M.ota(q, body);
 M.save();
 return r;
});
})();
