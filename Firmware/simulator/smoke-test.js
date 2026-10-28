// smoke-test.js - opens the built simulator in a browser and goes through its pages, as a check before it is published.
// Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
// Written 2026 by Claude (Anthropic) for renatorgr.
// Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
// 2026-10-27 (Claude): the Sprayer (PGN 160) card and the sprayer page's readings only with the sprayer's sensors.
// 2026-10-27 (Claude): the seed drill has 19 sections on the KC868-A32's 32 relays, on the network cable (Sprayer_ESP32 v0.8).
// 2026-10-28 (Claude): Seeder_ESP32 v0.2: the seed drill's board is the Relay24_ESP32, 24 relays.
// 2026-10-27 (Claude): the seed drill module's own page (Seeder_ESP32); the Teensy's Machine tab shows all 19 sections.
// 2026-10-27 (Claude): the AiO v26's machine part and two of our modules on the Teensy's Machine tab.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// node Firmware/simulator/smoke-test.js <site folder> [screenshot folder]   (needs the playwright package)
//
// Serves the site under /Boards/ as GitHub Pages does, opens the Teensy page with each receiver, the XIAO
// page and the seed drill and sprayer modules' pages, waits for the pretend module, clicks every tab and fails on
// a script error in the page or a tab that shows nothing. GitHub itself is not asked (the version check then says
// "no answer"). Also installs the firmware's own .hex on a parked module, end to end, saves the first section the
// seed drill's relays follow and the sprayer's calibration (their password prompt is answered).
"use strict";
const fs = require("fs"), path = require("path"), http = require("http");
const {chromium} = require("playwright");
const site = path.resolve(process.argv[2] || "_site"), shots = process.argv[3] ? path.resolve(process.argv[3]) : "";
const TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript", ".css": "text/css"};
const fail = [];
// the whole check takes about two minutes; a browser that hangs must not hold the Action for hours
const watchdog = setTimeout(() => { console.log("FAILED: the check did not finish in 6 minutes"); process.exit(1); }, 6 * 60 * 1000);

const server = http.createServer((q, r) => {
 const u = decodeURIComponent(q.url.split("?")[0]);
 if (!u.startsWith("/Boards/")) { r.writeHead(404); r.end(); return; }
 let f = path.join(site, u.slice("/Boards/".length));
 if (!f.startsWith(site)) { r.writeHead(403); r.end(); return; }
 if (fs.existsSync(f) && fs.statSync(f).isDirectory()) f = path.join(f, "index.html");
 if (!fs.existsSync(f)) { r.writeHead(404); r.end(); return; }
 r.writeHead(200, {"Content-Type": TYPES[path.extname(f)] || "application/octet-stream"});
 r.end(fs.readFileSync(f));
});

async function page(browser, url, what) {
 const pg = await browser.newPage({viewport: {width: 1000, height: 1100}});
 pg.on("pageerror", e => fail.push(what + ": " + e));
 pg.on("console", m => { if (m.type() === "error" && !/Failed to load resource|net::ERR/.test(m.text())) fail.push(what + ": console " + m.text()); });
 await pg.route(/^https:\/\/(api\.github\.com|github\.com|raw\.githubusercontent\.com)\//, r => r.abort());
 await pg.goto(url);
 return pg;
}
const shot = async (pg, name) => { if (shots) await pg.screenshot({path: path.join(shots, name + ".png"), fullPage: true}); };
async function expect(pg, what, sel, re) {
 const t = (await pg.locator(sel).first().textContent() || "").trim();
 if (!re.test(t)) fail.push(what + ": " + sel + " shows «" + t + "», expected " + re);
}

async function teensy(browser, port, q, name) {
 const what = "Teensy page " + (q || "(defaults)");
 const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/teensy/" + q, what);
 await pg.waitForTimeout(2500);
 await expect(pg, what, "#hdr", /_AiO_WEB · 192\.168\.5\.126/);
 await expect(pg, what, "#simBar", /SIMULATOR/);
 if (!(await pg.locator("#dot.on").count())) fail.push(what + ": the page is not connected to the pretend module");
 await shot(pg, name + "-status");
 for (const t of ["log", "settings", "gps", "imu", "machine", "firmware", "status"]) {
  const b = pg.locator('nav button[data-t="' + t + '"]');
  if (!(await b.isVisible())) continue;
  await b.click();
  await pg.waitForTimeout(t === "gps" ? 2500 : 700);
  if (!(await pg.locator("#t-" + t).isVisible())) fail.push(what + ": tab " + t + " does not show");
  if (t !== "status") await shot(pg, name + "-" + t);
 }
 await expect(pg, what, "#log", /\[boot\]/);
 const spr = /impl=sprayer/.test(q), ours = !/mach=(aog|v26|0)/.test(q);   // our module chosen (not the official one, the AiO v26, none)
 if (!/mach=0|park=1/.test(q)) {       // the seed drill's 19 sections (PGN 229 and its module's PGN 237), or the sprayer's 5 (parked: nothing asked yet, the tab draws 5)
  const n = spr ? 5 : 19;
  if ((await pg.locator("#mSec span").count()) !== n) fail.push(what + ": the Machine tab does not show " + n + " sections");
  await expect(pg, what, "#mSecFrom", /mach=aog/.test(q) ? /^hello reply/ : /^PGN 237/);
  await expect(pg, what, "#mIp", /mach=two/.test(q) ? /^192\.168\.5\.123, 192\.168\.5\.124$/ : /^192\.168\.5\.123$/);
 }
 // our module: the Machine tab links to its page (the seed drill's or the sprayer's)
 const la = pg.locator("#simModuleLink a"), link = (await la.count()) ? await la.getAttribute("href") : null;
 if (!!link !== ours) fail.push(what + ": the link to the module's page " + (ours ? "is missing" : "shows without our module"));
 else if (ours && !(spr ? /^\.\.\/sprayer\//.test(link) : /^\.\.\/seeder\//.test(link))) fail.push(what + ": the module link goes to " + link);
 const p160 = ours && spr;   // only the sprayer has sensors: the seed drill's module sends no PGN 160
 if ((await pg.locator("#mSprayCard.hidden").count() === 0) !== p160) fail.push(what + ": the Sprayer (PGN 160) card " + (p160 ? "is missing" : "shows without PGN 160"));
 return pg;
}

(async () => {
 await new Promise(r => server.listen(0, "127.0.0.1", r));
 const port = server.address().port;
 if (shots) fs.mkdirSync(shots, {recursive: true});
 const browser = await chromium.launch();
 const start = await page(browser, "http://127.0.0.1:" + port + "/Boards/", "start page");
 await expect(start, "start page", "h1", /simulator/);
 await shot(start, "start");
 await start.close();
 for (const [q, name] of [["", "um982"], ["?gps=x20p&xiao=0&mach=0", "x20p"], ["?gps=f9p&imu=bno&motor=keya&prog=aog&mach=aog", "f9p"],
  ["?gps=dualf9p&imu=rvc&motor=ibt2&prog=aog&mach=v26", "dualf9p"], ["?imu=none&motor=danfoss&impl=sprayer&mach=two", "noimu"], ["?mach=two", "two"]]) {
  const pg = await teensy(browser, port, q, name);
  await pg.close();
 }
 // a parked tractor: the settings saved, a gyro calibration, the firmware's own .hex installed
 {
  const what = "Teensy page, parked";
  const pg = await teensy(browser, port, "?park=1", "parked");
  await pg.click('nav button[data-t="settings"]'); await pg.waitForTimeout(500);
  await pg.click("#fs button[type=submit]"); await pg.waitForTimeout(800);
  await expect(pg, what, "#setMsg", /Saved/);
  pg.on("dialog", d => d.accept());
  await pg.click('nav button[data-t="imu"]'); await pg.waitForTimeout(300);
  await pg.click("#tmCalBtn"); await pg.waitForTimeout(2500);
  await expect(pg, what, "#tmCal2", /in progress|starting/);
  const hex = fs.readdirSync(path.join(__dirname, "..")).filter(d => /^v[\d.]+_AiO_WEB$/.test(d)).sort().map(d => path.join(__dirname, "..", d))
   .map(d => fs.readdirSync(d).filter(f => f.endsWith(".hex")).map(f => path.join(d, f))).flat().pop();
  if (hex) {
   await pg.click('nav button[data-t="firmware"]'); await pg.waitForTimeout(300);
   await pg.setInputFiles("#hexFile", hex); await pg.waitForTimeout(1500);
   await expect(pg, what, "#hexInfo", /_AiO_WEB, built/);
   await pg.click("#install");
   await pg.waitForFunction(() => /Updated\. Running/.test(document.getElementById("fwMsg").textContent), null, {timeout: 60000}).catch(() => {});
   await expect(pg, what, "#fwMsg", /Updated\. Running/);
   await shot(pg, "parked-updated");
  }
  await pg.close();
 }
 // the XIAO page
 {
  const what = "XIAO page";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/xiao/", what);
  await pg.waitForTimeout(2500);
  await expect(pg, what, "#hdr", /_XIAO_ESP32C3/);
  await expect(pg, what, "#mFw", /AiO/);
  await shot(pg, "xiao-status");
  for (const t of ["log", "wifi", "prog"]) {
   await pg.click('nav button[data-t="' + t + '"]'); await pg.waitForTimeout(800);
   if (!(await pg.locator("#t-" + t).isVisible())) fail.push(what + ": tab " + t + " does not show");
   if (t === "wifi") { await pg.click("#scan"); await pg.waitForTimeout(3500); await expect(pg, what, "#scanMsg", /networks/); }
   await shot(pg, "xiao-" + t);
  }
  await pg.close();
 }
 // the seed drill module's page: the Relay24_ESP32 on the network cable sowing a pass with 19 of its 24 relays (relay 1 =
 // section 1 and so on); then the relays set back to the program's pin config (the password asked in a prompt)
 {
  const what = "Seed drill page";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/seeder/", what);
  pg.on("dialog", d => d.accept("agopengps"));
  await pg.waitForTimeout(2500);
  await expect(pg, what, "h1", /^Seeder module$/);
  await expect(pg, what, "#sub", /^on the cable as 192\.168\.5\.123 · program talking/);
  if (!(await pg.locator("#dot.on").count())) fail.push(what + ": the page is not connected to the pretend module");
  if (await pg.locator("#simBar select").count() !== 2) fail.push(what + ": the strip should offer only the network and the tractor");
  await expect(pg, what, "#log", /ready: module IP 192\.168\.5\.123/);
  await expect(pg, what, "#log", /network cable connected/);
  await expect(pg, what, "#eth", /connected/);
  await expect(pg, what, "#ago229", /now/);
  await expect(pg, what, "#ver", /^Seeder_ESP32 v[\d.]+ · Relay24_ESP32$/);
  await expect(pg, what, "#invert", /relay on = pin high$/);
  if ((await pg.locator("#secs span").count()) !== 19) fail.push(what + ": expected the 19 sections");
  if ((await pg.locator("#secs span.on").count()) !== 19) fail.push(what + ": expected 19 sections open while sowing");
  if ((await pg.locator("#relays span").count()) !== 24) fail.push(what + ": expected the Relay24_ESP32's 24 relays");
  if ((await pg.locator("#relays span.on").count()) !== 19) fail.push(what + ": expected 19 relays on while sowing");
  await expect(pg, what, "#relayNote", /^24 relays, 24 of them sections: relay 1 follows section 1/);
  await expect(pg, what, "#r18", /^19S19$/);
  await shot(pg, "seeder-status");
  await pg.fill("#first", "0");
  await pg.click("button[onclick='saveRelays()']"); await pg.waitForTimeout(1500);
  await expect(pg, what, "#msgRelays", /done/);
  await expect(pg, what, "#log", /relays follow the program's pin config/);
  await expect(pg, what, "#relayNote", /^24 relays, 8 of them sections: what each relay does comes from the program/);
  await pg.close();
 }
 // the sprayer module's page: 5 sections spraying on the X8 board's 8 relays, the boom pressure and flow, the calibration
 // saved, the total set to 0
 {
  const what = "Sprayer page";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/sprayer/", what);
  pg.on("dialog", d => d.accept("agopengps"));
  await pg.waitForTimeout(2500);
  await expect(pg, what, "h1", /^Sprayer module$/);
  await expect(pg, what, "#sub", /^on Farm WiFi as 192\.168\.5\.123 · program talking/);
  if (!(await pg.locator("#dot.on").count())) fail.push(what + ": the page is not connected to the pretend module");
  await expect(pg, what, "#ver", /^Sprayer_ESP32 v[\d.]+ · ESP32_Relay_X8$/);
  if ((await pg.locator("#secs span.on").count()) !== 5) fail.push(what + ": expected 5 sections open while spraying");
  if ((await pg.locator("#relays span").count()) !== 8) fail.push(what + ": expected the X8 board's 8 relays");
  await expect(pg, what, "#relayNote", /^8 relays, 8 of them sections/);
  await expect(pg, what, "#bar", /^[23]\.\dbar$/);
  if (!(await pg.locator("#sprCard").isVisible())) fail.push(what + ": the Sprayer card is missing with both sensors");
  if (!(await pg.locator("#ppl").isVisible()) || !(await pg.locator("#pmax").isVisible())) fail.push(what + ": Calibration fields missing with both sensors");
  await shot(pg, "sprayer-status");
  await pg.click("button[onclick='saveCalib()']"); await pg.waitForTimeout(800);
  await expect(pg, what, "#msgCalib", /done/);
  await expect(pg, what, "#log", /saved: pressure sensor \+ flow meter, 450\.0 pulses\/L/);
  await pg.getByRole("button", {name: "Total back to 0"}).click(); await pg.waitForTimeout(1500);
  await expect(pg, what, "#msgTotal", /done/);
  await expect(pg, what, "#total", /^0\.\dL$/);
  await expect(pg, what, "#log", /total set to 0 from the page/);
  await pg.close();
 }
 // the seed drill's module with the network cable unplugged: sections closed, the program's request still outlined
 {
  const what = "Seed drill page, network lost";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/seeder/?wifi=lost", what);
  await pg.waitForTimeout(2500);
  await expect(pg, what, "#sub", /not on the tractor network · program silent/);
  await expect(pg, what, "#link", /silent, sections closed/);
  await expect(pg, what, "#log", /network cable: no link/);
  await expect(pg, what, "#eth", /no link/);
  if ((await pg.locator("#secs span.on").count()) !== 0) fail.push(what + ": a section is open with the program silent");
  if ((await pg.locator("#secs span.req").count()) !== 19) fail.push(what + ": expected 19 sections asked for but closed");
  await shot(pg, "seeder-lost");
  await pg.close();
 }
 // the sprayer's module that lost the tractor's Wi-Fi
 {
  const what = "Sprayer page, network lost";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/sprayer/?wifi=lost", what);
  await pg.waitForTimeout(2500);
  await expect(pg, what, "#sub", /not on the tractor network · program silent/);
  await expect(pg, what, "#log", /lost the tractor network 'Farm WiFi'/);
  if ((await pg.locator("#secs span.on").count()) !== 0) fail.push(what + ": a section is open with the program silent");
  await pg.close();
 }
 await browser.close();
 server.close();
 clearTimeout(watchdog);
 if (fail.length) { console.log("FAILED:\n" + fail.join("\n")); process.exit(1); }
 console.log("Simulator pages OK");
})().catch(e => { console.log(e); process.exit(1); });
