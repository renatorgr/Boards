// smoke-test.js - opens the built simulator in a browser and goes through its pages, as a check before it is published.
// Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
// Written 2026 by Claude (Anthropic) for renatorgr.
// The history of changes is in the README changelog and in the git log.
// Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// node Firmware/simulator/smoke-test.js <site folder> [screenshot folder]   (needs the playwright package)
//
// Serves the site under /Boards/ as GitHub Pages does, opens the Teensy page with each receiver, the XIAO
// page and the seed drill and sprayer modules' pages, waits for the pretend module, clicks every tab and fails on
// a script error in the page or a tab that shows nothing. GitHub itself is not asked: the version check gets the
// site's firmware.json (or a copy with a newer build in it) instead of https://renatorgr.github.io/Boards/firmware.json. Also installs the firmware's own .hex on a parked module, end to end, saves the first section the
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

// what the version check finds on GitHub: the site's firmware.json, or a copy changed by `newer`
const published = newer => { const j = JSON.parse(fs.readFileSync(path.join(site, "firmware.json"), "utf8")); if (newer) newer(j); return JSON.stringify(j); };
const LATER = "Dec 31 2099 23:59:59";

async function page(browser, url, what, newer) {
 const pg = await browser.newPage({viewport: {width: 1000, height: 1100}});
 pg.on("pageerror", e => fail.push(what + ": " + e));
 pg.on("console", m => { if (m.type() === "error" && !/Failed to load resource|net::ERR/.test(m.text())) fail.push(what + ": console " + m.text()); });
 await pg.route(/^https:\/\/(api\.github\.com|github\.com|raw\.githubusercontent\.com)\//, r => r.abort());
 await pg.route("https://renatorgr.github.io/Boards/firmware.json", r => r.fulfill({status: 200, contentType: "application/json",
  headers: {"Access-Control-Allow-Origin": "*"}, body: published(newer)}));
 await pg.goto(url);
 return pg;
}
const shot = async (pg, name) => { if (shots) await pg.screenshot({path: path.join(shots, name + ".png"), fullPage: true}); };
async function expect(pg, what, sel, re) {
 const t = (await pg.locator(sel).first().textContent() || "").trim();
 if (!re.test(t)) fail.push(what + ": " + sel + " shows «" + t + "», expected " + re);
}
// the same, for what comes a moment after a click (the module's answer, its events): waits up to ms for it
async function expectSoon(pg, what, sel, re, ms = 5000) {
 try {
  await pg.waitForFunction(([s, src, fl]) => new RegExp(src, fl).test(((document.querySelector(s) || {}).textContent || "").trim()),
   [sel, re.source, re.flags], {timeout: ms});
 } catch (e) { /* reported by expect() below */ }
 await expect(pg, what, sel, re);
}

async function teensy(browser, port, q, name) {
 const what = "Teensy page " + (q || "(defaults)");
 const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/teensy/" + q, what);
 await pg.waitForTimeout(2500);
 await expect(pg, what, "#ghVer", /^[\d.]+(, built [^—]+)? — the one installed$/);
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
 if (!/mach=0|park=1/.test(q)) {       // the seed drill's 19 sections (PGN 229 and its module's PGN 237), or the sprayer's 8 (parked: nothing asked yet)
  const n = spr ? 8 : 19;
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
 if (p160) await expect(pg, what, "#mAlarms", /^none$/);   // its PGN 161 read too: the alarm bits
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
 // the AgOpenWeb settings page: the worked example's sprayer and seed drill, the drawing, a wrong input refused
 {
  const what = "AgOpenWeb settings page";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/aow-settings/", what);
  await pg.waitForTimeout(500);
  await expect(pg, what, "h1", /^AgOpenWeb settings from the dimensions$/);
  await expect(pg, what, "#results tbody tr:nth-child(3) td.v", /^4 m$/);          // U-Turn radius: 1.2 × (2.7 ÷ tan 40°) = 3.9, floor 4 m
  await expect(pg, what, "#results tbody tr:nth-child(5) td.v", /^5\.2 m$/);       // distance from boundary: the 8 m boom's end plus the 1 m margin
  await expect(pg, what, "#results tbody tr:nth-child(6) td.v", /^16 m \(2 passes\)$/);
  if (!(await pg.locator("#draw svg").count())) fail.push(what + ": no drawing");
  await shot(pg, "aow-settings-sprayer");
  await pg.click("button:has-text('3 m seed drill')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#results tbody tr:nth-child(4) td.v", /^Sagitta, skip 2 or Keyhole, skip 0 or K-turn, skip 0$/);
  await expect(pg, what, "#results tbody tr:nth-child(6) td.v", /^12 m \(4 passes\) or 18 m \(6 passes\) or 15 m \(5 passes\)$/);
  await expect(pg, what, "#results tbody tr:nth-child(9) td.v", /^no match$/);     // tramlines: 8 ÷ 3 is not whole
  if ((await pg.locator("#opt button").count()) !== 3) fail.push(what + ": expected the three turn options to draw");
  await pg.click("#opt button:has-text('Keyhole')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#draw svg text:last-child", /^next pass 3 m away: Keyhole, radius 4 m, 9\.8 m deep$/);
  await shot(pg, "aow-settings-drill");
  await pg.click("#opt button:has-text('K-turn')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#draw svg text:last-child", /^next pass 3 m away: K-turn, radius 4 m, 6\.5 m deep \(the backing up included\)$/);
  if ((await pg.locator("#draw svg path[stroke-dasharray]").count()) !== 1) fail.push(what + ": the K-turn's backing up should be drawn dashed");
  await shot(pg, "aow-settings-drill-kturn");
  await pg.fill("#overlap", "3"); await pg.waitForTimeout(300);
  await expect(pg, what, "#problems", /overlap must be less than the implement width/);
  await pg.close();
 }
 for (const [q, name] of [["", "um982"], ["?gps=x20p&xiao=0&mach=0", "x20p"], ["?gps=f9p&imu=bno&motor=keya&prog=aog&mach=aog", "f9p"],
  ["?gps=dualf9p&imu=rvc&motor=ibt2&prog=aog&mach=v26", "dualf9p"], ["?imu=none&motor=danfoss&impl=sprayer&mach=two", "noimu"], ["?mach=two", "two"]]) {
  const pg = await teensy(browser, port, q, name);
  await pg.close();
 }
 // the firmware's own .hex, if the version folder has one (the Arduino IDE's export, vX.Y_AiO_WEB.ino.hex)
 const hex = fs.readdirSync(path.join(__dirname, "..")).filter(d => /^v[\d.]+_AiO_WEB$/.test(d)).sort().map(d => path.join(__dirname, "..", d))
  .map(d => fs.readdirSync(d).filter(f => f.endsWith(".hex")).map(f => path.join(d, f))).flat().pop();
 // a parked tractor: the settings saved, a gyro calibration, the firmware's own .hex installed, the previous one put back
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
  if (hex) {
   await pg.click('nav button[data-t="firmware"]'); await pg.waitForTimeout(300);
   await expect(pg, what, "#prevFw", /^none kept yet/);
   if (!(await pg.locator("#revert").isDisabled())) fail.push(what + ": Go back is offered with no previous firmware kept");
   await pg.setInputFiles("#hexFile", hex); await pg.waitForTimeout(1500);
   await expect(pg, what, "#hexInfo", /_AiO_WEB, built/);
   await pg.click("#install");
   await pg.waitForFunction(() => /Updated\. Running/.test(document.getElementById("fwMsg").textContent), null, {timeout: 60000}).catch(() => {});
   await expect(pg, what, "#fwMsg", /Updated\. Running/);
   await pg.waitForFunction(() => /_AiO_WEB, built/.test(document.getElementById("prevFw").textContent), null, {timeout: 10000}).catch(() => {});   // the next status poll fills it
   await expect(pg, what, "#prevFw", /_AiO_WEB, built/);   // the firmware that ran before is kept
   await shot(pg, "parked-updated");
   await pg.click("#revert");
   await pg.waitForFunction(() => /Going back/.test(document.getElementById("fwMsg").textContent), null, {timeout: 5000}).catch(() => {});
   await pg.waitForFunction(() => /Updated\. Running/.test(document.getElementById("fwMsg").textContent), null, {timeout: 60000}).catch(() => {});
   await expect(pg, what, "#fwMsg", /Updated\. Running/);
   await expect(pg, what, "#prevFw", /^none kept yet/);    // used up by going back
  }
  await pg.close();
 }
 // a newer build of the same version on GitHub: offered as new
 {
  const what = "Teensy page, newer build on GitHub";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/teensy/", what, j => { j.teensy.built = LATER; });
  await pg.waitForTimeout(2500);
  await expect(pg, what, "#ghVer", /built 31 Dec 2099 23:59 — newer than the installed one$/);
  if (await pg.locator("#ghInstall").isDisabled()) fail.push(what + ": Install from GitHub is not offered");
  if (await pg.locator("#ghNote").isHidden()) fail.push(what + ": no note about the new firmware");
  await pg.close();
 }
 // the XIAO page
 {
  const what = "XIAO page";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/xiao/", what);
  await pg.waitForTimeout(2500);
  await expect(pg, what, "#hdr", /_XIAO_ESP32C3/);
  // the seed drill's 19 sections (PGN 229): one square each, not the 16 of PGN 254 alone
  for (let i = 0; i < 40 && (await pg.locator("#aSecs span").count()) !== 19; i++) await pg.waitForTimeout(250);
  if ((await pg.locator("#aSecs span").count()) !== 19) fail.push(what + ": Program → module should show the seed drill's 19 sections");
  await expect(pg, what, "#mFw", /AiO/);
  await expect(pg, what, "#ghVer", /^[\d.]+(, built [^—]+)? — the one installed$/);
  await shot(pg, "xiao-status");
  for (const t of ["log", "wifi", "prog"]) {
   await pg.click('nav button[data-t="' + t + '"]'); await pg.waitForTimeout(800);
   if (!(await pg.locator("#t-" + t).isVisible())) fail.push(what + ": tab " + t + " does not show");
   if (t === "wifi") { await pg.click("#scan"); await pg.waitForTimeout(3500); await expect(pg, what, "#scanMsg", /networks/); }
   await shot(pg, "xiao-" + t);
  }
  await pg.close();
 }
 // the XIAO page with the module parked: the firmware's own .hex installed on the module over the link, then the
 // previous firmware put back (the XIAO password typed in the Update or restart card)
 if (hex) {
  const what = "XIAO page, module firmware over the link";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/xiao/?park=1", what);
  pg.on("dialog", d => d.accept());
  await pg.waitForTimeout(2500);
  await pg.click('nav button[data-t="prog"]'); await pg.waitForTimeout(800);
  await expect(pg, what, "#tFw", /_AiO_WEB/);
  await expectSoon(pg, what, "#tGh", /the one the module runs$/);
  await expect(pg, what, "#tPrev", /^none kept yet/);
  if (!(await pg.locator("#tRevert").isDisabled())) fail.push(what + ": Go back is offered with no previous firmware kept");
  if (!(await pg.locator("#tGhInstall").isDisabled())) fail.push(what + ": Install from GitHub is offered with the same build");
  await pg.fill("#pwP", "agopengps");
  await pg.setInputFiles("#tHexFile", hex); await pg.waitForTimeout(1500);
  await expect(pg, what, "#tHexInfo", /_AiO_WEB, built/);
  await pg.click("#tInstall");
  await pg.waitForFunction(() => /^Done\./.test(document.getElementById("tMsg").textContent), null, {timeout: 60000}).catch(() => {});
  await expect(pg, what, "#tMsg", /^Done\. The module runs/);
  await expect(pg, what, "#tPrev", /_AiO_WEB, built/);
  await expectSoon(pg, what, "#logX", /checked by the Teensy/, 8000);   // the XIAO's own messages (polled every few seconds on any tab)
  await shot(pg, "xiao-module-updated");
  await pg.click("#tRevert");
  await pg.waitForFunction(() => /going back/.test(document.getElementById("tMsg").textContent), null, {timeout: 5000}).catch(() => {});
  await pg.waitForFunction(() => /^Done\./.test(document.getElementById("tMsg").textContent), null, {timeout: 60000}).catch(() => {});
  await expect(pg, what, "#tMsg", /^Done\. The module runs/);
  await expect(pg, what, "#tPrev", /^none kept yet/);
  await pg.close();
 }
 // an S3 XIAO when only the S3's .bin on GitHub is a newer build: new for it (a C3 would stay "installed")
 {
  const what = "XIAO S3 page, newer S3 build on GitHub";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/xiao/?chip=s3", what, j => { if (j.xiao.S3) j.xiao.S3.built = LATER; });
  await pg.waitForTimeout(2500);
  await expect(pg, what, "#hdr", /_XIAO_ESP32S3/);
  await expect(pg, what, "#ghVer", /built 31 Dec 2099 23:59 — newer than the installed one$/);
  if (await pg.locator("#ghInstall").isDisabled()) fail.push(what + ": Install from GitHub is not offered");
  await pg.close();
 }
 // the seed drill module's page: the Relay24_ESP32 on the network cable sowing a pass with 19 of its 24 relays (relay 1 =
 // section 1 and so on); Settings: the relays set back to the program's pin config (the password asked in a prompt);
 // Module: the search for Wi-Fi networks; Firmware: the program on GitHub (none until the .bin is on main), wrong files
 // refused, the report downloaded
 {
  const what = "Seed drill page";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/seeder/", what);
  pg.on("dialog", d => d.accept("agopengps"));
  await pg.waitForTimeout(2500);
  await expect(pg, what, "h1", /^Seeder module$/);
  await expect(pg, what, "#sub", /^on the cable as 192\.168\.5\.123 · program talking/);
  if (!(await pg.locator("#dot.on").count())) fail.push(what + ": the page is not connected to the pretend module");
  if (!(await pg.locator("#t-work").isVisible())) fail.push(what + ": the Sowing tab does not show first");
  if (await pg.locator("#simBar select").count() !== 2) fail.push(what + ": the strip should offer only the network and the tractor");
  if ((await pg.locator("#secs span").count()) !== 19) fail.push(what + ": expected the 19 sections");
  if ((await pg.locator("#secs span.on").count()) !== 19) fail.push(what + ": expected 19 sections open while sowing");
  await expect(pg, what, "#detail", /^19 of 19 open · [\d.]+ km\/h · lift (idle|lowering|raising)/);
  if (await pg.locator("#silent").isVisible()) fail.push(what + ": 'program silent' shows while the program talks");
  await shot(pg, "seeder-status");
  await pg.click('nav button[data-t="settings"]'); await pg.waitForTimeout(700);
  if (!(await pg.locator("#t-settings").isVisible())) fail.push(what + ": tab Settings does not show");
  if ((await pg.locator("#relays span").count()) !== 24) fail.push(what + ": expected the Relay24_ESP32's 24 relays");
  if ((await pg.locator("#relays span.on").count()) !== 19) fail.push(what + ": expected 19 relays on while sowing");
  await expect(pg, what, "#relayNote", /^24 relays, 24 of them sections: relay 1 follows section 1/);
  await expect(pg, what, "#r18", /^19S19$/);
  await pg.fill("#first", "0");
  await pg.click("button[onclick='saveRelays()']"); await pg.waitForTimeout(1500);
  await expectSoon(pg, what, "#msgRelays", /done/);
  await expectSoon(pg, what, "#log", /relays follow the program's pin config/);
  await expect(pg, what, "#relayNote", /^24 relays, 8 of them sections: what each relay does comes from the program/);
  await shot(pg, "seeder-settings");
  await pg.click('nav button[data-t="module"]'); await pg.waitForTimeout(700);
  if (!(await pg.locator("#t-module").isVisible())) fail.push(what + ": tab Module does not show");
  await expect(pg, what, "#log", /ready: module IP 192\.168\.5\.123/);
  await expect(pg, what, "#log", /network cable connected/);
  await expect(pg, what, "#eth", /connected/);
  await expect(pg, what, "#ago229", /now/);
  await expect(pg, what, "#invert", /relay on = pin high$/);
  // the Wi-Fi settings as on the XIAO page: "Search for networks" lists what the module hears, strongest first, and a tap
  // chooses one; Save wants the password typed in its card (nothing is saved here: the module would restart)
  await pg.click("#scan");
  await expectSoon(pg, what, "#scanMsg", /^4 networks · tap one to choose it$/);
  if ((await pg.locator("#netList button").count()) !== 4) fail.push(what + ": expected 4 networks in the list");
  await expect(pg, what, "#netList button:first-child", /^Farm WiFi-58 dBm · strong$/);
  await pg.locator("#netList button", {hasText: "Phone hotspot"}).click();
  if (await pg.inputValue("#staSsid") !== "Phone hotspot" || !(await pg.isChecked("#staOn"))) fail.push(what + ": a tap on a network does not choose it");
  if ((await pg.locator("#netList button.sel").count()) !== 1) fail.push(what + ": the network chosen is not marked");
  await pg.click("button[onclick='saveWifi()']");
  await expectSoon(pg, what, "#msgWifi", /^Type the password of the own Wi-Fi network\.$/);
  await shot(pg, "seeder-network");
  await pg.click('nav button[data-t="firmware"]'); await pg.waitForTimeout(700);
  if (!(await pg.locator("#t-firmware").isVisible())) fail.push(what + ": tab Firmware does not show");
  await expect(pg, what, "#ver", /^Seeder_ESP32 v[\d.]+$/);
  await expect(pg, what, "#board", /^Relay24_ESP32$/);
  // the .bin on main, once there is one: the one the pretend module runs (or an older one, when the code changed and the
  // .bin did not yet); none yet: said so
  await expect(pg, what, "#ghVer", /^(no seed drill program on GitHub|v[\d.]+, built [^—]+ — (the one installed|older than the installed one \(the new \.bin is not on GitHub yet\)))$/);
  if (!(await pg.locator("#ghInstall").isDisabled())) fail.push(what + ": Install from GitHub is offered with nothing newer there");
  if (await pg.locator('nav button[data-t="firmware"].new').count()) fail.push(what + ": the Firmware tab is marked new with nothing newer");
  await shot(pg, "seeder-firmware");
  // wrong files: the .merged.bin, a program for the other board, the sprayer's program
  const sdBin = (fw, ver, built, board) => Buffer.concat([Buffer.from([0xE9]), Buffer.alloc(31),
   Buffer.from('{"fw":"' + fw + '","ver":"' + ver + '","built":"' + built + '"\0' + (board ? ',"board":"' + board + '"\0' : ""))]);
  for (const [name, buf, re] of [["Seeder_ESP32.ino.merged.bin", Buffer.alloc(64, 0xFF), /^This is the wrong file: use Seeder_ESP32\.ino\.bin/],
   ["Seeder_ESP32.ino.bin", sdBin("Seeder_ESP32", "0.2", LATER, "KC868-A32"), /^This program is for the KC868-A32 board, but this module is on the Relay24_ESP32 board/],
   ["Sprayer_ESP32.ino.bin", sdBin("Sprayer_ESP32", "0.10", LATER, ""), /^This is the program Sprayer_ESP32, not this module's/]]) {
   await pg.setInputFiles("#bin", {name: name, mimeType: "application/octet-stream", buffer: buf});
   await pg.click("#install"); await pg.waitForTimeout(500);
   await expect(pg, what + " (" + name + ")", "#msgProg", re);
  }
  if (/program update/.test(await pg.locator("#log").textContent())) fail.push(what + ": a wrong file was sent to the module");
  try {
   const [dl] = await Promise.all([pg.waitForEvent("download", {timeout: 5000}), pg.click("#dumpBtn")]);
   const name = dl.suggestedFilename(), r = JSON.parse(require("fs").readFileSync(await dl.path(), "utf8"));
   if (!/^Seeder_ESP32_report_\d{4}-\d\d-\d\d_\d{4}\.json$/.test(name)) fail.push(what + ": the report is called " + name);
   if (!r.status || !r.status.out || !r.settings || r.settings.first !== 0)
    fail.push(what + ": the report misses the status or the settings");
   if (JSON.stringify(r).includes("agopengps")) fail.push(what + ": the report has the password in it");
  } catch (e) { fail.push(what + ": no report downloaded (" + e.message + ")"); }
  await expect(pg, what, "#dumpMsg", /^Downloaded: Seeder_ESP32_report_/);
  await pg.close();
 }
 // a newer build of the seed drill program on GitHub: offered, marked on the Firmware tab, downloaded from GitHub (here: a
 // made-up .bin with the version running and a later build date, since none is on main yet) and sent to the module,
 // which then runs it
 {
  const what = "Seed drill page, newer build on GitHub";
  const html = fs.readFileSync(path.join(site, "seeder", "index.html"), "utf8");
  const running = JSON.parse(/window\.SIM_BUILD=(\{.*?\});<\/script>/.exec(html)[1]).seeder.version;
  const bin = Buffer.concat([Buffer.from([0xE9]), Buffer.alloc(31),
   Buffer.from('{"fw":"Seeder_ESP32","ver":"' + running + '","built":"' + LATER + '"\0,"board":"Relay24_ESP32"\0'), Buffer.alloc(4000, 0x5A)]);
  const raw = "https://raw.githubusercontent.com/renatorgr/Boards/main/Seeder/Seeder_ESP32/Seeder_ESP32.ino.bin";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/seeder/#firmware", what,
   j => { j.seeder = {file: "Seeder/Seeder_ESP32/Seeder_ESP32.ino.bin", name: "Seeder_ESP32", version: running, built: LATER, board: "Relay24_ESP32", size: bin.length}; });
  const cors = {"Access-Control-Allow-Origin": "*"};
  await pg.route("https://api.github.com/repos/renatorgr/Boards/contents/Seeder/Seeder_ESP32?ref=main", r => r.fulfill({status: 200,
   contentType: "application/json", headers: cors, body: JSON.stringify([{type: "file", name: "Seeder_ESP32.ino.bin", size: bin.length, download_url: raw}])}));
  await pg.route(raw, r => r.fulfill({status: 200, contentType: "application/octet-stream", headers: cors, body: bin}));
  pg.on("dialog", d => d.accept("agopengps"));
  await pg.waitForTimeout(2500);
  if (!(await pg.locator("#t-firmware").isVisible())) fail.push(what + ": #firmware in the address does not open the Firmware tab");
  await expect(pg, what, "#ghVer", /built 31 Dec 2099 23:59 — newer than the installed one$/);
  if (!(await pg.locator('nav button[data-t="firmware"].new').count())) fail.push(what + ": the Firmware tab is not marked new");
  if (await pg.locator("#ghInstall").isDisabled()) fail.push(what + ": Install from GitHub is not offered");
  else {
   await pg.click("#ghInstall");
   try { await pg.waitForFunction(() => /^Back:/.test(document.getElementById("msgProg").textContent), null, {timeout: 25000}); }
   catch (e) { fail.push(what + ": the module did not come back with the new program (" + (await pg.locator("#msgProg").textContent()) + ")"); }
   await expect(pg, what, "#msgProg", /^Back: it runs Seeder_ESP32 v[\d.]+, built 31 Dec 2099 23:59 for the Relay24_ESP32\.$/);
   await expect(pg, what, "#ghVer", /built 31 Dec 2099 23:59 — the one installed$/);
   if (await pg.locator('nav button[data-t="firmware"].new').count()) fail.push(what + ": the Firmware tab is still marked new after the update");
  }
  await shot(pg, "seeder-firmware-updated");
  await pg.close();
 }
 // the sprayer module's page: 8 sections spraying on the X8 board's 8 relays, the rate on target, the boom pressure;
 // Settings: the section widths from the program, calibration and spraying settings saved (a higher target asks to slow
 // down), the total set to 0; Module: the relays, the search for Wi-Fi networks; Firmware: the program on GitHub, wrong
 // files refused, the report downloaded
 {
  const what = "Sprayer page";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/sprayer/", what);
  pg.on("dialog", d => d.accept("agopengps"));
  await pg.waitForTimeout(2500);
  await expect(pg, what, "h1", /^Sprayer module$/);
  await expect(pg, what, "#sub", /^on Farm WiFi as 192\.168\.5\.123 · program talking/);
  if (!(await pg.locator("#dot.on").count())) fail.push(what + ": the page is not connected to the pretend module");
  if (!(await pg.locator("#t-work").isVisible())) fail.push(what + ": the Spraying tab does not show first");
  if ((await pg.locator("#secs span.on").count()) !== 8) fail.push(what + ": expected 8 sections open while spraying");
  await expect(pg, what, "#bar", /^[23]\.\dbar$/);
  await expect(pg, what, "#lha", /^1[0-4]\dL\/ha$/);
  if (!(await pg.locator("#lha.on-target").count())) fail.push(what + ": the rate is not shown on target");
  await expect(pg, what, "#target", /^target 120 L\/ha$/);
  await expect(pg, what, "#need", /^set [23]\.\d bar for the target$/);
  if (await pg.locator("#alarms").isVisible()) fail.push(what + ": an alarm shows while spraying as it should");
  await expect(pg, what, "#detail", /L\/min · [\d.]+ km\/h · 8\.00 m open \(16 nozzles\)/);
  if (!(await pg.locator("#sprCard").isVisible())) fail.push(what + ": the rate and pressure are missing with both sensors");
  await shot(pg, "sprayer-status");
  await pg.click('nav button[data-t="settings"]'); await pg.waitForTimeout(700);
  if (!(await pg.locator("#t-settings").isVisible())) fail.push(what + ": tab Settings does not show");
  if (!(await pg.locator("#ppl").isVisible()) || !(await pg.locator("#pmax").isVisible())) fail.push(what + ": calibration fields missing with both sensors");
  await expect(pg, what, "#tool", /^8 sections of 1\.00 m$/);
  await shot(pg, "sprayer-settings");
  await pg.click("button[onclick='saveCalib()']"); await pg.waitForTimeout(800);
  await expectSoon(pg, what, "#msgCalib", /done/);
  await expectSoon(pg, what, "#log", /saved: pressure sensor \+ flow meter, 450\.0 pulses\/L/);
  await pg.fill("#rate", "200");
  await pg.click("button[onclick='saveSpray()']"); await pg.waitForTimeout(800);
  await expectSoon(pg, what, "#msgSpray", /done/);
  await expectSoon(pg, what, "#log", /saved: target 200 L\/ha, nozzles every 0\.50 m, 0\.80 L\/min at 3\.0 bar, 1\.0-6\.0 bar, flow \+-20 %/);
  await pg.fill("#pLow", "7");
  await pg.click("button[onclick='saveSpray()']"); await pg.waitForTimeout(800);
  await expectSoon(pg, what, "#msgSpray", /the minimum pressure must be below the maximum/);
  await pg.getByRole("button", {name: "Total back to 0"}).click(); await pg.waitForTimeout(1500);
  await expectSoon(pg, what, "#msgTotal", /done/);
  await expectSoon(pg, what, "#total", /^0\.\d L$/);
  await expectSoon(pg, what, "#log", /total set to 0 from the page/);
  await pg.click('nav button[data-t="work"]'); await pg.waitForTimeout(1200);
  try { await pg.waitForSelector("#lha.off-target", {timeout: 5000}); } catch (e) { /* reported below */ }
  if (!(await pg.locator("#lha.off-target").count())) fail.push(what + ": the rate is not shown off target with a 200 L/ha target");
  await expect(pg, what, "#need", /^\d\.\d bar needed, slow down to \d\.\d km\/h$/);
  await pg.click('nav button[data-t="module"]'); await pg.waitForTimeout(700);
  if (!(await pg.locator("#t-module").isVisible())) fail.push(what + ": tab Module does not show");
  if ((await pg.locator("#relays span").count()) !== 8) fail.push(what + ": expected the X8 board's 8 relays");
  await expect(pg, what, "#relayNote", /^8 relays, 8 of them sections/);
  // the Wi-Fi settings as on the XIAO page: "Search for networks" lists what the module hears, strongest first, and a tap
  // chooses one; Save wants the password typed in its card (nothing is saved here: the module would restart)
  await pg.click("#scan");
  await expectSoon(pg, what, "#scanMsg", /^4 networks · tap one to choose it$/);
  if ((await pg.locator("#netList button").count()) !== 4) fail.push(what + ": expected 4 networks in the list");
  await expect(pg, what, "#netList button:first-child", /^Farm WiFi-58 dBm · strong$/);
  await pg.locator("#netList button", {hasText: "Phone hotspot"}).click();
  if (await pg.inputValue("#staSsid") !== "Phone hotspot" || !(await pg.isChecked("#staOn"))) fail.push(what + ": a tap on a network does not choose it");
  if ((await pg.locator("#netList button.sel").count()) !== 1) fail.push(what + ": the network chosen is not marked");
  await pg.click("button[onclick='saveWifi()']");
  await expectSoon(pg, what, "#msgWifi", /^Type the password of the own Wi-Fi network\.$/);
  await shot(pg, "sprayer-module");
  await pg.click('nav button[data-t="firmware"]'); await pg.waitForTimeout(700);
  if (!(await pg.locator("#t-firmware").isVisible())) fail.push(what + ": tab Firmware does not show");
  await expect(pg, what, "#ver", /^Sprayer_ESP32 v[\d.]+$/);
  await expect(pg, what, "#board", /^ESP32_Relay_X8$/);
  // the .bin on main: the one the pretend module runs (or an older one, when the code changed and the .bin did not yet)
  await expect(pg, what, "#ghVer", /^v[\d.]+, built [^—]+ — (the one installed|older than the installed one \(the new \.bin is not on GitHub yet\))$/);
  if (!(await pg.locator("#ghInstall").isDisabled())) fail.push(what + ": Install from GitHub is offered with nothing newer there");
  if (await pg.locator('nav button[data-t="firmware"].new').count()) fail.push(what + ": the Firmware tab is marked new with nothing newer");
  await shot(pg, "sprayer-firmware");
  // wrong files: the .merged.bin, and a program for another board
  const sprBin = (ver, built, board) => Buffer.concat([Buffer.from([0xE9]), Buffer.alloc(31),
   Buffer.from('{"fw":"Sprayer_ESP32","ver":"' + ver + '","built":"' + built + '"\0,"board":"' + board + '"\0')]);
  for (const [name, buf, re] of [["Sprayer_ESP32.ino.merged.bin", Buffer.alloc(64, 0xFF), /^This is the wrong file: use Sprayer_ESP32\.ino\.bin/],
   ["Sprayer_ESP32.ino.bin", sprBin("0.10.0", LATER, "ESP32 dev module"), /^This program is for the ESP32 dev module board, but this module is on the ESP32_Relay_X8 board/],
   ["Seeder_ESP32.ino.bin", Buffer.concat([Buffer.from([0xE9]), Buffer.alloc(31), Buffer.from('{"fw":"Seeder_ESP32","ver":"0.2","built":"' + LATER + '"')]), /^This is the program Seeder_ESP32, not this module's/]]) {
   await pg.setInputFiles("#bin", {name: name, mimeType: "application/octet-stream", buffer: buf});
   await pg.click("#install"); await pg.waitForTimeout(500);
   await expect(pg, what + " (" + name + ")", "#msgProg", re);
  }
  if (/program update/.test(await pg.locator("#log").textContent())) fail.push(what + ": a wrong file was sent to the module");
  try {
   const [dl] = await Promise.all([pg.waitForEvent("download", {timeout: 5000}), pg.click("#dumpBtn")]);
   const name = dl.suggestedFilename(), r = JSON.parse(require("fs").readFileSync(await dl.path(), "utf8"));
   if (!/^Sprayer_ESP32_report_\d{4}-\d\d-\d\d_\d{4}\.json$/.test(name)) fail.push(what + ": the report is called " + name);
   if (!r.status || !r.status.rate || !r.settings || r.settings.rate !== 200)
    fail.push(what + ": the report misses the status or the settings");
   if (JSON.stringify(r).includes("agopengps")) fail.push(what + ": the report has the password in it");
  } catch (e) { fail.push(what + ": no report downloaded (" + e.message + ")"); }
  await expect(pg, what, "#dumpMsg", /^Downloaded: Sprayer_ESP32_report_/);
  await pg.close();
 }
 // a newer build of the sprayer program on GitHub: offered, marked on the Firmware tab, downloaded from GitHub (here: the
 // .bin on main with the version running and a later build date) and sent to the module, which then runs it
 {
  const what = "Sprayer page, newer build on GitHub";
  const html = fs.readFileSync(path.join(site, "sprayer", "index.html"), "utf8");
  const running = JSON.parse(/window\.SIM_BUILD=(\{.*?\});<\/script>/.exec(html)[1]).sprayer.version;
  const onMain = fs.readFileSync(path.join(__dirname, "..", "..", "Sprayer", "Sprayer_ESP32", "Sprayer_ESP32.ino.bin"));
  const bin = Buffer.from(onMain.toString("latin1").replace(/\{"fw":"Sprayer_ESP32","ver":"[^"]*","built":"[^"]*"/,
   '{"fw":"Sprayer_ESP32","ver":"' + running + '","built":"' + LATER + '"'), "latin1");
  const raw = "https://raw.githubusercontent.com/renatorgr/Boards/main/Sprayer/Sprayer_ESP32/Sprayer_ESP32.ino.bin";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/sprayer/#firmware", what,
   j => { Object.assign(j.sprayer, {version: running, built: LATER, size: bin.length}); });
  const cors = {"Access-Control-Allow-Origin": "*"};
  await pg.route("https://api.github.com/repos/renatorgr/Boards/contents/Sprayer/Sprayer_ESP32?ref=main", r => r.fulfill({status: 200,
   contentType: "application/json", headers: cors, body: JSON.stringify([{type: "file", name: "Sprayer_ESP32.ino.bin", size: bin.length, download_url: raw}])}));
  await pg.route(raw, r => r.fulfill({status: 200, contentType: "application/octet-stream", headers: cors, body: bin}));
  pg.on("dialog", d => d.accept("agopengps"));
  await pg.waitForTimeout(2500);
  if (!(await pg.locator("#t-firmware").isVisible())) fail.push(what + ": #firmware in the address does not open the Firmware tab");
  await expect(pg, what, "#ghVer", /built 31 Dec 2099 23:59 — newer than the installed one$/);
  if (!(await pg.locator('nav button[data-t="firmware"].new').count())) fail.push(what + ": the Firmware tab is not marked new");
  if (await pg.locator("#ghInstall").isDisabled()) fail.push(what + ": Install from GitHub is not offered");
  else {
   await pg.click("#ghInstall");
   try { await pg.waitForFunction(() => /^Back:/.test(document.getElementById("msgProg").textContent), null, {timeout: 25000}); }
   catch (e) { fail.push(what + ": the module did not come back with the new program (" + (await pg.locator("#msgProg").textContent()) + ")"); }
   await expect(pg, what, "#msgProg", /^Back: it runs Sprayer_ESP32 v[\d.]+, built 31 Dec 2099 23:59 for the ESP32_Relay_X8\.$/);
   await expect(pg, what, "#ghVer", /built 31 Dec 2099 23:59 — the one installed$/);
   if (await pg.locator('nav button[data-t="firmware"].new').count()) fail.push(what + ": the Firmware tab is still marked new after the update");
  }
  await shot(pg, "sprayer-firmware-updated");
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
