// smoke-test.js - opens the built simulator in a browser and goes through its pages, as a check before it is published.
// Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator, see README.md there).
// Written 2026 by Claude (Anthropic) for renatorgr.
// Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// node Firmware/simulator/smoke-test.js <site folder> [screenshot folder]   (needs the playwright package)
//
// Serves the site under /Boards/ as GitHub Pages does, opens the Teensy page with each receiver and the XIAO
// page, waits for the pretend module, clicks every tab and fails on a script error in the page or a tab that
// shows nothing. GitHub itself is not asked (the version check then says "no answer"). Also installs the
// firmware's own .hex on a parked module, end to end.
"use strict";
const fs = require("fs"), path = require("path"), http = require("http");
const {chromium} = require("playwright");
const site = path.resolve(process.argv[2] || "_site"), shots = process.argv[3] ? path.resolve(process.argv[3]) : "";
const TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript", ".css": "text/css"};
const fail = [];

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
  ["?gps=dualf9p&imu=rvc&motor=ibt2&prog=aog", "dualf9p"], ["?imu=none&motor=danfoss", "noimu"]]) {
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
 await browser.close();
 server.close();
 if (fail.length) { console.log("FAILED:\n" + fail.join("\n")); process.exit(1); }
 console.log("Simulator pages OK");
})().catch(e => { console.log(e); process.exit(1); });
