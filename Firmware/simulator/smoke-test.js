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
// seed drill's relays follow and the sprayer's calibration (the pages' own password question is answered). Also opens the sprayer
// and Teensy pages inside a frame with the sandbox of an AgOpenWeb web card, where the browser's confirm() and prompt() do
// nothing: the pages' own question box must show there, fit the card and answer. A browser dialog on any page fails the check.
"use strict";
const fs = require("fs"), path = require("path"), http = require("http");
const {chromium} = require("playwright");
const site = path.resolve(process.argv[2] || "_site"), shots = process.argv[3] ? path.resolve(process.argv[3]) : "";
const TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript", ".css": "text/css"};
const fail = [];
// an AgOpenWeb web card's frame (AgOpenWeb Docs/ADD_ON_CARDS.md): no allow-modals, so alert / confirm / prompt show nothing in it
const CARD_SANDBOX = "allow-scripts allow-same-origin allow-forms allow-popups allow-downloads";
// the whole check takes about two minutes; a browser that hangs must not hold the Action for hours
const watchdog = setTimeout(() => { console.log("FAILED: the check did not finish in 6 minutes"); process.exit(1); }, 6 * 60 * 1000);

const server = http.createServer((q, r) => {
 const u = decodeURIComponent(q.url.split("?")[0]);
 if (!u.startsWith("/Boards/")) { r.writeHead(404); r.end(); return; }
 if (u === "/Boards/_card") {   // the check only: a pretend AgOpenWeb web card around a page of the site (?src=/Boards/...), served from
  // the same address as the page because Chromium refuses a loopback page inside a frame of a page from nowhere (about:blank)
  const src = (new URL(q.url, "http://127.0.0.1").searchParams.get("src") || "").replace(/"/g, "&quot;");
  r.writeHead(200, {"Content-Type": "text/html; charset=utf-8"});
  r.end('<!doctype html><html><body style="margin:0"><iframe sandbox="' + CARD_SANDBOX + '" src="' + src + '" style="width:480px;height:320px;border:0;display:block"></iframe></body></html>');
  return;
 }
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
 // inside an AgOpenWeb web card the browser's alert / confirm / prompt show nothing (the frame has no allow-modals): the pages ask
 // with their own question box (#ask) instead, so a browser dialog is a fault
 pg.on("dialog", d => { fail.push(what + ": the browser's own dialog opened («" + d.message() + "»); it would show nothing in a web card"); d.dismiss(); });
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
// the pages' own question box (ask / askText in the pages) answered with OK as soon as it opens, as a person would; `text` is typed
// into its field first (the module's password, when it asks for one). Stays on after the page reloads itself (a restart)
async function answerAsk(pg, text) {
 const js = "(()=>{if(window.__askAuto)return;window.__askAuto=1;" +
  "const go=()=>{const a=document.getElementById('ask');if(!a||a.hidden)return;const i=document.getElementById('askIn');" +
  "if(i&&!i.hidden)i.value=" + JSON.stringify(text || "") + ";document.getElementById('askYes').click();};" +
  "const start=()=>{const a=document.getElementById('ask');if(!a)return;new MutationObserver(go).observe(a,{attributes:true,attributeFilter:['hidden']});go();};" +
  "document.readyState==='loading'?document.addEventListener('DOMContentLoaded',start):start();})()";
 await pg.addInitScript(js);
 await pg.evaluate(js);
}
// a page of the site inside an AgOpenWeb web card: the server's /Boards/_card, an iframe with the card's sandbox at the smallest
// card size, 480 x 320. There the browser's confirm() and prompt() return false / null without showing anything, which the check
// first makes sure of (otherwise the frame is not testing what a card does). Gives the page and its frame (null: not loaded)
async function card(browser, port, src, what) {
 const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/_card?src=" + encodeURIComponent(src), what);
 pg.removeAllListeners("dialog"); pg.removeAllListeners("console");
 let native = 0, probing = true;   // the check's own confirm() and prompt() below: Chromium logs "Ignored call to 'confirm()'" for them
 pg.on("dialog", d => { native++; d.dismiss(); });
 pg.on("console", m => { if (m.type() === "error" && !/Failed to load resource|net::ERR/.test(m.text()) && !(probing && /Ignored call to '(confirm|prompt)\(\)'/.test(m.text()))) fail.push(what + ": console " + m.text()); });
 await pg.waitForTimeout(2500);
 const base = "http://127.0.0.1:" + port + src.split("#")[0], fr = pg.frames().find(f => f.url().startsWith(base));
 if (!fr) { fail.push(what + ": the page did not load inside the frame"); return [pg, null]; }
 const r = await fr.evaluate(() => [window.confirm("test"), window.prompt("test")]);
 if (native || r[0] !== false || r[1] !== null) fail.push(what + ": the frame does not behave as a web card (confirm " + r[0] + ", prompt " + r[1] + ", dialogs " + native + ")");
 await pg.waitForTimeout(300); probing = false;
 return [pg, fr];
}
// the question box open in the card: its text, whether it asks to type something, inside the 480 x 320, the OK button 44 px high
async function askShown(fr, what, re, withInput) {
 await fr.waitForTimeout(200);
 if (!(await fr.locator("#ask").isVisible())) { fail.push(what + ": the question box did not open"); return; }
 await expect(fr, what, "#askText", re);
 if ((await fr.locator("#askIn").isVisible()) !== withInput) fail.push(what + ": the question " + (withInput ? "has no field to type in" : "should not ask to type anything"));
 const b = await fr.locator(".askBox").boundingBox(), y = await fr.locator("#askYes").boundingBox();
 if (!b || b.x < 0 || b.y < 0 || b.x + b.width > 480 || b.y + b.height > 320) fail.push(what + ": the question box does not fit the card (" + JSON.stringify(b) + ")");
 if (!y || y.height < 44) fail.push(what + ": the OK button is under 44 px high (" + (y && y.height) + ")");
}
async function askClosed(fr, what, how) {
 await fr.waitForTimeout(200);
 if (await fr.locator("#ask").isVisible()) fail.push(what + ": " + how + " does not close the question");
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
  await expect(pg, what, "h1", /^Set up AgOpenWeb from your measures$/);
  // it opens on Renato's Case and front sprayer
  await expect(pg, what, "#results tbody tr[data-key=radius] td.v", /^4 m$/);          // U-Turn radius: 1.2 × (2.7 ÷ tan 42°) = 3.6, floor 4 m
  await expect(pg, what, "#results tbody tr[data-key=distance] td.v", /^6\.4 m$/);     // distance from boundary: the 8 m boom's end plus the 1 m margin
  await expect(pg, what, "#results tbody tr[data-key=hitch] td.v", /^4\.8 m ahead of the rear axle$/);   // the boom 2.1 m ahead of the front axle + the 2.7 m wheelbase
  await expect(pg, what, "#results tbody tr[data-key=headland] td.v", /^16 m \(2 passes\)$/);
  // the reasons hide behind an arrow on each row, and the header's arrow opens them all
  if (await pg.locator("#results tbody tr.why:visible").count()) fail.push(what + ": the reasons should start hidden");
  await pg.click("#results tbody tr[data-key=radius] button.why"); await pg.waitForTimeout(200);
  await expect(pg, what, "#results tbody tr[data-key=radius] + tr.why td", /tightest radius 3 m/);
  if ((await pg.locator("#results tbody tr.why:visible").count()) !== 1) fail.push(what + ": one arrow should open one reason");
  await pg.click("#allWhy"); await pg.waitForTimeout(200);
  if ((await pg.locator("#results tbody tr.why:visible").count()) !== (await pg.locator("#results tbody tr[data-key]").count())) fail.push(what + ": the header arrow should open every reason");
  await pg.click("#allWhy"); await pg.waitForTimeout(200);
  if (await pg.locator("#results tbody tr.why:visible").count()) fail.push(what + ": the header arrow should close every reason again");
  if ((await pg.locator("#turns tbody tr").count()) !== 1) fail.push(what + ": a plain semicircle is the only turn for the sprayer");
  if (!(await pg.locator("#draw svg").count())) fail.push(what + ": no drawing");
  if (await pg.locator("#draw svg circle[stroke-dasharray]").count()) fail.push(what + ": the drawing should show the implement's ends, not a reach circle");
  await expect(pg, what, "#legend", /^Black line: the centre of the rear axle/);
  await expect(pg, what, ".ver", /^version \d+\.\d+$/);
  if ((await pg.locator("#legend li").count()) < 8) fail.push(what + ": the legend should have one line per colour");
  await shot(pg, "aow-settings-sprayer");
  await pg.click("button:has-text('3 m no-till drill')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#results tbody tr[data-key=style] td.v", /^Keyhole, skip 1$/);   // the smallest headland; backing it up jackknifes: no K-turn
  await expect(pg, what, "#results tbody tr[data-key=headland] td.v", /^18\.3 m \(6 passes\)$/);
  await expect(pg, what, "#results tbody tr[data-key=tram] td.v", /^no match$/);     // tramlines: 8 ÷ 3 is not whole
  if ((await pg.locator("#turns tbody tr").count()) !== 5) fail.push(what + ": expected five turns for the drill (Sagitta skip 0-2, Keyhole skip 0-1)");
  await expect(pg, what, "#turns tbody tr:first-child", /Keyhole, skip 1.*8\.8 m.*3\.4 m.*18\.3 m \(6 passes\)/);
  await expect(pg, what, "#turnsNote", /^No K-turn: /);
  await expect(pg, what, "#draw svg text:last-child", /^next pass 6\.1 m away: Keyhole, radius 4\.3 m, 8\.8 m deep$/);
  await pg.click("#turns tbody tr:has-text('Sagitta, skip 2')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#results tbody tr[data-key=headland] td.v", /^21\.35 m \(7 passes\)$/);   // the rows follow the pick
  await expect(pg, what, "#draw svg text:last-child", /^next pass 9\.15 m away: Sagitta, radius 4\.3 m, 4\.3 m deep$/);   // 3 × 3.05, no float tail
  await pg.click("#turns tbody tr:has-text('Keyhole, skip 0')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#draw svg text:last-child", /^next pass 3\.05 m away: Keyhole, radius 4\.3 m, 10\.6 m deep$/);
  await shot(pg, "aow-settings-drill");
  // Renato's one-axle manure spreader, then two examples with typical measures: a trailed disc harrow, a reversible plough
  await pg.click("button:has-text('10 m manure spreader')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#presetNote", /^Renato's manure spreader: one axle 4\.2 m behind the drawbar pin/);
  await expect(pg, what, "#results tbody tr[data-key=radius] td.v", /^5\.4 m$/);     // its drawbar pin inside the rear tyres: they set the radius (the tyre at the drawbar's 0.5 m height)
  await expect(pg, what, "#results tbody tr[data-key=style] td.v", /^Keyhole, skip 0$/);   // its short body is straight again soon: the Keyhole's headland is the smallest
  await expect(pg, what, "#results tbody tr[data-key=headland] td.v", /^20 m \(2 passes\)$/);
  await expect(pg, what, "#results tbody tr[data-key=fold] td.v", /^touches the tyres at 65°$/);
  if (!(await pg.locator("#drawbarHeight").isVisible())) fail.push(what + ": a trailed implement needs the drawbar's height");
  await shot(pg, "aow-settings-spreader");
  // the tractor buttons: the Farmall fills the Machine card and moves the spreader to its own drawbar pin, then back to the Maxxum
  await expect(pg, what, "#tractors button.on", /^Case IH Maxxum 135$/);
  await expect(pg, what, "#implements button.on", /^10 m manure spreader/);
  await pg.click("button:has-text('Case IH Farmall 115C')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#tractors button.on", /^Case IH Farmall 115C$/);
  await expect(pg, what, "#tractorNote", /^Renato's Case IH Farmall 115C: 2\.3 m wheelbase/);
  if ((await pg.inputValue("#wheelbase")) !== "2.3" || (await pg.inputValue("#hitch")) !== "0.74") fail.push(what + ": the Farmall button should set its wheelbase and its drawbar pin");
  await expect(pg, what, "#results tbody tr[data-key=radius] td.v", /^4\.8 m$/);     // its lower 540/65 R34 tyres let the drawbar fold to 76°
  await expect(pg, what, "#results tbody tr[data-key=fold] td.v", /^touches the tyres at 76°$/);
  await shot(pg, "aow-settings-farmall-spreader");
  await pg.click("button:has-text('Case IH Puma 180 CVX')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#tractors button.on", /^Case IH Puma 180 CVX$/);
  if ((await pg.inputValue("#wheelbase")) !== "2.88" || (await pg.inputValue("#steer")) !== "33" || (await pg.inputValue("#hitch")) !== "0.9") fail.push(what + ": the Puma button should set its wheelbase, its steering and its drawbar pin");
  await expect(pg, what, "#results tbody tr[data-key=radius] td.v", /^5\.4 m$/);     // 33° from its manual: the tractor sets the radius, the drawbar only folds to 82.5°
  await expect(pg, what, "#results tbody tr[data-key=fold] td.v", /^touches the tyres at 82\.5°$/);
  await pg.click("button:has-text('Case IH Maxxum 135')"); await pg.waitForTimeout(300);
  if ((await pg.inputValue("#hitch")) !== "0.7") fail.push(what + ": the Maxxum button should put the spreader back on its 0.7 m pin");
  await expect(pg, what, "#results tbody tr[data-key=radius] td.v", /^5\.4 m$/);
  // Renato's centre-pivot mower conditioner comes with the Farmall, on its drawbar pin; then back to the Maxxum
  await pg.click("button:has-text('3.2 m mower conditioner')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#implements button.on", /^3\.2 m mower conditioner, centre pivot$/);
  await expect(pg, what, "#tractors button.on", /^Case IH Farmall 115C$/);
  await expect(pg, what, "#presetNote", /^Renato's centre-pivot mower conditioner, used with the Farmall/);
  if ((await pg.inputValue("#wheelbase")) !== "2.3" || (await pg.inputValue("#hitch")) !== "0.74" || (await pg.inputValue("#width")) !== "3.2") fail.push(what + ": the mower button should press the Farmall and put the mower on its 0.74 m pin");
  await expect(pg, what, "#results tbody tr[data-key=radius] td.v", /^5\.6 m$/);     // its 5 m tongue reaches the tyres at 76°
  await expect(pg, what, "#results tbody tr[data-key=style] td.v", /^Keyhole, skip 2$/);
  await expect(pg, what, "#results tbody tr[data-key=headland] td.v", /^21\.7 m \(7 passes\)$/);
  await shot(pg, "aow-settings-mower");
  await pg.click("button:has-text('Case IH Maxxum 135')"); await pg.waitForTimeout(300);
  if ((await pg.inputValue("#hitch")) !== "0.7") fail.push(what + ": the Maxxum button should put the mower on its 0.7 m pin");
  await pg.click("button:has-text('4 m disc harrow')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#results tbody tr[data-key=style] td.v", /^Sagitta, skip 2$/);
  await pg.click("button:has-text('4-furrow reversible plough')"); await pg.waitForTimeout(300);
  if (await pg.locator("#drawbarHeight").isVisible()) fail.push(what + ": the drawbar height is for trailed implements only");
  if (!(await pg.locator("#trackWidth").isVisible())) fail.push(what + ": the rear track gives the tractor's width for every implement");
  await expect(pg, what, "#results tbody tr[data-key=style] td.v", /^K-turn, skip 0$/);   // the preset picks the K-turn: a plough skips no rows
  await expect(pg, what, "#results tbody tr[data-key=distance] td.v", /^6\.6 m$/);
  await expect(pg, what, "#draw svg text:last-child", /^next pass 1\.6 m away: K-turn, radius 4 m, 8\.7 m deep \(the backing up included\)$/);   // a plough skips no rows
  if ((await pg.locator("#draw svg polyline[stroke-dasharray='1 4']").count()) !== 1) fail.push(what + ": the plough's rear corner should be drawn dotted");
  await expect(pg, what, "#legend", /Blue dots: the implement's outer rear corner/);
  if (!(await pg.locator("#length").isVisible())) fail.push(what + ": the plough needs its length");
  await shot(pg, "aow-settings-plough");
  await pg.click("button:has-text('3.2 m roller')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#results tbody tr[data-key=radius] td.v", /^4\.2 m$/);     // its own 60° stop sets the radius
  await expect(pg, what, "#results tbody tr[data-key=fold] td.v", /^touches the tyres at 113\.5°, at most 60°$/);
  await pg.fill("#foldClearanceDeg", "10"); await pg.waitForTimeout(300);
  await expect(pg, what, "#results tbody tr[data-key=radius] td.v", /^4\.6 m$/);     // more room before the stop: a wider turn
  await shot(pg, "aow-settings-roller");
  await pg.click("button:has-text('2.7 m quick disc harrow')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#results tbody tr[data-key=style] td.v", /^Sagitta, skip 2$/);
  if ((await pg.locator("#turns tbody tr").count()) !== 6) fail.push(what + ": expected six turns for the quick harrow, the K-turn among them");
  await pg.fill("#overlap", "3"); await pg.waitForTimeout(300);
  await expect(pg, what, "#problems", /overlap must be less than the implement width/);
  // the front sprayer on a tractor with 35° on the stop: two turns too tight for a semicircle, and a K-turn
  await pg.click("button:has-text('8 m sprayer')"); await pg.waitForTimeout(200);
  await pg.selectOption("#mount", "front"); await pg.fill("#steer", "35"); await pg.fill("#overhang", "1.5"); await pg.fill("#axleToTool", "2"); await pg.waitForTimeout(300);
  await expect(pg, what, "#axleToToolLabel", /^Front axle to the working line/);
  if (await pg.locator("#length").isVisible()) fail.push(what + ": the implement length is for rear and trailed implements only");
  await expect(pg, what, "#results tbody tr[data-key=distance] td.v", /^6\.2 m$/);                     // distance from boundary
  await expect(pg, what, "#results tbody tr[data-key=headland] td.v", /^16 m \(2 passes\)$/);   // Sagitta, skip 1; the boom on the next pass 4.7 m back
  await expect(pg, what, "#results tbody tr[data-key=hitch] td.v", /^4\.7 m ahead of the rear axle$/); // AgOpenWeb's hitch length
  await shot(pg, "aow-settings-front-sprayer");
  await pg.click("#turns tbody tr:has-text('K-turn')"); await pg.waitForTimeout(300);
  await expect(pg, what, "#draw svg text:last-child", /^next pass 8 m away: K-turn, radius 4\.7 m, 5\.8 m deep \(the backing up included\)$/);
  if ((await pg.locator("#draw svg path[stroke-dasharray='7 5']").count()) !== 1) fail.push(what + ": the K-turn's backing up should be drawn dashed");
  await shot(pg, "aow-settings-front-kturn");
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
  await answerAsk(pg);
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
  await answerAsk(pg);
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
 // section 1 and so on, 19 sections set); Settings: the relays set back to the program's pin config (the password asked in a
 // prompt);
 // Module: the search for Wi-Fi networks; Firmware: the program on GitHub (none until the .bin is on main), wrong files
 // refused, the report downloaded
 {
  const what = "Seed drill page";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/seeder/", what);
  await answerAsk(pg, "agopengps");
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
  await expect(pg, what, "#relayNote", /^24 relays, 19 of them sections: relay 1 follows section 1/);
  await expect(pg, what, "#r18", /^19S19$/);
  await expect(pg, what, "#r19", /^20–$/);
  if ((await pg.locator("#drillSecs").inputValue()) !== "19") fail.push(what + ": the Settings tab should show 19 sections on the seed drill");
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
  await answerAsk(pg, "agopengps");
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
  await answerAsk(pg, "agopengps");
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
  await answerAsk(pg, "agopengps");
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
 // the sprayer page inside a web card: Save and restart (Module tab) asks yes / no (Escape is a no), Restart (Firmware tab) asks
 // the password in the box (Cancel does nothing, Enter after the password restarts the module)
 {
  const what = "Sprayer page in a web card";
  const [pg, fr] = await card(browser, port, "/Boards/sprayer/#module", what);
  if (fr) {
   if (!(await fr.locator("#t-module").isVisible())) fail.push(what + ": #module in the address does not open the Module tab");
   await fr.fill("#pwW", "agopengps");
   await fr.click("button:has-text('Save and restart the module')");
   await askShown(fr, what, /^Save and restart the module\? Its Wi-Fi goes off/, false);
   await shot(pg, "sprayer-card-ask");
   await fr.locator("#askYes").press("Escape"); await askClosed(fr, what, "Escape");
   await expect(fr, what, "#msgWifi", /^$/);
   await fr.click('nav button[data-t="firmware"]'); await fr.waitForTimeout(300);
   await fr.click("button[onclick*='/api/restart']");
   await askShown(fr, what, /^Password of the own Wi-Fi network$/, true);
   await shot(pg, "sprayer-card-password");
   await fr.click("#askNo"); await askClosed(fr, what, "Cancel");
   await expect(fr, what, "#msgProg", /^$/);
   await fr.click("button[onclick*='/api/restart']");
   await fr.fill("#askIn", "agopengps"); await fr.locator("#askIn").press("Enter"); await askClosed(fr, what, "Enter");
   await expectSoon(fr, what, "#msgProg", /^done, restarting/);
  }
  await pg.close();
 }
 // the Teensy page inside a web card: Restart Teensy asks yes / no
 {
  const what = "Teensy page in a web card";
  const [pg, fr] = await card(browser, port, "/Boards/teensy/?park=1", what);
  if (fr) {
   await fr.click('nav button[data-t="firmware"]'); await fr.waitForTimeout(300);
   await fr.click("#reboot");
   await askShown(fr, what, /^Restart the Teensy\? Autosteer and GPS stop for a few seconds\.$/, false);
   await shot(pg, "teensy-card-ask");
   await fr.click("#askNo"); await askClosed(fr, what, "Cancel");
   await expect(fr, what, "#fwMsg", /^$/);
   await fr.click("#reboot");
   await fr.click("#askYes"); await askClosed(fr, what, "OK");
   await expectSoon(fr, what, "#fwMsg", /^Restarting/);
  }
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
 // the seed drill's module before the program ever talked (the cable not plugged in): the 19 sections set on the Settings
 // tab show, not the 16 of the program's pin config
 {
  const what = "Seed drill page, cable unplugged";
  const pg = await page(browser, "http://127.0.0.1:" + port + "/Boards/seeder/?wifi=ap", what);
  await pg.waitForTimeout(2500);
  await expect(pg, what, "#sub", /not on the tractor network · program silent/);
  if ((await pg.locator("#secs span").count()) !== 19) fail.push(what + ": expected the 19 sections set, with the program silent");
  if ((await pg.locator("#secs span.on, #secs span.req").count()) !== 0) fail.push(what + ": a section is open or asked for with the program silent");
  await expect(pg, what, "#detail", /^0 of 19 open/);
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
