// test.js - checks settings.js with known machines, as a step of the simulator Action.
// Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator/aow-settings, see README.md there).
// Written 2026 by Claude (Anthropic) for renatorgr.
// The history of changes is in the README changelog and in the git log.
// Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// node Firmware/simulator/aow-settings/test.js   (no packages needed)
"use strict";
const S = require("./settings.js");
const fail = [];
const eq = (what, got, want) => { if (got !== want) fail.push(what + ": got " + JSON.stringify(got) + ", expected " + JSON.stringify(want)); };
const near = (what, got, want, tol) => { if (Math.abs(got - want) > tol) fail.push(what + ": got " + got + ", expected " + want + " ± " + tol); };

// the tractor of the worked example: 2.7 m wheelbase, 40° on the stop, 1.3 m from the front axle to the nose
const tractor = {machine: "tractor", wheelbase: 2.7, maxSteerDeg: 40, overhang: 1.3};

// an 8 m sprayer mounted 1.8 m behind the rear axle: the next pass is 8 m away, 2 × 4 m, a plain semicircle (1 m margin to the edge)
{
 const s = S.suggest(Object.assign({mount: "rear", width: 8, overlap: 0, axleToTool: 1.8, sprayerWidth: 8}, tractor));
 eq("sprayer problems", s.problems.length, 0);
 eq("sprayer spacing", s.spacing, 8);
 eq("sprayer rMin", s.rMin, 3.2);
 eq("sprayer radius", s.radius, 4);
 eq("sprayer semicircle", s.semicircle, true);
 eq("sprayer skip", s.skip, 0);
 eq("sprayer options", s.options.map(o => o.key).join(","), "semicircle");
 eq("sprayer no K-turn", s.kturn, null);   // a plain semicircle needs no backing up
 eq("sprayer nose reach", s.noseReach, 1.7);
 eq("sprayer tool reach", s.toolReach, 4.2);
 eq("sprayer distance", s.distance, 5.2);   // the boom's end 4.2 m past the pivot, plus the 1 m margin
 eq("sprayer headland", s.options[0].headland.width, 16);
 eq("sprayer headland passes", s.options[0].headland.passes, 2);
 eq("sprayer extension", s.extension, 8);
 eq("sprayer tram", s.tram.whole && s.tram.passes, 1);
 eq("sprayer rows", s.rows.map(r => r.key).join(","), "spacing,rMin,radius,style,distance,headland,extension,goAround,tram");
}

// a 3 m no-till seed drill trailed 3 m behind the rear axle (1 m to the hitch pin, a 2 m drawbar), spraying with the 8 m sprayer:
// the next pass is 3 m away, closer than 2 × 4 m, so skip 2 passes (land 9 m away) or loop with a Keyhole; backing it up in a
// K-turn jackknifes it
{
 const s = S.suggest(Object.assign({mount: "trailed", sections: 19, sectionWidth: 15.8, overlap: 0.002, hitchLength: 1, drawbar: 2, pivotToTool: 0, sprayerWidth: 8}, tractor));
 eq("drill width from the sections", s.spacing, 3);   // 19 × 15.8 cm = 3.002 m, less the 2 mm
 eq("drill problems", s.problems.length, 0);
 eq("drill radius", s.radius, 4);
 eq("drill semicircle", s.semicircle, false);
 eq("drill skip", s.skip, 2);
 eq("drill landing", s.landing, 9);
 eq("drill keyhole depth", s.keyhole, 9.8);
 near("drill sagitta depth", s.sagitta, 11.4, 0.15);
 eq("drill nose reach", s.noseReach, 1.7);
 eq("drill distance", s.distance, 2.7);
 eq("drill axle to tool", s.axleToTool, 3);
 eq("drill options", s.options.map(o => o.key).join(","), "skip,keyhole");
 if (!/No K-turn: .*jackknife/.test(s.rows.find(r => r.key === "style").why)) fail.push("drill: should say why there is no K-turn");
 eq("drill headland with skip", s.options[0].headland.width, 12);   // 4 + 2.7 + 3.1 (straight on the pass 3.1 m back) = 9.8 m, 4 passes
 eq("drill low with skip", s.options[0].low, -3.1);
 eq("drill headland keyhole", s.options[1].headland.width, 18);   // 9.8 + 2.7 + 3 (the drill finishing its pass) = 15.5 m, 6 passes
 eq("drill extension", s.extension, 9);
 eq("drill tram", s.tram.whole, false);
}

// the turns walked step by step: each lands on its pass heading south
{
 const kh = S.turnPath("keyhole", 4, 3, 0), khEnd = kh.poses[kh.turnEnd];
 near("keyhole path depth", S.depthOf(kh.poses), S.keyholeDepth(4, 3), 0.05);
 near("keyhole lands on the pass", khEnd.e, 3, 0.02);
 near("keyhole ends level with its start", khEnd.n, 0, 0.02);
 near("keyhole heading", khEnd.h, Math.PI, 0.01);
 const sk = S.turnPath("skip", 4, 9, 0), skEnd = sk.poses[sk.turnEnd];
 near("skip turn lands on the pass", skEnd.e, 9, 0.02);
 near("skip turn depth", S.depthOf(sk.poses), 4, 0.02);
 // K-turn: the 126° arc and the 4.5 m tail forward, then straight back and the last 54° backing up onto the pass
 const b = S.kTurnBack(4, 3);
 near("K-turn back", b.back, 6.6, 0.05);
 near("K-turn ends on the pass", b.end.e, 3, 1e-9);
 near("K-turn depth", b.end.n, 7.7, 0.05);
 const k = S.turnPath("kturn", 4, 3, 0), kEnd = k.poses[k.turnEnd];
 near("K-turn arc end depth", k.poses[k.arcEnd].n, 4 * Math.sin(2.2), 0.02);
 near("K-turn tail tip", k.poses[k.forwardEnd].n, b.tip.n, 0.02);
 near("K-turn walked onto the pass", kEnd.e, 3, 0.02);
 near("K-turn walked depth", kEnd.n, b.end.n, 0.02);
 near("K-turn heading south", kEnd.h, Math.PI, 0.01);
 eq("K-turn backs up", kEnd.dir, -1);
 eq("K-turn depth from the radius", S.kTurnDepth(4, 3), b.end.n);
}

// the same tractor with 35° on the stop: 2.7 ÷ tan 35° = 3.9 m, × 1.2 = 4.7 m; the sprayer's 8 m pass is now closer than 2 × 4.7
{
 const s = S.suggest(Object.assign({mount: "rear", width: 8, overlap: 0, axleToTool: 1.8}, tractor, {maxSteerDeg: 35}));
 eq("35° radius", s.radius, 4.7);
 eq("35° semicircle", s.semicircle, false);
 eq("35° skip", s.skip, 1);
 eq("35° keyhole depth", s.keyhole, 8.3);
 eq("35° K-turn depth", s.kturn, 5.8);     // backed up onto the next pass 5.8 m in from where the turn started
}

// a small tractor: the 4 m floor; a long drawbar: the hitch angle sets the radius
{
 const s = S.suggest({wheelbase: 1.8, maxSteerDeg: 45, frontOverhang: 0.8, mount: "rear", width: 4, overlap: 0.2, axleToTool: 1.2});
 eq("floor radius", s.radius, 4);
 eq("floor spacing", s.spacing, 3.8);
 const t = S.suggest(Object.assign({mount: "trailed", width: 6, overlap: 0, hitchLength: 1, drawbar: 6, pivotToTool: 1}, tractor));
 near("long drawbar radius", t.radius, 6 / Math.sin(75 * Math.PI / 180), 0.11);
 eq("long drawbar follows", t.problems.length, 0);
}

// an articulated machine (the joint midway) and a harvester (the rear wheels steer, the header ahead of the front axle)
{
 const a = S.suggest({machine: "articulated", wheelbase: 3.0, maxSteerDeg: 40, overhang: 1.0, mount: "rear", width: 8, overlap: 0, axleToTool: 1.8});
 eq("articulated rMin", a.rMin, 4.1);       // (3 ÷ 2) ÷ tan 20°
 eq("articulated radius", a.radius, 5);     // × 1.2 = 4.95, rounded up
 eq("articulated nose reach", a.noseReach, 0.1);   // the front half swings with the turn: √(5² + 1²) − 5
 eq("articulated tool reach", a.toolReach, 4.2);
 eq("articulated distance", a.distance, 5.2);
 eq("articulated skip", a.skip, 1);         // 8 m passes, closer than 2 × 5
 eq("articulated keyhole", a.keyhole, 9.4);
 const h = S.suggest({machine: "harvester", wheelbase: 3.6, maxSteerDeg: 50, overhang: 2.0, mount: "front", width: 6, overlap: 0.3, axleToTool: 2.5});
 eq("harvester rMin", h.rMin, 3);           // 3.6 ÷ tan 50°
 eq("harvester radius", h.radius, 4);       // × 1.2 = 3.6, the 4 m floor
 eq("harvester spacing", h.spacing, 5.7);
 eq("harvester back end reach", h.noseReach, 2.9);   // √(4² + (3.6 + 2)²) − 4
 eq("harvester header reach", h.toolReach, 3.4);     // √((4 + 3)² + 2.5²) − 4
 eq("harvester distance", h.distance, 4.5);     // 3.43 + 1, rounded up
 eq("harvester skip", h.skip, 1);
 eq("harvester low", h.options[0].low, -2.5);   // the header 2.5 m ahead of the pivot on the next pass when the turn ends
 eq("harvester headland with skip", h.options[0].headland.width, 11.4);   // 4 + 4 = 8 m, 2 passes of 5.7
 eq("harvester extension", h.extension, 8); // a rigid implement: 2 × the radius only
 eq("harvester K-turn distance", h.options.find(o => o.key === "kturn").distance, 6.7);   // backing up, the header goes 5.7 m past the deepest point
}

// an offset sprayer, a long mounted implement, a TBT
{
 const o = S.suggest(Object.assign({mount: "rear", width: 8, overlap: 0, offset: 1, axleToTool: 1.8}, tractor));
 eq("offset tool reach", o.toolReach, 5.2);      // √((4 + 5)² + 1.8²) − 4
 eq("offset distance", o.distance, 6.2);
 const l = S.suggest(Object.assign({mount: "rear", width: 3, overlap: 0, axleToTool: 1.5, hitchLength: 1, length: 4}, tractor));
 near("long implement rear corner", l.toolReach, Math.sqrt(5.5 * 5.5 + 25) - 4, 0.06);   // 1 + 4 m behind the axle, not the 1.5 m working line
 if (!/rear corner/.test(l.rows[4].why)) fail.push("long implement: the reason should name the rear corner");
 const t = S.suggest(Object.assign({mount: "tbt", width: 3, overlap: 0, hitchLength: 1, tankHitch: 3, drawbar: 2, pivotToTool: 0.5}, tractor));
 eq("tbt axle to tool", t.axleToTool, 6.5);     // 1 + 3 + 2 + 0.5
 eq("tbt extension", t.extension, 19.5);        // 3 × 6.5
 eq("tbt tool radius", t.toolRadius, 2);        // hitch pin √(4² + 1²) = √17, tank √(17 − 9) = √8, tool √(8 − 4) = 2 m
 eq("tbt follows", t.problems.length, 0);
 const far = S.suggest(Object.assign({mount: "tbt", width: 3, overlap: 0, hitchLength: 1, tankHitch: 4, drawbar: 4, pivotToTool: 0}, tractor));
 eq("tbt that cannot follow", far.problems.length, 1);
 eq("no width", S.suggest(Object.assign({mount: "rear", width: 0}, tractor)).problems.length, 1);
}

// the margin to the field edge: 1 m by default, what the farmer wants on his irregular edges
{
 const base = Object.assign({mount: "rear", width: 8, overlap: 0, axleToTool: 1.8}, tractor);
 eq("margin 0.5", S.suggest(Object.assign({margin: 0.5}, base)).distance, 4.7);
 eq("margin 1.5", S.suggest(Object.assign({margin: 1.5}, base)).distance, 5.7);
 eq("margin 1.5 headland", S.suggest(Object.assign({margin: 1.5}, base)).options[0].headland.width, 16);   // 4 + 5.7 + 1.8 = 11.5 m, still 2 passes
 eq("negative margin", S.suggest(Object.assign({margin: -1}, base)).problems.length, 1);
}

// a front implement on a tractor is measured from the front axle; AgOpenWeb wants it from the rear axle, the pivot
{
 const f = {machine: "tractor", wheelbase: 2.7, maxSteerDeg: 35, overhang: 1.5, mount: "front", width: 8, overlap: 0, axleToTool: 2, margin: 1};
 const s = S.suggest(f);
 eq("front sprayer problems", s.problems.length, 0);
 eq("front sprayer from the pivot", s.axleToTool, 4.7);   // 2.7 m wheelbase + 2 m
 eq("front sprayer reach", s.toolReach, 5.2);           // √((4.7 + 4)² + 4.7²) − 4.7
 eq("front sprayer distance", s.distance, 6.2);
 eq("front sprayer hitch row", s.rows.find(r => r.key === "hitch").value, "4.7 m ahead of the rear axle");
 // the boom 4.7 m ahead of the pivot is already on the next pass when the turn ends, that far back into the field
 eq("front sprayer headland", s.rows.find(r => r.key === "headland").value, "16 m (2 passes) or 24 m (3 passes) or 24 m (3 passes)");
 eq("front sprayer low", s.options[0].low, -4.7);
 // the K-turn backs the boom up first: its own distance from boundary, and its tail takes the boom 9.3 m back into the field
 const k = s.options.find(o => o.key === "kturn");
 eq("front sprayer K-turn distance", k.distance, 6.2);
 eq("front sprayer K-turn low", k.low, -9.3);
 eq("front boom behind the nose", S.suggest(Object.assign({}, f, {axleToTool: 0})).problems.length, 1);
 const a = S.suggest({machine: "articulated", wheelbase: 3, maxSteerDeg: 40, overhang: 1, mount: "front", width: 8, overlap: 0, axleToTool: 1.5});
 eq("articulated front reach", a.toolReach, 4.1);       // the front half is tangent at the front axle: √(9² + 1.5²) − 5
 eq("articulated front hitch row", a.rows.find(r => r.key === "hitch").value, "4.5 m ahead of the rear axle");
 const h = S.suggest({machine: "harvester", wheelbase: 3.6, maxSteerDeg: 50, overhang: 2, mount: "rear", width: 6, overlap: 0, axleToTool: 1});
 eq("harvester rear implement from the pivot", h.axleToTool, 4.6);
 eq("no hitch row when measured from the pivot", S.suggest(Object.assign({mount: "rear", width: 8, overlap: 0, axleToTool: 1.8}, tractor)).rows.some(r => r.key === "hitch"), false);
}

// every distance printed with one decimal at most (51 × 0.1 is 5.1000000000000005 in floating point)
{
 const long = [];
 for (let m = 0; m <= 20; m++) for (const w of [3, 4.5, 6, 8, 12]) for (const deg of [30, 35, 40, 45]) {
  const s = S.suggest(Object.assign({}, tractor, {maxSteerDeg: deg, mount: "rear", width: w, axleToTool: 1.8, margin: m / 10}));
  for (const x of [s.distance, s.extension, s.radius]) if (!/^\d+(\.\d)?$/.test(String(x))) long.push(String(x));
 }
 eq("numbers printed short", long.join(" "), "");
}

// refused inputs
{
 eq("overlap too big", S.suggest(Object.assign({mount: "rear", width: 3, overlap: 3}, tractor)).problems.length, 1);
 eq("no wheelbase", S.suggest({wheelbase: 0, maxSteerDeg: 35, width: 3}).problems.length, 1);
}

if (fail.length) { console.log("FAILED:\n " + fail.join("\n ")); process.exit(1); }
console.log("settings.js: all checks passed");
