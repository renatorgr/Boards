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
 eq("sprayer options", s.options.map(o => o.key).join(","), "sagitta0");
 eq("sprayer no K-turn", s.kturn, null);   // a plain semicircle needs no backing up
 eq("sprayer nose reach", s.noseReach, 1.7);
 eq("sprayer tool reach", s.toolReach, 4.2);
 eq("sprayer distance", s.distance, 5.2);   // the boom's end 4.2 m past the pivot, plus the 1 m margin
 eq("sprayer headland", s.options[0].headland.width, 16);
 eq("sprayer headland passes", s.options[0].headland.passes, 2);
 eq("sprayer extension", s.extension, 8);
 eq("sprayer tram", s.tram.whole && s.tram.passes, 1);
 eq("sprayer rows", s.rows.map(r => r.key).join(","), "spacing,rMin,radius,style,distance,headland,extension,goAround,tram");
 eq("sprayer chosen", s.chosen, "sagitta0");
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
 eq("drill options, the smallest headland first", s.options.map(o => o.key).join(","), "sagitta2,keyhole1,sagitta1,keyhole0,sagitta0");   // no K-turn: it jackknifes
 if (!/No K-turn: .*jackknife/.test(s.rows.find(r => r.key === "style").why)) fail.push("drill: should say why there is no K-turn");
 const byKey = k => s.options.find(o => o.key === k);
 eq("drill headland with skip", byKey("sagitta2").headland.width, 12);   // 4 + 2.7 + 3.1 (straight on the pass 3.1 m back) = 9.8 m, 4 passes
 eq("drill low with skip", byKey("sagitta2").low, -3.1);
 eq("drill headland keyhole", byKey("keyhole0").headland.width, 18);   // 9.8 + 2.7 + 3 (the drill finishing its pass) = 15.5 m, 6 passes
 eq("drill headland keyhole skip 1", byKey("keyhole1").headland.width, 15);   // between the two: lands 6 m away, loops less deep
 eq("drill rows follow the smallest headland", s.rows.find(r => r.key === "style").value + " / " + s.rows.find(r => r.key === "headland").value, "Sagitta, skip 2 / 12 m (4 passes)");
 const picked = S.suggest(Object.assign({mount: "trailed", sections: 19, sectionWidth: 15.8, overlap: 0.002, hitchLength: 1, drawbar: 2, pivotToTool: 0, sprayerWidth: 8}, tractor), "keyhole0");
 eq("drill rows follow the pick", picked.chosen + " / " + picked.rows.find(r => r.key === "headland").value, "keyhole0 / 18 m (6 passes)");
 eq("an unknown pick falls back", S.suggest(Object.assign({mount: "trailed", sections: 19, sectionWidth: 15.8, overlap: 0.002, hitchLength: 1, drawbar: 2, pivotToTool: 0}, tractor), "nothing").chosen, "sagitta2");
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
 const sk = S.turnPath("sagitta", 4, 9, 0), skEnd = sk.poses[sk.turnEnd];
 near("skip turn lands on the pass", skEnd.e, 9, 0.02);
 near("skip turn depth", S.depthOf(sk.poses), 4, 0.02);
 const sg = S.turnPath("sagitta", 4, 3, 0), sgEnd = sg.poses[sg.turnEnd];   // closer than 2 × 4: the offset arc
 near("sagitta arc lands on the pass", sgEnd.e, 3, 0.02);
 near("sagitta arc heading", sgEnd.h, Math.PI, 0.01);
 near("sagitta arc depth", S.depthOf(sg.poses), S.depthOf(S.sagittaPath(4, 3)), 0.05);
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
 eq("harvester low", h.options.find(o => o.key === "sagitta1").low, -2.5);   // the header 2.5 m ahead of the pivot on the next pass when the turn ends
 eq("harvester headland with skip", h.options.find(o => o.key === "sagitta1").headland.width, 11.4);   // 4 + 4 = 8 m, 2 passes of 5.7
 eq("harvester extension", h.extension, 8); // a rigid implement: 2 × the radius only
 eq("harvester K-turn distance", h.options.find(o => o.key === "kturn0").distance, 6.7);   // backing up, the header goes 5.7 m past the deepest point
}

// an offset sprayer, a long mounted implement, a TBT
{
 const o = S.suggest(Object.assign({mount: "rear", width: 8, overlap: 0, offset: 1, axleToTool: 1.8}, tractor));
 eq("offset tool reach", o.toolReach, 5.2);      // √((4 + 5)² + 1.8²) − 4
 eq("offset distance", o.distance, 6.2);
 const l = S.suggest(Object.assign({mount: "rear", width: 3, overlap: 0, axleToTool: 1.5, hitchLength: 1, length: 4}, tractor));
 near("long implement rear corner", l.toolReach, Math.sqrt(5.5 * 5.5 + 25) - 4, 0.06);   // 1 + 4 m behind the axle, not the 1.5 m working line
 if (!/rear corner/.test(l.rows.find(r => r.key === "distance").why)) fail.push("long implement: the reason should name the rear corner");
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
 eq("front sprayer options", s.options.map(o => o.key).join(","), "sagitta1,sagitta0,kturn0,keyhole0");   // the smallest headland first, the shallowest on a tie
 eq("front sprayer headland", s.rows.find(r => r.key === "headland").value, "16 m (2 passes)");
 eq("front sprayer low", s.options[0].low, -4.7);
 eq("front sprayer keyhole headland", s.options.find(o => o.key === "keyhole0").headland.width, 24);
 // the K-turn backs the boom up first: its own distance from boundary, and its tail takes the boom 9.3 m back into the field
 const k = s.options.find(o => o.key === "kturn0");
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

// Renato's machines, measured 8 Nov 2026: the Case (2.7 m, 42° from the steer wizard, nose 1.5 m), the 8 m boom on the front
// linkage 2 m ahead of the front axle, and the semi-mounted no-till drill (pivot 1.54 m behind the rear axle, wheels 4.15 m
// behind the pivot, the middle of its two rows of discs 1.6 m ahead of the wheels, a 5 cm gap between passes)
{
 const caseTractor = {machine: "tractor", wheelbase: 2.7, maxSteerDeg: 42, overhang: 1.5, margin: 1};
 const sp = S.suggest(Object.assign({mount: "front", width: 8, overlap: 0, axleToTool: 2, sprayerWidth: 8}, caseTractor));
 eq("Renato's sprayer problems", sp.problems.length, 0);
 eq("Renato's sprayer radius", sp.radius, 4);           // 1.2 × 2.7 ÷ tan 42° = 3.6, the 4 m floor
 eq("Renato's sprayer turn", sp.rows.find(r => r.key === "style").value, "Sagitta, skip 0");   // 8 m = 2 × 4 m: a plain semicircle
 eq("Renato's sprayer distance", sp.distance, 6.3);
 eq("Renato's sprayer headland", sp.rows.find(r => r.key === "headland").value, "16 m (2 passes)");
 const d = S.suggest(Object.assign({mount: "trailed", sections: 19, sectionWidth: 15.8, width: 3, overlap: -0.05, hitchLength: 1.54, drawbar: 4.15, pivotToTool: -1.6, sprayerWidth: 8}, caseTractor));
 eq("Renato's drill problems", d.problems.length, 0);
 eq("Renato's drill spacing", d.spacing, 3.05);         // the 5 cm gap shows
 eq("Renato's drill radius", d.radius, 4.3);           // 4.15 m ÷ sin 75°
 eq("Renato's drill axle to the discs", d.axleToTool, 4.1);   // 1.54 + 4.15 − 1.6
 eq("Renato's drill options", d.options.map(o => o.key).join(","), "keyhole1,keyhole0,sagitta2,sagitta1,sagitta0");
 if (!/lifted rigid on the 3-point linkage .* Rear fixed/.test(d.rows.find(r => r.key === "style").why)) fail.push("Renato's drill: should say a lifted 3-point implement is Rear fixed");
 eq("Renato's drill distance", d.distance, 2.8);       // the tractor's nose
 eq("Renato's drill turn", d.rows.find(r => r.key === "style").value, "Keyhole, skip 1");   // the smallest headland: lands 6.1 m away, loops 8.8 m deep
 eq("Renato's drill headland", d.rows.find(r => r.key === "headland").value, "18.3 m (6 passes)");   // the 18 m he uses
 eq("Renato's drill Sagitta skip 2 headland", d.options.find(o => o.key === "sagitta2").headland.width, 21.35);   // the drill takes 11 m to get straight on the pass
 eq("Renato's drill extension", d.extension, 12.5);
}

// the three examples with typical measures, on Renato's Case (its rear tyres guessed: 650/65 R38 on a 1.81 m track)
{
 const caseTractor = {machine: "tractor", wheelbase: 2.7, maxSteerDeg: 42, overhang: 1.5, margin: 1, trackWidth: 1.81, tyreWidth: 0.65, wheelDiameter: 1.8, drawbarWidth: 0.3};
 const row = (r, key) => r.rows.find(x => x.key === key);
 // a tandem-axle manure spreader on the drawbar, its pin 0.7 m behind the rear axle (inside the tyres' 0.9 m back edge):
 // 4.8 m from the eye to the middle of its axles, the beaters 3 m behind, 10 m spread. The drawbar touches a tyre at 57°
 const m = S.suggest(Object.assign({mount: "trailed", width: 10, overlap: 0, hitchLength: 0.7, drawbar: 4.8, pivotToTool: 3, length: 3.3}, caseTractor));
 eq("spreader problems", m.problems.length, 0);
 eq("spreader touches the tyre", m.touchDeg, 57);
 eq("spreader radius", m.radius, 6.7);                // the drawbar kept under 52° on the turn's circle, not 4.8 ÷ sin 75° = 5 m
 if (!/touches a rear tyre when it folds 57°: kept under 52°/.test(row(m, "radius").why)) fail.push("spreader: the radius should give the tyre as its reason");
 eq("spreader turn", row(m, "style").value, "Sagitta, skip 1");
 eq("spreader options", m.options.map(o => o.key).join(","), "sagitta1,keyhole0,sagitta0");   // no K-turn: the drawbar would fold onto the tyre
 eq("spreader headland", row(m, "headland").value, "30 m (3 passes)");
 eq("spreader tyres row", row(m, "fold").value, "touches the tyres at 57°");
 if (!(Math.max(...m.options.map(o => o.sweep.firstFoldDeg)) <= 52)) fail.push("spreader: a turn folds the drawbar onto the tyre");
 // the same spreader on a hitch 1 m back: its tail swings out as the tractor straightens, past the formula's reach
 const m1 = S.suggest(Object.assign({mount: "trailed", width: 10, overlap: 0, hitchLength: 1, drawbar: 4.8, pivotToTool: 3, length: 3.3}, caseTractor, {trackWidth: 0}));
 eq("spreader on a 1 m hitch radius", m1.radius, 5);   // 4.8 m drawbar ÷ sin 75°
 eq("spreader on a 1 m hitch distance", row(m1, "distance").value, "4.6 m");
 if (!/swings out as the tractor straightens, 3\.6 m/.test(row(m1, "distance").why)) fail.push("spreader: the distance should say its tail swings out (not 'backing up')");
 eq("spreader on a 1 m hitch extension", m1.extension, 26.5);   // 3 × 8.8 m from the rear axle to the beaters
 // a trailed disc harrow on the drawbar, its wheels between the gangs, 20 cm of overlap: backing it up would jackknife
 const h = S.suggest(Object.assign({mount: "trailed", width: 4, overlap: 0.2, hitchLength: 0.7, drawbar: 3.5, pivotToTool: 0, length: 1.5}, caseTractor));
 eq("harrow spacing", h.spacing, 3.8);
 eq("harrow radius", h.radius, 5);                    // the tyre again, not 4 m
 eq("harrow options", h.options.map(o => o.key).join(","), "sagitta2,keyhole1,sagitta1,keyhole0,sagitta0");
 eq("harrow headland", row(h, "headland").value, "19 m (5 passes)");
 eq("harrow keyhole skip 0 headland", h.options.find(o => o.key === "keyhole0").headland.width, 22.8);   // the nose's outer corner needs 3.6 m from the edge
 // the drawbar sits 0.5 m up, where the 1.8 m tyre reaches 0.81 m behind the axle, not 0.9: it touches later (65°), so a tighter turn
 const hl = S.suggest(Object.assign({mount: "trailed", width: 4, overlap: 0.2, hitchLength: 0.7, drawbar: 3.5, pivotToTool: 0, length: 1.5, drawbarHeight: 0.5}, caseTractor));
 eq("harrow at the drawbar's height touches", hl.touchDeg, 65);
 eq("harrow at the drawbar's height radius", hl.radius, 4.5);
 eq("harrow at the drawbar's height headland", row(hl, "headland").value, "19 m (5 passes)");
 const mh = S.suggest(Object.assign({mount: "trailed", width: 10, overlap: 0, hitchLength: 0.7, drawbar: 4.8, pivotToTool: 3, length: 3.3, drawbarHeight: 0.5}, caseTractor));
 eq("spreader at the drawbar's height radius", mh.radius, 6);
 eq("spreader at the drawbar's height row", row(mh, "fold").value, "touches the tyres at 65°");
 if (!/0\.81 m at the drawbar's 0\.5 m height/.test(row(mh, "fold").why)) fail.push("spreader: the tyres row should give the tyre's reach at the drawbar's height");
 // Renato's Farmall 115C: 540/65 R34 tyres (1.57 m) reach only 0.73 m back at the drawbar's 0.5 m height and its pin is 0.74 m back,
 // so the drawbar touches late (76°) and the spreader turns tighter than on the Maxxum; its shorter, narrower nose needs less room
 const farmall = {machine: "tractor", wheelbase: 2.3, maxSteerDeg: 45, overhang: 0.85, trackWidth: 1.7, tyreWidth: 0.54, wheelDiameter: 1.57, drawbarHeight: 0.5, margin: 1};
 const fs = S.suggest(Object.assign({mount: "trailed", width: 10, overlap: 0, hitchLength: 0.74, drawbar: 4.8, pivotToTool: 3, length: 3.3, drawbarWidth: 0.3}, farmall));
 eq("Farmall spreader problems", fs.problems.length, 0);
 eq("Farmall spreader touches the tyre", fs.touchDeg, 76);
 eq("Farmall spreader radius", fs.radius, 5.4);
 eq("Farmall spreader headland", row(fs, "headland").value, "30 m (3 passes)");
 eq("Farmall tightest turn", fs.rMin, 2.3);                 // 2.3 m ÷ tan 45°: AgOpenWeb's 4 m floor is wider anyway
 // Renato's own manure spreader: one axle 4.2 m behind the pin, the body and its beaters 1.5 m past it. Short behind its axle, it is
 // straight on the next pass soon, so on the Maxxum the Keyhole needs the smallest headland; on the Farmall (R 4.8) the 10 m passes
 // fit a plain semicircle, but its headland sum is 20.5 m: three passes
 const own = {mount: "trailed", width: 10, overlap: 0, drawbar: 4.2, pivotToTool: 1.5, length: 1.5, drawbarWidth: 0.3};
 const om = S.suggest(Object.assign({}, own, {hitchLength: 0.7}, caseTractor, {drawbarHeight: 0.5, overhang: 1.0}));
 eq("own spreader on the Maxxum radius", om.radius, 5.4);
 eq("own spreader on the Maxxum turn", row(om, "style").value, "Keyhole, skip 0");
 eq("own spreader on the Maxxum headland", row(om, "headland").value, "20 m (2 passes)");
 // Renato's Puma 180 CVX: tall 650/65 R42 tyres (1.91 m) but a wide 2 m track and the pin 0.9 m back: the drawbar touches at 82.5°;
 // its manual's turning circle (8.87 m at the rear axle's centre, 2.884 m wheelbase) gives 33° of steering, so the tractor sets the radius
 const puma = {machine: "tractor", wheelbase: 2.88, maxSteerDeg: 33, overhang: 1.5, trackWidth: 2.0, tyreWidth: 0.65, wheelDiameter: 1.91, drawbarHeight: 0.5, margin: 1};
 const ps = S.suggest(Object.assign({mount: "trailed", width: 10, overlap: 0, hitchLength: 0.9, drawbar: 4.8, pivotToTool: 3, length: 3.3, drawbarWidth: 0.3}, puma));
 eq("Puma spreader touches the tyre", ps.touchDeg, 82.5);
 eq("Puma spreader radius", ps.radius, 5.4);   // 1.2 × 2.88 ÷ tan 33° = 5.3: wider than the tyres need
 eq("Puma spreader headland", row(ps, "headland").value, "30 m (3 passes)");
 const pf = S.suggest(Object.assign({mount: "front", width: 8, overlap: 0, axleToTool: 2, hitchLength: 1, length: 0, sprayerWidth: 8}, puma));
 eq("Puma tightest turn", pf.rMin, 4.4);                     // 2.88 ÷ tan 33° = 4.43: half the manual's 8.87 m circle
 eq("Puma front sprayer radius", pf.radius, 5.4);
 eq("Puma front sprayer turn", row(pf, "style").value, "Sagitta, skip 1");   // 8 m passes no longer fit a plain semicircle (twice 5.4 m)
 eq("Puma front sprayer headland", row(pf, "headland").value, "24 m (3 passes)");
 const fd = S.suggest(Object.assign({mount: "trailed", width: 3, overlap: -0.05, hitchLength: 1.54, drawbar: 4.15, pivotToTool: -1.6, drawbarWidth: 0.3}, farmall));
 eq("Farmall drill distance", row(fd, "distance").value, "3 m");     // the nose's corner: (1.7 + 0.54) ÷ 2 = 1.12 m off the centre line, 0.85 m ahead
 eq("Farmall drill turn", row(fd, "style").value, "Keyhole, skip 1");
 const of = S.suggest(Object.assign({}, own, {hitchLength: 0.74}, farmall));
 eq("own spreader on the Farmall radius", of.radius, 4.8);
 eq("own spreader on the Farmall turn", row(of, "style").value, "Sagitta, skip 0");
 eq("own spreader on the Farmall headland", row(of, "headland").value, "30 m (3 passes)");
 // the tractor's width: its nose's outer corner, (1.81 + 0.65) ÷ 2 = 1.23 m off the centre line, sweeps 2.6 m past the pivot's path, not 1.7
 eq("tractor half width", m.bodyHalf, 1.23);
 eq("nose corner reach", m.noseReach, 2.3);   // √((6.7 + 1.23)² + 4.2²) − 6.7
 if (!/outer corner 1\.23 m off the centre line/.test(row(h, "distance").why)) fail.push("harrow: the distance should name the nose's outer corner");
 eq("nose on the centre line without a track", S.suggest(Object.assign({mount: "trailed", width: 10, hitchLength: 0.7, drawbar: 4.8, pivotToTool: 3}, caseTractor, {trackWidth: 0})).noseReach, 1.5);   // no tyre check either: radius 5
 // a 4-furrow reversible plough on the 3-point linkage, 4.5 m long: a K-turn backs its rear corner towards the edge
 const plough = {mount: "rear", width: 1.6, overlap: 0, axleToTool: 3, hitchLength: 1, length: 4.5};
 const p = S.suggest(Object.assign({}, plough, caseTractor));
 eq("plough options", p.options.map(o => o.key).join(","), "sagitta4,keyhole3,keyhole2,sagitta3,keyhole1,kturn0,keyhole0,sagitta2,sagitta1,sagitta0");
 eq("plough distance", row(p, "distance").value, "4.3 m");
 const pK = S.suggest(Object.assign({}, plough, caseTractor), "kturn0");
 eq("plough K-turn picked", pK.rows.find(r => r.key === "style").value + " / " + row(pK, "distance").value, "K-turn, skip 0 / 6.6 m");
 const pk = p.options.find(o => o.key === "kturn0"), short = S.suggest(Object.assign({}, plough, caseTractor, {length: 0})).options.find(o => o.key === "kturn0");
 if (!pk.corner || short.corner) fail.push("plough: the rear corner is followed only when the implement is longer than its working line");
 if (!(pk.distance > short.distance)) fail.push("plough: backing up the 4.5 m plough should need more room than its working line alone");
}

// the drawbar and the rear tyres (Renato, 8 Nov 2026: a drawbar pin is usually inside the tyres' back edge, so the drawbar
// can never fold to 90°; a semi-mounted implement's pivot is further back)
{
 eq("pin inside the tyres", S.drawbarTouchDeg(0.7, 0.3, 4.8, 1.81, 0.65, 1.8), 57);
 eq("pin inside the tyres, drawbar 0.5 m up", S.drawbarTouchDeg(0.7, 0.3, 4.8, 1.81, 0.65, 1.8, 0.5), 65);
 near("tyre reach at 0.5 m", S.tyreReach(1.8, 0.5), 0.806, 0.001);   // √(0.9² − 0.4²)
 eq("tyre reach not known", S.tyreReach(1.8, 0), 0.9);
 eq("tyre reach at the axle", S.tyreReach(1.8, 1.2), 0.9);
 if (!(S.drawbarTouchDeg(0.5, 0.3, 4.8, 1.81, 0.65, 1.8) < 57)) fail.push("tyres: a pin further in should touch sooner");
 if (!(S.drawbarTouchDeg(0.7, 0.6, 4.8, 1.81, 0.65, 1.8) < 57)) fail.push("tyres: a wider drawbar should touch sooner");
 if (!(S.drawbarTouchDeg(0.7, 0.3, 4.8, 2.1, 0.65, 1.8) > 57)) fail.push("tyres: a wider track should touch later");
 eq("semi-mounted pivot behind the tyres", S.drawbarTouchDeg(1.54, 0.3, 4.15, 1.81, 0.65, 1.8), 111.5);
 near("steady fold", S.steadyFoldDeg(4.3, 1.54, 4.15), 85, 0.1);   // atan(1.54 ÷ 4.3) + asin(4.15 ÷ √(4.3² + 1.54²))
 // Renato's drill: its pivot behind the tyres, so nothing changes
 const d = S.suggest({machine: "tractor", wheelbase: 2.7, maxSteerDeg: 42, overhang: 1.5, margin: 1, trackWidth: 1.81, tyreWidth: 0.65, wheelDiameter: 1.8, drawbarWidth: 0.3,
  mount: "trailed", width: 3, overlap: -0.05, hitchLength: 1.54, drawbar: 4.15, pivotToTool: -1.6});
 eq("drill with its tyres radius", d.radius, 4.3);
 eq("drill with its tyres row", d.rows.find(r => r.key === "fold").value, "touches the tyres at 111.5°");
 eq("drill with the tractor's width nose", d.noseReach, 2.6);     // the Case's front corner, 1.23 m off the centre line
 eq("drill with the tractor's width distance", d.distance, 3.7);
 eq("drill with the tractor's width headland", d.rows.find(r => r.key === "headland").value, "18.3 m (6 passes)");   // Keyhole, skip 1
 eq("drill with the tractor's width, the Keyhole onto the next pass", d.options.find(o => o.key === "keyhole0").headland.width, 21.35);
 if (d.rows.find(r => r.key === "fold").why.indexOf("behind the tyres' back edge") < 0) fail.push("drill: its pivot is behind the tyres' back edge");
 eq("no tyre check for a front implement", S.suggest({mount: "front", width: 8, axleToTool: 2, trackWidth: 1.81}).rows.some(r => r.key === "fold"), false);
 // Renato's roller, semi-mounted: its pivot turns 60° to each side (120° in all) and that stop, not the tyres, sets the radius
 const r = S.suggest({machine: "tractor", wheelbase: 2.7, maxSteerDeg: 42, overhang: 1.5, margin: 1, trackWidth: 1.81, tyreWidth: 0.65, wheelDiameter: 1.8, drawbarWidth: 0.3,
  mount: "trailed", width: 3.2, hitchLength: 1.5, drawbar: 2.5, pivotToTool: 0, swingDeg: 60});
 eq("roller radius", r.radius, 4.2);
 if (!/reaches the most it may turn when it folds 60°: kept under 55°/.test(r.rows.find(x => x.key === "radius").why)) fail.push("roller: the radius should give the stop as its reason");
 eq("roller fold row", r.rows.find(x => x.key === "fold").value, "touches the tyres at 110°, at most 60°");
 if (!(Math.max(...r.options.map(o => o.sweep.firstFoldDeg)) <= 55)) fail.push("roller: a turn folds it past its stop");
 eq("a stop out of range", S.suggest({mount: "trailed", width: 3, swingDeg: 200}).problems.length, 1);
 eq("clearance out of range", S.suggest({mount: "trailed", width: 3, foldClearanceDeg: 60}).problems.length, 1);
 const c10 = S.suggest({machine: "tractor", wheelbase: 2.7, maxSteerDeg: 42, overhang: 1.5, margin: 1, trackWidth: 1.81, tyreWidth: 0.65, wheelDiameter: 1.8, drawbarWidth: 0.3,
  mount: "trailed", width: 3.2, hitchLength: 1.5, drawbar: 2.5, pivotToTool: 0, swingDeg: 60, foldClearanceDeg: 10});
 eq("roller with 10° of room", c10.radius, 4.6);   // kept under 50° instead of 55°: 4.2 m was enough for 55°
 if (!/kept under 50°/.test(c10.rows.find(x => x.key === "radius").why)) fail.push("roller: the radius should give the 10° of room");
 eq("track narrower than a tyre", S.suggest({mount: "trailed", width: 3, trackWidth: 0.6, tyreWidth: 0.65}).problems.length, 1);
 eq("track narrower than a tyre, rigid", S.suggest({mount: "rear", width: 3, trackWidth: 0.6, tyreWidth: 0.65}).problems.length, 1);
 eq("drawbar below the ground", S.suggest({mount: "trailed", width: 3, trackWidth: 1.81, tyreWidth: 0.65, drawbarHeight: -1}).problems.length, 1);
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
eq("version is a number pair, as the page shows it", /^\d+\.\d+$/.test(S.VERSION), true);

// index.html asks for settings.js with the version in the address: a browser that kept the old file (GitHub Pages lets it for
// 10 minutes) fetches the new one with the new page, instead of showing "version undefined" and stopping half way.
const page = require("fs").readFileSync(require("path").join(__dirname, "index.html"), "utf8");
eq("index.html loads settings.js?v= with the current version", (page.match(/<script src="settings\.js\?v=([^"]+)"><\/script>/) || [])[1], S.VERSION);

console.log("settings.js: all checks passed");
