// settings.js - the AgOpenWeb settings worked out from the tractor and implement measures, and every turn it can make.
// Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards (Firmware/simulator/aow-settings, see README.md there).
// Written 2026 by Claude (Anthropic) for renatorgr.
// The history of changes is in the README changelog and in the git log.
// Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure arithmetic, no page: the same file runs in the browser (window.AowSettings) and in Node (test.js), and is the
// model for a C# version on AgOpenWeb's Vehicle & Tool page. Everything in metres and degrees. The rules follow
// AgOpenWeb's own U-turn code: the turn is drawn by the tractor's pivot (the middle of the rear axle) and has to stay
// inside the field edge pulled in by "Distance from boundary"; a pass closer than twice the radius needs a loop
// (Sagitta or Keyhole), a pass at twice the radius or more gets a plain semicircle.
"use strict";
(function (root) {
 const VERSION = "1.3";       // shown under the page's title and in index.html's "settings.js?v=": raise both with every change
 const RADIUS_FACTOR = 1.2;    // the turn radius over the tightest the tractor can do: the steering stays off its stop
 const RADIUS_FLOOR = 4.0;     // AgOpenWeb's simple turn path never goes below 4 m
 const MARGIN = 1.0;           // room kept between what sweeps the widest and the field edge, unless `margin` says otherwise
 const MAX_HITCH_DEG = 75;     // drawbar ÷ sin 75°: the radius where a trailed implement's wheels still roll round a circle of
                               // their own; backing up, a drawbar folded past it is a jackknife
 const FOLD_CLEARANCE_DEG = 5; // and it stays this far from where it touches a rear tyre or reaches its own stop
 const GO_AROUND_MARGIN = 1.0; // AgOpenWeb's safety margin round a Go around obstacle
 const K_TURN_ARC = 2.2;      // radians: the forward arc AgOpenWeb draws for a K-turn before the tractor backs up
 const rad = d => d * Math.PI / 180;
 const deg = r => r * 180 / Math.PI;
 const r1 = v => Math.round(v * 10) / 10;
 const r2 = v => Math.round(v * 100) / 100;   // the track spacing: a 5 cm gap must show
 const up = (v, step) => Math.round(Math.ceil(v / step - 1e-9) * step * 1000) / 1000;   // rounded again: 51 × 0.1 is 5.1000000000000005

 // How far past its start a Keyhole turn onto a pass `lateral` metres away reaches (AgOpenWeb
 // SagittaTurnGeometry.KeyholeDepth): the main arc's centre plus its radius.
 function keyholeDepth(radius, lateral) {
  const across = Math.min(Math.max(lateral, 0), 2 * radius) / 2 + radius;
  return radius + Math.sqrt(Math.max(0, 4 * radius * radius - across * across));
 }

 // Points along a constant-radius arc, as AgOpenWeb's SagittaTurnGeometry.AddArc steps them: `pos` is {e, n}
 // (east, north), `theta` the heading clockwise from north. Returns the new heading; the points go into `out`.
 function addArc(pos, theta, out, length, right, radius) {
  const segments = Math.max(1, Math.ceil(length / (radius * 0.1)));
  const dist = length / segments, turn = (dist / radius) * (right ? 1 : -1), r = right ? radius : -radius;
  for (let i = 0; i < segments; i++) {
   pos.e += Math.cos(theta) * r; pos.n -= Math.sin(theta) * r;
   theta = (theta + turn) % (2 * Math.PI);
   pos.e -= Math.cos(theta) * r; pos.n += Math.sin(theta) * r;
   out.push({e: pos.e, n: pos.n});
  }
  return theta;
 }

 // The Keyhole turn from (0, 0) heading north onto the pass `lateral` metres to the right: a counter-arc, the
 // main arc, a counter-arc (AgOpenWeb SagittaTurnGeometry.BuildKeyhole).
 function keyholePath(radius, lateral) {
  const l = Math.min(Math.max(lateral, 0), 2 * radius), across = l / 2 + radius;
  const ahead = Math.sqrt(Math.max(0, 4 * radius * radius - across * across)), counter = Math.atan2(ahead, across);
  const out = [{e: 0, n: 0}], pos = {e: 0, n: 0};
  let theta = 0;
  if (counter > 0) theta = addArc(pos, theta, out, counter * radius, false, radius);
  theta = addArc(pos, theta, out, (Math.PI + 2 * counter) * radius, true, radius);
  if (counter > 0) addArc(pos, theta, out, counter * radius, false, radius);
  return out;
 }

 // The Sagitta turn from (0, 0) heading north onto the pass `lateral` metres to the right (AgOpenWeb
 // SagittaTurnGeometry.BuildOffsetArc with a semicircle's sweep): a counter-arc lead-in, then one offset arc.
 function sagittaPath(radius, lateral) {
  const offset = Math.max(0, 2 * radius - lateral);
  const first = Math.acos(Math.min(1, Math.max(-1, 1 - (offset * 0.5) / radius)));
  const out = [{e: 0, n: 0}], pos = {e: 0, n: 0};
  let theta = 0;
  if (offset > 0) theta = addArc(pos, theta, out, first * radius, false, radius);
  addArc(pos, theta, out, (Math.PI + first) * radius, true, radius);
  return out;
 }
 const depthOf = path => path.reduce((m, p) => Math.max(m, p.n), 0);

 // The K-turn from (0, 0) heading north onto the pass `lateral` metres to the right. AgOpenWeb draws only its first part
 // (YouTurnCreationService.CreateKStyleTurnCurve, AgOpenGPS's KStyleTurn): an arc of 2.2 rad (126°) and a straight tail
 // of 1.5 × the distance to the next pass. As soon as the tractor reverses the turn is over for AgOpenWeb and its guidance
 // is on the next pass, backing up too (TrackGuidanceService steers in reverse). The shortest way back onto that pass at
 // the same radius: straight back along the tail by `back` metres, then a reverse arc through the remaining 54°, which
 // ends on the pass heading south at `end`.
 function kTurnBack(radius, lateral) {
  const tip = {e: radius * (1 - Math.cos(K_TURN_ARC)) + 1.5 * lateral * Math.sin(K_TURN_ARC), n: radius * Math.sin(K_TURN_ARC) + 1.5 * lateral * Math.cos(K_TURN_ARC)};
  const back = (tip.e - lateral - radius * (1 + Math.cos(K_TURN_ARC))) / Math.sin(K_TURN_ARC);
  return {tip, back, end: {e: lateral, n: tip.n - back * Math.cos(K_TURN_ARC) + radius * Math.sin(K_TURN_ARC)}};
 }
 // the pivot's deepest point: the top of the forward arc, or where the backing up ends
 const kTurnDepth = (radius, lateral) => Math.max(radius, kTurnBack(radius, lateral).end.n);

 // ---- the drawbar and the rear tyres ----

 // How far behind (and ahead of) the axle a wheel `dia` high reaches at `height` above the ground: the half chord of its
 // circle there. The drawbar sits low, where the tyre is shorter than at the axle. 0 (not known) or a height at the axle
 // or above: the whole wheel, dia ÷ 2.
 function tyreReach(dia, height) {
  const r = dia / 2;
  return height > 0 && height < r ? Math.sqrt(r * r - (r - height) * (r - height)) : r;
 }
 // Does the drawbar, folded `angle` degrees against the tractor, overlap a rear tyre? Seen from above, in the tractor's
 // frame (x to the side it swings to, y back from the rear axle): the drawbar a strip `bar` wide from the hitch pin `hitch`
 // metres behind the axle, `len` long; the tyre a rectangle `tyre` wide round its centre `track` ÷ 2 out, reaching
 // tyreReach(dia, height) behind and ahead of the axle. Separating axes of two rectangles.
 function drawbarTouches(angle, hitch, bar, len, track, tyre, dia, height) {
  const a = rad(angle), u = [Math.sin(a), Math.cos(a)], n = [Math.cos(a), -Math.sin(a)], strip = [];
  for (const s of [0, len]) for (const w of [-bar / 2, bar / 2]) strip.push([s * u[0] + w * n[0], hitch + s * u[1] + w * n[1]]);
  const x0 = track / 2 - tyre / 2, x1 = track / 2 + tyre / 2, back = tyreReach(dia, height), box = [[x0, -back], [x1, -back], [x1, back], [x0, back]];
  for (const ax of [[1, 0], [0, 1], u, n]) {
   const A = strip.map(p => p[0] * ax[0] + p[1] * ax[1]), B = box.map(p => p[0] * ax[0] + p[1] * ax[1]);
   if (Math.max(...A) < Math.min(...B) || Math.max(...B) < Math.min(...A)) return false;
  }
  return true;
 }
 // The angle (every 0.5°) where the drawbar first touches a rear tyre; null when it never does before 180°. A drawbar pin
 // inside the tyres' back edge touches them long before 90°.
 function drawbarTouchDeg(hitch, bar, len, track, tyre, dia, height) {
  for (let d = 0; d <= 180; d += 0.5) if (drawbarTouches(d, hitch, bar, len, track, tyre, dia, height)) return d;
  return null;
 }
 // How far a drawbar `len` long from a hitch pin `hitch` behind the pivot folds against the tractor when everything runs
 // round a circle of `radius` at the pivot: the pin is at √(radius² + hitch²) from the centre and the implement's axle,
 // with no side slip, where the drawbar is tangent to its own circle. 180 when it cannot follow.
 function steadyFoldDeg(radius, hitch, len) {
  const rh = Math.hypot(radius, hitch);
  return len >= rh ? 180 : deg(Math.atan2(hitch, radius) + Math.asin(len / rh));
 }

 // ---- the whole turn step by step: the pivot's position and heading every 0.1 m, then where the implement goes ----

 const STEP = 0.1;
 const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
 // `pieces`: {len, dir: 1 forward or -1 backing up, turn: 0 straight, 1 right (the heading grows) or -1 left, r}. Returns
 // the poses {e, n, h (heading clockwise from north), dir} and the index where each piece ends.
 function walk(pieces) {
  const poses = [{e: 0, n: 0, h: 0, dir: 1}], ends = [];
  let e = 0, n = 0, h = 0;
  for (const p of pieces) {
   const steps = Math.max(1, Math.ceil(p.len / STEP - 1e-9)), ds = p.len / steps, dir = p.dir || 1;
   for (let k = 0; k < steps; k++) {
    const dh = p.turn ? p.turn * ds / p.r : 0, hm = h + dh / 2;
    e += dir * ds * Math.sin(hm); n += dir * ds * Math.cos(hm); h += dh;
    poses.push({e, n, h, dir});
   }
   ends.push(poses.length - 1);
  }
  return {poses, ends};
 }

 // The pivot's path for a turn kind ("sagitta": a semicircle with a straight middle when the pass is 2 × radius or more
 // away, else the offset arc with its counter-arc lead-in; "keyhole"; "kturn") from (0, 0) heading north onto the pass
 // `lateral` metres to the right, then `exit` metres down that pass. `turnEnd`: where the turn ends; a K-turn's `arcEnd`
 // and `forwardEnd` (the tail between them), the backing up after it.
 function turnPath(key, radius, lateral, exit) {
  const R = radius, pieces = [];
  if (key === "keyhole") {
   const across = lateral / 2 + R, c = Math.atan2(Math.sqrt(Math.max(0, 4 * R * R - across * across)), across);
   pieces.push({len: c * R, turn: -1, r: R}, {len: (Math.PI + 2 * c) * R, turn: 1, r: R}, {len: c * R, turn: -1, r: R});
  } else if (key === "kturn") {
   pieces.push({len: K_TURN_ARC * R, turn: 1, r: R}, {len: 1.5 * lateral, turn: 0},
    {len: kTurnBack(R, lateral).back, dir: -1, turn: 0}, {len: (Math.PI - K_TURN_ARC) * R, dir: -1, turn: 1, r: R});
  } else if (lateral < 2 * R - 1e-9) {
   const offset = 2 * R - lateral, first = Math.acos(Math.min(1, Math.max(-1, 1 - (offset * 0.5) / R)));
   pieces.push({len: first * R, turn: -1, r: R}, {len: (Math.PI + first) * R, turn: 1, r: R});
  } else pieces.push({len: Math.PI / 2 * R, turn: 1, r: R}, {len: Math.max(0, lateral - 2 * R), turn: 0}, {len: Math.PI / 2 * R, turn: 1, r: R});
  pieces.push({len: exit, turn: 0});
  const w = walk(pieces), turnEnd = w.ends[w.ends.length - 2];
  return {poses: w.poses, turnEnd, arcEnd: key === "kturn" ? w.ends[0] : turnEnd, forwardEnd: key === "kturn" ? w.ends[1] : turnEnd};
 }

 // Where the implement's two ends go along `poses`, and the machine's far end (`body` metres ahead of the pivot, behind
 // when negative; null: not followed; `bodyHalf`: half its width, so its two corners `bodyLeft` and `bodyRight` are
 // followed too). `imp`: rigid {along: + ahead of the pivot, − behind} or trailed {hitch: behind the pivot, bars: [tank
 // hitch,] drawbar, pivotToTool}; offset (right +), half (the width ÷ 2). A trailed implement's axle follows its hitch
 // with no side slip: its heading turns by the hitch's sideways move ÷ the drawbar, forwards and backing up. `foldDeg`:
 // the most a drawbar folds against what pulls it; `firstFoldDeg`: the first one against the machine.
 function sweep(poses, imp) {
  const u = h => ({e: Math.sin(h), n: Math.cos(h)}), rt = h => ({e: Math.cos(h), n: -Math.sin(h)});
  const left = [], right = [], centre = [], heading = [], body = [], bodyLeft = [], bodyRight = [];
  const bars = imp.bars || [], phis = bars.map(() => poses[0].h), prev = bars.map(() => null), bh = imp.bodyHalf || 0;
  let fold = 0, firstFold = 0;
  for (const p of poses) {
   const f = u(p.h);
   if (imp.body != null) {
    const b = {e: p.e + imp.body * f.e, n: p.n + imp.body * f.n}, s = rt(p.h);
    body.push(b); bodyLeft.push({e: b.e - bh * s.e, n: b.n - bh * s.n}); bodyRight.push({e: b.e + bh * s.e, n: b.n + bh * s.n});
   }
   let c, th;
   if (!bars.length) { c = {e: p.e + imp.along * f.e, n: p.n + imp.along * f.n}; th = p.h; }
   else {
    let pt = {e: p.e - imp.hitch * f.e, n: p.n - imp.hitch * f.n}, pull = p.h;
    bars.forEach((bar, j) => {
     if (prev[j]) { const r = rt(phis[j]); phis[j] += ((pt.e - prev[j].e) * r.e + (pt.n - prev[j].n) * r.n) / bar; }
     prev[j] = pt;
     fold = Math.max(fold, Math.abs(wrap(phis[j] - pull)));
     if (j === 0) firstFold = Math.max(firstFold, Math.abs(wrap(phis[j] - pull)));
     pull = phis[j];
     const b = u(phis[j]); pt = {e: pt.e - bar * b.e, n: pt.n - bar * b.n};
    });
    th = phis[phis.length - 1];
    const b = u(th); c = {e: pt.e - imp.pivotToTool * b.e, n: pt.n - imp.pivotToTool * b.n};
   }
   const r = rt(th);
   c = {e: c.e + imp.offset * r.e, n: c.n + imp.offset * r.n};
   centre.push(c); heading.push(th);
   left.push({e: c.e - imp.half * r.e, n: c.n - imp.half * r.n}); right.push({e: c.e + imp.half * r.e, n: c.n + imp.half * r.n});
  }
  return {left, right, centre, heading, body, bodyLeft, bodyRight, foldDeg: deg(fold), firstFoldDeg: deg(firstFold)};
 }

 // `input`: machine ("tractor": the front wheels steer, the pivot is the rear axle; "articulated": the machine bends at a
 // joint midway between the axles; "harvester": the rear wheels steer, the pivot is the front axle), wheelbase, maxSteerDeg
 // (the front wheels', the articulation's or the rear wheels' angle on the stop), overhang (tractor and articulated: front
 // axle to the nose; harvester: rear axle to the back end). The implement as AgOpenWeb's Tool page has it: mount ("front",
 // "rear", "trailed" or "tbt"), width (or sections × sectionWidth in cm), overlap (a gap is negative), offset (the working
 // line's centre right of the machine's, left is negative: Tool offset), axleToTool (rigid: the axle nearest the implement to
 // its working line, what a tape measures: the rear axle for a rear implement, the front axle for a front one; AgOpenWeb's
 // Working center is from the pivot, so the wheelbase is added when that axle is not the pivot), hitchLength (rear axle to the hitch pin: the Hitch length of the Vehicle page), tankHitch (TBT:
 // hitch pin to the tank's axle, Tank trailing hitch), drawbar (hitch pin, or the tank's axle, to the implement's axle:
 // Trailing hitch), pivotToTool (its axle to the working line, behind = +: Pivot distance), length (from its attachment,
 // the 3-point hitch or its own axle, to its rearmost point: Implement length; 0 = not known), sprayerWidth (0 = no
 // tramline check), trackWidth and tyreWidth (the rear wheels, centre to centre, and the tyre's width: together the machine's
 // width, whose front corner sweeps the widest; trackWidth 0 = the centre line only, and no tyre check), wheelDiameter (the
 // rear wheel's height), drawbarHeight (the hitch pin above the ground: the tyre is shorter there than at the axle; 0 = the
 // whole wheel counts), drawbarWidth (a trailed implement's drawbar near the hitch, where it can reach those tyres),
 // swingDeg (the most the drawbar may turn to each side against the machine: a semi-mounted pivot's stop, or what the
 // farmer knows it takes; 0 = only the tyres limit it), foldClearanceDeg (how far short of that the turn keeps the drawbar:
 // 5° by default), margin (the room kept between what sweeps the widest in the turn and the field edge: 1 m by default,
 // more where the edge is irregular). `pick`: the key of the turn option the rows describe (the one with the smallest
 // headland when missing or unknown). Returns the numbers, `options` (every turn AgOpenWeb can make here, the smallest
 // headland first), `chosen` (the key described) and `rows`, one per suggestion, with the reason.
 function suggest(input, pick) {
  const i = Object.assign({machine: "tractor", wheelbase: 2.5, maxSteerDeg: 35, overhang: 1.2, mount: "rear", width: 6, sections: 0, sectionWidth: 0,
   overlap: 0, offset: 0, axleToTool: 1.8, hitchLength: 1.0, tankHitch: 3.0, drawbar: 3.0, pivotToTool: 0, length: 0, sprayerWidth: 0, margin: MARGIN,
   trackWidth: 0, tyreWidth: 0.6, wheelDiameter: 1.8, drawbarHeight: 0, drawbarWidth: 0.3, swingDeg: 0, foldClearanceDeg: FOLD_CLEARANCE_DEG}, input);
  if (i.frontOverhang !== undefined && input.overhang === undefined) i.overhang = i.frontOverhang;
  if (i.sections > 0 && i.sectionWidth > 0 && !(input.width > 0)) i.width = i.sections * i.sectionWidth / 100;
  const trailed = i.mount === "trailed" || i.mount === "tbt", tbt = i.mount === "tbt", front = i.mount === "front", problems = [];
  const articulated = i.machine === "articulated", harvester = i.machine === "harvester";
  const spacing = i.width - i.overlap, offset = Math.abs(i.offset), length = Math.max(0, i.length);
  if (!(i.wheelbase > 0) || !(i.maxSteerDeg > 0 && i.maxSteerDeg < 90)) problems.push("The wheelbase and the maximum steering angle must be above 0 (the angle below 90°).");
  if (!(i.width > 0)) problems.push("The implement needs a width (or sections and a section width).");
  else if (!(spacing > 0.01)) problems.push("The overlap must be less than the implement width.");
  if (!(i.margin >= 0)) problems.push("The safety margin to the field edge cannot be negative.");
  // the machine's width, from its rear track and tyres: its front corner (a harvester's rear corner) sweeps wider than its centre line
  const tyresOk = i.tyreWidth > 0 && i.trackWidth > i.tyreWidth;
  if (i.trackWidth > 0 && !tyresOk) problems.push("The rear tyres need a width, and the track must be wider than a tyre.");
  const bodyHalf = i.trackWidth > 0 ? (i.trackWidth + i.tyreWidth) / 2 : 0;
  // the drawbar against the rear tyres: a tractor's or an articulated machine's (a harvester's rear wheels steer)
  const tyreCheck = trailed && !harvester && i.trackWidth > 0;
  if (tyreCheck && tyresOk && !(i.wheelDiameter > 0 && i.drawbarWidth >= 0 && i.drawbarWidth < i.trackWidth - i.tyreWidth && i.drawbarHeight >= 0))
   problems.push("The rear wheels need a height, the drawbar must pass between the tyres, and its height cannot be negative.");
  if (trailed && !(i.swingDeg >= 0 && i.swingDeg <= 180)) problems.push("The most the drawbar may turn must be between 0 (the tyres only) and 180°.");
  if (trailed && !(i.foldClearanceDeg >= 0 && i.foldClearanceDeg <= 45)) problems.push("The room kept before the drawbar touches must be between 0 and 45°.");
  if (problems.length) return {problems, rows: [], options: []};
  // the working line from the pivot axle, along the machine. A rigid implement is measured from the axle nearest to it; the
  // pivot is a tractor's rear axle and a harvester's front axle, so a front implement on a tractor (or a rear one on a
  // harvester) is the wheelbase further away
  const fromOtherAxle = !trailed && (front !== harvester);
  const axleToTool = trailed ? i.hitchLength + (tbt ? i.tankHitch : 0) + i.drawbar + i.pivotToTool : i.axleToTool + (fromOtherAxle ? i.wheelbase : 0);

  // the tightest turn: a bicycle model for a tractor (and a harvester, steered from the back, pivot on the front axle);
  // an articulated machine with the joint midway turns both axles on (wheelbase ÷ 2) ÷ tan(angle ÷ 2)
  const rMin = articulated ? (i.wheelbase / 2) / Math.tan(rad(i.maxSteerDeg / 2)) : i.wheelbase / Math.tan(rad(i.maxSteerDeg));
  const rMinWhy = articulated ? "(wheelbase ÷ 2) ÷ tan(articulation angle ÷ 2), with the joint midway between the axles: both axles run on the same circle"
   : harvester ? "wheelbase ÷ tan(rear wheel angle), the radius at the front axle (the pivot of a harvester) with the wheels on the stop"
   : "wheelbase ÷ tan(maximum steering angle), the radius at the pivot with the wheels on the stop";
  let radius = Math.max(RADIUS_FACTOR * rMin, RADIUS_FLOOR), why = [];
  why.push(RADIUS_FACTOR + " × the tightest radius " + r1(rMin) + " m (" + (articulated ? "(" + i.wheelbase + " ÷ 2) ÷ tan(" + i.maxSteerDeg + "° ÷ 2)" : i.wheelbase + " ÷ tan " + i.maxSteerDeg + "°") + ")");
  if (radius === RADIUS_FLOOR && RADIUS_FACTOR * rMin < RADIUS_FLOOR) why.push("AgOpenWeb's simple turn path goes no tighter than " + RADIUS_FLOOR + " m");
  const longestBar = trailed ? Math.max(i.drawbar, tbt ? i.tankHitch : 0) : 0;
  const hitchMin = longestBar / Math.sin(rad(MAX_HITCH_DEG));
  if (hitchMin > radius) { radius = hitchMin; why.push("the " + longestBar + " m drawbar ÷ sin " + MAX_HITCH_DEG + "°, so the implement's wheels still roll round a circle of their own: at least " + r1(hitchMin) + " m"); }
  // the drawbar that the machine pulls (a TBT's tank's) folds no further than where it touches a rear tyre or reaches its
  // own stop (a semi-mounted implement's pivot), with some room, on the turn's circle
  const firstBar = tbt ? i.tankHitch : i.drawbar;
  const tyreBack = tyreReach(i.wheelDiameter, i.drawbarHeight);   // how far behind the axle the tyre reaches at the drawbar's height
  const touchDeg = tyreCheck ? drawbarTouchDeg(i.hitchLength, i.drawbarWidth, firstBar, i.trackWidth, i.tyreWidth, i.wheelDiameter, i.drawbarHeight) : null;
  const stopDeg = trailed && i.swingDeg > 0 ? i.swingDeg : null;
  const limits = [];
  if (touchDeg !== null) limits.push({at: touchDeg, what: "touches a rear tyre", onto: "onto a rear tyre (it touches at " + touchDeg + "°)"});
  if (stopDeg !== null) limits.push({at: stopDeg, what: "reaches the most it may turn", onto: "past the most it may turn (" + stopDeg + "° to each side)"});
  const limit = limits.length ? limits.reduce((a, b) => b.at < a.at ? b : a) : null;
  const clearance = i.foldClearanceDeg, foldLimit = limit ? limit.at - clearance : null;
  if (limit) {
   let r = radius;
   while (steadyFoldDeg(r, i.hitchLength, firstBar) > foldLimit && r < 200) r += 0.1;
   if (r >= 200) problems.push("The drawbar " + limit.what + " almost straight behind the tractor (at " + limit.at + "°): check the hitch, the drawbar, the track and the swing.");
   else if (r > radius + 1e-9) { radius = r; why.push("the drawbar " + limit.what + " when it folds " + limit.at + "°: kept under " + foldLimit + "° on the turn's circle, at least " + r1(up(r, 0.1)) + " m"); }
  }
  if (problems.length) return {problems, rows: [], options: []};
  radius = up(radius, 0.1);

  // the turn onto the next pass: a semicircle when the pass is 2 × radius or more away, else skip passes or loop
  const semicircle = spacing >= 2 * radius - 1e-9;
  const skip = semicircle ? 0 : Math.ceil(2 * radius / spacing - 1e-9) - 1;
  const landing = (skip + 1) * spacing;
  const keyhole = semicircle ? null : r1(keyholeDepth(radius, spacing));
  const sagitta = semicircle ? null : r1(depthOf(sagittaPath(radius, spacing)));
  const kturn = semicircle ? null : r1(kTurnDepth(radius, spacing));

  // what sweeps the widest in the turn: the machine's body (a tractor's nose, an articulated machine's nose, which swings
  // with its front half, a harvester's back end), at its outer corner, half the machine's width out from the pivot's
  // circle; or the implement's outer corner: its half width plus the offset out from that circle, and the working line
  // or its rearmost point along it
  const bodyName = harvester ? "the back end" : "the nose";
  const noseAlong = articulated ? i.overhang : i.wheelbase + i.overhang;
  const noseReach = Math.sqrt(Math.pow(radius + bodyHalf, 2) + noseAlong * noseAlong) - radius;
  const noseFormula = "√((" + r1(radius) + (bodyHalf > 0 ? " + " + r2(bodyHalf) : "") + ")² + " + (articulated ? i.overhang : "(" + i.wheelbase + " + " + i.overhang + ")") + "²) − " + r1(radius)
   + (bodyHalf > 0 ? ": its outer corner " + r2(bodyHalf) + " m off the centre line, half the " + r2(i.trackWidth + i.tyreWidth) + " m over the rear tyres" : "")
   + (articulated ? "; the front half swings with the turn" : "");
  const halfOut = i.width / 2 + offset, outName = "half the width" + (offset > 0 ? " plus the " + r1(offset) + " m offset" : "");
  let toolReach, toolWhy, follows = true, toolRadius = radius;
  if (!trailed) {
   // the working line, or the rearmost corner of a rear implement (its length from the 3-point hitch). An articulated
   // machine's front half runs tangent to the circle at the front axle: a front implement there is measured from it
   const line = front && articulated ? i.axleToTool : axleToTool;
   const corner = !front && length > 0 ? (harvester ? i.wheelbase : 0) + i.hitchLength + length : 0;
   const along = Math.max(line, corner);
   toolReach = Math.sqrt(Math.pow(radius + halfOut, 2) + along * along) - radius;
   const alongName = corner > line ? "its length behind the hitch"
    : front && articulated ? "the working line " + r1(line) + " m ahead of the front axle, which swings with the turn"
    : "the working line " + r1(line) + " m " + (front ? "ahead of" : "behind") + " the pivot" + (fromOtherAxle ? " (" + i.wheelbase + " m wheelbase + " + i.axleToTool + " m from the " + (front ? "front" : "rear") + " axle)" : "");
   toolWhy = "the " + (front ? "front " : "") + "implement's outer " + (corner > line ? "rear corner" : "end") + " " + r1(toolReach) + " m (√((" + r1(radius) + " + " + r1(halfOut) + ")² + " + r1(along) + "²) − " + r1(radius) + ": " + outName + ", " + alongName + ")";
  } else {
   // a trailed implement cuts inside: its axle runs on a smaller circle than the hitch pin (and a TBT's tank between them)
   let r = Math.sqrt(radius * radius + i.hitchLength * i.hitchLength);
   for (const bar of tbt ? [i.tankHitch, i.drawbar] : [i.drawbar]) { if (bar >= r) { follows = false; r = 0; break; } r = Math.sqrt(r * r - bar * bar); }
   toolRadius = r;
   const along = Math.max(i.pivotToTool, length);
   toolReach = Math.max(0, Math.sqrt(Math.pow(toolRadius + halfOut, 2) + along * along) - radius);
   toolWhy = "the " + (tbt ? "TBT" : "trailed") + " implement's axle runs " + r1(toolRadius) + " m from the turn's centre, inside the " + (harvester ? "machine" : "tractor") + ": its outer " + (along > i.pivotToTool ? "rear corner" : "end") + " (" + outName + ") reaches " + r1(toolReach) + " m past the pivot's path";
  }
  const reach = Math.max(noseReach, toolReach);
  const margin = i.margin, distance = up(reach + margin, 0.1);
  const noseWhy = bodyName + " of the machine sweeps " + r1(noseReach) + " m past the pivot's path (" + noseFormula + ")";
  const distanceWhy = (noseReach >= toolReach ? noseWhy + ", more than " + toolWhy : toolWhy + ", more than " + bodyName + " of the machine (" + r1(noseReach) + " m)") + "; plus the " + r1(margin) + " m safety margin";

  // Each turn followed step by step (the implement and the machine's far end with it): how deep the pivot goes, what goes
  // further than the distance from boundary allows (a K-turn backs the implement up first), and how far back into the field
  // anything not yet straight on a pass reaches: the implement finishing its pass when the turn starts, its ends swinging,
  // a front implement or a trailed one until it is straight on the next pass (within 10 cm and 2°), a K-turn's tail
  const imp = trailed ? {bars: tbt ? [i.tankHitch, i.drawbar] : [i.drawbar], hitch: i.hitchLength, pivotToTool: i.pivotToTool, offset: i.offset, half: i.width / 2}
   : {along: front ? axleToTool : -axleToTool, offset: i.offset, half: i.width / 2};
  imp.body = articulated ? null : harvester ? -(i.wheelbase + i.overhang) : i.wheelbase + i.overhang;
  imp.bodyHalf = bodyHalf;
  // a long implement's rear corners, past its working line (Implement length): followed too, as a K-turn backs them up first
  const cornerAlong = (harvester ? i.wheelbase : 0) + i.hitchLength + length;
  const cornerImp = front || !(length > 0) ? null : trailed ? (length > i.pivotToTool ? Object.assign({}, imp, {pivotToTool: length}) : null)
   : cornerAlong > axleToTool ? Object.assign({}, imp, {along: -cornerAlong}) : null;
  const exitLen = Math.max(20, 15 * axleToTool);
  const implName = front ? "the front implement" : trailed ? "the " + (tbt ? "TBT" : "trailed") + " implement" : "the implement";
  function study(key, lateral, depth) {
   const path = turnPath(key, radius, lateral, exitLen), sw = sweep(path.poses, imp), P = path.poses, end = path.turnEnd;
   const corner = cornerImp ? sweep(path.poses, cornerImp) : null;
   // the furthest anything goes, the way out included: a trailed implement's tail still swings out after the pivot's turn
   let top = 0;
   for (let k = 0; k < P.length; k++) top = Math.max(top, sw.left[k].n, sw.right[k].n, imp.body != null && k <= end ? Math.max(sw.bodyLeft[k].n, sw.bodyRight[k].n) : -Infinity,
    corner ? Math.max(corner.left[k].n, corner.right[k].n) : -Infinity);
   const own = top - depth > reach + 0.01 ? top - depth : reach;   // a K-turn: what backs up first reaches past the pivot
   const dist = up(own + margin, 0.1);
   let low = 0, lowWhy = "";
   const take = (n, what) => { if (n < low - 1e-6) { low = n; lowWhy = what; } };
   const inTail = k => key === "kturn" && k >= path.arcEnd && k <= path.forwardEnd;
   for (let k = 0; k < end; k++) take(Math.min(sw.left[k].n, sw.right[k].n), k === 0 ? "start" : inTail(k) ? "tail" : "swing");
   if (key === "kturn" && imp.body > 0) for (let k = path.arcEnd; k <= path.forwardEnd; k++) take(Math.min(sw.bodyLeft[k].n, sw.bodyRight[k].n), "nose");
   const target = sw.centre[P.length - 1].e;
   let aligned = P.length - 1;
   for (let k = end; k < P.length; k++) {
    if (Math.abs(sw.centre[k].e - target) < 0.1 && Math.abs(wrap(sw.heading[k] - Math.PI)) < rad(2)) { aligned = k; take(sw.centre[k].n, "exit"); break; }
    take(Math.min(sw.left[k].n, sw.right[k].n), "exit");
   }
   const sum = r1(depth) + dist - low, passes = Math.ceil(sum / spacing - 0.001);
   const lowText = lowWhy === "start" ? implName + " " + r1(-low) + " m behind the pivot finishing its pass when the turn starts"
    : lowWhy === "exit" ? (trailed ? implName + " straight on the next pass (within 10 cm) only " + r1(-low) + " m back from where the turn started"
     : implName + " " + r1(-low) + " m ahead of the pivot when the turn ends, already on the next pass")
    : lowWhy === "nose" ? "the K-turn's tail taking the tractor's nose " + r1(-low) + " m back into the field"
    : lowWhy === "tail" ? "the K-turn's tail taking " + implName + "'s end " + r1(-low) + " m back into the field"
    : lowWhy === "swing" ? implName + "'s end swinging " + r1(-low) + " m back into the field" : "";
   return {path: Object.assign(path, {aligned}), sweep: sw, corner, distance: dist, top: r1(top), low: r1(low), lowText,
    headland: {sum: r1(sum), passes, width: r2(passes * spacing)}};
  }
  // Every turn AgOpenWeb can make onto the passes within reach: Sagitta with 0 to `skip` rows skipped (a plain semicircle
  // once the landing pass is 2 × radius away, the offset arc before that), Keyhole with the rows that still need a loop,
  // and the K-turn onto the next pass. Sorted by the headland each one needs, the shallowest turn first on a tie.
  const option = (kind, style, skipRows, lateral, depth, text) => Object.assign({key: kind + skipRows, kind, style, skip: skipRows, lateral: r2(lateral), depth}, study(kind, lateral, depth), {text});
  const options = [];
  let kTurnOut = "";
  const rows_ = s => s + " row" + (s > 1 ? "s" : "");
  for (let s = 0; s <= skip; s++) {
   const lat = (s + 1) * spacing, plain = lat >= 2 * radius - 1e-9, where = s ? "the turn lands " + r2(lat) + " m away" : "the next pass is " + r2(lat) + " m away";
   const worked = s ? "; the skipped passes are worked on the way back" : "";
   if (plain) options.push(option("sagitta", "Sagitta", s, lat, r1(radius), "Sagitta, skip " + rows_(s) + ": " + where + ", 2 × " + r1(radius) + " m or more, so the turn is a plain semicircle " + r1(radius) + " m deep" + worked));
   else {
    const sag = r1(depthOf(sagittaPath(radius, lat))), key = r1(keyholeDepth(radius, lat));
    options.push(option("sagitta", "Sagitta", s, lat, sag, "Sagitta, skip " + rows_(s) + ": " + where + ", closer than 2 × " + r1(radius) + " m, so the turn first swings out the other way, then loops " + sag + " m deep" + worked));
    options.push(option("keyhole", "Keyhole", s, lat, key, "Keyhole, skip " + rows_(s) + ": " + where + ", closer than 2 × " + r1(radius) + " m, so the turn loops " + key + " m deep (a Sagitta turn would go " + sag + " m deep and swing further out)" + worked));
   }
  }
  if (!semicircle) {
   // the K-turn backs up: a trailed implement pushed back along that path folds; past the hitch limit it is left out
   const k = option("kturn", "K-turn", 0, spacing, kturn, "");
   const kb = kTurnBack(radius, spacing), backed = r1(kb.back + (Math.PI - K_TURN_ARC) * radius);
   k.reverse = true;
   k.text = "K-turn, skip 0: AgOpenWeb drives a 126° arc forward and a straight tail of " + r1(1.5 * spacing) + " m; as soon as you back up the turn is over for AgOpenWeb and its guidance is on the next pass, backing up too: you back up " + backed + " m until you are on it and straight, " + r1(kb.end.n) + " m deep, then drive forward. A stop and a reverse at every pass";
   if (foldLimit !== null && k.sweep.firstFoldDeg > foldLimit) kTurnOut = "No K-turn: backing the " + (tbt ? "TBT" : "trailed") + " implement up those " + backed + " m folds its drawbar to " + Math.round(k.sweep.firstFoldDeg) + "°, " + limit.onto;
   else if (trailed && k.sweep.foldDeg > MAX_HITCH_DEG) kTurnOut = "No K-turn: backing the " + (tbt ? "TBT" : "trailed") + " implement up those " + backed + " m folds its drawbar past " + MAX_HITCH_DEG + "° (a jackknife); an implement lifted rigid on the 3-point linkage for the turn is Rear fixed here";
   else { if (trailed) k.text += "; backing up a trailed implement takes care: its drawbar folds to " + Math.round(k.sweep.foldDeg) + "°"; options.push(k); }
  }
  options.sort((a, b) => a.headland.width - b.headland.width || a.depth - b.depth || a.skip - b.skip);
  const chosen = options.find(o => o.key === pick) || options[0];
  const headlandOf = o => o.headland.width + " m (" + o.headland.passes + " pass" + (o.headland.passes > 1 ? "es" : "") + ")";
  const extension = up(Math.max(2 * radius, trailed ? 3 * axleToTool : 0), 0.5);
  const extensionWhy = !trailed || 2 * radius >= 3 * axleToTool ? "2 × the radius" : "3 × the " + r1(axleToTool) + " m from the rear axle to the working line, so the " + (tbt ? "TBT" : "trailed") + " implement is straight behind the tractor before the sections come on";
  let tram = null;
  if (i.sprayerWidth > 0) {
   const passes = i.sprayerWidth / spacing, whole = Math.abs(passes - Math.round(passes)) < 0.01;
   tram = {width: i.sprayerWidth, passes: whole ? Math.round(passes) : r1(passes), whole};
  }

  const o = chosen, others = options.length > 1 ? ". The other turns are in the list below, the smallest headland first; pick one and these rows follow it" : "";
  const rows = [
   {key: "spacing", name: "Track spacing (not a setting)", value: r2(spacing) + " m", why: "implement width " + i.width + " m − overlap " + i.overlap + " m; AgOpenWeb works it out itself"},
   {key: "rMin", name: "Tightest turn of the machine (not a setting)", value: r1(rMin) + " m", why: rMinWhy},
   {key: "radius", name: "U-Turn → Radius", value: r1(radius) + " m", why: why.join("; ")},
   {key: "style", name: "U-Turn → Turn style and skip rows", value: o.style + ", skip " + o.skip, why: o.text + (kTurnOut ? ". " + kTurnOut : "") + others},
   {key: "distance", name: "U-Turn → Distance from boundary", value: o.distance + " m",
    why: distanceWhy + (o.distance !== distance ? ". " + o.style + ", skip " + o.skip + ": " + (o.reverse ? "backing up, " + implName + " or the machine reaches "
     : implName + (trailed ? " swings out as the tractor straightens, " : " reaches ")) + r1(o.top - o.depth) + " m past the pivot's deepest point; plus the margin" : "")},
   {key: "headland", name: "Headland width (Field builder → Headland)", value: headlandOf(o),
    why: o.style + ", skip " + o.skip + ": turn depth " + o.depth + " + distance from boundary " + o.distance + (o.low < 0 ? " + " + r1(-o.low) + " m (" + o.lowText + ")" : "") + " = " + o.headland.sum + " m"
     + ", rounded up to whole passes, so nothing that is not straight on a pass goes past the headland line"},
   {key: "extension", name: "U-Turn → Extension", value: extension + " m", why: extensionWhy + ": straight legs into and out of the turn"},
   {key: "goAround", name: "U-Turn → Going around obstacles", value: "radius " + r1(radius) + " m, margin " + GO_AROUND_MARGIN + " m", why: "the same radius as the U-turn; AgOpenWeb refuses anything tighter than the tractor's tightest turn"},
  ];
  if (fromOtherAxle) rows.push({key: "hitch", name: "Tool → Hitch length (working center)", value: r1(axleToTool) + " m " + (front ? "ahead of the rear axle" : "behind the front axle"),
   why: "AgOpenWeb measures a rigid implement from the pivot, the " + (harvester ? "front" : "rear") + " axle: the " + i.wheelbase + " m wheelbase + the " + i.axleToTool + " m from the " + (front ? "front" : "rear") + " axle to the working line"});
  if (tram) rows.push({key: "tram", name: "Tram → Tram width and passes", value: tram.whole ? tram.width + " m, every " + tram.passes + " pass" + (tram.passes > 1 ? "es" : "") : "no match",
   why: tram.whole ? "sprayer width ÷ track spacing" : "sprayer width " + tram.width + " ÷ track spacing " + r2(spacing) + " = " + tram.passes + ", not a whole number: the sprayer's wheels would not land on the same tracks every pass"});
  if (tyreCheck || stopDeg !== null) {
   const where = i.hitchLength < tyreBack ? "inside the tyres' back edge" : "behind the tyres' back edge";
   const most = Math.max(...options.map(o => o.sweep.firstFoldDeg)), parts = [], value = [];
   if (tyreCheck) {
    value.push(touchDeg === null ? "never touches the tyres" : "touches the tyres at " + touchDeg + "°");
    parts.push("the " + i.drawbarWidth + " m wide drawbar from the hitch " + i.hitchLength + " m behind the rear axle (" + where + ": " + r2(tyreBack) + " m"
     + (tyreBack < i.wheelDiameter / 2 - 1e-9 ? " at the drawbar's " + i.drawbarHeight + " m height, where the " + i.wheelDiameter + " m wheel is shorter than at its axle" : "") + ") and the rear tyres, "
     + i.tyreWidth + " m wide, " + i.trackWidth + " m apart" + (touchDeg === null ? ", never meet" : ", meet when the drawbar folds " + touchDeg + "° against the tractor"));
   }
   if (stopDeg !== null) { value.push("at most " + stopDeg + "°"); parts.push("the drawbar may turn " + stopDeg + "° to each side at most (its stop, or what you typed)"); }
   rows.push({key: "fold", name: "How far the drawbar may fold (not a setting)", value: value.join(", "),
    why: parts.join("; ") + "; the turns here fold it " + Math.round(most) + "° at most" + (limit ? " (" + clearance + "° or more away)" : "")});
  }
  if (front && !harvester && i.axleToTool < i.overhang) problems.push("The front implement's working line is " + i.axleToTool + " m ahead of the front axle, behind the nose (" + i.overhang + " m): measure from the front axle to the boom or the cutter bar.");
  if (!follows) problems.push("The drawbar is longer than the hitch's turn radius: the implement cannot follow this turn. Set a bigger radius or a shorter drawbar.");
  return {problems, rows, machine: i.machine, spacing: r2(spacing), rMin: r1(rMin), radius: r1(radius), semicircle, skip, landing: r1(landing), keyhole, sagitta, kturn,
   bodyName, noseReach: r1(noseReach), toolReach: r1(toolReach), reach: r1(reach), margin: r1(margin), distance, axleToTool: r1(axleToTool), toolRadius: r1(toolRadius), halfOut: r1(halfOut), extension,
   options, chosen: o.key, kTurnOut, headlandOf, tram, goAround: {radius: r1(radius), margin: GO_AROUND_MARGIN}, touchDeg, stopDeg, foldLimit, bodyHalf: r2(bodyHalf), tyreBack: r2(tyreBack)};
 }

 const api = {suggest, keyholeDepth, keyholePath, sagittaPath, kTurnBack, kTurnDepth, turnPath, sweep, depthOf, drawbarTouchDeg, steadyFoldDeg, tyreReach,
  VERSION, RADIUS_FACTOR, RADIUS_FLOOR, MARGIN, MAX_HITCH_DEG, FOLD_CLEARANCE_DEG, K_TURN_ARC};
 if (typeof module !== "undefined" && module.exports) module.exports = api; else root.AowSettings = api;
})(typeof window !== "undefined" ? window : globalThis);
