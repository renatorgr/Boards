// settings.js - the AgOpenWeb settings worked out from the tractor and implement dimensions.
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
 const RADIUS_FACTOR = 1.2;    // the turn radius over the tightest the tractor can do: the steering stays off its stop
 const RADIUS_FLOOR = 4.0;     // AgOpenWeb's simple turn path never goes below 4 m
 const MARGIN = 1.0;           // room kept between what sweeps the widest and the field edge, unless `margin` says otherwise
 const MAX_HITCH_DEG = 75;     // the most a drawbar may fold against the tractor in the turn
 const GO_AROUND_MARGIN = 1.0; // AgOpenWeb's safety margin round a Go around obstacle
 const K_TURN_ARC = 2.2;      // radians: the forward arc AgOpenWeb draws for a K-turn before the tractor backs up
 const rad = d => d * Math.PI / 180;
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

 // The pivot's path for a turn option ("semicircle" or "skip": quarter, straight, quarter; "keyhole"; "kturn") from (0, 0)
 // heading north onto the pass `lateral` metres to the right, then `exit` metres down that pass. `turnEnd`: where the
 // turn ends; a K-turn's `arcEnd` and `forwardEnd` (the tail between them), the backing up after it.
 function turnPath(key, radius, lateral, exit) {
  const R = radius, pieces = [];
  if (key === "keyhole") {
   const across = lateral / 2 + R, c = Math.atan2(Math.sqrt(Math.max(0, 4 * R * R - across * across)), across);
   pieces.push({len: c * R, turn: -1, r: R}, {len: (Math.PI + 2 * c) * R, turn: 1, r: R}, {len: c * R, turn: -1, r: R});
  } else if (key === "kturn") {
   pieces.push({len: K_TURN_ARC * R, turn: 1, r: R}, {len: 1.5 * lateral, turn: 0},
    {len: kTurnBack(R, lateral).back, dir: -1, turn: 0}, {len: (Math.PI - K_TURN_ARC) * R, dir: -1, turn: 1, r: R});
  } else pieces.push({len: Math.PI / 2 * R, turn: 1, r: R}, {len: Math.max(0, lateral - 2 * R), turn: 0}, {len: Math.PI / 2 * R, turn: 1, r: R});
  pieces.push({len: exit, turn: 0});
  const w = walk(pieces), turnEnd = w.ends[w.ends.length - 2];
  return {poses: w.poses, turnEnd, arcEnd: key === "kturn" ? w.ends[0] : turnEnd, forwardEnd: key === "kturn" ? w.ends[1] : turnEnd};
 }

 // Where the implement's two ends go along `poses`, and the machine's far end (`body` metres ahead of the pivot, behind
 // when negative; null: not followed). `imp`: rigid {along: + ahead of the pivot, − behind} or trailed {hitch: behind the
 // pivot, bars: [tank hitch,] drawbar, pivotToTool}; offset (right +), half (the width ÷ 2). A trailed implement's axle
 // follows its hitch with no side slip: its heading turns by the hitch's sideways move ÷ the drawbar, forwards and backing
 // up. `foldDeg`: the most a drawbar folds against what pulls it.
 function sweep(poses, imp) {
  const u = h => ({e: Math.sin(h), n: Math.cos(h)}), rt = h => ({e: Math.cos(h), n: -Math.sin(h)});
  const left = [], right = [], centre = [], heading = [], body = [];
  const bars = imp.bars || [], phis = bars.map(() => poses[0].h), prev = bars.map(() => null);
  let fold = 0;
  for (const p of poses) {
   const f = u(p.h);
   if (imp.body != null) body.push({e: p.e + imp.body * f.e, n: p.n + imp.body * f.n});
   let c, th;
   if (!bars.length) { c = {e: p.e + imp.along * f.e, n: p.n + imp.along * f.n}; th = p.h; }
   else {
    let pt = {e: p.e - imp.hitch * f.e, n: p.n - imp.hitch * f.n}, pull = p.h;
    bars.forEach((bar, j) => {
     if (prev[j]) { const r = rt(phis[j]); phis[j] += ((pt.e - prev[j].e) * r.e + (pt.n - prev[j].n) * r.n) / bar; }
     prev[j] = pt;
     fold = Math.max(fold, Math.abs(wrap(phis[j] - pull)));
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
  return {left, right, centre, heading, body, foldDeg: fold * 180 / Math.PI};
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
 // tramline check), margin (the room kept between what sweeps the widest in the turn and the field edge: 1 m by default,
 // more where the edge is irregular). Returns the numbers and `rows`, one per suggestion, with the reason.
 function suggest(input) {
  const i = Object.assign({machine: "tractor", wheelbase: 2.5, maxSteerDeg: 35, overhang: 1.2, mount: "rear", width: 6, sections: 0, sectionWidth: 0,
   overlap: 0, offset: 0, axleToTool: 1.8, hitchLength: 1.0, tankHitch: 3.0, drawbar: 3.0, pivotToTool: 0, length: 0, sprayerWidth: 0, margin: MARGIN}, input);
  if (i.frontOverhang !== undefined && input.overhang === undefined) i.overhang = i.frontOverhang;
  if (i.sections > 0 && i.sectionWidth > 0 && !(input.width > 0)) i.width = i.sections * i.sectionWidth / 100;
  const trailed = i.mount === "trailed" || i.mount === "tbt", tbt = i.mount === "tbt", front = i.mount === "front", problems = [];
  const articulated = i.machine === "articulated", harvester = i.machine === "harvester";
  const spacing = i.width - i.overlap, offset = Math.abs(i.offset), length = Math.max(0, i.length);
  if (!(i.wheelbase > 0) || !(i.maxSteerDeg > 0 && i.maxSteerDeg < 90)) problems.push("The wheelbase and the maximum steering angle must be above 0 (the angle below 90°).");
  if (!(i.width > 0)) problems.push("The implement needs a width (or sections and a section width).");
  else if (!(spacing > 0.01)) problems.push("The overlap must be less than the implement width.");
  if (!(i.margin >= 0)) problems.push("The safety margin to the field edge cannot be negative.");
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
  if (hitchMin > radius) { radius = hitchMin; why.push("the " + longestBar + " m drawbar must not fold past " + MAX_HITCH_DEG + "°: at least " + r1(hitchMin) + " m"); }
  radius = up(radius, 0.1);

  // the turn onto the next pass: a semicircle when the pass is 2 × radius or more away, else skip passes or loop
  const semicircle = spacing >= 2 * radius - 1e-9;
  const skip = semicircle ? 0 : Math.ceil(2 * radius / spacing - 1e-9) - 1;
  const landing = (skip + 1) * spacing;
  const keyhole = semicircle ? null : r1(keyholeDepth(radius, spacing));
  const sagitta = semicircle ? null : r1(depthOf(sagittaPath(radius, spacing)));
  const kturn = semicircle ? null : r1(kTurnDepth(radius, spacing));

  // what sweeps the widest in the turn: the machine's body (a tractor's nose, an articulated machine's nose, which swings
  // with its front half, a harvester's back end), or the implement's outer corner: its half width plus the offset out
  // from the pivot's circle, and the working line or its rearmost point along it
  const bodyName = harvester ? "the back end" : "the nose";
  const noseReach = articulated ? Math.sqrt(radius * radius + i.overhang * i.overhang) - radius
   : Math.sqrt(radius * radius + Math.pow(i.wheelbase + i.overhang, 2)) - radius;
  const noseFormula = articulated ? "√(" + r1(radius) + "² + " + i.overhang + "²) − " + r1(radius) + ", the front half swings with the turn"
   : "√(" + r1(radius) + "² + (" + i.wheelbase + " + " + i.overhang + ")²) − " + r1(radius);
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
  const exitLen = Math.max(20, 15 * axleToTool);
  const implName = front ? "the front implement" : trailed ? "the " + (tbt ? "TBT" : "trailed") + " implement" : "the implement";
  function study(key, lateral, depth) {
   const path = turnPath(key, radius, lateral, exitLen), sw = sweep(path.poses, imp), P = path.poses, end = path.turnEnd;
   let top = 0;
   for (let k = 0; k <= end; k++) top = Math.max(top, sw.left[k].n, sw.right[k].n, imp.body != null ? sw.body[k].n : -Infinity);
   const own = top - depth > reach + 0.01 ? top - depth : reach;   // a K-turn: what backs up first reaches past the pivot
   const dist = up(own + margin, 0.1);
   let low = 0, lowWhy = "";
   const take = (n, what) => { if (n < low - 1e-6) { low = n; lowWhy = what; } };
   const inTail = k => key === "kturn" && k >= path.arcEnd && k <= path.forwardEnd;
   for (let k = 0; k < end; k++) take(Math.min(sw.left[k].n, sw.right[k].n), k === 0 ? "start" : inTail(k) ? "tail" : "swing");
   if (key === "kturn" && imp.body > 0) for (let k = path.arcEnd; k <= path.forwardEnd; k++) take(sw.body[k].n, "nose");
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
   return {path: Object.assign(path, {aligned}), sweep: sw, distance: dist, top: r1(top), low: r1(low), lowText,
    headland: {sum: r1(sum), passes, width: r2(passes * spacing)}};
  }
  const option = (key, style, skipRows, lateral, depth, text) => Object.assign({key, style, skip: skipRows, depth}, study(key, lateral, depth), {text});
  const options = [];
  let kTurnOut = "";
  if (semicircle) options.push(option("semicircle", "Sagitta", 0, spacing, r1(radius),
   "Sagitta, skip 0: the next pass is " + r2(spacing) + " m away, 2 × " + r1(radius) + " m or more, so the turn is a plain semicircle " + r1(radius) + " m deep"));
  else {
   options.push(option("skip", "Sagitta", skip, landing, r1(radius),
    "Sagitta, skip " + skip + " row" + (skip > 1 ? "s" : "") + ": the turn lands " + r1(landing) + " m away, 2 × " + r1(radius) + " m or more, a plain semicircle " + r1(radius) + " m deep; the skipped passes are worked on the way back"));
   options.push(option("keyhole", "Keyhole", 0, spacing, keyhole,
    "Keyhole, skip 0: the next pass is only " + r2(spacing) + " m away, closer than 2 × " + r1(radius) + " m, so the turn loops " + keyhole + " m deep (a Sagitta turn would go " + sagitta + " m deep and swing further out)"));
   // the K-turn backs up: a trailed implement pushed back along that path folds; past the hitch limit it is left out
   const k = option("kturn", "K-turn", 0, spacing, kturn, "");
   const kb = kTurnBack(radius, spacing), backed = r1(kb.back + (Math.PI - K_TURN_ARC) * radius);
   k.reverse = true;
   k.text = "K-turn, skip 0: AgOpenWeb drives a 126° arc forward and a straight tail of " + r1(1.5 * spacing) + " m; as soon as you back up the turn is over for AgOpenWeb and its guidance is on the next pass, backing up too: you back up " + backed + " m until you are on it and straight, " + r1(kb.end.n) + " m deep, then drive forward. A stop and a reverse at every pass";
   if (trailed && k.sweep.foldDeg > MAX_HITCH_DEG) kTurnOut = "No K-turn: backing the " + (tbt ? "TBT" : "trailed") + " implement up those " + backed + " m folds its drawbar past " + MAX_HITCH_DEG + "° (a jackknife); an implement lifted rigid on the 3-point linkage for the turn is Rear fixed here";
   else { if (trailed) k.text += "; backing up a trailed implement takes care: its drawbar folds to " + Math.round(k.sweep.foldDeg) + "°"; options.push(k); }
  }
  const extension = up(Math.max(2 * radius, trailed ? 3 * axleToTool : 0), 0.5);
  const extensionWhy = !trailed || 2 * radius >= 3 * axleToTool ? "2 × the radius" : "3 × the " + r1(axleToTool) + " m from the rear axle to the working line, so the " + (tbt ? "TBT" : "trailed") + " implement is straight behind the tractor before the sections come on";
  let tram = null;
  if (i.sprayerWidth > 0) {
   const passes = i.sprayerWidth / spacing, whole = Math.abs(passes - Math.round(passes)) < 0.01;
   tram = {width: i.sprayerWidth, passes: whole ? Math.round(passes) : r1(passes), whole};
  }

  const widths = options.map(o => o.headland.width + " m (" + o.headland.passes + " pass" + (o.headland.passes > 1 ? "es" : "") + ")");
  const sameHeadland = widths.every(w => w === widths[0]);
  const rows = [
   {key: "spacing", name: "Track spacing (not a setting)", value: r2(spacing) + " m", why: "implement width " + i.width + " m − overlap " + i.overlap + " m; AgOpenWeb works it out itself"},
   {key: "rMin", name: "Tightest turn of the machine (not a setting)", value: r1(rMin) + " m", why: rMinWhy},
   {key: "radius", name: "U-Turn → Radius", value: r1(radius) + " m", why: why.join("; ")},
   {key: "style", name: "U-Turn → Turn style and skip rows", value: options.map(o => o.style + ", skip " + o.skip).join(" or "), why: options.map(o => o.text).join(". ") + (kTurnOut ? ". " + kTurnOut : "")},
   {key: "distance", name: "U-Turn → Distance from boundary", value: options.some(o => o.distance !== distance) ? options.map(o => o.distance + " m" + (options.length > 1 ? " (" + o.style + ")" : "")).join(", ") : distance + " m",
    why: distanceWhy + options.filter(o => o.distance !== distance).map(o => ". " + o.style + ": backing up, " + implName + " or the machine reaches " + r1(o.top - o.depth) + " m past the pivot's deepest point; plus the margin").join("")},
   {key: "headland", name: "Headland width (Field builder → Headland)", value: sameHeadland ? widths[0] : widths.join(" or "),
    why: options.map(o => (options.length > 1 ? o.style + ", skip " + o.skip + ": " : "") + "turn depth " + o.depth + " + distance from boundary " + o.distance + (o.low < 0 ? " + " + r1(-o.low) + " m (" + o.lowText + ")" : "") + " = " + o.headland.sum + " m").join("; ")
     + ", rounded up to whole passes, so nothing that is not straight on a pass goes past the headland line" + (sameHeadland && options.length > 1 ? "; the same headland for every turn" : "")},
   {key: "extension", name: "U-Turn → Extension", value: extension + " m", why: extensionWhy + ": straight legs into and out of the turn"},
   {key: "goAround", name: "U-Turn → Going around obstacles", value: "radius " + r1(radius) + " m, margin " + GO_AROUND_MARGIN + " m", why: "the same radius as the U-turn; AgOpenWeb refuses anything tighter than the tractor's tightest turn"},
  ];
  if (fromOtherAxle) rows.push({key: "hitch", name: "Tool → Hitch length (working center)", value: r1(axleToTool) + " m " + (front ? "ahead of the rear axle" : "behind the front axle"),
   why: "AgOpenWeb measures a rigid implement from the pivot, the " + (harvester ? "front" : "rear") + " axle: the " + i.wheelbase + " m wheelbase + the " + i.axleToTool + " m from the " + (front ? "front" : "rear") + " axle to the working line"});
  if (tram) rows.push({key: "tram", name: "Tram → Tram width and passes", value: tram.whole ? tram.width + " m, every " + tram.passes + " pass" + (tram.passes > 1 ? "es" : "") : "no match",
   why: tram.whole ? "sprayer width ÷ track spacing" : "sprayer width " + tram.width + " ÷ track spacing " + r2(spacing) + " = " + tram.passes + ", not a whole number: the sprayer's wheels would not land on the same tracks every pass"});
  if (front && !harvester && i.axleToTool < i.overhang) problems.push("The front implement's working line is " + i.axleToTool + " m ahead of the front axle, behind the nose (" + i.overhang + " m): measure from the front axle to the boom or the cutter bar.");
  if (!follows) problems.push("The drawbar is longer than the hitch's turn radius: the implement cannot follow this turn. Set a bigger radius or a shorter drawbar.");
  return {problems, rows, machine: i.machine, spacing: r2(spacing), rMin: r1(rMin), radius: r1(radius), semicircle, skip, landing: r1(landing), keyhole, sagitta, kturn,
   bodyName, noseReach: r1(noseReach), toolReach: r1(toolReach), reach: r1(reach), margin: r1(margin), distance, axleToTool: r1(axleToTool), toolRadius: r1(toolRadius), halfOut: r1(halfOut), extension, options, tram,
   goAround: {radius: r1(radius), margin: GO_AROUND_MARGIN}};
 }

 const api = {suggest, keyholeDepth, keyholePath, sagittaPath, kTurnBack, kTurnDepth, turnPath, sweep, depthOf, RADIUS_FACTOR, RADIUS_FLOOR, MARGIN, MAX_HITCH_DEG, K_TURN_ARC};
 if (typeof module !== "undefined" && module.exports) module.exports = api; else root.AowSettings = api;
})(typeof window !== "undefined" ? window : globalThis);
