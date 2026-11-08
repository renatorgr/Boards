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
const K_TURN_ARC = 2.2;       // radians: the forward arc AgOpenWeb draws for a K-turn before the tractor backs up
 const rad = d => d * Math.PI / 180;
 const r1 = v => Math.round(v * 10) / 10;
 const up = (v, step) => Math.ceil(v / step - 1e-9) * step;

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

 // The K-turn from (0, 0) heading north onto the pass `lateral` metres to the right. AgOpenWeb draws only its first
 // part (YouTurnCreationService.CreateKStyleTurnCurve, AgOpenGPS's KStyleTurn): an arc of 2.2 rad (126°) and a straight
 // tail of 1.5 × the distance to the next pass; the turn is over for it as soon as the tractor reverses, and guidance
 // moves to the next pass. `forward` is that part; `reverse` is the backing up done by hand: back along the tail, then a
 // reverse arc of the remaining 54° at the same radius, ending heading south at `end`, the next pass to the left.
 function kTurnPath(radius, lateral) {
  const forward = [{e: 0, n: 0}], pos = {e: 0, n: 0}, tail = 1.5 * Math.max(lateral, 0);
  const theta = addArc(pos, 0, forward, K_TURN_ARC * radius, true, radius);
  const tip = {e: pos.e + tail * Math.sin(theta), n: pos.n + tail * Math.cos(theta)};
  forward.push(tip);
  const reverse = [tip, {e: pos.e, n: pos.n}], back = {e: pos.e, n: pos.n};
  const segments = 12, step = (Math.PI - K_TURN_ARC) / segments;
  for (let k = 1; k <= segments; k++) {   // backing up while the heading goes on to south: the pivot moves against its heading
   const h0 = theta + (k - 1) * step, h1 = theta + k * step;
   back.e += radius * (Math.cos(h1) - Math.cos(h0)); back.n -= radius * (Math.sin(h1) - Math.sin(h0));
   reverse.push({e: back.e, n: back.n});
  }
  return {forward, reverse, end: {e: back.e, n: back.n}};
 }
 const kTurnDepth = radius => 2 * radius * Math.sin(K_TURN_ARC);   // the forward arc's depth and the reverse arc's again

 // `input`: machine ("tractor": the front wheels steer, the pivot is the rear axle; "articulated": the machine bends at a
 // joint midway between the axles; "harvester": the rear wheels steer, the pivot is the front axle), wheelbase, maxSteerDeg
 // (the front wheels', the articulation's or the rear wheels' angle on the stop), overhang (tractor and articulated: front
 // axle to the nose; harvester: rear axle to the back end). The implement as AgOpenWeb's Tool page has it: mount ("front",
 // "rear", "trailed" or "tbt"), width (or sections × sectionWidth in cm), overlap (a gap is negative), offset (the working
 // line's centre right of the machine's, left is negative: Tool offset), axleToTool (rigid: the pivot axle to the working
 // line, Working center), hitchLength (rear axle to the hitch pin: the Hitch length of the Vehicle page), tankHitch (TBT:
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
  // the working line from the pivot axle, along the machine
  const axleToTool = trailed ? i.hitchLength + (tbt ? i.tankHitch : 0) + i.drawbar + i.pivotToTool : i.axleToTool;

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
  const kturn = semicircle ? null : r1(kTurnDepth(radius));

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
   // the rearmost corner of a rigid implement: its length from the 3-point hitch (a front implement: from its attachment)
   const along = Math.max(axleToTool, length > 0 ? (front ? 0 : i.hitchLength) + length : 0);
   toolReach = Math.sqrt(Math.pow(radius + halfOut, 2) + along * along) - radius;
   toolWhy = "the " + (front ? "front " : "") + "implement's outer " + (along > axleToTool ? "rear corner" : "end") + " " + r1(toolReach) + " m (√((" + r1(radius) + " + " + r1(halfOut) + ")² + " + r1(along) + "²) − " + r1(radius) + ": " + outName + ", " + (along > axleToTool ? "its length behind the hitch" : "the working line " + (front ? "ahead of" : "behind") + " the axle") + ")";
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

  // the headland: the turn's depth, the distance from boundary and the implement behind the tractor, in whole passes
  const behind = front ? 0 : r1(axleToTool);   // a front implement is ahead of the pivot: it enters the headland before the turn starts
  const headland = depth => { const sum = r1(depth) + distance + behind; const passes = Math.ceil(sum / spacing - 0.001); return {sum: r1(sum), passes, width: r1(passes * spacing)}; };
  const options = [];
  if (semicircle) options.push({key: "semicircle", style: "Sagitta", skip: 0, depth: r1(radius), headland: headland(radius),
   text: "Sagitta, skip 0: the next pass is " + r1(spacing) + " m away, 2 × " + r1(radius) + " m or more, so the turn is a plain semicircle " + r1(radius) + " m deep"});
  else {
   options.push({key: "skip", style: "Sagitta", skip, depth: r1(radius), headland: headland(radius),
    text: "Sagitta, skip " + skip + " row" + (skip > 1 ? "s" : "") + ": the turn lands " + r1(landing) + " m away, 2 × " + r1(radius) + " m or more, a plain semicircle " + r1(radius) + " m deep; the skipped passes are worked on the way back"});
   options.push({key: "keyhole", style: "Keyhole", skip: 0, depth: keyhole, headland: headland(keyhole),
    text: "Keyhole, skip 0: the next pass is only " + r1(spacing) + " m away, closer than 2 × " + r1(radius) + " m, so the turn loops " + keyhole + " m deep (a Sagitta turn would go " + sagitta + " m deep and swing further out)"});
   options.push({key: "kturn", style: "K-turn", skip: 0, depth: kturn, headland: headland(kturn), reverse: true,
    text: "K-turn, skip 0: AgOpenWeb drives a 126° arc forward, " + r1(radius * Math.sin(K_TURN_ARC)) + " m deep, and a straight tail of " + r1(1.5 * spacing) + " m; then you stop and back up by hand (the turn is over for AgOpenWeb as soon as you reverse, and guidance is on the next pass). Backing up through the remaining 54° at the same radius goes another " + r1(radius * Math.sin(K_TURN_ARC)) + " m in: " + kturn + " m deep in all, with a stop and a reverse at every pass" + (trailed ? "; a trailed implement is hard to back up" : "")});
  }
  const extension = up(Math.max(2 * radius, trailed ? 3 * axleToTool : 0), 0.5);
  const extensionWhy = !trailed || 2 * radius >= 3 * axleToTool ? "2 × the radius" : "3 × the " + r1(axleToTool) + " m from the rear axle to the working line, so the " + (tbt ? "TBT" : "trailed") + " implement is straight behind the tractor before the sections come on";
  let tram = null;
  if (i.sprayerWidth > 0) {
   const passes = i.sprayerWidth / spacing, whole = Math.abs(passes - Math.round(passes)) < 0.01;
   tram = {width: i.sprayerWidth, passes: whole ? Math.round(passes) : r1(passes), whole};
  }

  const rows = [
   {key: "spacing", name: "Track spacing (not a setting)", value: r1(spacing) + " m", why: "implement width " + i.width + " m − overlap " + i.overlap + " m; AgOpenWeb works it out itself"},
   {key: "rMin", name: "Tightest turn of the machine (not a setting)", value: r1(rMin) + " m", why: rMinWhy},
   {key: "radius", name: "U-Turn → Radius", value: r1(radius) + " m", why: why.join("; ")},
   {key: "style", name: "U-Turn → Turn style and skip rows", value: options.map(o => o.style + ", skip " + o.skip).join(" or "), why: options.map(o => o.text).join(". ")},
   {key: "distance", name: "U-Turn → Distance from boundary", value: distance + " m", why: distanceWhy},
   {key: "headland", name: "Headland width (Field builder → Headland)", value: options.map(o => o.headland.width + " m (" + o.headland.passes + " pass" + (o.headland.passes > 1 ? "es" : "") + ")").join(" or "),
    why: options.map(o => (options.length > 1 ? o.style + ", skip " + o.skip + ": " : "") + "turn depth " + o.depth + " + distance from boundary " + distance + (front ? "" : " + " + behind + " m of implement behind the axle") + " = " + o.headland.sum + " m, rounded up to whole passes" + (front ? "; the front implement is ahead of the pivot and enters the headland first" : "")).join(". ")},
   {key: "extension", name: "U-Turn → Extension", value: extension + " m", why: extensionWhy + ": straight legs into and out of the turn"},
   {key: "goAround", name: "U-Turn → Going around obstacles", value: "radius " + r1(radius) + " m, margin " + GO_AROUND_MARGIN + " m", why: "the same radius as the U-turn; AgOpenWeb refuses anything tighter than the tractor's tightest turn"},
  ];
  if (tram) rows.push({key: "tram", name: "Tram → Tram width and passes", value: tram.whole ? tram.width + " m, every " + tram.passes + " pass" + (tram.passes > 1 ? "es" : "") : "no match",
   why: tram.whole ? "sprayer width ÷ track spacing" : "sprayer width " + tram.width + " ÷ track spacing " + r1(spacing) + " = " + tram.passes + ", not a whole number: the sprayer's wheels would not land on the same tracks every pass"});
  if (!follows) problems.push("The drawbar is longer than the hitch's turn radius: the implement cannot follow this turn. Set a bigger radius or a shorter drawbar.");
  return {problems, rows, machine: i.machine, spacing: r1(spacing), rMin: r1(rMin), radius: r1(radius), semicircle, skip, landing: r1(landing), keyhole, sagitta, kturn,
   bodyName, noseReach: r1(noseReach), toolReach: r1(toolReach), reach: r1(reach), margin: r1(margin), distance, axleToTool: r1(axleToTool), behind, toolRadius: r1(toolRadius), halfOut: r1(halfOut), extension, options, tram,
   goAround: {radius: r1(radius), margin: GO_AROUND_MARGIN}};
 }

 const api = {suggest, keyholeDepth, keyholePath, sagittaPath, kTurnPath, kTurnDepth, depthOf, RADIUS_FACTOR, RADIUS_FLOOR, MARGIN, MAX_HITCH_DEG, K_TURN_ARC};
 if (typeof module !== "undefined" && module.exports) module.exports = api; else root.AowSettings = api;
})(typeof window !== "undefined" ? window : globalThis);
