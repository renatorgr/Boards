// fake-boards.js - self-test of hand-tests.ps1 with no boards: one pretend board on http://localhost:18090 that answers
// what the Teensy, the XIAO (and its port 8080) and the sprayer module answer, enough for each test to run to its end.
//   node fake-boards.js <test> [seconds]      (test: 7, 17, 42, 44, 46 or 48)
// It does not prove the boards right: the harnesses in Tests/ do that. It shows the script runs and reads what it should.
// SPDX-License-Identifier: GPL-3.0-or-later
'use strict';
const http = require('http');
const mode = process.argv[2] || '42';
const life = Number(process.argv[3] || 300) * 1000;
const t0 = Date.now();
const sec = () => (Date.now() - t0) / 1000;
let upBase = 500, upT = t0;                 // "up" restarts from 0 after a pretend power cut
const up = () => Math.floor(upBase + (Date.now() - upT) / 1000);
const xev = [{ s: 1, t: 10, x: 'Program started (power on)' }];
const tlog = [{ s: 1, t: 10, x: '[status] GPS receiver: temperature 31 C (UM982 HWSTATUSA, first field 31234, read as thousandths of a degree)' }];
let captiveDone = false, total = 12.3, start = 'power on', startBad = false, offUntil = 0, n239 = 0, lastN = Date.now(), cuts = 0;

function sprayer() {
  const s = sec();
  let on = '7', req = '7', cut = false;
  if (mode === '17') {                       // the program goes silent from 6 s to 14 s of every 20 s
    const k = s % 20;
    cut = k >= 6 && k < 14;
    if (k >= 8 && k < 14.5) on = '0';
  }
  if (mode === '44') { const k = s % 30; if (k < 12) { on = '0'; req = '0'; } }
  if (!cut) n239 += Math.round((Date.now() - lastN) / 100);
  lastN = Date.now();
  if (on !== '0') total += 0.01;
  return {
    fw: 'Sprayer_ESP32', ver: '0.10.16', built: 'Nov 22 2026 17:01:00', board: 'ESP32 relay board', kind: 'Sprayer',
    up: up(), heap: 150000, start, startBad, name: 'sprayer',
    wifi: { ap: 'Sprayer', apIp: '192.168.4.1', up: true, ip: '192.168.5.123', subnet: '192.168.5' },
    aog: { link: !cut, via: '', ago239: cut ? 3 : 0, n239, req }, out: { bits: 8, on, noRelay: '0' },
    spr: { bar: 2.1, lpm: 12.0, total: +total.toFixed(1), master: true, volts: 1.2, pulses: 1234, unplugged: false },
    log: ['start: ' + start],
  };
}
function xiaoState() {
  const steer = mode === '46';
  return {
    fw: 'v4.5_XIAO_ESP32C3', ver: '4.5', chip: 'ESP32-C3', built: 'Nov 23 2026 04:43:00', up: up(), reset: 'power on', resetBad: false,
    heap: 100000, heapMin: 60000, wifi: { ap: 'XIAO-TEST', apIp: '192.168.5.1', apCl: 1, staOn: false },
    bridge: { on: true, teensy: true, state: 2, prog: '192.168.5.2', steer, in: Math.floor(sec() * 50), out: Math.floor(sec() * 30), lost: 0, mods: [] },
    captive: { ap: true, done: captiveDone, n: captiveDone ? 1 : 0 },
    tpage: { ready: true, why: '', req: 3, refused: 0, broken: 0, lowMem: 0 },
    link: { talking: true, lastAgo: 0, frames: 1000 },
  };
}
function teensyStatus() {
  return { fw: 'v4.5_AiO_WEB 4.5', built: 'Nov 22 2026 19:00:00', up: up(), steer: { active: mode === '46', set: 1.5, was: 1.4 } };
}
const PAGE = '<html><body>' + ['status', 'log', 'settings', 'gps', 'imu', 'machine', 'firmware'].map(t => `<a data-t="${t}">${t}</a>`).join('') +
  ' module page: http://192.168.5.1:8080/ </body></html>';

function send(res, code, obj) { const b = typeof obj === 'string' ? obj : JSON.stringify(obj); res.writeHead(code, { 'Content-Type': typeof obj === 'string' ? 'text/html' : 'application/json', 'Connection': 'close' }); res.end(b); }

const srv = http.createServer((req, res) => {
  const u = new URL(req.url, 'http://x');
  const p = u.pathname;
  if (mode === '44' && Date.now() < offUntil) { req.socket.destroy(); return; }
  // a pretend power cut: the first (test 44 b) while the sections are off (6-7 s of 30), the next (c) while spraying (24-25 s)
  const k = sec() % 30, cutNow = cuts === 0 ? k > 6 && k < 7 : k > 24 && k < 25;
  if (mode === '44' && p === '/api/status' && cutNow) {
    const sprayingAtCut = sprayer().out.on !== '0';
    cuts++;
    offUntil = Date.now() + 3000; upBase = 0; upT = offUntil;
    start = sprayingAtCut ? 'power on while spraying (power cut?)' : 'power on'; startBad = sprayingAtCut;
    req.socket.destroy(); return;
  }
  if (req.method === 'POST' && p === '/update') {
    let got = 0;
    req.on('data', d => { got += d.length; });
    req.on('close', () => { xev.push({ s: xev.length + 1, t: Date.now() - t0, x: `Program update STOPPED after ${Math.floor(got / 1024)} kB: the browser's connection dropped (weak Wi-Fi?), old program kept` }); });
    return;                                   // never answers: the script stalls, then closes
  }
  let body = '';
  req.on('data', d => { body += d; });
  req.on('end', () => {
    if (p === '/') return send(res, 200, PAGE);
    if (p === '/api/state') return send(res, 200, xiaoState());
    if (p === '/api/log') {
      const after = Number(u.searchParams.get('after') || 0), xafter = Number(u.searchParams.get('xafter') || 0);
      return send(res, 200, { epoch: 1, last: tlog.length, now: Date.now() - t0, t: tlog.filter(l => l.s > after), xlast: xev.length, x: xev.filter(l => l.s > xafter), more: false, next: tlog.length + 1, lines: tlog.filter(l => l.s > after) });
    }
    if (p === '/api/status') return send(res, 200, mode === '17' || mode === '44' ? sprayer() : teensyStatus());
    if (p === '/api/gps') return send(res, 200, { ok: true, kind: 'um982', model: 'UM982', temp: Math.round(31 + sec() / 2), tempSt: 'ok' });
    if (p === '/api/settings') return send(res, 200, { settings: {} });
    if (p === '/api/captive' && req.method === 'POST') { captiveDone = true; return send(res, 200, { ok: true }); }
    if (p === '/selftest/dns') return send(res, 200, { a: captiveDone ? null : '192.168.5.1' });   // what the XIAO's DNS tells Windows' check
    if (p === '/api/ota' || p === '/api/revert') return send(res, 403, { error: 'not through the XIAO: use the XIAO page, Firmware tab' });
    if (p === '/api/total' && req.method === 'POST') { total = 0; return send(res, 200, { ok: true }); }
    if (p === '/api/restart' && req.method === 'POST') { offUntil = Date.now() + 2000; upBase = 0; upT = offUntil; start = 'restart asked'; startBad = false; return send(res, 200, { ok: true, restart: true }); }
    send(res, 404, { error: 'no such page' });
  });
});
srv.listen(18090, '127.0.0.1', () => console.log(`fake boards for test ${mode} on http://localhost:18090 for ${life / 1000} s`));
setTimeout(() => process.exit(0), life);
