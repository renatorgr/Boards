# LED comb for the AiO v4.5 boards

A small 3D-printed comb that keeps the light of the three front LEDs of the AiO v4.5 Micro and Standard (PWR/ETH, GPS/RTK, AS-ACT) apart. On the board the LEDs stand upright at the edge, 6.6 mm apart, so only 1.6 mm separates one dome from the next. In an enclosure with light windows on the front panel (the RayCorp case, for one) the light reaches each window from the side of the domes and spreads to the windows next to it: an orange LED beside a red one cannot be told apart. The comb slides down over the three LEDs and gives each one a cell that is closed on the top, the back and both sides, open only towards the panel, with the dividers reaching the panel.

| File | What |
|---|---|
| `led_bezel.scad` | The model, in [OpenSCAD](https://openscad.org/). Every size is a number at the top of the file. |
| `led_bezel.stl` | The model ready to slice and print, as the `.scad` is set. |
| `led_bezel.png`, `led_bezel_front.png` | What it looks like, from the panel side and from below. |

![LED comb, from the panel side](led_bezel.png)

![LED comb, from below](led_bezel_front.png)

## How it stays in place

Hand-soldered LEDs never stand exactly on their centres, so the cells are 0.3 mm wider than the domes all round and three small ribs in each cell (0.25 mm, one at the back and one on each side) take up the difference and grip the dome.

Two pegs under the back wall go into two free plated holes of the board, 1.0 mm, right behind the end LEDs: the near pads of the through-hole resistors R32 and R36 of the AiO v4.5 Micro. They stay empty because the board is built with the SMD resistors (R29, R39) instead. The pegs hold the comb against sliding along the row and give it the right place even when the LEDs are a little off. They are 0.9 mm thick and 1.4 mm long, so they do not come out under the board.

If the pegs are not wanted (another board, or holes that are not free), set `pegs = false` and, for tape or glue, give `foot` a width such as 2 mm: that adds a thin plate behind the back wall to stick to the board with double-sided tape or a drop of hot glue. A drop of glue on the back wall alone does the same.

## Sizes as set

- Three cells of 5.6 mm, 6.6 mm apart (the pitch of the LEDs on both boards, from the pick-and-place files of the [official AiO v4.5](https://github.com/AgOpenGPS-Official/Boards/tree/main/TeensyModules)), 10 mm deep under a 1 mm roof, with three 0.25 mm grip ribs each.
- A 6.2 mm counterbore, 1.5 mm high, at the bottom of each cell for the LED's base flange.
- Dividers 1.0 mm thick (what the pitch leaves), back wall 2.2 mm with the two pegs in it, end walls 0.8 mm at the PWR/ETH end and 0.7 mm at the AS-ACT end: on the Micro the RJ45 shield is about 1 mm from the AS-ACT dome and the fuse holder about 1.5 mm from the PWR/ETH dome.
- The front face 3.0 mm from the LED centres, that is at the board edge (3.05 mm from the centres on the Micro, 3.3 mm on the Standard). In the RayCorp case the board edge touches the front panel, so the dividers end against the panel.
- Pegs 7.17 mm from the board edge, 0.63 mm past the AS-ACT LED and 0.76 mm past the PWR/ETH LED along the row (the Micro drill file).

Overall about 20 x 8 x 11 mm, plus the pegs.

## Before printing: check two things

- If there is a gap between the board edge and the inside of the front panel in your case, add it to `front` so that the dividers just touch the panel. Dividers that stop short of the panel let the light leak across again; dividers that are too long keep the comb from sitting on the board, so a bit short beats a bit long.
- On the Standard, or on a Micro with the through-hole resistors fitted, the pegs have nowhere to go: set `pegs = false`.

## Printing and fitting

- Print in **black or another opaque** filament, roof down on the bed, no supports. Opaque matters: the walls are there to stop one LED lighting the next window. A 0.4 mm nozzle and 0.2 mm layers are fine; the pegs are small, so slow down the last layers or print two combs at once.
- Hold the comb over the three LEDs, open side towards the panel, pegs towards the inside of the board, and push it down until it rests on the LED flanges and the pegs are in their holes. The ribs should grip; if the comb is hard to push on, pass a round file through the cells or set `rib = 0.15`. If the pegs will not go in, sand them a touch or set `peg_d = 0.8`.
- If it still moves, a drop of hot glue between the back wall and the board holds it and comes off again.
- If it touches the RJ45 or the fuse holder, set `end_wall_right` or `end_wall_left` to 0.5, or sand that end.

## Without a printer

Wrap each dome with a strip of black insulating tape, about 8 mm wide, around the back and both sides, leaving only the side that faces the panel uncovered. Each LED then lights only its own window. A strip of black card folded into a comb between the LEDs and the panel does the same.

## Licence

CERN Open Hardware Licence Version 2 - Strongly Reciprocal (CERN-OHL-S v2) or later, as the AgOpenGPS boards it fits. No warranty.
