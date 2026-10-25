# LED bezel for the AiO v4.5 boards

A small 3D-printed hood that separates the three 5 mm LEDs on the front of the AiO v4.5 Micro and Standard (PWR/ETH, GPS/RTK, AS-ACT). On the board the LEDs sit 6.6 mm apart, so only 1.6 mm separates one dome from the next, and on the panel the colours blend: an orange LED next to a red one is hard to tell apart. The hood gives each LED its own tube and a name.

| File | What |
|---|---|
| `led_bezel.scad` | The model, in [OpenSCAD](https://openscad.org/). Every size is a number at the top of the file. |
| `led_bezel.stl` | The model ready to slice and print, as the `.scad` is set. |
| `led_bezel.png` | What it looks like. |

![LED bezel](led_bezel.png)

## Sizes as set

- Three holes of 5.4 mm, 6.6 mm apart (the pitch of the LEDs on both boards, from the pick-and-place files of the [official AiO v4.5](https://github.com/AgOpenGPS-Official/Boards/tree/main/TeensyModules)).
- Tubes 7 mm deep, walls 1.6 mm at the ends, 1.2 mm between the holes (what the pitch leaves).
- A 1.2 mm back plate 2 mm wider than the tubes, to glue to the panel.
- A 4 mm strip under the holes with the names `PWR`, `GPS`, `STR` in raised letters.

Overall about 24 x 14 mm, 8 mm deep.

## Printing and fitting

- Print in **black or another opaque** filament, tubes facing up, no supports. Opaque matters: the walls are there to stop one LED lighting the next.
- A 0.4 mm nozzle prints the letters; if they come out blurred, set `text_h` to 2.2 and `lip` to 5 (the names then nearly touch), or set `lip` to 0 for no names.
- If the LED domes stick out of the panel 4 mm or more, the hood slides over them. If they do not, glue the back plate to the panel around the holes (the holes line up with the domes).
- A 5 mm dome should enter a 5.4 mm hole; a tight print takes `hole = 5.6`.

## Without a printer

A short piece of black heat-shrink tube (6.4 mm, about 5 mm long) over each dome does the same job: shrink it lightly so it stays, and the light of each LED only comes out of its own end.

## Licence

CERN Open Hardware Licence Version 2 - Strongly Reciprocal (CERN-OHL-S v2) or later, as the AgOpenGPS boards it fits. No warranty.
