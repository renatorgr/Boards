// led_bezel.scad - a comb that keeps the light of the three front LEDs of the AiO v4.5 apart.
// Part of renatorgr/Boards. Written 2026 by Claude (Anthropic) for renatorgr.
// Licence: CERN-OHL-S v2 or later, as the AgOpenGPS boards it fits. WITHOUT ANY WARRANTY.
//
// The AiO v4.5 Micro and Standard carry three two-colour 5 mm LEDs (PWR/ETH, GPS/RTK, AS-ACT) standing
// upright at the board edge, 6.6 mm apart, so only 1.6 mm separates one dome from the next. In an
// enclosure with light windows on the front panel (e.g. the RayCorp case), the light reaches each window
// from the side of the domes and spreads to the neighbouring windows, so an orange LED next to a red one
// cannot be told apart. This comb slides down over the three LEDs: each LED gets its own cell, closed on
// the top, the back and the sides, open only towards the panel, and the dividers reach the panel.
// Print it in black (opaque) filament.
//
// Measurements from the AiO v4.5 pick-and-place files (AgOpenGPS-Official/Boards): LED pitch 6.6 mm,
// LED DY-339URUYGW/SN (5 mm dome, 5.8 mm flange), LED centres 3.05 mm (Micro) / 3.3 mm (Standard) from
// the board edge; on the Micro the RJ45 shield is about 1 mm from the AS-ACT LED and the fuse holder about
// 1.5 mm from the PWR/ETH LED, so both end walls are thin. Change the numbers below
// to fit your board and panel.

pitch          = 6.6;    // centre to centre of the LEDs (mm)
hole           = 5.4;    // cell diameter for a 5 mm dome; 5.6 if the print comes out tight
flange_d       = 6.2;    // counterbore at the bottom for the LED's base flange (5.8 mm)
flange_h       = 1.5;    // height of that counterbore
led_h          = 10;     // top of the domes above the board (mm): the cells are this deep
roof           = 1;      // closed top, this thick (0 = open top)
back_wall      = 1.2;    // wall behind the LEDs (towards the inside of the board)
end_wall_left  = 0.8;    // wall at the PWR/ETH end (the fuse holder is about 1.5 mm away on the Micro)
end_wall_right = 0.8;    // wall at the AS-ACT end (the RJ45 shield is about 1 mm away on the Micro)
front          = 3.0;    // from the LED centres to the front face, towards the panel (mm). The board edge
                         // is 3.05 mm from the centres on the Micro (3.3 on the Standard); in the RayCorp
                         // case the board edge touches the panel, so 3.0 puts the front face at the edge,
                         // against the panel. With a gap between the board edge and the panel, add it here.
n              = 3;
$fn = 48;

width  = (n - 1) * pitch + hole + end_wall_left + end_wall_right;
depth  = front + hole / 2 + back_wall;          // along the LED-to-panel direction
height = led_h + roof;
cx     = front;                                  // LED centre line, from the front face

module body() { cube([depth, width, height]); }

module cells() {
    for (i = [0 : n - 1]) {
        y = end_wall_right + hole / 2 + i * pitch;
        // the round cell around the dome, from the bottom up to the roof
        translate([cx, y, -1]) cylinder(d = hole, h = led_h + 1);
        // the counterbore for the base flange
        translate([cx, y, -1]) cylinder(d = flange_d, h = flange_h + 1);
        // the opening towards the panel, as wide as the cell, full height under the roof
        translate([-1, y - hole / 2, -1]) cube([cx + 1, hole, led_h + 1]);
    }
}

difference() { body(); cells(); }
