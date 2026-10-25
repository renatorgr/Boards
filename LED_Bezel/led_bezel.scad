// led_bezel.scad - a hood that separates the three 5 mm LEDs on the front of the AiO v4.5 boards.
// Part of renatorgr/Boards. Written 2026 by Claude (Anthropic) for renatorgr.
// Licence: CERN-OHL-S v2 or later, as the AgOpenGPS boards it fits. WITHOUT ANY WARRANTY.
//
// The AiO v4.5 Micro and Standard carry three two-colour 5 mm LEDs (PWR/ETH, GPS/RTK, AS-ACT) 6.6 mm
// apart, so only 1.6 mm separates one dome from the next and the colours blend when you look at the
// panel. This hood gives each LED its own tube and a name. Print it in black (opaque) filament, slide
// it over the three domes, or glue the flange to the panel if the domes do not stick out enough.
//
// Measurements from the AiO v4.5 pick-and-place files (AgOpenGPS-Official/Boards): LED pitch 6.6 mm,
// LED DY-339URUYGW/SN, 5 mm dome. Change the numbers below to fit your panel.

pitch      = 6.6;    // centre to centre of the LEDs (mm)
hole       = 5.4;    // hole for a 5 mm dome; 5.6 if the print comes out tight
tube       = 7;      // how far the tubes stand out from the flange (mm)
end_wall   = 1.6;    // wall at both ends of the row
top_wall   = 1.6;    // wall above and below the holes
flange     = 1.2;    // thickness of the back plate (glue face)
flange_rim = 2;      // how far the flange sticks out around the tubes (0 = no flange)
lip        = 4;      // extra height under the holes for the names (0 = no names)
names      = ["PWR", "GPS", "STR"];   // left to right, as on the board: PWR/ETH, GPS/RTK, AS-ACT (steer); 3 letters fit the 6.6 mm
text_h     = 1.8;    // letter height (mm)
text_depth = 0.5;    // letters stand out this much
$fn = 48;

n      = len(names);
width  = (n - 1) * pitch + hole + 2 * end_wall;
height = hole + 2 * top_wall + lip;
cy     = lip + top_wall + hole / 2;          // centre line of the holes, from the bottom edge

module body() {
    // flange (back plate) and the block with the tubes
    translate([-flange_rim, -flange_rim, 0]) cube([width + 2 * flange_rim, height + 2 * flange_rim, flange]);
    cube([width, height, flange + tube]);
}

module holes() {
    for (i = [0 : n - 1])
        translate([end_wall + hole / 2 + i * pitch, cy, -1]) cylinder(d = hole, h = flange + tube + 2);
}

module labels() {
    if (lip > 0)
        for (i = [0 : n - 1])
            translate([end_wall + hole / 2 + i * pitch, lip / 2, flange + tube])
                linear_extrude(text_depth)
                    text(names[i], size = text_h, halign = "center", valign = "center", font = "Liberation Sans:style=Bold");
}

difference() { body(); holes(); }
labels();
