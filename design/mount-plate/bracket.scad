// Angle bracket for the USB-to-vacuum dongle, glued to the plate.
// The 4-pin 1/2" panel connector goes through the hole in the upright face.
// Origin: outside corner of the angle; base on z=0, upright face at y=0 going up.
$fn = 128;

W      = 22;     // width of the bracket (along the angle)
H      = 20;     // height of the upright face
D      = 20;     // depth of the base leg
TH     = 3;      // wall thickness
HOLE_D = 12.74;  // connector hole
FIL    = 3;      // inside fillet between the legs, for strength

// hole centred on the free part of the face (above the base)
HOLE_Z = TH + (H - TH) / 2;

difference() {
    union() {
        cube([W, D, TH]);                 // base
        cube([W, TH, H]);                 // upright
        // inside fillet
        translate([0, TH, TH]) rotate([0, 90, 0]) rotate([0, 0, 90])
            linear_extrude(W) difference() {
                square([FIL, FIL]);
                translate([FIL, FIL]) circle(r = FIL);
            }
    }
    translate([W / 2, -1, HOLE_Z]) rotate([-90, 0, 0]) cylinder(d = HOLE_D, h = TH + 2);
}
