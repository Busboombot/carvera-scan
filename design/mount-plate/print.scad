// Print layout: the plate on its back face, the bracket on its holed face.
translate([0, 0, 5]) import("plate.stl");
translate([55, -11, 0]) rotate([90, 0, 0]) translate([0, 0, 0]) import("bracket.stl");
