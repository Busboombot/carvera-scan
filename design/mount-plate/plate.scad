// Mounting plate, from Eric's hand measurements (2026-11-04), for comparison with the scan.
// Origin: plate centre on the front face; front face at z=0, back face at z=-T.
// X along the long side, Y along the short side, +Y = "top" edge.
$fn = 96;

L  = 82.7;   // long dimension
W  = 45.7;   // narrow dimension
T  = 5.0;    // thickness
R  = 10;     // corner radius (guess)

// Pocket on the back, centred on the top long edge, open to that edge
P_W = 25;    // along the edge
P_D = 11;    // in from the edge
P_THROUGH = true;  // pocket cut all the way through, leaving a notch
P_Z = P_THROUGH ? T + 0.02 : 3.3;   // into the thickness
P_R = 2;     // radius of the two inside corners
P_RO = 2;    // radius of the two outside corners where a through notch meets the edge

// Two counterbored holes on the long centreline, 50 mm c-c
H_PITCH = 50;
H_D  = 4.2;  // through hole
CB_D = 8.5;  // counterbore
CB_Z = 3.2;  // counterbore depth
CB_FRONT = true;   // counterbore opens on the front face (screw heads flush there)

e = 0.01;

module outline() offset(r = R) square([L - 2*R, W - 2*R], center = true);

module pocket() hull() {
    for (sx = [-1, 1])
        translate([sx * (P_W/2 - P_R), W/2 - P_D + P_R]) circle(r = P_R);
    translate([-P_W/2, W/2 - P_D + P_R]) square([P_W, P_D - P_R + 1]);
}

difference() {
    if (P_THROUGH)
        // notch in the outline; the opening rounds the convex corners at the edge to P_RO
        // and leaves the concave ones (already P_R) and the R corners alone
        translate([0, 0, -T]) linear_extrude(T)
            offset(r = P_RO) offset(delta = -P_RO) difference() { outline(); pocket(); }
    else {
        translate([0, 0, -T]) linear_extrude(T) outline();
        // back pocket, inner corners rounded to P_R (open edge stays square)
        translate([0, 0, -T - e]) linear_extrude(P_Z + e) pocket();
    }

    for (sx = [-1, 1]) translate([sx * H_PITCH/2, 0, 0]) {
        translate([0, 0, -T - e]) cylinder(d = H_D, h = T + 2*e);
        if (CB_FRONT) translate([0, 0, -CB_Z]) cylinder(d = CB_D, h = CB_Z + e);
        else          translate([0, 0, -T - e]) cylinder(d = CB_D, h = CB_Z + e);
    }
}
