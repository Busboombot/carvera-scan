"""Cutebot Pro mounting features, in the robot's own frame, from robot_ortho.py
images (camera A) and robot_laser.py heights.

    python projects/cutebot-pro/robot_features.py data/2026-11-04/robot1

Robot frame: +x to the robot's right, +y forward (toward the sonar), +z up,
mm. Every boss and hole lies on one 8 mm (LEGO) grid, so the frame sits on it:
origin on the bed at the grid point on the centreline in line with the middle
upper holes, ~0.4 mm from the wheel axle; positions are reported snapped to the
grid (and as measured).

- Lower-deck bosses (4): circles in the top-down image at the lower deck's height.
- Upper-deck holes: two rows of 7 beside the battery (rear screw, hole, boss,
  hole, boss, hole, front screw), fitted as a regular row.
- Battery box: edges of the top-down image at its laser height.
- micro:bit: the top-down image is sharpest at 67-68 mm on its USB and JST
  connectors (both cameras); the laser points above that around the board are
  reflections off it, so its laser maximum (79 mm) is not used.
Writes OUT/features.json, OUT/cutebot.scad, OUT/features.png.
"""
import argparse
import json
import os

import cv2
import numpy as np
from scipy.optimize import least_squares

LOWER = (-140, -230, 0.1)  # robot_ortho views: X0, Y0, res
UPPER = (-105, -160, 0.1)
BOSS_SEEDS = {"front_right": (-84.1, -204.9), "front_left": (-67.7, -204.7),
              "side_right": (-107.9, -189.1), "side_left": (-44.5, -188.3)}
ROW_SEEDS = {"right": -92.5, "left": -60.3}
ROW_Y = [-101.0, -108.5, -116.75, -125.0, -132.75, -141.0, -149.0]  # rear to front
ROW_KIND = ["screw", "hole", "boss", "hole", "boss", "hole", "screw"]
BATTERY = {"x": (-86.9, -69.1), "y": (-162.5, -91.5)}  # ortho_ridge edges (camera A)
GRID = 8.0  # LEGO pitch: every boss and hole is on this grid
MICROBIT = {"top": 68.0, "y": -176.5, "usb_x": -75.2, "width": 51.6, "thick": 1.6}  # focus on USB/JST, A and B


def gray(img):
    v = img[np.isfinite(img)]
    lo, hi = np.percentile(v, [1, 99.5])
    return np.clip((np.nan_to_num(img, nan=lo) - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)


def circle(g, view, seed, w=4.5, rmin=1.8, rmax=4.0):
    x0, y0, res = view
    cx, cy = (seed[0] - x0) / res, (seed[1] - y0) / res
    n = int(w / res)
    a, b = int(cx - n), int(cy - n)
    cs = cv2.HoughCircles(g[b:b + 2 * n, a:a + 2 * n], cv2.HOUGH_GRADIENT, 1, 100, param1=40, param2=8,
                          minRadius=int(rmin / res), maxRadius=int(rmax / res))
    if cs is None:
        return None
    c = cs[0][0]
    return x0 + (a + c[0]) * res, y0 + (b + c[1]) * res, 2 * c[2] * res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    a = ap.parse_args()
    las = json.load(open(os.path.join(a.dir, "laser.json")))

    gl = cv2.GaussianBlur(gray(np.load(os.path.join(a.dir, "ortho_lower_A.npy"))), (0, 0), 1.5)
    gu = cv2.GaussianBlur(gray(np.load(os.path.join(a.dir, "ortho_upper_A.npy"))), (0, 0), 1.5)
    bosses = {k: circle(gl, LOWER, s, rmax=4.0) for k, s in BOSS_SEEDS.items()}
    rows = {}
    for side, x in ROW_SEEDS.items():
        found = [(k, circle(gu, UPPER, (x, y), rmax=3.6)) for k, y in enumerate(ROW_Y)]
        found = [(k, c) for k, c in found if c is not None and abs(c[0] - x) < 2 and abs(c[1] - ROW_Y[k]) < 2]
        k = np.array([f[0] for f in found], float)
        ys = np.array([f[1][1] for f in found])
        pitch, y3 = np.polyfit(k - 3, ys, 1)
        rows[side] = {"x": float(np.mean([f[1][0] for f in found])), "y3": y3, "pitch": pitch, "n": len(found),
                      "rms": float(np.std(ys - (y3 + pitch * (k - 3))))}
    pitch = np.mean([r["pitch"] for r in rows.values()])
    y3 = np.mean([r["y3"] for r in rows.values()])

    # robot frame: centreline from the boss pairs, hole rows and wheels; origin under the axle
    sl, sr = np.array(bosses["side_left"][:2]), np.array(bosses["side_right"][:2])
    left = (sl - sr) / np.linalg.norm(sl - sr)
    right, fwd = -left, np.array([left[1], -left[0]])
    wl, wr = las["wheel_L"], las["wheel_R"]  # L/R here = machine -X/+X
    xc = np.mean([(bosses["side_left"][0] + bosses["side_right"][0]) / 2,
                  (bosses["front_left"][0] + bosses["front_right"][0]) / 2,
                  (rows["left"]["x"] + rows["right"]["x"]) / 2,
                  (np.mean(wl["x"]) + np.mean(wr["x"])) / 2])
    yc = (wl["axle_y"] + wr["axle_y"]) / 2
    O = np.array([xc, yc])

    # Every boss and hole sits on one 8 mm (LEGO) grid. Fit it (offset and
    # rotation, pitch fixed) and move the frame onto it: origin at the grid
    # point on the centreline in line with the middle holes (L4/R4).
    meas = {("boss", k): c[:2] for k, c in bosses.items()}
    for side in ("left", "right"):
        for k in range(7):
            meas[(side, k)] = (rows[side]["x"], y3 + pitch * (k - 3))
    keys = list(meas)
    P = np.array([[(np.array(meas[k]) - O) @ right, (np.array(meas[k]) - O) @ fwd] for k in keys])
    g0 = (P[keys.index(("left", 3))] + P[keys.index(("right", 3))]) / 2
    idx = np.round((P - g0) / GRID)

    def lattice(p, s=GRID):
        R = np.array([[np.cos(p[2]), -np.sin(p[2])], [np.sin(p[2]), np.cos(p[2])]])
        return p[:2] + s * idx @ R.T

    p = least_squares(lambda p: (lattice(p) - P).ravel(), [*g0, 0.0]).x
    d = np.hypot(*(lattice(p) - P).T)
    free = least_squares(lambda q: (lattice(q[:3], q[3]) - P).ravel(), [*p, GRID]).x
    O = O + p[0] * right + p[1] * fwd
    right, fwd = np.cos(p[2]) * right + np.sin(p[2]) * fwd, -np.sin(p[2]) * right + np.cos(p[2]) * fwd
    grid = {k: (int(i), int(j)) for k, (i, j) in zip(keys, idx)}
    left = -right

    def rf(X, Y):
        p = np.array([X, Y]) - O
        return round(float(p @ right), 2), round(float(p @ fwd), 2)

    deck_lo = las["lower_deck"]
    deck_hi = (las["upper_deck_L"] + las["upper_deck_R"]) / 2

    def on_grid(key, X, Y, z, **extra):
        i, j = grid[key]
        return {"grid": [i, j], "xy": [GRID * i, GRID * j], "xy_measured": rf(X, Y), "z": round(z, 1), **extra}

    out = {"frame": "8 mm LEGO grid: origin on the bed at the grid point on the centreline in line with the middle "
                    "upper holes (L4/R4), ~0.4 mm from the wheel axle; +x robot's right, +y forward, +z up; mm",
           "grid": {"pitch": GRID, "features": len(keys), "rms": round(float(np.sqrt(np.mean(d ** 2))), 2),
                    "max": round(float(d.max()), 2), "free_pitch": round(float(free[3]), 3),
                    "note": "xy = pitch * grid (snapped); xy_measured = as scanned"},
           "machine_origin": [round(float(O[0]), 2), round(float(O[1]), 2)],
           "rotation_deg": round(float(np.degrees(np.arctan2(left[1], left[0]))), 2),
           "lower_deck_z": round(deck_lo, 1), "upper_deck_z": round(deck_hi, 1),
           "lower_bosses": {k: on_grid(("boss", k), *c[:2], deck_lo, dia=round(float(c[2]), 1))
                            for k, c in bosses.items()},
           "upper_rows": {}}
    for side in ("left", "right"):
        r = rows[side]
        out["upper_rows"][side] = [on_grid((side, k), r["x"], y3 + pitch * (k - 3), deck_hi, kind=ROW_KIND[k])
                                   for k in range(7)]
        out["upper_rows"][side + "_fit"] = {"found": r["n"], "rms": round(r["rms"], 2)}
    bx, by = BATTERY["x"], BATTERY["y"]
    c1, c2 = rf(bx[0], by[0]), rf(bx[1], by[1])
    out["battery_box"] = {"x": sorted([c1[0], c2[0]]), "y": sorted([c1[1], c2[1]]), "top_z": round(las["ridge"]["top"], 1)}
    for nm, w in (("right", wl), ("left", wr)):  # machine -X is the robot's right
        xa, ya = rf(w["x"][0], w["axle_y"]), rf(w["x"][1], w["axle_y"])
        out["wheel_" + nm] = {"x": sorted([xa[0], ya[0]]), "axle_y": xa[1], "top_z": round(w["top"], 1),
                              "dia": round(w["top"], 1), "axle_z": round(w["top"] / 2, 1)}
    mx = rf(MICROBIT["usb_x"], MICROBIT["y"])
    out["microbit"] = {"y_board": mx[1], "x_centre": mx[0], "width": MICROBIT["width"], "top_z": MICROBIT["top"],
                       "note": "vertical, LEDs facing forward; y_board +-1.5, top_z +-1"}
    out = json.loads(json.dumps(out, default=float))
    json.dump(out, open(os.path.join(a.dir, "features.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))

    # OpenSCAD keep-out / reference model
    L = ["// Cutebot Pro reference, from the Carvera line-laser scan (projects/cutebot-pro/robot_features.py).",
         "// Robot frame: +x right, +y forward, +z up, mm; origin on the 8 mm LEGO grid (see features.json).",
         "lower_deck_z = %.1f; upper_deck_z = %.1f;" % (deck_lo, deck_hi), "",
         "module lower_bosses(d = 3.2, h = 10) {"]
    L += ["    translate([%.2f, %.2f, %.1f - h / 2]) cylinder(d = d, h = h, $fn = 24);  // %s" % (*v["xy"], deck_lo, k)
          for k, v in out["lower_bosses"].items()]
    L += ["}", "", "module upper_row(kind = \"boss\", d = 3.2, h = 10) {"]
    for side in ("left", "right"):
        for f in out["upper_rows"][side]:
            L.append("    if (kind == \"%s\") translate([%.2f, %.2f, %.1f - h / 2]) cylinder(d = d, h = h, $fn = 24);"
                     % (f["kind"], *f["xy"], deck_hi))
    L += ["}", "", "module envelope() {"]
    b = out["battery_box"]
    L.append("    translate([%.2f, %.2f, 0]) cube([%.2f, %.2f, %.1f]);  // battery box"
             % (b["x"][0], b["y"][0], b["x"][1] - b["x"][0], b["y"][1] - b["y"][0], b["top_z"]))
    for nm in ("left", "right"):
        w = out["wheel_" + nm]
        L.append("    translate([%.2f, %.2f, %.1f]) rotate([0, 90, 0]) cylinder(d = %.1f, h = %.2f, $fn = 64);  // wheel %s"
                 % (w["x"][0], w["axle_y"], w["axle_z"], w["dia"], w["x"][1] - w["x"][0], nm))
    m = out["microbit"]
    L.append("    translate([%.2f, %.2f, %.1f]) cube([%.1f, %.1f, %.1f]);  // micro:bit"
             % (m["x_centre"] - m["width"] / 2, m["y_board"] - MICROBIT["thick"] / 2, m["top_z"] - 42,
                m["width"], MICROBIT["thick"], 42))
    L += ["}", "", "%envelope();", "color(\"red\") lower_bosses();", "color(\"red\") upper_row(\"boss\");",
          "color(\"blue\") upper_row(\"hole\", d = 4.8);", "color(\"gray\") upper_row(\"screw\");", ""]
    open(os.path.join(a.dir, "cutebot.scad"), "w").write("\n".join(L))

    # annotated top view on the merged laser height map (0-80 mm, turbo), north up, 0.1 mm/px
    g = np.load(os.path.join(a.dir, "merged-grid.npz"))
    (x0, y0), res = g["origin"], 0.1
    hm = np.clip(np.nan_to_num(g["height"], nan=0) / 80 * 255, 0, 255).astype(np.uint8)
    hm = cv2.resize(hm, None, fx=float(g["cell"]) / res, fy=float(g["cell"]) / res, interpolation=cv2.INTER_NEAREST)
    img = cv2.applyColorMap(hm, cv2.COLORMAP_TURBO)
    img[cv2.resize((~np.isfinite(g["height"])).astype(np.uint8), hm.shape[::-1], interpolation=cv2.INTER_NEAREST) > 0] = 255
    img = img[::-1].copy()
    Hpx = img.shape[0]
    px = lambda X, Y: (int((X - x0) / res), int(Hpx - (Y - y0) / res))
    inv = lambda x, y: O + x * right + y * fwd
    for k, c in bosses.items():
        cv2.circle(img, px(*c[:2]), int(c[2] / 2 / res), (0, 0, 255), 2)
        xy = out["lower_bosses"][k]["xy"]
        cv2.putText(img, "(%.1f, %.1f)" % tuple(xy), (px(*c[:2])[0] - 60, px(*c[:2])[1] + 55), 0, .55, (0, 0, 255), 2)
    for side in ("left", "right"):
        for f in out["upper_rows"][side]:
            P = inv(*f["xy"])
            col = {"boss": (0, 0, 255), "hole": (255, 128, 0), "screw": (128, 128, 128)}[f["kind"]]
            cv2.circle(img, px(*P), 25, col, 2)
    P = inv(0, 0)
    cv2.drawMarker(img, px(*P), (0, 255, 0), cv2.MARKER_CROSS, 60, 2)
    cv2.arrowedLine(img, px(*P), px(*inv(0, 30)), (0, 255, 0), 2)
    cv2.arrowedLine(img, px(*P), px(*inv(30, 0)), (0, 255, 0), 2)
    cv2.putText(img, "+y fwd", px(*inv(2, 32)), 0, .6, (0, 255, 0), 2)
    cv2.putText(img, "+x", px(*inv(31, -3)), 0, .6, (0, 255, 0), 2)
    cv2.imwrite(os.path.join(a.dir, "features.png"), img)

    # the scanned surface in the robot frame, for index.html: float32 x, y, z per 0.5 mm cell
    hh, c = g["height"], float(g["cell"])
    iy, ix = np.nonzero(np.isfinite(hh))
    P = np.c_[x0 + (ix + .5) * c, y0 + (iy + .5) * c] - O
    np.c_[P @ right, P @ fwd, hh[iy, ix]].astype("<f4").tofile(os.path.join(a.dir, "points.bin"))


if __name__ == "__main__":
    main()
