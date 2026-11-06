"""Merge two cameras' line-laser points (sweep_height.py) into camera A's frame
and measure the Cutebot Pro on the plate: deck heights, battery ridge, wheels,
micro:bit top. Camera B's offset comes from bed holes seen by both.

    python projects/cutebot-pro/robot_laser.py data/2026-11-04/robot1_2.2.npz data/2026-11-04/robot1_2.3.npz \
        --grid-a data/2026-11-04/robot1_2.2-grid.npz --grid-b data/2026-11-04/robot1_2.3-grid.npz \
        --out data/2026-11-04/robot1

Writes OUT/laser.json and OUT/merged-grid.npz (0.5 mm, levelled to the bed).
"""
import argparse
import json
import os

import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares

from carvera_scan.process.grid import grid

BOX = (-150, -240, 0, -70)  # X0 Y0 X1 Y1 around the robot, camera A frame


def bed_holes(path):
    """Centres of round gaps in the bed cells of a grid (the spoilboard holes)."""
    d = np.load(path)
    h, (x0, y0), c = d["height"], d["origin"], float(d["cell"])
    bed = np.isfinite(h) & (np.abs(h) < 2)
    lab, n = ndimage.label(ndimage.binary_closing(bed, iterations=12) & ~bed)
    out = []
    for k in range(1, n + 1):
        ys, xs = np.nonzero(lab == k)
        if 60 < len(ys) < 300 and np.ptp(xs) < 16 and np.ptp(ys) < 16 and abs(np.ptp(xs) - np.ptp(ys)) < 4:
            out.append((x0 + (xs.mean() + .5) * c, y0 + (ys.mean() + .5) * c))
    return np.array(out)


def offset_b_to_a(ga, gb, guess=(0, -120), search=10):
    A, B = bed_holes(ga), bed_holes(gb)
    best = None
    for dx in np.arange(guess[0] - search, guess[0] + search + .1, .5):
        for dy in np.arange(guess[1] - search, guess[1] + search + .1, .5):
            s = (np.linalg.norm((B + [dx, dy])[:, None] - A[None], axis=2).min(1) < 2).sum()
            if best is None or s > best[0]:
                best = (s, dx, dy)
    dd = np.linalg.norm((B + best[1:])[:, None] - A[None], axis=2)
    m = dd.min(1) < 2
    off = (A[dd.argmin(1)[m]] - B[m]).mean(0)
    r = A[dd.argmin(1)[m]] - B[m] - off
    return off, int(m.sum()), float(np.sqrt((r ** 2).sum(1).mean()))


def wheel(x, y, h, x0, x1):
    """Circle through the tread's top profile (median height per 1 mm of Y)."""
    m = (x > x0) & (x < x1) & (y > -165) & (y < -90) & (h > 30) & (h < 60)
    yc, hc = [], []
    for b in np.arange(-165, -90, 1.0):
        q = m & (y >= b) & (y < b + 1)
        if q.sum() > 30:
            yc.append(b + .5)
            hc.append(np.median(h[q]))
    yc, hc = np.array(yc), np.array(hc)
    f = lambda p: p[2] + np.sqrt(np.maximum(p[1] ** 2 - (yc - p[0]) ** 2, 0)) - hc
    p = least_squares(f, [-128, 26, 25], loss="soft_l1", f_scale=0.5).x
    r = f(p)
    xs = x[m & (h > 44)]
    return {"axle_y": p[0], "radius": p[1], "axle_h": p[2], "top": p[1] + p[2],
            "x": [np.percentile(xs, 1), np.percentile(xs, 99)], "rms": float(np.sqrt(np.mean(r[np.abs(r) < 1.5] ** 2)))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--grid-a", required=True)
    ap.add_argument("--grid-b", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    off, n, rms = offset_b_to_a(a.grid_a, a.grid_b)
    print("camera B -> A offset (%.2f, %.2f) mm from %d bed holes, rms %.2f" % (*off, n, rms))
    pa, pb = np.load(a.a), np.load(a.b)
    xy = np.r_[pa["xy"], pb["xy"] + off]
    h = np.r_[pa["height"], pb["height"]]
    src = np.r_[np.zeros(len(pa["height"]), int), np.ones(len(pb["height"]), int)]
    box = (xy[:, 0] > BOX[0]) & (xy[:, 1] > BOX[1]) & (xy[:, 0] < BOX[2]) & (xy[:, 1] < BOX[3])
    xy, h, src = xy[box], h[box], src[box]
    bed = np.abs(h) < 2
    coef = np.linalg.lstsq(np.c_[xy[bed], np.ones(bed.sum())], h[bed], rcond=None)[0]
    h = h - np.c_[xy, np.ones(len(h))] @ coef
    x, y = xy[:, 0], xy[:, 1]
    med, cnt, org = grid(xy, h, 0.5)
    np.savez_compressed(os.path.join(a.out, "merged-grid.npz"), height=med, count=cnt, origin=np.array(org), cell=0.5)

    def sel(x0, x1, y0, y1, h0, h1):
        return (x > x0) & (x < x1) & (y > y0) & (y < y1) & (h > h0) & (h < h1)

    res = {"offset_b_to_a": off.tolist(), "offset_holes": n, "offset_rms": rms}
    m = sel(-140, -10, -225, -150, 15, 25)
    res["lower_deck"] = float(np.median(h[m]))
    for side, xr in (("L", (-100, -88)), ("R", (-65, -53))):
        m = sel(*xr, -150, -95, 25, 45)
        res["upper_deck_" + side] = float(np.median(h[m]))
    m = sel(-90, -66, -170, -95, 50, 66)
    res["ridge"] = {"top": float(np.median(h[m])), "x": np.percentile(x[m], [1, 99]).tolist(),
                    "y": np.percentile(y[m], [0.5, 99.5]).tolist()}
    m = sel(-110, -45, -200, -168, 60, 95)
    res["microbit_top"] = float(np.percentile(h[m], 99))
    for k, nm in ((0, "A"), (1, "B")):
        mm = m & (src == k)
        top = mm & (h > np.percentile(h[mm], 95))
        res["microbit_" + nm] = {"top": float(np.median(h[top])), "y": float(np.median(y[top])),
                                 "x": np.percentile(x[mm], [1, 99]).tolist()}
    res["wheel_L"] = wheel(x, y, h, -131, -113)
    res["wheel_R"] = wheel(x, y, h, -38, -21)
    res = json.loads(json.dumps(res, default=float))
    json.dump(res, open(os.path.join(a.out, "laser.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))


def boss_heights(merged_pts, bosses):
    """Deck around (5-8 mm) and on top of (< 2.8 mm) each boss."""
    x, y, h = merged_pts
    out = {}
    for nm, (bx, by) in bosses.items():
        r = np.hypot(x - bx, y - by)
        out[nm] = (float(np.median(h[(r > 5) & (r < 8) & (h > 10) & (h < 30)])),
                   float(np.median(h[(r < 2.8) & (h > 10) & (h < 30)])))
    return out


if __name__ == "__main__":
    main()
