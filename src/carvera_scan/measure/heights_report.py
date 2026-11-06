"""Heights of labelled tops (1-2-3 blocks, setup bars) from merged cross-laser
points, against their nominal (imperial) sizes.

    python -m carvera_scan.measure.heights_report data/2026-11-05/blocks_all.npz projects/blocks-123/objects.json \
        --out data/2026-11-05/heights.json

objects.json entries: name, nominal (inches, e.g. "1/2"), box [x0 x1 y0 y1] (camera-1.2 map
frame), band [lo hi] (mm), optional "on": name of the object it lies on (its height is then
top minus that object's top). Each camera/arm combination gets its own bed plane (bare
spoilboard 8-40 mm around the whole group), so per-arm calibration offsets cancel; the
reported height is the median over the four combinations, the spread is max - min.
"""
import argparse
import json
from fractions import Fraction

import numpy as np

MM = 25.4


def frac(inches):
    f = Fraction(inches).limit_denominator(64)
    w, r = divmod(f.numerator, f.denominator)
    return ("%d %d/%d\"" % (w, r, f.denominator) if w and r else "%d\"" % w if not r else "%d/%d\"" % (r, f.denominator))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("points")
    ap.add_argument("objects")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    d = np.load(a.points)
    xy, H, cam, arm = d["xy"].astype(float), d["height"].astype(float), d["cam"], d["arm"]
    objs = json.load(open(a.objects))
    boxes = np.array([o["box"] for o in objs])
    gx0, gx1, gy0, gy1 = boxes[:, 0].min(), boxes[:, 1].max(), boxes[:, 2].min(), boxes[:, 3].max()
    ring = (xy[:, 0] > gx0 - 40) & (xy[:, 0] < gx1 + 40) & (xy[:, 1] > gy0 - 40) & (xy[:, 1] < gy1 + 40) & \
           ~((xy[:, 0] > gx0 - 8) & (xy[:, 0] < gx1 + 8) & (xy[:, 1] > gy0 - 8) & (xy[:, 1] < gy1 + 8)) & (np.abs(H) < 1.5)
    tops = {}
    groups = [(c, r) for c in (0, 1) for r in (0, 1)]
    beds = {}
    for g in groups:
        m = ring & (cam == g[0]) & (arm == g[1])
        A = np.c_[xy[m], np.ones(m.sum())]
        p, *_ = np.linalg.lstsq(A, H[m], rcond=None)
        for _ in range(3):
            r = H[m] - A @ p
            k = np.abs(r) < 3 * np.std(r)
            p, *_ = np.linalg.lstsq(A[k], H[m][k], rcond=None)
        beds[g] = p
        print("bed plane cam %s arm %d: %+.3f at (0,0), slopes %+.4f %+.4f, rms %.2f, n %d" % (
            ("1.2", "1.1")[g[0]], g[1], p[2], p[0], p[1], np.std(H[m][k] - A[k] @ p), k.sum()))
    for o in objs:
        x0, x1, y0, y1 = o["box"]
        lo, hi = o["band"]
        sel = (xy[:, 0] > x0 + 2) & (xy[:, 0] < x1 - 2) & (xy[:, 1] > y0 + 2) & (xy[:, 1] < y1 - 2) & (H >= lo) & (H <= hi)
        per = {}
        for g in groups:
            m = sel & (cam == g[0]) & (arm == g[1])
            if m.sum() < 200:
                continue
            b = beds[g][0] * xy[m, 0] + beds[g][1] * xy[m, 1] + beds[g][2]
            per[g] = (float(np.median(H[m] - b)), int(m.sum()))
        tops[o["name"]] = per
    rows = []
    for o in objs:
        per = dict(tops[o["name"]])
        if o.get("on"):
            base = tops[o["on"]]
            per = {g: (v[0] - base[g][0], v[1]) for g, v in per.items() if g in base}
        vals = np.array([v[0] for v in per.values()])
        nom_in = float(Fraction(o["nominal"]))
        meas = float(np.median(vals))
        rows.append({"name": o["name"], "label": o.get("label", o["nominal"] + "\""), "nominal_in": nom_in,
                     "nominal_mm": nom_in * MM, "measured_mm": meas, "measured_in": meas / MM,
                     "error_mm": meas - nom_in * MM, "error_in": (meas - nom_in * MM) / MM,
                     "spread_mm": float(np.ptp(vals)), "per_group": {"%s/arm%d" % (("1.2", "1.1")[g[0]], g[1]): v[0] for g, v in per.items()},
                     "points": int(sum(v[1] for v in per.values())), "on": o.get("on")})
    print("\n%-22s %-8s %9s %9s %10s %9s %8s %8s %8s" % ("object", "label", "nom mm", "meas mm", "nom in", "meas in", "err mm", "err in", "spread"))
    for r in rows:
        print("%-22s %-8s %9.2f %9.2f %10.4f %9.4f %+8.2f %+8.4f %8.2f" % (
            r["name"], frac(r["nominal_in"]), r["nominal_mm"], r["measured_mm"], r["nominal_in"], r["measured_in"],
            r["error_mm"], r["error_in"], r["spread_mm"]))
    json.dump(rows, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
