"""Fit a square lattice (e.g. the 8 mm LEGO grid) to measured feature centres.

    python -m carvera_scan.measure.lattice RESULT.json [RESULT2.json ...] --pitch 8 [--ids H01 H02 ...] [--object 2]

Many parts put their holes on a grid. Fitting one (offset and rotation, pitch fixed,
then pitch free) gives each feature its grid index and a residual. The rms residual is a
direct accuracy check of the measurements, with no reference part needed, and the
snapped grid positions are usually better than any single measurement.

Input: any features JSON with {"features": [{id, xy, ...}]} (measure.features,
process.spot measure, measure.features refine).
"""
import argparse
import json

import numpy as np
from scipy.optimize import least_squares


def fit(P, pitch, anchor=None):
    """P: N x 2 centres. Returns dict: origin, angle_deg, idx (N x 2 ints), residuals (N), rms,
    free_pitch. The lattice is a0 + pitch * R(angle) idx; anchor (index into P) sets idx 0."""
    P = np.asarray(P, float)
    a0 = P[anchor if anchor is not None else 0]
    # angle: the direction of the nearest-neighbour vectors, folded into one 90 deg sector
    d = P[:, None] - P[None]
    dist = np.hypot(d[..., 0], d[..., 1])
    np.fill_diagonal(dist, np.inf)
    nn = d[np.arange(len(P)), dist.argmin(1)]
    ang = np.arctan2(nn[:, 1], nn[:, 0]) % (np.pi / 2)
    th = np.arctan2(np.sin(4 * ang).mean(), np.cos(4 * ang).mean()) / 4

    def rot(t):
        return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])

    idx = np.round((P - a0) @ rot(th) / pitch)

    def model(q, s=pitch):
        return q[:2] + s * idx @ rot(q[2]).T

    q = least_squares(lambda q: (model(q) - P).ravel(), [*a0, th]).x
    idx = np.round((P - q[:2]) @ rot(q[2]) / pitch)  # re-index once with the fitted lattice
    q = least_squares(lambda q: (model(q) - P).ravel(), q).x
    res = np.hypot(*(model(q) - P).T)
    free = least_squares(lambda v: (model(v[:3], v[3]) - P).ravel(), [*q, pitch]).x
    return {"origin": q[:2].tolist(), "angle_deg": float(np.degrees(q[2])), "idx": idx.astype(int).tolist(),
            "residuals": res.tolist(), "rms": float(np.sqrt(np.mean(res ** 2))), "max": float(res.max()),
            "free_pitch": float(free[3]), "snapped": model(q).tolist()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results", nargs="+")
    ap.add_argument("--pitch", type=float, default=8.0)
    ap.add_argument("--ids", nargs="+", help="only these feature ids")
    ap.add_argument("--box", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"), help="only features inside")
    ap.add_argument("--out", help="write the fit (one per input) as JSON")
    a = ap.parse_args()
    outs = {}
    for path in a.results:
        fs = json.load(open(path))["features"]
        fs = [f for f in fs if "xy" in f and f.get("ok", True)]
        if a.ids:
            fs = [f for f in fs if f["id"] in a.ids]
        if a.box:
            x0, x1, y0, y1 = a.box
            fs = [f for f in fs if x0 <= f["xy"][0] <= x1 and y0 <= f["xy"][1] <= y1]
        r = fit([f["xy"] for f in fs], a.pitch)
        print("%s: %d features, rms %.3f mm, max %.3f mm, free pitch %.4f, angle %.2f deg"
              % (path, len(fs), r["rms"], r["max"], r["free_pitch"], r["angle_deg"]))
        for f, i, e in zip(fs, r["idx"], r["residuals"]):
            print("   %-5s grid (%3d, %3d)  off %.3f" % (f["id"], i[0], i[1], e))
        outs[path] = {**r, "ids": [f["id"] for f in fs]}
    if a.out:
        json.dump(outs, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
