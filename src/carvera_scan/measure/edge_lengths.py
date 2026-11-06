"""Lengths of flat tops from machine motion: each fixed point along a laser
arm (turned column) sees a top surface at one fixed spot relative to the head,
so the head travel between stepping onto a top and stepping off it is the
top's length along the travel direction. Image scale and latency cancel
(both edges in one pass at constant speed).

    python -m carvera_scan.measure.edge_lengths --objects objects.json \
        --scan data/…/blocks1_1.2 data/…/blocks1_1.1 data/…/blocksX_1.2 data/…/blocksX_1.1 \
        --zcal data/…/zcal45deep --shift -3.5 53.5 --out lengths.json

objects.json: [{"name", "band": [lo, hi] (top height, mm), "box": [x0, x1, y0, y1]
(camera-1.2 map frame, the top's extent), "skip": [[x0, x1, y0, y1], ...] (optional holes)}].
Per scan (passes along Y give Y lengths, along X give X lengths), camera,
pass, arm and column: the first and last frames on the top inside the box,
each paired with the neighbouring frame just off it (<= 2 frames away); the
edge is the midpoint. Columns whose path runs within 3 mm of a side edge
are dropped (corner clips).
"""
import argparse
import json
import os

import numpy as np

from carvera_scan.process.cross import head_xy, turn
from carvera_scan.process.sweep_height import load


def scan_lengths(scan, npz, zcal, shift, objects, latency=0.004, margin=3.0):
    s, t, p, _ = load(scan)
    d = np.load(npz)
    cal = json.load(open(zcal))
    Xh, Yh = head_xy(s, t, p, latency)
    along = s["passes"][0].get("axis", "y")
    trav = Yh if along == "y" else Xh
    fr, arm, h = d["frame"], d["arm"], d["height"]
    xy = d["xy"] + shift
    uv = d["uv"].astype(float)
    col = np.zeros(len(h), int)
    for ai, a in enumerate(cal["arms"]):
        m = arm == ai
        M = turn(a["theta"])
        col[m] = np.round(uv[m] @ M[0, :2] + M[0, 2]).astype(int)
    passno = p[fr]
    out = {o["name"]: [] for o in objects}
    for o in objects:
        x0, x1, y0, y1 = o["box"]
        lo, hi = o["band"]
        # along-travel map coordinate of a point (for checking entry/exit sit at the box ends)
        a_lo, a_hi = (y0, y1) if along == "y" else (x0, x1)
        c_lo, c_hi = (x0, x1) if along == "y" else (y0, y1)
        near = (xy[:, 0] > x0 - 6) & (xy[:, 0] < x1 + 6) & (xy[:, 1] > y0 - 6) & (xy[:, 1] < y1 + 6)
        idx = np.flatnonzero(near)
        if not len(idx):
            continue
        top = (h[idx] >= lo) & (h[idx] <= hi) & (xy[idx, 0] > x0 - 1.5) & (xy[idx, 0] < x1 + 1.5) & \
              (xy[idx, 1] > y0 - 1.5) & (xy[idx, 1] < y1 + 1.5)
        key = (passno[idx].astype(np.int64) * 4 + arm[idx]) * 4096 + col[idx]
        order = np.lexsort((trav[fr[idx]], key))
        idx, top, key = idx[order], top[order], key[order]
        starts = np.flatnonzero(np.r_[True, np.diff(key) != 0])
        ends = np.r_[starts[1:], len(key)]
        for s0, s1 in zip(starts, ends):
            tp = top[s0:s1]
            if tp.sum() < 8:
                continue
            ii = idx[s0:s1]
            pos = trav[fr[ii]]
            frames = fr[ii]
            k_first, k_last = np.flatnonzero(tp)[[0, -1]]
            if k_first == 0 or k_last == len(tp) - 1:
                continue
            # neighbours just off the top: valid detections, not on the top, within 2 frames
            if frames[k_first] - frames[k_first - 1] > 2 * (1 if frames[k_first] > frames[k_first - 1] else -1) and \
                    abs(frames[k_first] - frames[k_first - 1]) > 2:
                continue
            if abs(frames[k_last + 1] - frames[k_last]) > 2 or abs(frames[k_first] - frames[k_first - 1]) > 2:
                continue
            e1 = (pos[k_first] + pos[k_first - 1]) / 2
            e2 = (pos[k_last] + pos[k_last + 1]) / 2
            # the top points' map coordinates: cross position inside the box, ends at the box ends
            q = xy[ii[tp]]
            cross = np.median(q[:, 0] if along == "y" else q[:, 1])
            if not (c_lo + margin < cross < c_hi - margin):
                continue
            ends_map = q[[0, -1], 1 if along == "y" else 0]
            if min(abs(ends_map.min() - a_lo), abs(ends_map.max() - a_hi)) > 3.0:
                continue
            if abs(ends_map.min() - a_lo) > 3.0 or abs(ends_map.max() - a_hi) > 3.0:
                continue
            out[o["name"]].append({"len": float(abs(e2 - e1)), "cross": float(cross), "arm": int(arm[ii[0]]),
                                   "pass": int(passno[ii[0]])})
    return along, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objects", required=True)
    ap.add_argument("--scan", nargs="+", required=True)
    ap.add_argument("--zcal", required=True, help="dir with <cam>.json")
    ap.add_argument("--shift", type=float, nargs=2, default=(0, 0), help="camera 1.1 -> 1.2 map shift")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    objects = json.load(open(a.objects))
    res = {}
    for scan in a.scan:
        cam = scan.rstrip("/").split("_")[-1]
        sh = np.array(a.shift) if cam == "1.1" else np.zeros(2)
        along, out = scan_lengths(scan, scan.rstrip("/") + ".npz", os.path.join(a.zcal, cam + ".json"), sh, objects)
        for name, rows in out.items():
            for r in rows:
                r["cam"] = cam
            res.setdefault(name, {}).setdefault(along, []).extend(rows)
    summary = {}
    for name, by in res.items():
        summary[name] = {}
        for along, rows in by.items():
            if not rows:
                continue
            L = np.array([r["len"] for r in rows])
            med = np.median(L)
            keep = np.abs(L - med) < 1.0
            groups = {}
            for r, k in zip(rows, keep):
                if k:
                    groups.setdefault("%s/arm%d" % (r["cam"], r["arm"]), []).append(r["len"])
            gm = {g: (float(np.median(v)), len(v)) for g, v in groups.items() if len(v) >= 5}
            vals = np.array([v[0] for v in gm.values()])
            summary[name][along] = {"median": float(np.median(L[keep])), "n": int(keep.sum()), "n_all": len(L),
                                    "mad": float(1.4826 * np.median(np.abs(L[keep] - np.median(L[keep])))),
                                    "groups": gm, "group_spread": float(np.ptp(vals)) if len(vals) > 1 else None}
            g = "  ".join("%s %.2f (%d)" % (k, v[0], v[1]) for k, v in sorted(gm.items()))
            print("%-14s along %s: %7.2f mm  (n %d of %d, MAD %.2f; by camera/arm: %s)" % (
                name, along.upper(), summary[name][along]["median"], keep.sum(), len(L), summary[name][along]["mad"], g))
    json.dump(summary, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
