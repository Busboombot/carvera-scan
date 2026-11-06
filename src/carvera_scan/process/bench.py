"""Benchmark the cross-laser chain on recorded captures (no machine needed).

    python -m carvera_scan.process.bench --out scratch/bench/default                     # pipeline defaults
    python -m carvera_scan.process.bench --out scratch/bench/try --set grid=median consistency=0   # override Params

The regression test for `carvera_scan.process.pipeline`: runs the standard chain (pipeline
defaults, any Params field overridable with --set) from the raw frames, with the
calibration sets in calib/, and scores two cases recorded on 2026-11-05. The cases
(scan names, object boxes, probe spots, the sheet mask) are fixtures of this data;
the processing is not. Run it before and after any change to the pipeline:

blocks  1-2-3 blocks + setup bars (blocks1_* Y passes, blocksX_* X passes, zcal45deep):
        heights vs labels (projects/blocks-123/objects.json), spread between the four camera/arm
        combinations, points/cells beyond the calibrated range, flagged cells
        (> 3 mm above their 7x7 median) and their clusters, roughness of flat tops,
        coverage.
sheet   1.5 mm sheet, bare (plate3_*, zcal45): laser vs the nine M466 probe touches
        (probe1/probe1.jsonl), flatness of the sheet core (flat2.npz mask).

Writes OUT/metrics.json and prints a summary. Compare runs with --compare A B.
"""
import argparse
import json
import os
import time
from fractions import Fraction

import numpy as np
from scipy import ndimage

from dataclasses import asdict, fields  # noqa: E402

from carvera_scan import calib as calib_mod
from carvera_scan.process import pipeline
from carvera_scan.process.grid import needles

MM = 25.4
TO_MACHINE_SHEET = np.array([-71.7, -68.1])   # scan-2 map frame -> machine (anchor corner)
CAM12_TO_SCAN2 = np.array([1.0, 1.0])          # camera 1.2 frame (after the laser turn) -> scan-2 frame


# ---------------------------------------------------------------- fixtures
BLOCK_SCANS = [("blocks1_1.2", "1.2"), ("blocks1_1.1", "1.1"), ("blocksX_1.2", "1.2"), ("blocksX_1.1", "1.1")]
SHEET_SCANS = [("plate3_1.2", "1.2"), ("plate3_1.1", "1.1")]


# ---------------------------------------------------------------- self-calibration
def selfcal(P, cell=1.0, iters=6, max_spread=1.5):
    """Per source (camera x arm) height offset and scale from overlaps: in each 1 mm cell
    the source medians m_cs = t_c (1 + k_s) + o_s; solved by alternating least squares
    with mean(o) = mean(k) = 0, so the Z-sweep calibration sets the average. Cells where
    the sources differ by more than max_spread (walls, reflections) are left out."""
    xy, h, s = P["xy"], P["height"], P["src"]
    ns = int(s.max()) + 1
    ix = np.floor(xy / cell).astype(np.int64)
    ix -= ix.min(0)
    cid = ix[:, 1] * (ix[:, 0].max() + 1) + ix[:, 0]
    key = cid * ns + s
    o = np.argsort(key, kind="stable")
    k_, h_ = key[o], h[o]
    st = np.flatnonzero(np.r_[True, np.diff(k_) != 0])
    cnt = np.diff(np.r_[st, len(k_)])
    med = np.array([np.median(h_[a:a + n]) for a, n in zip(st, cnt)])
    good = cnt >= 5
    cc, ss, mm = k_[st][good] // ns, k_[st][good] % ns, med[good]
    # cells seen by >= 2 sources that agree within max_spread
    uc, inv, nper = np.unique(cc, return_inverse=True, return_counts=True)
    lo = np.full(len(uc), np.inf); hi = np.full(len(uc), -np.inf)
    np.minimum.at(lo, inv, mm); np.maximum.at(hi, inv, mm)
    use = (nper[inv] >= 2) & ((hi - lo)[inv] < max_spread)
    cc, ss, mm, inv = cc[use], ss[use], mm[use], inv[use]
    off, scl = np.zeros(ns), np.zeros(ns)
    for _ in range(iters):
        corr = (mm - off[ss]) / (1 + scl[ss])
        t = np.bincount(inv, weights=corr, minlength=len(uc)) / np.maximum(np.bincount(inv, minlength=len(uc)), 1)
        tc = t[inv]
        for q in range(ns):
            m = ss == q
            A = np.c_[np.ones(m.sum()), tc[m]]
            (off[q], scl[q]), *_ = np.linalg.lstsq(A, mm[m] - tc[m], rcond=None)
        off -= off.mean(); scl -= scl.mean()
    resid = mm - (tc * (1 + scl[ss]) + off[ss])
    out = dict(P)
    out["height"] = (h - off[s]) / (1 + scl[s])
    info = {"offset_mm": off.round(3).tolist(), "scale": scl.round(5).tolist(), "cells": int(len(uc)),
            "pairs": int(len(mm)), "resid_mad": float(1.4826 * np.median(np.abs(resid)))}
    return out, info


# ---------------------------------------------------------------- metrics
def spike_stats(g):
    med = ndimage.median_filter(np.nan_to_num(g, nan=0.0), size=7)
    spike = np.isfinite(g) & (g > med + 3.0)
    lab, n = ndimage.label(spike)
    sizes = ndimage.sum(spike, lab, range(1, n + 1)) if n else np.zeros(0)
    return int(spike.sum()), int((np.asarray(sizes) >= 10).sum())


def plane_fit(X, Y, Z, iters=3):
    A = np.c_[X, Y, np.ones(len(Z))]
    p, *_ = np.linalg.lstsq(A, Z, rcond=None)
    for _ in range(iters):
        r = Z - A @ p
        k = np.abs(r) < max(3 * np.std(r), 0.3)
        p, *_ = np.linalg.lstsq(A[k], Z[k], rcond=None)
    return p


# ---------------------------------------------------------------- blocks case
def blocks(args, prm, cache):
    data = args.data
    cal = calib_mod.load(args.calib)
    P, info = pipeline.merge([(os.path.join(data, s_), port) for s_, port in BLOCK_SCANS], cal, prm, cache)
    m = {"register": info, "points": int(len(P["height"])), "points_above_80": int((P["height"] > 80).sum())}
    P, ci = pipeline.clean(P, prm)
    m.update(ci)
    cell = prm.cell_mm
    origin, shape = (-305.0, -220.0), (int(318 / cell), int(359 / cell))
    g, origin, gi = pipeline.grid(P, prm, origin, shape)
    m.update(gi)
    yy, xx = np.mgrid[0:shape[0], 0:shape[1]]
    GX, GY = origin[0] + (xx + .5) * cell, origin[1] + (yy + .5) * cell
    roi = (GX > -240) & (GX < -10) & (GY > -120) & (GY < 25)
    nd = needles(g) & roi
    m["needle_cells"] = int(nd.sum())
    m["needle_cells_5mm"] = int((needles(g, 5.0) & roi).sum())
    above = 0
    for a, b, e, f, t in [(-181, -154, -82, -29, 76.2), (-137.5, -110.5, -83, -4, 50.8), (-99.5, -46.5, -83, -3.5, 25.4)]:
        mm_ = (GX > a) & (GX < b) & (GY > e) & (GY < f) & np.isfinite(g)
        above += int((mm_ & (g > t + 2.3)).sum())
    m["cells_above_block_tops"] = above
    m["filled_roi"] = float(np.isfinite(g[roi]).mean())
    m["cells_above_80"] = int((g > 80).sum())
    m["flagged_cells"], m["flagged_clusters"] = spike_stats(np.where(roi, g, np.nan))
    # roughness of flat tops (plane residual, robust sd) and bed
    rough = {}
    for nm, (a, b, c0, c1) in {"1in_left": (-96, -81, -78, -8), "1/2bar": (-153, -138, -78, -8),
                               "3/16bar": (-102, -32, -93, -86), "bed": (-232, -214, -100, 10)}.items():
        mm_ = np.isfinite(g) & (GX > a) & (GX < b) & (GY > c0) & (GY < c1)
        z = g[mm_]; zmed = np.median(z); keep = np.abs(z - zmed) < 2.0
        p = plane_fit(GX[mm_][keep], GY[mm_][keep], z[keep])
        r = z[keep] - np.c_[GX[mm_][keep], GY[mm_][keep], np.ones(keep.sum())] @ p
        rough[nm] = float(1.4826 * np.median(np.abs(r - np.median(r))))
    m["roughness_mad"] = rough
    # heights vs labels, per source against its own bed plane
    objs = json.load(open(args.objects))
    xy, H, S = P["xy"], P["height"], P["src"]
    boxes = np.array([o["box"] for o in objs])
    gx0, gx1, gy0, gy1 = boxes[:, 0].min(), boxes[:, 1].max(), boxes[:, 2].min(), boxes[:, 3].max()
    ring = (xy[:, 0] > gx0 - 40) & (xy[:, 0] < gx1 + 40) & (xy[:, 1] > gy0 - 40) & (xy[:, 1] < gy1 + 40) & \
           ~((xy[:, 0] > gx0 - 8) & (xy[:, 0] < gx1 + 8) & (xy[:, 1] > gy0 - 8) & (xy[:, 1] < gy1 + 8)) & (np.abs(H) < 1.5)
    beds = {}
    for q in range(4):
        k = ring & (S == q)
        beds[q] = plane_fit(xy[k, 0], xy[k, 1], H[k])
    tops = {}
    for o in objs:
        x0, x1, y0, y1 = o["box"]; lo, hi = o["band"]
        sel = (xy[:, 0] > x0 + 2) & (xy[:, 0] < x1 - 2) & (xy[:, 1] > y0 + 2) & (xy[:, 1] < y1 - 2) & (H >= lo) & (H <= hi)
        per = {}
        for q in range(4):
            k = sel & (S == q)
            if k.sum() >= 200:
                b = beds[q][0] * xy[k, 0] + beds[q][1] * xy[k, 1] + beds[q][2]
                per[q] = float(np.median(H[k] - b))
        tops[o["name"]] = per
    rows = []
    for o in objs:
        per = dict(tops[o["name"]])
        if o.get("on"):
            base = tops[o["on"]]
            per = {q: v - base[q] for q, v in per.items() if q in base}
        vals = np.array(list(per.values()))
        nom = float(Fraction(o["nominal"])) * MM
        rows.append({"name": o["name"], "nominal_mm": nom, "measured_mm": float(np.median(vals)),
                     "error_mm": float(np.median(vals) - nom), "spread_mm": float(np.ptp(vals)),
                     "per_source": {"cam%d/arm%d" % (q // 2, q % 2): round(v - nom, 3) for q, v in per.items()}})
    m["heights"] = rows
    errs = np.array([r["error_mm"] for r in rows]); spr = np.array([r["spread_mm"] for r in rows])
    m["height_abs_error_mean"] = float(np.abs(errs).mean()); m["height_abs_error_max"] = float(np.abs(errs).max())
    m["spread_mean"] = float(spr.mean()); m["spread_max"] = float(spr.max())
    if args.save:
        pipeline.save_points(os.path.join(args.out, "blocks_points.npz"), P, g, origin, cell, m["register"])
        hrows = []
        for o, r in zip(objs, rows):
            meas = r["measured_mm"]
            hrows.append({"name": o["name"], "label": o.get("label", o["nominal"] + "\""), "nominal_mm": r["nominal_mm"],
                          "nominal_in": r["nominal_mm"] / MM, "measured_mm": meas, "measured_in": meas / MM,
                          "error_mm": r["error_mm"], "error_in": r["error_mm"] / MM, "spread_mm": r["spread_mm"],
                          "on": o.get("on")})
        json.dump(hrows, open(os.path.join(args.out, "heights.json"), "w"), indent=1)
    return m, g, (origin, cell)


# ---------------------------------------------------------------- sheet case
def sheet(args, prm, cache):
    data = args.data
    cal = calib_mod.load(args.calib_sheet)
    # the sheet's mask and probe offset live in the older scan-2 frame: camera A frame + CAM12_TO_SCAN2
    P, info = pipeline.merge([(os.path.join(data, s_), port) for s_, port in SHEET_SCANS], cal, prm, cache,
                             extra_shift=CAM12_TO_SCAN2)
    m = {"register": info, "points": int(len(P["height"])), "points_above_10": int((P["height"] > 10).sum())}
    P, ci = pipeline.clean(P, prm)
    m.update(ci)
    f = np.load(os.path.join(data, "flat2.npz"))
    origin, cell, core = tuple(f["origin"]), float(f["cell"]), f["core"]
    if abs(cell - prm.cell_mm) > 1e-9:
        raise SystemExit("sheet case needs cell_mm = %.2f (its mask)" % cell)
    g, origin, gi = pipeline.grid(P, prm, origin, core.shape)
    m.update(gi)
    yy, xx = np.mgrid[0:core.shape[0], 0:core.shape[1]]
    GX, GY = origin[0] + (xx + .5) * cell, origin[1] + (yy + .5) * cell
    # bed plane: bare spoilboard 10-40 mm around the sheet
    sx0, sx1, sy0, sy1 = -288.0, -132.5, -174.0, -65.0
    around = (GX > sx0 - 40) & (GX < sx1 + 40) & (GY > sy0 - 40) & (GY < sy1 + 40) & \
             ~((GX > sx0 - 10) & (GX < sx1 + 10) & (GY > sy0 - 10) & (GY < sy1 + 10)) & np.isfinite(g)
    z = g[around]; keep = np.abs(z - np.median(z)) < 1.0
    bp = plane_fit(GX[around][keep], GY[around][keep], z[keep])
    hb = g - (bp[0] * GX + bp[1] * GY + bp[2])
    # flatness on the sheet core
    c = core & np.isfinite(hb)
    sp = plane_fit(GX[c], GY[c], hb[c])
    res = np.where(c, hb - (sp[0] * GX + sp[1] * GY + sp[2]), 0)
    smooth = ndimage.uniform_filter(res.astype(np.float64), 11) / np.maximum(ndimage.uniform_filter(c.astype(float), 11), 1e-6)
    sm = smooth[c]
    m["sheet_thickness_median"] = float(np.median(hb[c]))
    m["flat_resid_rms"] = float(np.std(res[c])); m["flat_smoothed_p1_p99"] = [float(np.percentile(sm, 1)), float(np.percentile(sm, 99))]
    m["filled_core"] = float(np.isfinite(g[core]).mean())
    # probe comparison (heights above each method's own bed plane)
    recs = {}
    for ln in open(os.path.join(data, "probe1", "probe1.jsonl")):
        r = json.loads(ln); recs.setdefault(r["spot"], []).append(r)
    spots = {k: (v[0]["x"], v[0]["y"], np.mean([r["z_machine"] for r in v])) for k, v in recs.items()}
    bed = [k for k in spots if k.startswith("B")]
    Ab = np.array([[1, spots[k][0], spots[k][1]] for k in bed]); zb = np.array([spots[k][2] for k in bed])
    pc, *_ = np.linalg.lstsq(Ab, zb, rcond=None)
    rows = []
    for k, (x, y, zt) in sorted(spots.items()):
        hp = zt - (pc[0] + pc[1] * x + pc[2] * y)
        mx, my = x - TO_MACHINE_SHEET[0], y - TO_MACHINE_SHEET[1]
        near = np.isfinite(hb) & ((GX - mx) ** 2 + (GY - my) ** 2 <= 9.0)
        hl = float(np.median(hb[near]))
        rows.append({"spot": k, "probe_h": float(hp), "laser_h": hl, "diff": hl - float(hp)})
    d = np.array([r["diff"] for r in rows if not r["spot"].startswith("B")])
    db = np.array([r["diff"] for r in rows if r["spot"].startswith("B")])
    m["probe"] = rows
    m["probe_sheet_mean"], m["probe_sheet_sd"] = float(d.mean()), float(d.std())
    m["probe_bed_absmax"] = float(np.abs(db).max())
    return m, g, (origin, cell)


SUMMARY = [("blocks", "points_above_80", "%d"), ("blocks", "cells_above_80", "%d"), ("blocks", "needle_cells", "%d"),
           ("blocks", "needle_cells_5mm", "%d"), ("blocks", "cells_above_block_tops", "%d"), ("blocks", "flagged_cells", "%d"),
           ("blocks", "flagged_clusters", "%d"), ("blocks", "filled_roi", "%.3f"), ("blocks", "height_abs_error_mean", "%.3f"),
           ("blocks", "height_abs_error_max", "%.3f"), ("blocks", "spread_mean", "%.3f"), ("blocks", "spread_max", "%.3f"),
           ("sheet", "probe_sheet_mean", "%+.3f"), ("sheet", "probe_sheet_sd", "%.3f"), ("sheet", "probe_bed_absmax", "%.3f"),
           ("sheet", "flat_resid_rms", "%.3f"), ("sheet", "filled_core", "%.3f")]


def summary(m):
    out = {}
    for case, k, fmt in SUMMARY:
        if case in m and k in m[case]:
            out["%s.%s" % (case, k)] = fmt % m[case][k]
    for nm, v in m.get("blocks", {}).get("roughness_mad", {}).items():
        out["blocks.rough." + nm] = "%.3f" % v
    return out


def parse_set(items):
    """--set key=value ... -> dict typed by the Params fields."""
    types = {f.name: f.type for f in fields(pipeline.Params)}
    out = {}
    for it in items or []:
        k, _, v = it.partition("=")
        if k not in types:
            raise SystemExit("unknown parameter %r; known: %s" % (k, ", ".join(sorted(types))))
        t = types[k] if isinstance(types[k], type) else {"bool": bool, "int": int, "float": float, "str": str}[types[k]]
        out[k] = (v.lower() in ("1", "true", "yes", "on")) if t is bool else t(v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="data/2026-11-05")
    ap.add_argument("--objects", default="projects/blocks-123/objects.json", help="labelled tops for the blocks case")
    ap.add_argument("--calib", default="config/calib/2026-11-05-cross45-deep.json", help="calibration set for the blocks case")
    ap.add_argument("--calib-sheet", default="config/calib/2026-11-05-cross45-shallow.json", help="calibration set for the sheet case")
    ap.add_argument("--out")
    ap.add_argument("--cache", help="extraction cache (default: OUT/../cache)")
    ap.add_argument("--case", nargs="+", default=["blocks", "sheet"])
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="override pipeline.Params defaults")
    ap.add_argument("--save", action="store_true", help="also write OUT/blocks_points.npz and OUT/heights.json (view3d input)")
    ap.add_argument("--compare", nargs="+", help="metrics.json files (or run dirs) to tabulate")
    a = ap.parse_args()
    if a.compare:
        runs = [json.load(open(p if p.endswith(".json") else os.path.join(p, "metrics.json"))) for p in a.compare]
        sums = [summary(r) for r in runs]
        keys = list(dict.fromkeys(k for s in sums for k in s))
        names = [os.path.basename(os.path.dirname(p)) if p.endswith(".json") else os.path.basename(p.rstrip("/")) for p in a.compare]
        print("%-30s" % "metric" + "".join("%14s" % n[:13] for n in names))
        for k in keys:
            print("%-30s" % k + "".join("%14s" % s.get(k, "-") for s in sums))
        return
    prm = pipeline.Params(**parse_set(a.set))
    cache = a.cache or os.path.join(os.path.dirname(a.out.rstrip("/")), "cache")
    os.makedirs(a.out, exist_ok=True)
    m = {"params": asdict(prm), "calib": a.calib, "calib_sheet": a.calib_sheet}
    for case in a.case:
        t = time.time()
        res, g, (origin, cell) = {"blocks": blocks, "sheet": sheet}[case](a, prm, cache)
        res["seconds"] = round(time.time() - t, 1)
        m[case] = res
        np.savez_compressed(os.path.join(a.out, "%s_grid.npz" % case), grid=g, origin=origin, cell=cell)
    json.dump(m, open(os.path.join(a.out, "metrics.json"), "w"), indent=1, default=str)
    for k, v in summary(m).items():
        print("%-30s %s" % (k, v))


if __name__ == "__main__":
    main()
