"""Superseded by `carvera_scan.process.pipeline` (merge + cleaning + grid with the standard
defaults). Only best_shift() is still used, by pipeline.merge(). The command line
below is the old chain (no cleaning, median grid), kept for reproducing old results.

Merge cross-laser point sets (several scans, both cameras) into camera 1.2's
map frame and grid them.

    python -m carvera_scan.process.blocks_merge --ref data/…/blocks1_1.2.npz --other data/…/blocks1_1.1.npz \
        --extra data/…/blocksX_1.2.npz data/…/blocksX_1.1.npz --out data/…/blocks_all.npz

Camera 1.1's frame is shifted onto 1.2's by matching 1 mm height grids
(heights clipped to 0..80). Points from each scan keep their camera, arm and
scan index; the merged 0.5 mm grid is the median of all points per cell.
"""
import argparse
import os

import numpy as np

from carvera_scan.process.grid import grid


def best_shift(ref_pts, oth_pts, guess=(0.0, 0.0), span=70, step=1.0):
    def g(xy, h, o):
        med, cnt, org = grid(xy, h, 1.0, 3)
        return med, org
    A, oa = g(ref_pts["xy"], ref_pts["height"], None)
    B, ob = g(oth_pts["xy"] + guess, oth_pts["height"], None)
    best, errs = None, {}
    for dy in np.arange(-span, span + 1, step):
        for dx in np.arange(-10, 11, step):
            ix, iy = int(round(ob[0] + dx - oa[0])), int(round(ob[1] + dy - oa[1]))
            ys = slice(max(iy, 0), min(iy + B.shape[0], A.shape[0])); xs = slice(max(ix, 0), min(ix + B.shape[1], A.shape[1]))
            if ys.stop <= ys.start or xs.stop <= xs.start:
                continue
            a = A[ys, xs]; b = B[ys.start - iy:ys.stop - iy, xs.start - ix:xs.stop - ix]
            ok = np.isfinite(a) & np.isfinite(b)
            if ok.sum() < 5000:
                continue
            e = np.mean(np.abs(np.clip(a[ok], 0, 80) - np.clip(b[ok], 0, 80)))
            if best is None or e < best[0]:
                best = (e, dx, dy)
            errs[(round(dx / step), round(dy / step))] = e
    # sub-step: a parabola through the best shift and its two neighbours, per axis
    e0, bx, by = best
    k = (round(bx / step), round(by / step))
    sub = []
    for d in ((1, 0), (0, 1)):
        em, ep = errs.get((k[0] - d[0], k[1] - d[1])), errs.get((k[0] + d[0], k[1] + d[1]))
        den = (em - 2 * e0 + ep) if em is not None and ep is not None else 0
        sub.append(float(np.clip(0.5 * (em - ep) / den, -0.5, 0.5)) * step if den > 0 else 0.0)
    return np.array(guess) + np.array([bx + sub[0], by + sub[1]]), e0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--other", required=True, help="same scan, other camera")
    ap.add_argument("--extra", nargs="*", default=[], help="more scans: camera 1.2 ones need no shift, 1.1 ones get the same shift")
    ap.add_argument("--guess", type=float, nargs=2, default=(-2.5, 52.5), help="1.1 -> 1.2 shift guess")
    ap.add_argument("--cell", type=float, default=0.5)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    ref, oth = np.load(a.ref), np.load(a.other)
    sh, e = best_shift(ref, oth, a.guess, span=8)
    print("camera 1.1 -> 1.2 shift (%.1f, %.1f) mm, mean |dh| %.2f mm" % (*sh, e))
    files = [a.ref, a.other] + a.extra
    XY, H, CAM, ARM, SC = [], [], [], [], []
    for k, f in enumerate(files):
        d = np.load(f)
        cam11 = "_1.1" in os.path.basename(f)
        XY.append(d["xy"] + (sh if cam11 else 0)); H.append(d["height"])
        CAM.append(np.full(len(d["height"]), 1 if cam11 else 0, np.int8)); ARM.append(d["arm"]); SC.append(np.full(len(d["height"]), k, np.int8))
        print("  %s: %d points%s" % (f, len(d["height"]), " (shifted)" if cam11 else ""))
    XY, H, CAM, ARM, SC = map(np.concatenate, (XY, H, CAM, ARM, SC))
    med, cnt, origin = grid(XY, H, a.cell, 4)
    np.savez_compressed(a.out, xy=XY.astype(np.float32), height=H.astype(np.float32), cam=CAM, arm=ARM, scan=SC,
                        files=np.array(files), shift=sh, grid=med.astype(np.float32), count=cnt, origin=origin, cell=a.cell)
    print("%d points; grid %s, %.0f%% filled; wrote %s" % (len(H), med.shape, 100 * np.isfinite(med).mean(), a.out))


if __name__ == "__main__":
    main()
