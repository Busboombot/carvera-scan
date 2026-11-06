"""Flatness of a sheet on the bed from a merged two-camera laser map.

    python -m carvera_scan.measure.flatness data/2026-11-05/plate2_merged.npz --out data/2026-11-05/flat2

Cells where the two cameras disagree by > 1 mm are dropped. The bed plane is
fitted to bare spoilboard around the sheet; the sheet's top is fitted with a
plane after eroding its mask 4 mm from every edge and window, and the
residuals are the flatness. Writes OUT.png (thickness map and residuals) and
OUT.json.
"""
import argparse
import json

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("merged")
    ap.add_argument("--sheet", type=float, nargs=2, default=(0.2, 2.6), help="height band of the sheet top, mm")
    ap.add_argument("--box", type=float, nargs=4, default=(-300, -130, -185, -60), help="x0 x1 y0 y1 around the sheet")
    ap.add_argument("--margin", type=float, default=4.0, help="mm kept from edges and windows")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    g = np.load(a.merged)
    A, B = g["A"], g["B"]
    X0, Y0 = g["origin"]
    cell = float(g["cell"])
    h = np.where(np.isfinite(A) & np.isfinite(B), np.where(np.abs(A - B) < 1.0, (A + B) / 2, np.nan),
                 np.where(np.isfinite(A), A, B))
    yy, xx = np.mgrid[0:h.shape[0], 0:h.shape[1]]
    X, Y = X0 + (xx + .5) * cell, Y0 + (yy + .5) * cell
    x0, x1, y0, y1 = a.box
    inbox = (X > x0) & (X < x1) & (Y > y0) & (Y < y1)

    # sheet mask: in-band cells, closed over small gaps, then the largest component
    band = np.isfinite(h) & (h > a.sheet[0]) & (h < a.sheet[1]) & inbox
    k = int(round(3 / cell))
    m = cv2.morphologyEx(band.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    big = 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])
    sheet = lab == big
    filled = cv2.morphologyEx(sheet.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((41, 41), np.uint8)).astype(bool)
    e = int(round(a.margin / cell))
    core = cv2.erode(sheet.astype(np.uint8), np.ones((2 * e + 1, 2 * e + 1), np.uint8)).astype(bool) & np.isfinite(h)

    # bed plane: bare spoilboard within 40 mm of the sheet, outside it
    near = cv2.dilate(filled.astype(np.uint8), np.ones((161, 161), np.uint8)).astype(bool)
    away = ~cv2.dilate(filled.astype(np.uint8), np.ones((21, 21), np.uint8)).astype(bool)
    bedm = near & away & np.isfinite(h) & (np.abs(h - np.nanmedian(h[near & away])) < 1.0)
    Ab = np.c_[X[bedm], Y[bedm], np.ones(bedm.sum())]
    cb, *_ = np.linalg.lstsq(Ab, h[bedm], rcond=None)
    bed = cb[0] * X + cb[1] * Y + cb[2]

    As = np.c_[X[core], Y[core], np.ones(core.sum())]
    cs, *_ = np.linalg.lstsq(As, h[core], rcond=None)
    res = h - (cs[0] * X + cs[1] * Y + cs[2])
    r = res[core]
    # smoothed residual (5 mm boxes) separates real shape from per-cell noise
    sm = cv2.blur(np.where(core, res, 0).astype(np.float32), (11, 11)) / np.maximum(
        cv2.blur(core.astype(np.float32), (11, 11)), 1e-6)
    smc = sm[core]
    thick = (h - bed)[core]
    ys, xs = np.nonzero(sheet)
    out = {
        "bed_plane": {"dh_dx": cb[0], "dh_dy": cb[1], "rms": float(np.std(h[bedm] - Ab @ cb))},
        "sheet_plane": {"dh_dx": cs[0], "dh_dy": cs[1]},
        "sheet_minus_bed_tilt_deg": [float(np.degrees(np.arctan(cs[0] - cb[0]))), float(np.degrees(np.arctan(cs[1] - cb[1])))],
        "thickness_median": float(np.median(thick)), "thickness_p5_p95": np.percentile(thick, [5, 95]).tolist(),
        "residual_rms": float(r.std()), "residual_p1_p99": np.percentile(r, [1, 99]).tolist(),
        "smoothed_residual_pp": float(smc.max() - smc.min()), "smoothed_residual_rms": float(smc.std()),
        "smoothed_residual_p1_p99": np.percentile(smc, [1, 99]).tolist(),
        "cells": int(core.sum()), "cell_mm": cell,
        "sheet_extent": [float(X0 + xs.min() * cell), float(X0 + (xs.max() + 1) * cell),
                         float(Y0 + ys.min() * cell), float(Y0 + (ys.max() + 1) * cell)],
    }
    print(json.dumps(out, indent=1))
    json.dump(out, open(a.out + ".json", "w"), indent=1)
    np.savez_compressed(a.out + ".npz", height=h, bed=bed, sheet=sheet, core=core, smooth_res=np.where(core, sm, np.nan),
                        origin=(X0, Y0), cell=cell, sheet_plane=cs, bed_plane=cb)

    ext = [X0, X0 + h.shape[1] * cell, Y0, Y0 + h.shape[0] * cell]
    fig, axs = plt.subplots(1, 2, figsize=(16, 7))
    im = axs[0].imshow(np.where(sheet | filled, h - bed, np.nan), origin="lower", extent=ext, cmap="viridis", vmin=1.0, vmax=2.2)
    axs[0].set_title("sheet height above fitted bed, mm (median %.2f)" % out["thickness_median"])
    plt.colorbar(im, ax=axs[0], shrink=.7)
    im = axs[1].imshow(np.where(core, sm, np.nan), origin="lower", extent=ext, cmap="RdBu_r", vmin=-0.3, vmax=0.3)
    axs[1].set_title("flatness: residual from best plane, 5 mm average (p-p %.2f mm)" % out["smoothed_residual_pp"])
    plt.colorbar(im, ax=axs[1], shrink=.7)
    for ax in axs:
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.grid(alpha=.3)
        ax.set_xlabel("X (camera frame, mm)"); ax.set_ylabel("Y")
    plt.tight_layout()
    plt.savefig(a.out + ".png", dpi=80)


if __name__ == "__main__":
    main()
