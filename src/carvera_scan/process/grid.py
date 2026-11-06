"""Grid line-laser points into a height map: median per cell, and only cells
where enough points agree (thin streaks from reflections drop out).

    python -m carvera_scan.process.grid data/2026-11-04/plate2.npz --cell 0.5 --out data/2026-11-04/plate2-grid.npz
"""
import argparse

import numpy as np


def grid(xy, h, cell, min_count=4, max_mad=1.0):
    x0, y0 = np.floor(xy.min(0))
    ix = ((xy[:, 0] - x0) / cell).astype(np.int64)
    iy = ((xy[:, 1] - y0) / cell).astype(np.int64)
    nx = ix.max() + 1
    key = iy * nx + ix
    order = np.argsort(key, kind="stable")
    key, hs = key[order], h[order]
    starts = np.flatnonzero(np.r_[True, np.diff(key) != 0])
    counts = np.diff(np.r_[starts, len(key)])
    shape = (iy.max() + 1, nx)
    med = np.full(shape, np.nan)
    cnt = np.zeros(shape, int)
    for s, n in zip(starts, counts):
        v = hs[s:s + n]
        m = np.median(v)
        good = np.abs(v - m) <= max(max_mad, 2.5 * np.median(np.abs(v - m)))
        k = key[s]
        cnt[k // nx, k % nx] = good.sum()
        if good.sum() >= min_count:
            med[k // nx, k % nx] = np.median(v[good])
    return med, cnt, (x0, y0)


def grid_mode(xy, h, cell, min_count=3, band=1.0, lo=-3.0, hi=90.0, origin=None, shape=None):
    """Height map from the densest `band`-mm height bin per cell (ties: the higher bin),
    refined by the mean of the points within one bin of it. Unlike the median, a cell
    on a wall (points at every height) snaps to the top or the bed instead of a
    mid-wall value. Returns (height, count in the chosen band, share of the cell's
    points in it, origin)."""
    m = (h >= lo) & (h < hi)
    xy, h = xy[m], h[m]
    if origin is None:
        origin = tuple(np.floor(xy.min(0)))
    x0, y0 = origin
    ix = ((xy[:, 0] - x0) / cell).astype(np.int64)
    iy = ((xy[:, 1] - y0) / cell).astype(np.int64)
    if shape is None:
        shape = (int(iy.max()) + 1, int(ix.max()) + 1)
    ok = (ix >= 0) & (iy >= 0) & (ix < shape[1]) & (iy < shape[0])
    ix, iy, h = ix[ok], iy[ok], h[ok]
    cellid = iy * shape[1] + ix
    nb = int(np.ceil((hi - lo) / band)) + 1
    b = ((h - lo) / band).astype(np.int64)
    u, cnt = np.unique(cellid * nb + b, return_counts=True)
    uc, ub = u // nb, u % nb
    order = np.lexsort((ub, cnt, uc))
    last = np.r_[uc[order][1:] != uc[order][:-1], True]
    bc, bb, bn = uc[order][last], ub[order][last], cnt[order][last]
    lut = np.full(shape[0] * shape[1], -1, np.int64)
    lut[bc[bn >= min_count]] = bb[bn >= min_count]
    pb = lut[cellid]
    has = pb >= 0
    # seed at the densest bin's centre, then mean-shift (flat kernel +-radius) so the
    # estimate doesn't stick to the bin grid
    est = np.full(lut.size, np.nan)
    est[bc[bn >= min_count]] = lo + (bb[bn >= min_count] + 0.5) * band
    radius = 1.5 * band
    for _ in range(3):
        e = est[cellid]
        inb = has & (np.abs(h - e) <= radius)
        s = np.bincount(cellid[inb], weights=h[inb], minlength=lut.size)
        n = np.bincount(cellid[inb], minlength=lut.size)
        est = np.where(n > 0, s / np.maximum(n, 1), est)
    tot = np.bincount(cellid, minlength=lut.size)
    med = np.where(n > 0, est, np.nan).reshape(shape)
    share = np.where(tot > 0, n / np.maximum(tot, 1), 0.0).reshape(shape)
    return med, n.reshape(shape), share, origin


def _neighbours(g):
    pad = np.pad(g, 1, constant_values=np.nan)
    H, W = g.shape
    return np.stack([pad[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
                     for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dy, dx) != (0, 0)])


def needles(g, tol=3.0):
    """Cells more than tol mm above the median of their 8 neighbours (what renders as a spike)."""
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        nmed = np.nanmedian(_neighbours(g), 0)
    return np.isfinite(g) & (g - nmed > tol)


def support_filter(g, tol=1.5, min_nb=3, iters=3):
    """A surface holds together: keep a cell only if >= min_nb of its 8 neighbours lie
    within tol mm of it; otherwise give it the median of its neighbours. Walls keep
    their edges (an edge cell has neighbours on its own level on one side); single
    cells and pairs that stand out (reflections, mixed wall points) are replaced.
    Returns (filtered grid, number of cells changed)."""
    import warnings
    g = g.astype(np.float64).copy()
    changed = np.zeros(g.shape, bool)
    for _ in range(iters):
        nb = _neighbours(g)
        agree = (np.abs(nb - g[None]) <= tol).sum(0)
        bad = np.isfinite(g) & (agree < min_nb) & (np.isfinite(nb).sum(0) >= min_nb)
        if not bad.any():
            break
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            med = np.nanmedian(nb, 0)
        g[bad] = med[bad]
        changed |= bad
    return g, int(changed.sum())


def remove_specks(g, min_cells=20, tol=2.0):
    """Drop small islands: connected groups of cells (8-connected, neighbouring cells
    within tol mm) smaller than min_cells, e.g. a few stray points at the edge of the
    coverage or a reflection patch with nothing around it at its height.
    Returns (grid, cells removed)."""
    from scipy import ndimage
    g = g.copy()
    valid = np.isfinite(g)
    # label surfaces: cells joined when their heights agree within tol
    lab = np.zeros(g.shape, np.int64)
    # quantise heights into tol-wide levels and label each level separately (two passes
    # with offset levels so a surface on a level boundary is not split)
    removed = np.zeros(g.shape, bool)
    for shift in (0.0, tol / 2):
        q = np.floor((np.nan_to_num(g, nan=-1e6) + shift) / tol).astype(np.int64)
        small = np.zeros(g.shape, bool)
        for lv in np.unique(q[valid]):
            m = valid & (q == lv)
            l, n = ndimage.label(m, structure=np.ones((3, 3)))
            if n == 0:
                continue
            sizes = np.bincount(l.ravel())
            sizes[0] = 0
            small |= m & (sizes[l] < min_cells)
        removed = small if shift == 0.0 else (removed & small)
    g[removed] = np.nan
    return g, int(removed.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("points")
    ap.add_argument("--cell", type=float, default=0.5)
    ap.add_argument("--min-count", type=int, default=4)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    d = np.load(a.points)
    med, cnt, origin = grid(d["xy"], d["height"], a.cell, a.min_count)
    np.savez_compressed(a.out, height=med, count=cnt, origin=origin, cell=a.cell)
    print("grid %s, %.0f%% filled; wrote %s" % (med.shape, 100 * np.isfinite(med).mean(), a.out))


if __name__ == "__main__":
    main()
