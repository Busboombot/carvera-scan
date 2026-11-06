"""Heights from continuous line-laser sweeps (`carvera_scan.acquire.line_scan`, frames.mjpg
+ scan.json), calibrated by a Z sweep (`carvera_scan.process.zcal`).

    python -m carvera_scan.process.sweep_height data/2026-11-04/sweep1 --zcal data/2026-11-04/zcal1.json \
        --axis data/2026-11-04/axis1 --latency-search
    python -m carvera_scan.process.sweep_height data/2026-11-04/plate2 --zcal … --axis … --latency 0.004 --out plate2.npz

Per frame: the line (line_height.ridge/pick/runs). h = A(u) d / (1 + B d) from
the Z sweep, with d = the Z sweep's bed row - line row, minus the bed's own
reading at this scan's Z. This scan's most common row per column (its bed
line) sets which columns the fan covers and rejects rows below the bed.

Head position per frame: status samples during each pass give Y(t) (time =
middle of each status round trip); a frame's exposure is `latency` s before
its V4L2 timestamp. --latency-search finds the latency that makes passes in
opposite directions agree.

X, Y of a line point: image offset from the zoom centre, through the
motion-test matrix (bed px per mm), times (H - h) / H, plus the head position.
"""
import argparse
import json
import os

import cv2
import numpy as np

from carvera_scan.process.line_height import ROT, axis_matrix, orient, pick, ridge, runs


def load(scan):
    s = json.load(open(os.path.join(scan, "scan.json")))
    raw = open(os.path.join(scan, "frames.mjpg"), "rb").read()
    fr = s["frames"]
    jpg = [raw[o:o + n] for o, n in zip(fr["off"], fr["len"])]
    return s, np.array(fr["t"]), np.array(fr["pass"]), jpg


def lines(jpg, min_strength, rot="none"):
    """Line row per column for every frame (NaN where there is none)."""
    out = []
    for b in jpg:
        img = orient(cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_GRAYSCALE), rot)
        out.append(runs(pick(ridge(img), None, min_strength)[0]))
    return np.array(out)


def bed_row(V, nrows=800):
    """Most common row per column, smoothed by a quadratic: the line on the bed."""
    W = V.shape[1]
    u = np.arange(W, dtype=float)
    mode = np.full(W, np.nan)
    for j in range(W):
        c = V[:, j][np.isfinite(V[:, j])]
        if len(c) < 20:
            continue
        hist, edges = np.histogram(c, bins=np.arange(0, nrows + 2, 2))
        k = hist.argmax()
        if hist[k] >= 20:
            mode[j] = np.median(c[(c >= edges[k] - 2) & (c <= edges[k + 1] + 2)])
    # start from columns near the overall median row: junk modes at the frame
    # edge (rows 0, 728, ...) would otherwise bend the first fit
    ok = np.isfinite(mode) & (np.abs(mode - np.nanmedian(mode)) < 40)
    coef = np.polyfit(u[ok], mode[ok], 1)
    for tol in (10, 5, 3, 3):
        ok = np.isfinite(mode) & (np.abs(mode - np.polyval(coef, u)) < tol)
        coef = np.polyfit(u[ok], mode[ok], 2)
    # the laser fan: the longest stretch of agreeing columns (gaps <= 80);
    # stray agreeing columns at the frame edge are texture, not the line
    cols = np.flatnonzero(ok)
    breaks = np.flatnonzero(np.diff(cols) > 80)
    starts, ends = np.r_[0, breaks + 1], np.r_[breaks, len(cols) - 1]
    k = np.argmax(cols[ends] - cols[starts])
    return np.polyval(coef, u), (cols[starts[k]], cols[ends[k]])


def head_xy(s, t, p, latency):
    """Head X, Y at each frame's exposure time."""
    X, Y = np.full(len(t), np.nan), np.full(len(t), np.nan)
    for k, ps in enumerate(s["passes"]):
        st = np.array(ps["status"], float)
        ts = (st[:, 0] + st[:, 1]) / 2
        order = np.argsort(ts)
        m = p == k
        X[m] = ps["x"]
        Y[m] = np.interp(t[m] - latency, ts[order], st[order, 3])
        moving = (t[m] - latency > ts[order][0]) & (t[m] - latency < ts[order][-1])
        Y[np.flatnonzero(m)[~moving]] = np.nan
    return X, Y


def heights(d, u, cal):
    W = cal["width"]
    a0, a1 = cal["A"]
    return (a0 + a1 * (u - W / 2) / W) * d / (1 + cal["B"] * d)


def profile(Y, H, ybins):
    """Median height along Y over all line points (a 1-D signature of a pass)."""
    out = np.full(len(ybins) - 1, np.nan)
    idx = np.digitize(Y, ybins) - 1
    for i in range(len(out)):
        v = H[idx == i]
        if len(v) > 20:
            out[i] = np.percentile(v, 80)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan")
    ap.add_argument("--zcal", required=True)
    ap.add_argument("--axis", required=True)
    ap.add_argument("--latency", type=float, default=0.0, help="s from exposure to frame timestamp")
    ap.add_argument("--latency-search", action="store_true")
    ap.add_argument("--min-strength", type=float, default=10.0)
    ap.add_argument("--rot", choices=ROT, default="none", help="frame rotation (as for zcal.py)")
    ap.add_argument("--out")
    a = ap.parse_args()

    cal = json.load(open(a.zcal))
    s, t, p, jpg = load(a.scan)
    print("%d frames, %d passes, F%d" % (len(t), len(s["passes"]), s["feed"]))
    V = lines(jpg, a.min_strength, a.rot)
    W = V.shape[1]
    u = np.arange(W, dtype=float)
    nrows = orient(cv2.imdecode(np.frombuffer(jpg[0], np.uint8), cv2.IMREAD_GRAYSCALE), a.rot).shape[0]
    vbed, span = bed_row(V, nrows)
    V[:, (u < span[0]) | (u > span[1])] = np.nan
    V[V > vbed[None, :] + 4] = np.nan  # below the bed line: reflections
    # d from the Z sweep's own bed line (its first Z), so the calibration curve
    # applies as fitted; then subtract the bed's height on that curve at this
    # scan's Z (the curve isn't linear, so shifting the reference row would bend it)
    D = np.polyval(cal["bed_line"], u)[None, :] - V
    Hh = heights(D, u[None, :], cal) - (cal["z0"] - s["z"])
    on_bed = np.abs(V - vbed[None, :]) < 1.5
    print("bed line columns %d..%d; %d line points; bed points read %.3f mm (median), sd %.3f"
          % (*span, np.isfinite(D).sum(), np.nanmedian(Hh[on_bed]), np.nanstd(Hh[on_bed])))

    M = axis_matrix(a.axis, a.rot)
    Minv = np.linalg.inv(M)
    # H was measured at the Z sweep's first Z; the camera is higher by however
    # much the scan's Z is above that
    Hc, (cx, cy) = cal["H"] + (s["z"] - cal["z0"]), cal["zoom_centre"]
    print("camera %.1f mm above the bed at scan Z %.1f; zoom centre (%.0f, %.0f)" % (Hc, s["z"], cx, cy))
    rel = np.einsum("ij,jfu->ifu", Minv, np.stack([np.broadcast_to(u[None, :] - cx, V.shape), V - cy]))
    shrink = (Hc - Hh) / Hc

    def points(latency):
        Xh, Yh = head_xy(s, t, p, latency)
        return Xh[:, None] + rel[0] * shrink, Yh[:, None] + rel[1] * shrink

    if a.latency_search:
        # passes over the same X in opposite directions must give the same profile
        groups = {}
        for k, ps in enumerate(s["passes"]):
            groups.setdefault(ps["x"], []).append(k)
        pairs = [g[:2] for g in groups.values() if len(g) >= 2]
        ok = np.isfinite(Hh)
        best = None
        for lat in np.arange(-0.05, 0.1001, 0.002):
            Xp, Yp = points(lat)
            ybins = np.arange(np.nanmin(Yp), np.nanmax(Yp), 0.5)
            err = []
            for k1, k2 in pairs:
                m1, m2 = ok & (p[:, None] == k1), ok & (p[:, None] == k2)
                p1, p2 = profile(Yp[m1], Hh[m1], ybins), profile(Yp[m2], Hh[m2], ybins)
                both = np.isfinite(p1) & np.isfinite(p2)
                err.append(np.mean(np.abs(p1[both] - p2[both])))
            e = float(np.mean(err))
            if best is None or e < best[1]:
                best = (lat, e)
            print("latency %+6.3f s: mean |profile difference| %.3f mm" % (lat, e))
        print("best latency %.3f s (%.3f mm)" % best)
        a.latency = best[0]

    Xp, Yp = points(a.latency)
    good = np.isfinite(Hh) & np.isfinite(Xp) & np.isfinite(Yp) & (Hh > -10)
    if a.out:
        fi, ci = np.nonzero(good)
        np.savez_compressed(a.out, xy=np.stack([Xp[good], Yp[good]], 1), height=Hh[good],
                            ok=np.ones(good.sum(), bool), frame=fi, col=ci, row=V[good], passno=p[fi],
                            latency=a.latency)
        print("wrote %s: %d points (latency %.3f s)" % (a.out, good.sum(), a.latency))


if __name__ == "__main__":
    main()
