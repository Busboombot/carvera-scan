"""Heights from a line-laser sweep (`carvera_scan.acquire.line_scan`) with the Arducam on the head.

    python -m carvera_scan.process.line_height data/2026-11-04/sweep1 --axis data/2026-11-04/axis1 --hist
    python -m carvera_scan.process.line_height data/2026-11-04/sweep1 --axis data/2026-11-04/axis1 \
        --cal 38.1:25.4 79.5:50.8 --out data/2026-11-04/sweep1.npz

0. Head position: a sweep (frames.mjpg + scan.json) records status samples
   while the head moves. Per pass a straight line through them (the cruise,
   inside --y) gives Y against time; each frame is placed at its timestamp
   minus --tau, the camera's delay from exposure to timestamp. Passes at one
   X in opposite directions must agree; the script reports the tau that
   lines them up. Stepped scans (NNNN.jpg + log.jsonl) still load.

1. Line: a vertical top-hat picks thin bright ridges; per image column the
   strongest ridge at or above the bed line (heights are >= 0, so anything
   below it is a reflection) is taken, refined to sub-pixel by a centroid.
2. Bed line: the most common row per column over all frames (most columns
   see the bed most of the time), smoothed by a quadratic in u.
3. Height: d = bed row - line row (px). A camera looking down at a tilted
   laser plane gives h = A d / (1 + B d); --cal d:h pairs (block tops) fit A
   and B. Then H = A / B is the camera height above the bed.
4. Position: the image offset from the centre maps to bed mm through the
   motion-test matrix (axis_test.py: image px per mm of X and Y, roll
   included), shrunk by (H - h) / H for raised points, plus the head position.

Writes xy (head-referenced machine X, Y), height, ok for heightmap.py.
"""
import argparse
import glob
import json
import os
from concurrent.futures import ProcessPoolExecutor

import cv2
import numpy as np

KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 11))


def ridge(img):
    """Top-hat response: thin bright horizontal-ish lines stand out."""
    return cv2.morphologyEx(cv2.GaussianBlur(img, (3, 3), 0), cv2.MORPH_TOPHAT, KERNEL).astype(np.float32)


def pick(th, limit=None, min_strength=20.0):
    """Per column: sub-pixel row of the strongest ridge (rows <= limit[u]), and its strength."""
    t = th.copy()
    if limit is not None:
        rows = np.arange(t.shape[0])[:, None]
        t[rows > limit[None, :]] = 0
    v = t.argmax(axis=0)
    s = t[v, np.arange(t.shape[1])]
    lo = np.clip(v - 3, 0, t.shape[0] - 7)
    idx = lo[None, :] + np.arange(7)[:, None]
    w = np.take_along_axis(t, idx, axis=0)
    w = np.maximum(w - 0.3 * s[None, :], 0)
    vs = (w * idx).sum(0) / np.maximum(w.sum(0), 1e-6)
    vs[s < min_strength] = np.nan
    return vs, s


def runs(v, max_step=2.0, min_len=12):
    """Keep only points in runs of >= min_len neighbouring columns whose rows differ by < max_step.

    The laser line is continuous along a surface; texture picked where the
    line is hidden is scattered.
    """
    out = np.full_like(v, np.nan)
    link = np.abs(np.diff(v)) < max_step  # NaN compares False
    start = 0
    for j in range(1, len(v) + 1):
        if j == len(v) or not link[j - 1]:
            if j - start >= min_len:
                out[start:j] = v[start:j]
            start = j
    return out


def head_track(p, lo, hi, max_rtt=0.02):
    """Y(t) for one pass: a line through the status samples inside [lo, hi].

    Each sample is timed at the midpoint of its request and reply; slow
    replies (> max_rtt s) are dropped, then 3-sigma outliers.
    """
    a = np.array(p["status"])
    t, y = (a[:, 0] + a[:, 1]) / 2, a[:, 3]
    sel = (a[:, 1] - a[:, 0] < max_rtt) & (y >= lo) & (y <= hi)
    for _ in range(3):
        c = np.polyfit(t[sel], y[sel], 1)
        r = y - np.polyval(c, t)
        sel &= np.abs(r) < 3 * r[sel].std() + 1e-3
    return c, r[sel].std(), sel.sum()


def load(scan, tau):
    """JPEG bytes, head machine (X, Y) and pass number per frame.

    Sweep frames outside the --y range given to line_scan.py (the run-up)
    are dropped.
    """
    if not os.path.exists(os.path.join(scan, "scan.json")):
        recs = [json.loads(l) for l in open(os.path.join(scan, "log.jsonl"))]
        frames = sorted(glob.glob(os.path.join(scan, "[0-9][0-9][0-9][0-9].jpg")))
        assert len(frames) == len(recs), (len(frames), len(recs))
        head = np.array([r["mpos"][:2] for r in recs])
        return [open(f, "rb").read() for f in frames], head, np.zeros(len(recs), int), []
    s = json.load(open(os.path.join(scan, "scan.json")))
    raw = open(os.path.join(scan, "frames.mjpg"), "rb").read()
    fr = s["frames"]
    t, k = np.array(fr["t"]), np.array(fr["pass"])
    lo, hi = sorted(s["y"])
    head = np.full((len(t), 2), np.nan)
    tracks = []
    for i, p in enumerate(s["passes"]):
        c, rms, n = head_track(p, lo, hi)
        tracks.append((p["x"], c))
        print("pass %d X %.1f: %.3f mm/s, track rms %.3f mm (%d samples)" % (i, p["x"], c[0], rms, n))
        head[k == i] = np.c_[np.full((k == i).sum(), p["x"]), np.polyval(c, t[k == i] - tau)]
    keep = (head[:, 1] >= lo) & (head[:, 1] <= hi)
    jpegs = [raw[o:o + n] for o, n, kp in zip(fr["off"], fr["len"], keep) if kp]
    return jpegs, head[keep], k[keep], [(x, c, t[(k == i) & keep]) for i, (x, c) in enumerate(tracks)]


def frame_line(args):
    """Decode one frame and pick its line (worker for the process pool)."""
    jpeg, limit, min_strength = args
    return runs(pick(ridge(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)),
                     limit, min_strength)[0])


def lines(pool, jpegs, limit, min_strength):
    return np.array(list(pool.map(frame_line, [(j, limit, min_strength) for j in jpegs], chunksize=16)))


def timing(d, k, tracks, taus=np.arange(-0.04, 0.0401, 0.001)):
    """Mismatch between opposite passes at one X against tau; prints the best tau.

    At the same head Y the camera sees the same thing whichever way the head
    is moving, so d(Y) per column must agree between the two passes once tau
    is right. A tau error e shifts the passes apart by 2 v e.
    """
    for i in range(len(tracks)):
        for j in range(i + 1, len(tracks)):
            (xi, ci, ti), (xj, cj, tj) = tracks[i], tracks[j]
            if xi != xj or np.sign(ci[0]) == np.sign(cj[0]):
                continue
            di, dj = d[k == i], d[k == j]
            cols = np.nonzero((np.isfinite(di).mean(0) > 0.5) & (np.isfinite(dj).mean(0) > 0.5))[0][::4]
            err = []
            for tau in taus:
                yi, yj = np.polyval(ci, ti - tau), np.polyval(cj, tj - tau)
                oj = np.argsort(yj)
                e = []
                for u in cols:
                    gi, gj = np.isfinite(di[:, u]), np.isfinite(dj[oj, u])
                    dv = np.interp(yi[gi], yj[oj][gj], dj[oj, u][gj], left=np.nan, right=np.nan) - di[gi, u]
                    e.append(dv[np.isfinite(dv)])
                e = np.abs(np.concatenate(e))
                err.append(np.mean(np.minimum(e, 10)))
            b = int(np.argmin(err))
            print("timing, passes %d/%d at X %.1f: best tau %.0f ms (mean |dd| %.2f px; at tau 0: %.2f px)"
                  % (i, j, xi, taus[b] * 1e3, err[b], err[np.argmin(np.abs(taus))]))


ROT = {"none": None, "cw": cv2.ROTATE_90_CLOCKWISE, "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE}


def orient(img, rot="none"):
    """Turn a frame so the laser line runs along the rows and rises (smaller
    row) with height: cameras mounted at 90 degrees need "cw" or "ccw"."""
    return img if ROT[rot] is None else cv2.rotate(img, ROT[rot])


def axis_matrix(axis_dir, rot="none"):
    """Image px per mm of (X, Y) of a bed point relative to the head, from axis_test.py frames.

    A matrix.json in axis_dir (a matrix found another way) takes precedence."""
    fixed = os.path.join(axis_dir, "matrix.json")
    if os.path.exists(fixed):
        return np.array(json.load(open(fixed))["matrix"])
    mv = json.load(open(os.path.join(axis_dir, "moves.json")))
    a = orient(cv2.imread(os.path.join(axis_dir, "0.png"), 0), rot).astype(np.float32)
    win = cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F)
    cols = []
    for k, step in ((1, mv["dx"]), (2, mv["dy"])):
        b = orient(cv2.imread(os.path.join(axis_dir, "%d.png" % k), 0), rot).astype(np.float32)
        (sx, sy), _ = cv2.phaseCorrelate(a, b, win)
        # the head moved by `step`; a bed point moved by -step relative to the head
        cols.append([sx / -step, sy / -step])
    return np.array(cols).T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan")
    ap.add_argument("--axis", required=True, help="axis_test.py output dir")
    ap.add_argument("--cal", nargs="*", default=[], help="d:h pairs (px:mm) from known tops")
    ap.add_argument("--min-strength", type=float, default=10.0,
                    help="top-hat peak; the line on the bed is ~85 (5th pct 29), dimmer on bare aluminum; "
                         "texture is ~22 but scattered, and runs() drops it")
    ap.add_argument("--hist", action="store_true", help="print a histogram of d to find the tops")
    ap.add_argument("--tau", type=float, default=0.0, help="s from exposure to frame timestamp")
    ap.add_argument("--out")
    a = ap.parse_args()

    jpegs, head, npass, tracks = load(a.scan, a.tau)
    H_img, W = cv2.imdecode(np.frombuffer(jpegs[0], np.uint8), cv2.IMREAD_GRAYSCALE).shape
    u = np.arange(W, dtype=float)
    pool = ProcessPoolExecutor()

    # bed line: most common row per column (the line spans only part of the
    # frame width; columns where no row repeats much are outside the fan)
    raw = lines(pool, jpegs, None, a.min_strength)
    mode, count = np.full(W, np.nan), np.zeros(W)
    for j in range(W):
        c = raw[:, j]
        c = c[np.isfinite(c)]
        hist, edges = np.histogram(c, bins=np.arange(0, H_img + 2, 2))
        b = hist.argmax()
        if hist[b] >= max(20, 0.03 * len(jpegs)):
            sel = c[(c >= edges[b] - 2) & (c <= edges[b + 1] + 2)]
            mode[j], count[j] = np.median(sel), hist[b]
    # weighted by how often the row repeats, so stray columns (a feature
    # that the line keeps hitting) don't drag the first fit off
    ok = np.isfinite(mode)
    for lim in (30, 10, 3, 3, 3):
        coef = np.polyfit(u[ok], mode[ok], 2, w=np.sqrt(count[ok]))
        res = mode - np.polyval(coef, u)
        ok = np.isfinite(mode) & (np.abs(res) < lim)
    vbed = np.polyval(coef, u)
    span = (u[ok].min(), u[ok].max())
    print("bed line: v = %.3g u^2 %+.4f u %+.1f  (%d columns, u %d..%d, rms %.2f px)"
          % (*coef, ok.sum(), *span, np.sqrt(np.mean((mode[ok] - vbed[ok]) ** 2))))

    v = lines(pool, jpegs, vbed + 4, a.min_strength)
    v[:, (u < span[0]) | (u > span[1])] = np.nan
    d = vbed[None, :] - v
    good = np.isfinite(d)
    print("%d frames x %d columns: %d line points" % (len(jpegs), W, good.sum()))
    timing(d, npass, tracks)

    if a.hist:
        dd = d[good]
        hist, edges = np.histogram(dd, bins=np.arange(-4, np.nanmax(dd) + 2, 1.0))
        for k in range(len(hist)):
            if hist[k] >= 50:
                print("  d %6.1f .. %6.1f px: %7d %s" % (edges[k], edges[k + 1], hist[k], "#" * min(80, hist[k] // 500)))

    if not a.cal:
        return
    pairs = np.array([[float(x) for x in p.split(":")] for p in a.cal] + [[0.0, 0.0]])
    dc, hc = pairs[:, 0], pairs[:, 1]
    # h (1 + B d) = A d  ->  A d - B h d = h : linear in A, B
    A_, B_ = np.linalg.lstsq(np.c_[dc, -hc * dc], hc, rcond=None)[0]
    Hcam = A_ / B_
    print("height model: h = %.4f d / (1 %+.6f d); camera %.1f mm above the bed" % (A_, B_, Hcam))
    for di, hi in zip(dc, hc):
        print("  d %6.1f px -> %6.2f mm (given %.2f)" % (di, A_ * di / (1 + B_ * di), hi))
    h = A_ * d / (1 + B_ * d)

    M = axis_matrix(a.axis)
    Minv = np.linalg.inv(M)
    print("image px per mm (columns X, Y):", M.round(3).tolist())
    du = np.broadcast_to(u[None, :] - W / 2, d.shape)
    dv = v - H_img / 2
    rel = np.einsum("ij,jfu->ifu", Minv, np.stack([du, dv]))  # bed mm, relative to head
    shrink = (Hcam - h) / Hcam
    X = head[:, 0, None] + rel[0] * shrink
    Y = head[:, 1, None] + rel[1] * shrink
    good &= h > -10
    if a.out:
        np.savez_compressed(a.out, xy=np.stack([X[good], Y[good]], 1), height=h[good],
                            ok=np.ones(good.sum(), bool), d=d[good], frame=np.nonzero(good)[0])
        print("wrote", a.out, good.sum(), "points")


if __name__ == "__main__":
    main()
