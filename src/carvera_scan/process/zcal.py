"""Line-laser calibration from a Z sweep over bare bed (`carvera_scan.acquire.z_sweep`).

    python -m carvera_scan.process.zcal data/2026-11-04/zcal1 --holes 205 290 458 302 962 322 975 72 \\
        --out data/2026-11-04/zcal1.json

Lowering the head by D mm is the same, to the camera and the laser, as the bed
rising D mm.
- Height: per frame the line is found per column (strongest ridge, kept where
  it continues); the bed part is the largest straight run (RANSAC), so the bar
  and screws drop out. Fits h = A(u) d / (1 + B d), A linear in the column u,
  d = row of the line on the bed at the first Z minus the row now.
- Zoom: bed holes picked on bare bed (--holes) are tracked through the frames
  by template matching. A point zooms about a fixed image point c:
  p(D) = c + (p0 - c) H / (H - D), H = camera height above the bed at the
  first Z. All tracks are fitted together. (Whole-image registration and blob
  detection both failed here: the frames are dark, the fixed sensor/JPEG
  pattern wins over the bed texture, and blob centroids are noisy.)

Writes JSON for line_height.py: bed line, A, B, H, zoom centre, residuals.
"""
import argparse
import json
import os

import cv2
import numpy as np
from scipy.optimize import least_squares

from carvera_scan.process.line_height import ROT, orient, pick, ridge, runs


def bed_line(v, u, tol=1.5):
    """RANSAC straight line through the finite points of v(u); returns inlier mask."""
    ok = np.flatnonzero(np.isfinite(v))
    rng = np.random.default_rng(0)
    best = np.zeros(len(v), bool)
    for _ in range(300):
        i, j = rng.choice(ok, 2, replace=False)
        m = (v[j] - v[i]) / (u[j] - u[i])
        inl = np.zeros(len(v), bool)
        inl[ok] = np.abs(v[ok] - (v[i] + m * (u[ok] - u[i]))) < tol
        if inl.sum() > best.sum():
            best = inl
    c = np.polyfit(u[best], v[best], 1)
    inl = np.zeros(len(v), bool)
    inl[ok] = np.abs(v[ok] - np.polyval(c, u[ok])) < tol
    return inl


def track_holes(imgs, D, seeds, H_guess=170.0, c_guess=(610.0, 430.0)):
    """Track bed holes (seeds: (x, y) in the first frame, picked on bare bed)
    by normalized cross-correlation with the first frame's patch, rescaled to
    the predicted zoom. Returns a list of {frame: (x, y)}."""
    prep = lambda im: cv2.GaussianBlur(cv2.createCLAHE(2.0, (16, 16)).apply(im), (0, 0), 1.5)
    imgs = [prep(im) for im in imgs]
    c = np.array(c_guess)
    zoom = lambda k: H_guess / (H_guess - D[k])
    tracks = []
    for x, y in seeds:
        w = imgs[0][y - 25:y + 25, x - 25:x + 25].astype(float)  # centre on the dark hole
        ys, xs = np.nonzero(w < np.percentile(w, 12))
        x, y = x - 25 + xs.mean(), y - 25 + ys.mean()
        T0 = imgs[0][int(y) - 30:int(y) + 30, int(x) - 30:int(x) + 30]
        pos = {0: np.array([x, y])}
        for k in range(1, len(imgs)):
            last = max(pos)
            pred = pos[last] + (pos[0] - c) * (zoom(k) - zoom(last))
            T = cv2.resize(T0, None, fx=zoom(k), fy=zoom(k), interpolation=cv2.INTER_LINEAR)
            th, tw = T.shape
            R = 30
            x0, y0 = int(pred[0] - tw / 2 - R), int(pred[1] - th / 2 - R)
            if x0 < 0 or y0 < 0 or x0 + tw + 2 * R > imgs[k].shape[1] or y0 + th + 2 * R > imgs[k].shape[0]:
                break
            res = cv2.matchTemplate(imgs[k][y0:y0 + th + 2 * R, x0:x0 + tw + 2 * R], T, cv2.TM_CCOEFF_NORMED)
            _, mx, _, (px, py) = cv2.minMaxLoc(res)
            if mx < 0.6:
                break
            dx = dy = 0.0
            if 0 < px < res.shape[1] - 1 and 0 < py < res.shape[0] - 1:  # parabolic sub-pixel peak
                dx = 0.5 * (res[py, px - 1] - res[py, px + 1]) / (res[py, px - 1] - 2 * res[py, px] + res[py, px + 1])
                dy = 0.5 * (res[py - 1, px] - res[py + 1, px]) / (res[py - 1, px] - 2 * res[py, px] + res[py + 1, px])
            pos[k] = np.array([x0 + px + dx + tw / 2, y0 + py + dy + th / 2])
        tracks.append(pos)
    return tracks


def fit_zoom(tracks, D, W, Hgt, max_rms=2.5):
    """Joint fit of H, zoom centre and each track's position at D = 0.

    A track that doesn't follow the bed's zoom (a mistrack) is dropped, worst
    first, until every track fits to max_rms px. Returns H, centre, rms,
    1-sigma (H, cx, cy), tracks kept.
    """
    def res(x, tr):
        H, c = x[0], x[1:3]
        out = []
        for i, t in enumerate(tr):
            p0 = x[3 + 2 * i:5 + 2 * i]
            for k, p in t.items():
                out.append(c + (p0 - c) * H / (H - D[k]) - p)
        return np.concatenate(out)
    tr = list(tracks)
    while True:
        x0 = [200.0, W / 2, Hgt / 2] + [v for t in tr for v in t[min(t)]]
        sol = least_squares(res, x0, args=(tr,))
        per, i = [], 0
        for t in tr:
            n = 2 * len(t)
            per.append(np.sqrt(np.mean(sol.fun[i:i + n] ** 2)))
            i += n
        worst = int(np.argmax(per))
        if per[worst] <= max_rms or len(tr) <= 3:
            break
        tr.pop(worst)
    cov = np.linalg.pinv(sol.jac.T @ sol.jac) * np.mean(sol.fun ** 2)
    return sol.x[0], sol.x[1:3], np.sqrt(np.mean(sol.fun ** 2)), np.sqrt(np.diag(cov)[:3]), tr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("zdir")
    ap.add_argument("--holes", type=int, nargs="+", required=True,
                    help="x y x y ...: bed holes on bare bed in the first frame, spread across it")
    ap.add_argument("--rot", choices=ROT, default="none",
                    help="turn frames so the line runs along the rows and rises with height")
    ap.add_argument("--out")
    a = ap.parse_args()
    recs = [json.loads(l) for l in open(os.path.join(a.zdir, "log.jsonl"))]
    z0 = recs[0]["mpos"][2]
    imgs = [orient(cv2.imread(os.path.join(a.zdir, r["file"]), 0), a.rot) for r in recs]
    Hgt, W = imgs[0].shape
    u = np.arange(W, dtype=float)
    Dk = np.array([z0 - r["mpos"][2] for r in recs])

    vs = []
    for img in imgs:
        v = runs(pick(ridge(img), None, 10)[0])
        v[~bed_line(v, u)] = np.nan
        vs.append(v)
    ok0 = np.isfinite(vs[0])
    cref = np.polyfit(u[ok0], vs[0][ok0], 2)
    print("first frame bed line: v = %.3g u^2 %+.4f u %+.2f, columns %d..%d (%d)"
          % (*cref, u[ok0].min(), u[ok0].max(), ok0.sum()))

    D, d, uu = [], [], []
    for dz, v in zip(Dk, vs):
        ok = np.isfinite(v)
        D.append(np.full(ok.sum(), dz)); d.append(np.polyval(cref, u[ok]) - v[ok]); uu.append(u[ok])
    D, d, uu = map(np.concatenate, (D, d, uu))
    uc = (uu - W / 2) / W
    # h (1 + B d) = (a0 + a1 uc) d  ->  linear in a0, a1, B
    (a0, a1, B), *_ = np.linalg.lstsq(np.c_[d, d * uc, -D * d], D, rcond=None)
    res = D - (a0 + a1 * uc) * d / (1 + B * d)
    rms = float(np.sqrt(np.mean(res ** 2)))
    print("h = (%.5f %+.5f uc) d / (1 %+.6f d), uc = (u - %d) / %d;  rms %.3f mm, max %.3f, %d points" %
          (a0, a1, B, W / 2, W, rms, np.abs(res).max(), len(D)))
    for k in np.unique(D)[::3]:
        m = D == k
        print("   D %5.1f: %4d columns, d %6.2f px, residual mean %+.3f sd %.3f"
              % (k, m.sum(), np.median(d[m]), res[m].mean(), res[m].std()))

    tracks = track_holes(imgs, Dk, list(zip(a.holes[0::2], a.holes[1::2])))
    H, c, zrms, sig, kept = fit_zoom(tracks, Dk, W, Hgt)
    print("zoom: %d of %d hole tracks fit; camera %.1f +- %.1f mm above the bed at Z %.2f; centre (%.1f, %.1f)"
          " +- (%.1f, %.1f) px; rms %.2f px" % (len(kept), len(tracks), H, sig[0], z0, *c, *sig[1:], zrms))
    for t in kept:
        k0, k1 = min(t), max(t)
        print("   track D %4.0f..%4.0f: (%4.0f, %4.0f) -> (%4.0f, %4.0f)" % (Dk[k0], Dk[k1], *t[k0], *t[k1]))

    if a.out:
        json.dump({"z0": z0, "width": W, "rot": a.rot, "bed_line": cref.tolist(), "A": [a0, a1], "B": B, "rms_mm": rms,
                   "D_max": float(D.max()), "H": H, "H_sd": sig[0], "zoom_centre": c.tolist()},
                  open(a.out, "w"), indent=1)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
