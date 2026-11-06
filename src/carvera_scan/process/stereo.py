"""Two-camera stereo from two line_scan.py sweeps over the same path (one per
camera: the two Arducams can't stream together on one USB 2 hub).

    python -m carvera_scan.process.stereo \
        --a data/2026-11-04/plate3_2.2 --a-zcal data/2026-11-04/zcal2-2.2.json --a-axis data/2026-11-04/axis3/2.2 \
        --b data/2026-11-04/plate3_2.3 --b-zcal data/2026-11-04/zcal2-2.3.json --b-axis data/2026-11-04/axis3/2.3 \
        --out data/2026-11-04/plate3-stereo.npz

No laser and no object sizes are used. Each camera has its own model from its
Z sweep and motion test (zcal.py, axis_test.py): a pixel p at height h is at
    head + r(p) (H - h) / H,   r(p) = M^-1 (p - c)
(rotated frame coordinates; c the zoom centre, H the camera's height above the
bed). Frames from the two sweeps are paired by head position. For a match pA,
pB in a pair:  rA (1 - h/HA) - rB (1 - h/HB) = O, with O the offset between
the two cameras' frames, so h follows by least squares (and the leftover is a
check that the match is real).

1. O and the bed-plane homography (A -> B) come from SIFT matches on many
   pairs: the bed (h = 0) is the dominant plane.
2. Dense: B and A warped onto B through the bed homography are matched with
   SGBM at half size (the bed lines up; a point h above it shifts along the
   baseline, up to ~600 px for a 3" top). Each match is triangulated.
"""
import argparse
import json

import cv2
import numpy as np

from carvera_scan.process.line_height import ROT, axis_matrix
from carvera_scan.process.sweep_height import head_xy, load


class Cam:
    def __init__(self, scan, zcal, axis, latency):
        self.s, self.t, self.p, self.jpg = load(scan)
        cal = json.load(open(zcal))
        self.rot = cal.get("rot", "none")
        self.H = cal["H"] + (self.s["z"] - cal["z0"])
        self.c = np.array(cal["zoom_centre"])
        self.Minv = np.linalg.inv(axis_matrix(axis, self.rot))
        self.X, self.Y = head_xy(self.s, self.t, self.p, latency)

    def img(self, k):
        return cv2.imdecode(np.frombuffer(self.jpg[k], np.uint8), cv2.IMREAD_GRAYSCALE)

    def r(self, pts):
        """Bed-scale offset (mm) from the camera's nadir, for raw (unrotated) pixel coordinates."""
        x, y = pts[..., 0], pts[..., 1]
        if self.rot == "cw":       # cv2.ROTATE_90_CLOCKWISE on an 800-row frame
            x, y = 799 - y, x
        elif self.rot == "ccw":    # ROTATE_90_COUNTERCLOCKWISE on a 1280-column frame
            x, y = y, 1279 - x
        q = np.stack([x - self.c[0], y - self.c[1]], -1)
        return q @ self.Minv.T


def pairs(A, B, every):
    """(frame in A, frame in B) at the same head position, same pass."""
    out = []
    for k in np.unique(A.p):
        ia = np.flatnonzero((A.p == k) & np.isfinite(A.Y))
        ib = np.flatnonzero((B.p == k) & np.isfinite(B.Y))
        if not len(ia) or not len(ib):
            continue
        order = np.argsort(B.Y[ib])
        yb = B.Y[ib][order]
        for i in ia[::every]:
            j = np.searchsorted(yb, A.Y[i])
            cand = [c for c in (j - 1, j) if 0 <= c < len(yb)]
            c = min(cand, key=lambda c: abs(yb[c] - A.Y[i]))
            if abs(yb[c] - A.Y[i]) < 0.15 and abs(A.X[i] - B.X[ib[order[c]]]) < 0.01:
                out.append((i, ib[order[c]]))
    return out


def triangulate(rA, rB, HA, HB, O):
    """Height from matched bed-scale offsets; returns h, residual (mm)."""
    lhs = rA / HA - rB / HB            # coefficient of h
    rhs = rA - rB - O
    h = (lhs * rhs).sum(-1) / np.maximum((lhs * lhs).sum(-1), 1e-12)
    res = np.linalg.norm(rhs - h[..., None] * lhs, axis=-1)
    return h, res


def main():
    ap = argparse.ArgumentParser()
    for k in "ab":
        ap.add_argument("--" + k, required=True)
        ap.add_argument("--%s-zcal" % k, required=True)
        ap.add_argument("--%s-axis" % k, required=True)
    ap.add_argument("--latency", type=float, default=0.004)
    ap.add_argument("--every", type=int, default=8, help="use every Nth frame (0.25 mm each)")
    ap.add_argument("--scale", type=float, default=0.5, help="image scale for dense matching")
    ap.add_argument("--max-res", type=float, default=1.5, help="mm: drop matches the two models disagree on")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    A = Cam(a.a, a.a_zcal, a.a_axis, a.latency)
    B = Cam(a.b, a.b_zcal, a.b_axis, a.latency)
    P = pairs(A, B, a.every)
    print("%d frame pairs; cameras %.1f / %.1f mm above the bed" % (len(P), A.H, B.H))

    # 1. bed homography and camera offset from SIFT matches on a spread of pairs
    sift, cl = cv2.SIFT_create(6000), cv2.createCLAHE(2.0, (8, 8))
    pa_all, pb_all = [], []
    for i, j in P[::max(1, len(P) // 60)]:
        ka, da = sift.detectAndCompute(cl.apply(A.img(i)), None)
        kb, db = sift.detectAndCompute(cl.apply(B.img(j)), None)
        if da is None or db is None:
            continue
        mt = [m for m, n in cv2.BFMatcher().knnMatch(da, db, k=2) if m.distance < 0.7 * n.distance]
        pa_all += [ka[m.queryIdx].pt for m in mt]
        pb_all += [kb[m.trainIdx].pt for m in mt]
    pa_all, pb_all = np.float32(pa_all), np.float32(pb_all)
    H0, inl = cv2.findHomography(pa_all, pb_all, cv2.RANSAC, 2.0)
    inl = inl.ravel().astype(bool)
    O = np.median(A.r(pa_all[inl]) - B.r(pb_all[inl]), 0)
    h_bed, res_bed = triangulate(A.r(pa_all[inl]), B.r(pb_all[inl]), A.H, B.H, O)
    print("bed homography from %d of %d matches; camera offset O = (%.2f, %.2f) mm; bed matches: h sd %.2f mm"
          % (inl.sum(), len(inl), *O, np.std(h_bed)))
    # sparse check: all matches, triangulated
    h_all, res_all = triangulate(A.r(pa_all), B.r(pb_all), A.H, B.H, O)
    ok = res_all < a.max_res
    print("all SIFT matches: %d, %d consistent (residual < %.1f mm); heights %s" % (
        len(h_all), ok.sum(), a.max_res, np.round(np.percentile(h_all[ok], [5, 25, 50, 75, 95]), 1).tolist()))

    # 2. dense: SGBM on B and A warped onto B, at reduced scale
    s = a.scale
    S = np.diag([s, s, 1.0])
    Hs = S @ H0 @ np.linalg.inv(S)
    w, hgt = int(1280 * s), int(800 * s)
    ndisp = 16 * int(np.ceil(700 * s / 16))
    sg = cv2.StereoSGBM_create(minDisparity=-16, numDisparities=ndisp, blockSize=7, P1=8 * 49, P2=32 * 49,
                               uniquenessRatio=10, speckleWindowSize=100, speckleRange=2, disp12MaxDiff=2)
    vv, uu = np.mgrid[:hgt, :w].astype(np.float32)
    Hinv = np.linalg.inv(Hs)
    xs, ys, hs, rs = [], [], [], []
    for n, (i, j) in enumerate(P):
        ib = cl.apply(cv2.resize(B.img(j), (w, hgt), interpolation=cv2.INTER_AREA))
        ia = cl.apply(cv2.warpPerspective(cv2.resize(A.img(i), (w, hgt), interpolation=cv2.INTER_AREA), Hs, (w, hgt)))
        valid = cv2.warpPerspective(np.full((hgt, w), 255, np.uint8), Hs, (w, hgt), flags=cv2.INTER_NEAREST) > 0
        # A raised point lands left of its B position in warped A (sparse matches:
        # -65 px at 15 mm, -311 px at 51 mm, full scale), so B is the left image
        d = sg.compute(ib, ia).astype(np.float32) / 16
        m = (d > -15) & valid
        ua, ub = uu[m] - d[m], uu[m]   # pixel u in B matches u - d in warped A
        va = vb = vv[m]
        pb = np.stack([ub, vb], -1) / s
        qa = np.stack([ua, va, np.ones_like(ua)], -1) @ Hinv.T
        pa = qa[:, :2] / qa[:, 2:] / s
        h, res = triangulate(A.r(pa), B.r(pb), A.H, B.H, O)
        good = (res < a.max_res) & (h > -10) & (h < 100)
        rb = B.r(pb[good])
        k = (B.H - h[good]) / B.H
        xs.append(B.X[j] + rb[:, 0] * k)   # in B's frame
        ys.append(B.Y[j] + rb[:, 1] * k)
        hs.append(h[good]); rs.append(res[good])
        if n % 100 == 0:
            print("  pair %d/%d: %d matched px, %d consistent" % (n, len(P), m.sum(), good.sum()), flush=True)
    xy = np.stack([np.concatenate(xs), np.concatenate(ys)], 1)
    h, r = np.concatenate(hs), np.concatenate(rs)
    np.savez_compressed(a.out, xy=xy, height=h, res=r, ok=np.ones(len(h), bool), O=O, H0=H0, frame="B")
    print("wrote %s: %d points (in camera B's frame)" % (a.out, len(h)))


if __name__ == "__main__":
    main()
