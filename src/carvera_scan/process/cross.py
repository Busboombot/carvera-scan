"""Line-laser *cross* at any angle: find each arm in a frame, calibrate height
per arm from a Z sweep, and turn sweeps into points.

    python -m carvera_scan.process.cross angles data/2026-11-05/zcal45/1.2
    python -m carvera_scan.process.cross zcal data/2026-11-05/zcal45/1.2 --out data/2026-11-05/zcal45/1.2.json
    python -m carvera_scan.process.cross sweep data/2026-11-05/plate3_1.2 --zcal …/1.2.json --axis …/axis45/1.2 --out plate3_1.2.npz

Arms: the frame is turned so the arm runs along the rows (other arm then
runs down the columns, and the row top-hat ignores it), the strongest ridge
per column near the arm's bed row is taken (line_height.ridge/pick/runs),
and the points are mapped back to the original frame.

Height: a laser plane seen by a pinhole camera gives a projective map from
image point to height, h = (a0 + a1 u + a2 v + quadratic) / (1 + b1 u + b2 v),
fitted (linearised) per arm to the Z sweep, where lowering the head D mm
puts the bed at height D. The quadratic terms absorb lens distortion.
"""
import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import cv2
import numpy as np

from carvera_scan.process.line_height import pick, ridge, runs

W, H = 1280, 800
SIDE = 1520  # canvas for the turned frame (holds the whole frame at any angle)


def turn(theta):
    """Affine taking original (u, v) to the turned canvas, arm at angle theta (deg) horizontal."""
    M = cv2.getRotationMatrix2D((W / 2, H / 2), theta, 1.0)
    M[:, 2] += (SIDE / 2 - W / 2, SIDE / 2 - H / 2)
    return M


def find_angles(img, lo=20, hi=70):
    """The two arm angles (deg, image u to the right, v down; positive = arm going up-right)."""
    th = cv2.morphologyEx(cv2.GaussianBlur(img, (3, 3), 0), cv2.MORPH_TOPHAT, np.ones((15, 15), np.uint8)).astype(np.float32)
    best = {}
    for sgn in (+1, -1):
        scores = []
        for a in np.arange(lo, hi, 0.1):
            t = cv2.warpAffine(th, turn(sgn * a), (SIDE, SIDE))
            scores.append((t.sum(1).max(), sgn * a))
        best[sgn] = max(scores)[1]
    return best[+1], best[-1]


def arm_points(img, theta, band=None, min_strength=10.0, ambig=None, ambig_sep=6, min_len=12):
    """(u, v) of one arm's ridge in the original frame. band = (row_lo, row_hi) per turned column (arrays).

    ambig: if set (e.g. 0.6), a column whose second-strongest ridge (more than
    ambig_sep rows from the strongest) reaches ambig x the strongest is dropped:
    two candidate lines (a reflection and the real one) can't be told apart.
    min_len: shortest run of neighbouring columns kept (line_height.runs)."""
    M = turn(theta)
    t = cv2.warpAffine(img, M, (SIDE, SIDE), flags=cv2.INTER_LINEAR)
    th = ridge(t)
    if band is not None:
        rows = np.arange(SIDE)[:, None]
        th[(rows < band[0][None, :]) | (rows > band[1][None, :])] = 0
    v, s = pick(th, None, min_strength)
    if ambig is not None:
        top = th.argmax(0)
        rows = np.arange(SIDE)[:, None]
        t2 = np.where(np.abs(rows - top[None, :]) > ambig_sep, th, 0)
        s2 = t2.max(0)
        v[(s2 >= ambig * s) & (s > 0)] = np.nan
    v = runs(v, min_len=min_len)
    cols = np.flatnonzero(np.isfinite(v))
    Mi = cv2.invertAffineTransform(M)
    pts = np.c_[cols, v[cols], np.ones(len(cols))] @ Mi.T
    inside = (pts[:, 0] >= 2) & (pts[:, 0] <= W - 3) & (pts[:, 1] >= 2) & (pts[:, 1] <= H - 3)
    return pts[inside], cols[inside], v[cols][inside]


def robust_quad(cl, rl, tol=2.0):
    k = np.polyfit(cl, rl, 2)
    for _ in range(4):
        ok = np.abs(rl - np.polyval(k, cl)) < max(tol, 3 * np.std(rl - np.polyval(k, cl)) if _ == 0 else tol)
        k = np.polyfit(cl[ok], rl[ok], 2)
    return k, np.abs(rl - np.polyval(k, cl)) < tol


def arm_height(arm, cols, rows):
    """Height from turned (col, row) of an arm: d = shift from the bed row toward 'up with height'."""
    bed = np.polyval(arm["bed_line"], cols)
    d = (bed - rows) if arm["sign"] < 0 else (rows - bed)
    sc = (cols - SIDE / 2) / SIDE
    if "P" in arm:
        a0, a1, a2, b0, b1 = arm["P"]
        return (a0 + a1 * sc + a2 * sc * sc) * d / (1 + (b0 + b1 * sc) * d)
    a0, a1, B = arm["A0"], arm["A1"], arm["B"]
    return (a0 + a1 * sc) * d / (1 + B * d)


def d_at(arm, h, sc=0.0):
    """Line shift d (px) for height h at turned-column coordinate sc (inverse of arm_height)."""
    if "P" in arm:
        a0, a1, a2, b0, b1 = arm["P"]
        a, b = a0 + a1 * sc + a2 * sc * sc, b0 + b1 * sc
    else:
        a, b = arm["A0"] + arm["A1"] * sc, arm["B"]
    return h / (a - b * h)


def arm_band(arm, below=4.0, above=None, gate=False):
    """Turned-row search band per column: from `below` px under the bed line up to
    the shift at ~D_max + 50% (default), or, with gate, at the calibrated depth + 3 mm."""
    allc = np.arange(SIDE)
    bed = np.polyval(arm["bed_line"], allc)
    if above is not None:
        reach = above
    elif gate:
        sc = (allc - SIDE / 2) / SIDE
        reach = d_at(arm, arm["D_max"] + 3.0, sc) + 3.0
    else:
        reach = arm["px_per_mm"] * arm["D_max"] * 1.5 + 10
    if arm["sign"] < 0:   # moves to smaller rows with height
        return bed - reach, bed + below
    return bed - below, bed + reach


def load_sweep(zdir):
    recs = [json.loads(l) for l in open(os.path.join(zdir, "log.jsonl"))]
    z0 = recs[0]["mpos"][2]
    imgs = [cv2.imread(os.path.join(zdir, r["file"]), 0) for r in recs]
    return imgs, np.array([z0 - r["mpos"][2] for r in recs]), z0


def calibrate(zdir):
    imgs, D, z0 = load_sweep(zdir)
    angles = find_angles(imgs[0])
    out = {"z0": z0, "arms": []}
    allc = np.arange(SIDE)
    for theta in angles:
        # bed row: first frame, around the strongest straight ridge
        _, c0, r0 = arm_points(imgs[0], theta)
        med = np.median(r0)
        _, c0, r0 = arm_points(imgs[0], theta, (np.full(SIDE, med - 40), np.full(SIDE, med + 40)))
        bed, ok0 = robust_quad(c0, r0)
        # direction: last frame within +-120 px of the bed row
        _, cl, rl = arm_points(imgs[-1], theta, (np.polyval(bed, allc) - 120, np.polyval(bed, allc) + 120))
        k, okl = robust_quad(cl, rl)
        sgn = float(np.sign(np.median(rl[okl] - np.polyval(bed, cl[okl]))))
        arm = {"theta": float(theta), "bed_line": bed.tolist(), "sign": sgn,
               "px_per_mm": float(abs(np.median(rl[okl] - np.polyval(bed, cl[okl]))) / D[-1]), "D_max": float(D.max())}
        lo, hi = arm_band(arm)
        C, R, Hh, per = [], [], [], []
        for img, dz in zip(imgs, D):
            _, cl, rl = arm_points(img, theta, (lo, hi))
            if len(cl) < 50:
                continue
            k, ok = robust_quad(cl, rl)
            C.append(cl[ok]); R.append(rl[ok]); Hh.append(np.full(ok.sum(), dz))
            per.append((dz, ok.sum()))
        C, R, Hh = map(np.concatenate, (C, R, Hh))
        d = (np.polyval(bed, C) - R) if sgn < 0 else (R - np.polyval(bed, C))
        sc = (C - SIDE / 2) / SIDE
        # h = (a0 + a1 s + a2 s^2) d / (1 + (b0 + b1 s) d), linearised; refit without outliers
        Amat = np.c_[d, d * sc, d * sc * sc, -Hh * d, -Hh * d * sc]
        ok = np.ones(len(Hh), bool)
        for _ in range(4):
            P, *_ = np.linalg.lstsq(Amat[ok], Hh[ok], rcond=None)
            a0, a1, a2, b0, b1 = P
            res = Hh - (a0 + a1 * sc + a2 * sc * sc) * d / (1 + (b0 + b1 * sc) * d)
            ok = np.abs(res) < 3.5 * np.std(res[ok])
        rms = float(np.sqrt(np.mean(res[ok] ** 2)))
        print("   model: %d of %d points kept (outliers > 3.5 sd dropped)" % (ok.sum(), len(ok)))
        res = np.where(ok, res, np.nan)
        arm.update({"P": P.tolist(), "rms_mm": rms})
        print("arm %+.1f deg: moves %s with height, %.2f px/mm at the bed; fit rms %.3f mm over %d points" % (
            theta, "toward smaller rows" if sgn < 0 else "toward larger rows", 1 / P[0], rms, len(Hh)))
        for dz in np.unique(Hh)[::3]:
            m = Hh == dz
            print("   D %4.1f: %4d points, d %6.2f px, residual mean %+.3f sd %.3f" % (dz, m.sum(), np.median(d[m]), np.nanmean(res[m]), np.nanstd(res[m])))
        out["arms"].append(arm)
    return out



# Legacy: principal point and camera height for the 2026-11-05 cameras, used only by
# place()/sweep() called with an axis directory. New code takes these from a
# calibration set (`carvera_scan.calib`) through place_cal().
CAMERA = {"1.2": {"centre": (622.1, 437.8), "H": 161.3}, "1.1": {"centre": (649.6, 430.1), "H": 162.6}}


def _frame_points(args):
    """Arm ridges in one frame: rows of (u, v, turned col, turned row, arm, frame)."""
    jpeg, arms, k, opts = args
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    out = []
    gate = opts.get("gate", False)
    for ai, arm in enumerate(arms):
        # with the gate, look a little further below the Z sweep's bed row: the scan's
        # own bed row (bedref "scan") may sit a few px from it
        band = arm_band(arm, below=8.0 if gate else 4.0, gate=gate)
        pts, cl, rl = arm_points(img, arm["theta"], band, ambig=opts.get("ambig"))
        if len(cl):
            out.append(np.c_[pts[:, :2], cl, rl, np.full(len(cl), ai), np.full(len(cl), k)])
    return np.concatenate(out) if out else np.zeros((0, 6))


def head_xy(s, t, p, latency):
    """Head X, Y at each frame's exposure; passes along Y (default) or X ("axis": "x")."""
    X, Y = np.full(len(t), np.nan), np.full(len(t), np.nan)
    for k, ps in enumerate(s["passes"]):
        st = np.array(ps["status"], float)
        ts = (st[:, 0] + st[:, 1]) / 2
        order = np.argsort(ts)
        m = p == k
        tt = t[m] - latency
        along = ps.get("axis", "y")
        moving = (tt > ts[order][0]) & (tt < ts[order][-1])
        if along == "y":
            X[m], Y[m] = ps["x"], np.interp(tt, ts[order], st[order, 3])
        else:
            Y[m], X[m] = ps["y"], np.interp(tt, ts[order], st[order, 2])
        idx = np.flatnonzero(m)[~moving]
        X[idx], Y[idx] = np.nan, np.nan
    return X, Y


def extract(scan, zcal, opts=None):
    """The expensive pass: arm ridges in every frame of a sweep. Returns u, v (image),
    col, row (turned frame), arm, frame. Depends on the calibration only through the
    arm angles and the search band."""
    opts = opts or {}
    from carvera_scan.process.sweep_height import load
    cal = json.load(open(zcal)) if isinstance(zcal, str) else zcal
    s, t, p, jpg = load(scan)
    with ProcessPoolExecutor() as ex:
        parts = list(ex.map(_frame_points, [(j, cal["arms"], k, opts) for k, j in enumerate(jpg)], chunksize=32))
    P = np.concatenate(parts)
    return {"u": P[:, 0].astype(np.float32), "v": P[:, 1].astype(np.float32), "col": P[:, 2].astype(np.int32),
            "row": P[:, 3].astype(np.float32), "arm": P[:, 4].astype(np.int8), "frame": P[:, 5].astype(np.int32)}


def scan_bed_line(col, row, arm, window=12.0, min_frames=30, strong=0.3, tol=1.0):
    """The scan's own bed row per turned column. Within `window` px of the Z sweep's bed
    line, rows are binned (0.5 px); of the bins holding >= `strong` x the column's
    biggest bin, the one lowest in height is the bed (the bed is the lowest large
    surface: a sheet or a block top covering many frames sits above it, holes are
    sparse). Then a robust quadratic over columns. Returns (coefficients, median
    offset from the Z sweep's line in px, columns used)."""
    cal_bed = np.polyval(arm["bed_line"], col)
    off = row - cal_bed
    m = np.abs(off) < window
    c, off = col[m].astype(np.int64), off[m]
    # height decreases toward larger rows when the arm moves to smaller rows with height (sign < 0)
    down = 1.0 if arm["sign"] < 0 else -1.0
    b = np.round(off * 2).astype(np.int64)
    nb = int(window * 4) + 3
    key = c * nb + (b + nb // 2)
    u, cnt = np.unique(key, return_counts=True)
    uc, ub = u // nb, (u % nb - nb // 2) / 2.0
    cmax = np.zeros(int(uc.max()) + 1); np.maximum.at(cmax, uc, cnt)
    ctot = np.bincount(uc, weights=cnt, minlength=len(cmax))
    ok = (cnt >= strong * cmax[uc]) & (ctot[uc] >= min_frames)
    # lowest strong bin per column: the one furthest in the "down" direction
    score = np.where(ok, ub * down, -np.inf)
    best = np.full(len(cmax), -np.inf); np.maximum.at(best, uc, score)
    sel = ok & (score == best[uc])
    cols_, mode_off = uc[sel], ub[sel]
    lut = np.full(len(cmax), np.nan); lut[cols_] = mode_off
    mo = lut[c]
    near = np.isfinite(mo) & (np.abs(off - mo) <= 1.0)
    o = np.argsort(c[near], kind="stable")
    cc, oo = c[near][o], off[near][o]
    st = np.flatnonzero(np.r_[True, np.diff(cc) != 0])
    med = np.array([np.median(oo[a:b_]) for a, b_ in zip(st, np.r_[st[1:], len(cc)])])
    ucol, w = cc[st].astype(float), np.diff(np.r_[st, len(cc)])
    y = med + np.polyval(arm["bed_line"], ucol)
    k = np.polyfit(ucol, y, 2, w=np.sqrt(w))
    for _ in range(4):
        good = np.abs(y - np.polyval(k, ucol)) < tol
        k = np.polyfit(ucol[good], y[good], 2, w=np.sqrt(w[good]))
    return k, float(np.median(med)), int(good.sum())


def compute_heights(E, cal, bedref="zsweep", gate=False):
    """Heights for extracted points. bedref "zsweep": d from the Z sweep's bed line;
    "scan": d from this scan's own bed line (scan_bed_line), so a bed line that the
    Z sweep got wrong (objects in its view) can't bias the scan."""
    cal = json.load(open(cal)) if isinstance(cal, str) else cal
    h = np.full(len(E["col"]), np.nan)
    info = []
    for ai, arm in enumerate(cal["arms"]):
        m = E["arm"] == ai
        a = dict(arm)
        if bedref == "scan":
            k, off, n = scan_bed_line(E["col"][m].astype(float), E["row"][m].astype(float), arm)
            a["bed_line"] = k.tolist()
            info.append({"arm": ai, "bed_offset_px": round(off, 2), "columns": n, "bed_line": k.tolist()})
        h[m] = arm_height(a, E["col"][m].astype(float), E["row"][m].astype(float))
        if gate:
            h[m & ~((h >= -3.0) & (h <= arm["D_max"] + 3.0))] = np.nan
    return h, info


def place_cal(E, h, scan, axis_matrix, centre, H, z0, latency=0.004):
    """Map-frame X, Y of extracted points: head position + image offset from the
    principal point through the inverse axis matrix, shrunk by (H - h) / H for raised
    points (H = camera height above the bed, corrected for the scan's Z)."""
    from carvera_scan.process.sweep_height import load
    s, t, p, _ = load(scan)
    Xh, Yh = head_xy(s, t, p, latency)
    Minv = np.linalg.inv(np.asarray(axis_matrix, float))
    c, Hc = centre, H + (s["z"] - z0)
    rel = Minv @ np.vstack([E["u"] - c[0], E["v"] - c[1]])
    shrink = (Hc - h) / Hc
    fr = E["frame"]
    return np.c_[Xh[fr] + rel[0] * shrink, Yh[fr] + rel[1] * shrink], p[fr]


def place(E, h, scan, axis_dir, cam, z0, latency=0.004):
    """Legacy wrapper: camera constants from CAMERA, axis matrix from axis_dir."""
    M = json.load(open(os.path.join(axis_dir, "matrix.json")))["matrix"]
    return place_cal(E, h, scan, M, CAMERA[cam]["centre"], CAMERA[cam]["H"], z0, latency)


def sweep(scan, zcal, axis_dir, cam, latency=0.004, opts=None):
    """Points from one sweep. opts: {"gate": bool, "ambig": float or None, "bedref": "zsweep"|"scan"}."""
    opts = opts or {}
    cal = json.load(open(zcal))
    E = extract(scan, cal, opts)
    h, info = compute_heights(E, cal, opts.get("bedref", "zsweep"), opts.get("gate", False))
    xy, passno = place(E, h, scan, axis_dir, cam, cal["z0"], latency)
    good = np.isfinite(xy).all(1) & np.isfinite(h)
    arm = E["arm"]
    print("%s: %d frames, %d points (arm 0: %d, arm 1: %d)%s" % (
        scan, int(E["frame"].max()) + 1 if len(arm) else 0, good.sum(), (good & (arm == 0)).sum(), (good & (arm == 1)).sum(),
        (" bed offsets %s" % [i["bed_offset_px"] for i in info]) if info else ""))
    return dict(xy=xy[good], height=h[good], arm=arm[good], frame=E["frame"][good],
                passno=passno[good], uv=np.c_[E["u"], E["v"]][good])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["angles", "zcal", "sweep"])
    ap.add_argument("path")
    ap.add_argument("--out")
    ap.add_argument("--zcal")
    ap.add_argument("--axis")
    ap.add_argument("--cam")
    ap.add_argument("--no-gate", dest="gate", action="store_false", help="search beyond the calibrated range (old behaviour)")
    ap.add_argument("--ambig", type=float, default=0.6, help="drop columns whose second ridge reaches this fraction of the first (0 = off)")
    ap.add_argument("--bedref", choices=["zsweep", "scan"], default="scan", help="bed row from the scan itself (default) or the Z sweep")
    a = ap.parse_args()
    if a.action == "sweep":
        d = sweep(a.path, a.zcal, a.axis, a.cam, opts={"gate": a.gate, "ambig": a.ambig or None, "bedref": a.bedref})
        np.savez_compressed(a.out, **d, ok=np.ones(len(d["height"]), bool))
        print("wrote", a.out)
        return
    if a.action == "angles":
        imgs, D, _ = load_sweep(a.path)
        print("arm angles first frame: %.2f %.2f; last frame: %.2f %.2f" % (*find_angles(imgs[0]), *find_angles(imgs[-1])))
    else:
        out = calibrate(a.path)
        if a.out:
            json.dump(out, open(a.out, "w"), indent=1)
            print("wrote", a.out)


if __name__ == "__main__":
    main()
