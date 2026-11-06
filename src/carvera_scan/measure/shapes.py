"""Fit shapes to a line-laser height map, refined with the camera frames.

    python -m carvera_scan.measure.shapes data/2026-11-04/plate2 --grid data/2026-11-04/plate2-grid1.npz \
        --zcal data/2026-11-04/zcal1.json --axis data/2026-11-04/axis1 --latency 0.004 \
        --out data/2026-11-04/plate2-shapes.json

1. Level: plane through the bed cells; objects are connected cells > --min mm.
2. Top: the most common height of an object; its top cells are those within
   1.5 mm of it. A minimum-area rectangle around them starts the fit.
3. Camera: a top-down image of the top at its own height is built from the
   sweep frames (each frame reprojected through the calibrated camera; rows
   near the laser line masked). The rectangle (centre, angle, width, length)
   is moved to where its four sides lie on the strongest edges of that image.
4. Objects whose top fills its rectangle (>= 0.85) are boxes; the rest keep
   their outline (a polygon, extruded to the top height).
"""
import argparse
import json

import cv2
import numpy as np
from scipy import ndimage
from scipy.optimize import minimize

from carvera_scan.process.line_height import axis_matrix, orient
from carvera_scan.process.sweep_height import bed_row, head_xy, lines, load

NOMINAL = [25.4, 50.8, 76.2]


def level(h, cell):
    yy, xx = np.mgrid[:h.shape[0], :h.shape[1]] * cell
    fl = np.isfinite(h) & (np.abs(h) < 2)
    for _ in range(4):
        A = np.c_[xx[fl], yy[fl], np.ones(fl.sum())]
        coef = np.linalg.lstsq(A, h[fl], rcond=None)[0]
        r = h - (coef[0] * xx + coef[1] * yy + coef[2])
        fl = np.isfinite(r) & (np.abs(r) < max(0.6, 3 * np.nanstd(r[fl])))
    return r, coef, fl


class Mosaic:
    """Top-down images at a given height, from the sweep frames."""

    def __init__(self, scan, zcal, axis, latency, step=4):
        s, t, p, jpg = load(scan)
        cal = json.load(open(zcal))
        rot = cal.get("rot", "none")
        self.H = cal["H"] + (s["z"] - cal["z0"])
        self.c = np.array(cal["zoom_centre"])
        self.M = axis_matrix(axis, rot)
        Xh, Yh = head_xy(s, t, p, latency)
        keep = np.flatnonzero(np.isfinite(Yh))[::step]
        self.head = np.c_[Xh[keep], Yh[keep]]
        self.imgs = [orient(cv2.imdecode(np.frombuffer(jpg[k], np.uint8), cv2.IMREAD_GRAYSCALE), rot) for k in keep]
        V = lines([jpg[k] for k in keep[::10]], 10, rot)
        self.vbed, _ = bed_row(V, self.imgs[0].shape[0])

    def image(self, x0, y0, x1, y1, h, res=0.1):
        """Mean of the frames over [x0, x1] x [y0, y1] (mm) at height h; res mm/px."""
        X, Y = np.meshgrid(np.arange(x0, x1, res), np.arange(y0, y1, res))
        acc, n = np.zeros(X.shape, np.float32), np.zeros(X.shape, np.float32)
        k = self.H / (self.H - h)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        for (hx, hy), img in zip(self.head, self.imgs):
            # quick reject: centre of the window must land in the frame
            du, dv = self.M @ np.array([cx - hx, cy - hy]) * k
            if not (-200 < self.c[0] + du < img.shape[1] + 200 and -200 < self.c[1] + dv < img.shape[0] + 200):
                continue
            u = self.c[0] + k * (self.M[0, 0] * (X - hx) + self.M[0, 1] * (Y - hy))
            v = self.c[1] + k * (self.M[1, 0] * (X - hx) + self.M[1, 1] * (Y - hy))
            ui = np.clip(u.astype(int), 0, img.shape[1] - 1)
            # the laser lies between the bed line and ~200 rows above it: don't use those rows
            ok = (u >= 0) & (u < img.shape[1] - 1) & (v >= 0) & (v < img.shape[0] - 1)
            ok &= (v > self.vbed[ui] + 15) | (v < self.vbed[ui] - 230)
            if not ok.any():
                continue
            smp = cv2.remap(img, u.astype(np.float32), v.astype(np.float32), cv2.INTER_LINEAR)
            acc[ok] += smp[ok]
            n[ok] += 1
        out = np.where(n > 0, acc / np.maximum(n, 1), np.nan)
        return out, n


def rect_corners(cx, cy, w, l, ang):
    c, s = np.cos(ang), np.sin(ang)
    ax, ay = np.array([c, s]) * w / 2, np.array([-s, c]) * l / 2
    return np.array([[cx, cy]]) + np.array([-ax - ay, ax - ay, ax + ay, -ax + ay])


def refine_rect(img, x0, y0, res, init):
    """Move a rectangle onto the strongest edges of img (window origin x0, y0)."""
    g = cv2.GaussianBlur(np.nan_to_num(img, nan=np.nanmean(img)).astype(np.float32), (0, 0), 3)
    gx, gy = cv2.Sobel(g, cv2.CV_32F, 1, 0) / 8, cv2.Sobel(g, cv2.CV_32F, 0, 1) / 8
    valid = np.isfinite(img).astype(np.float32)

    def score(q):
        cx, cy, w, l, ang = q
        pts, nrm = [], []
        cs, sn = np.cos(ang), np.sin(ang)
        ex, ey = np.array([cs, sn]), np.array([-sn, cs])
        for side, (o, e, n, L) in enumerate([(ey * l / 2, ex, ey, w), (-ey * l / 2, ex, -ey, w),
                                             (ex * w / 2, ey, ex, l), (-ex * w / 2, ey, -ex, l)]):
            tt = np.linspace(-L / 2 + 1.5, L / 2 - 1.5, 80)
            pts.append(np.array([cx, cy]) + o + tt[:, None] * e)
            nrm.append(np.repeat(n[None], len(tt), 0))
        pts, nrm = np.concatenate(pts), np.concatenate(nrm)
        px = ((pts[:, 0] - x0) / res).astype(np.float32)
        py = ((pts[:, 1] - y0) / res).astype(np.float32)
        sx = cv2.remap(gx, px[None], py[None], cv2.INTER_LINEAR)[0]
        sy = cv2.remap(gy, px[None], py[None], cv2.INTER_LINEAR)[0]
        vv = cv2.remap(valid, px[None], py[None], cv2.INTER_NEAREST)[0]
        # edge strength across each side, ignoring where the image has no data
        e = np.abs(sx * nrm[:, 0] + sy * nrm[:, 1]) * vv
        return -e.sum() / max(vv.sum(), 1)

    q0 = np.array(init, float)
    best = minimize(score, q0, method="Nelder-Mead",
                    options={"xatol": 0.02, "fatol": 1e-4, "maxiter": 3000,
                             "initial_simplex": q0 + np.vstack([np.zeros(5), np.diag([1.5, 1.5, 2.0, 2.0, 0.03])])})
    return best.x, -best.fun, -score(q0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan")
    ap.add_argument("--grid", required=True)
    ap.add_argument("--zcal", required=True)
    ap.add_argument("--axis", required=True)
    ap.add_argument("--latency", type=float, default=0.004)
    ap.add_argument("--min", type=float, default=3.0)
    ap.add_argument("--min-area", type=float, default=150.0, help="mm^2")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    G = np.load(a.grid)
    cell = float(G["cell"])
    x0g, y0g = G["origin"]
    h, plane, floor = level(G["height"], cell)
    print("bed plane: %.4f mm/mm X, %.4f mm/mm Y; floor cells sd %.2f mm" % (plane[0], plane[1], np.nanstd(h[floor])))
    # closing bridges strips of missing cells (occlusion, dark spots) a few mm wide
    mask = ndimage.binary_closing(np.nan_to_num(h, nan=0) > a.min, np.ones((5, 5)))
    mask = ndimage.binary_opening(mask, np.ones((3, 3)))
    lab, n = ndimage.label(mask, np.ones((3, 3)))
    mos = Mosaic(a.scan, a.zcal, a.axis, a.latency)
    objs = []
    for k in range(1, n + 1):
        m = lab == k
        if m.sum() * cell ** 2 < a.min_area:
            continue
        v = h[m]
        hist, e = np.histogram(v, bins=np.arange(a.min, np.nanmax(v) + 1, 0.5))
        top_h = e[np.argmax(hist)] + 0.25
        top = ndimage.binary_closing(m & (np.abs(h - top_h) < 1.5), np.ones((5, 5))) & m
        tl, tn = ndimage.label(top)
        if tn == 0:
            continue
        top = tl == (1 + np.argmax(ndimage.sum(top, tl, range(1, tn + 1))))
        top_h = float(np.nanmedian(h[top & (np.abs(h - top_h) < 1.5)]))
        yy, xx = np.nonzero(top)
        pts = np.c_[x0g + (xx + 0.5) * cell, y0g + (yy + 0.5) * cell].astype(np.float32)
        (rcx, rcy), (rw, rl), ang = cv2.minAreaRect(pts)
        fill = top.sum() * cell ** 2 / max(rw * rl, 1e-6)
        obj = {"id": len(objs) + 1, "height": round(top_h, 2), "top_cells": int(top.sum()),
               "top_sd": round(float(np.nanstd(h[top & (np.abs(h - top_h) < 1.5)])), 2), "footprint_mm2": round(float(m.sum() * cell ** 2)),
               "fill": round(float(fill), 2)}
        if fill >= 0.85:
            ang = np.radians(ang)
            if rw > rl:  # width along the shorter side
                rw, rl, ang = rl, rw, ang + np.pi / 2
            ang = (ang + np.pi / 2) % np.pi - np.pi / 2
            mg = 8
            xs, ys = pts[:, 0], pts[:, 1]
            wx0, wy0 = xs.min() - mg, ys.min() - mg
            img, cnt = mos.image(wx0, wy0, xs.max() + mg, ys.max() + mg, top_h)
            q, s1, s0 = refine_rect(img, wx0, wy0, 0.1, [rcx, rcy, rw, rl, ang])
            obj.update({"type": "box", "centre": [round(q[0], 2), round(q[1], 2)],
                        "size": [round(q[2], 2), round(q[3], 2)], "angle_deg": round(float(np.degrees(q[4])), 2),
                        "laser_rect": {"centre": [round(rcx, 2), round(rcy, 2)], "size": [round(rw, 2), round(rl, 2)],
                                       "angle_deg": round(float(np.degrees(ang)), 2)},
                        "edge_score": [round(s0, 2), round(s1, 2)]})
            cv2.imwrite(a.out.replace(".json", "-top%d.png" % obj["id"]),
                        cv2.normalize(np.nan_to_num(img, nan=0), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)[::-1])
        else:
            cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            c = max(cs, key=cv2.contourArea)[:, 0, :]
            c = cv2.approxPolyDP(c.reshape(-1, 1, 2), 1.0, True)[:, 0, :]
            obj.update({"type": "outline", "polygon": [[round(float(x0g + (x + 0.5) * cell), 2),
                                                        round(float(y0g + (y + 0.5) * cell), 2)] for x, y in c]})
        near = min(NOMINAL, key=lambda x: abs(x - top_h))
        obj["nearest_block_height"] = near
        objs.append(obj)
        print("object %d: %s, top %.2f mm (sd %.2f, %d cells), fill %.2f%s" % (
            obj["id"], obj["type"], top_h, obj["top_sd"], obj["top_cells"], fill,
            "" if obj["type"] != "box" else ", %.2f x %.2f mm at (%.1f, %.1f), %.2f deg (laser only: %.2f x %.2f)"
            % (*obj["size"], *obj["centre"], obj["angle_deg"], *obj["laser_rect"]["size"])))
    json.dump({"plane": plane.tolist(), "objects": objs}, open(a.out, "w"), indent=1)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
