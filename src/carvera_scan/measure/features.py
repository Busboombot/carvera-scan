"""Holes and bosses in a height map: the first, coarse pass that says where to look.

    python -m carvera_scan.measure.features OUT/points.npz --min-z 3 --out OUT/features.json [--png OUT/features.png]

Input: a pipeline points file, levelled to the bed with grid.bed_plane. By default the
points are gridded again (--regrid, 0.25 mm, densest band per cell) without the
pipeline's neighbour-support filter and speck removal: those are made to remove needles,
and they also fill small holes. --regrid 0 uses the file's own grid. All positions are in camera A's map frame (mm); z is above the bed.

- Hole: a pit or a no-data patch (the laser can't see the bottom of a small hole)
  that a grey closing with a disk fills, at least `depth` mm below the closing, round
  and small (diameter `hole_d`), with a flat rim: the ring of cells just outside it is
  nearly all measured and within a narrow band. The flat-rim test rejects shadows
  beside walls, which are also no-data but have a wall on one side.
- Boss: a small round raised spot, at least `rise` mm above a grey opening, diameter
  `boss_d`, standing on a flat ring. A threaded insert is a ring with a hole in it;
  the ring's inner hole is filled before it is measured, and the hole is also found
  as a hole (`in_boss` links them).
- Objects: connected regions over `obj_z` mm; each feature names the object it is on.

Positions here are good to about a cell. Two refining passes:
- process.spot: the laser cross centred in each hole (still frames, chord geometry);
- a fine-pitch line_scan over each object (process.spot passes), then `refine`:

    python -m carvera_scan.measure.features refine FINE/points.npz --seeds features.json --out refined.json

  around each seed the points are gridded at 0.1 mm, the hole (cells off the surface or
  empty) or the boss top (cells above mid-height) containing the seed is outlined, and
  a circle is fitted to the outline.
"""
import argparse
import json

import cv2
import numpy as np
from scipy import ndimage

from carvera_scan.process.grid import bed_plane, grid_mode


def disk(r_cells):
    r = max(int(round(r_cells)), 1)
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))


def _shape(mask, cell):
    """Equivalent diameter, roundness (area / area of the enclosing circle) and centroid (cells)."""
    ys, xs = np.nonzero(mask)
    pts = np.c_[xs, ys].astype(np.float32)
    (cx, cy), r = cv2.minEnclosingCircle(pts) if len(pts) > 2 else ((xs.mean(), ys.mean()), 1.0)
    area = len(xs)
    d = 2 * np.sqrt(area / np.pi) * cell
    roundness = area / (np.pi * max(r + 0.5, 1.0) ** 2)
    return d, roundness, (xs.mean(), ys.mean())


def _ring(mask, h, width):
    ring = cv2.dilate(mask.astype(np.uint8), disk(width)).astype(bool) & ~mask
    v = h[ring]
    fin = np.isfinite(v)
    return v[fin], fin.mean() if len(v) else 0.0


def find(h, origin, cell, hole_d=(1.0, 10.0), boss_d=(2.5, 14.0), depth=1.0, rise=0.6,
         min_round=0.5, ring_valid=0.75, ring_sd=0.8, min_z=None, obj_z=3.0, scales=(3.0, 6.0), min_depth=1.0):
    """Holes and bosses in a levelled height grid h (NaN = no data). Returns (features, objects)."""
    x0, y0 = origin
    to_xy = lambda c: [round(float(x0 + (c[0] + .5) * cell), 3), round(float(y0 + (c[1] + .5) * cell), 3)]
    low = np.nan_to_num(h, nan=-30.0).astype(np.float32)
    feats = []

    # objects: regions above obj_z, small gaps closed
    obj = cv2.morphologyEx((np.nan_to_num(h, nan=0) > obj_z).astype(np.uint8), cv2.MORPH_CLOSE, disk(2.0 / cell))
    olab, nobj = ndimage.label(obj)
    objects = []
    for k, sl in enumerate(ndimage.find_objects(olab), start=1):
        m = olab[sl] == k
        if m.sum() * cell * cell < 100:  # under 1 cm2: not an object
            olab[sl][m] = 0
            continue
        hv = h[sl][m]
        objects.append({"id": k, "bbox": [*to_xy((sl[1].start, sl[0].start)), *to_xy((sl[1].stop - 1, sl[0].stop - 1))],
                        "area_mm2": round(float(m.sum() * cell * cell)), "top": round(float(np.nanpercentile(hv, 99)), 1)})

    # holes: a closing fills pits narrower than its disk. Several disk sizes: a big disk
    # also bridges a small hole to a shadow beside a wall, a small one can't.
    holes = []
    for dmax in sorted({min(hole_d[1], s_) for s_ in scales} | {hole_d[1]}):
        closed = cv2.morphologyEx(low, cv2.MORPH_CLOSE, disk(dmax / 2 / cell + 1))
        deep = closed - low
        # stray surface-height cells inside a hole (blur, reflections) would split it: close
        # the mask, fill it, then open away single-cell texture pits
        pit = cv2.morphologyEx((deep > depth).astype(np.uint8), cv2.MORPH_CLOSE, disk(0.4 / cell))
        pit = cv2.morphologyEx(ndimage.binary_fill_holes(pit).astype(np.uint8), cv2.MORPH_OPEN, disk(0.3 / cell))
        lab, n = ndimage.label(pit)
        for k, sl in enumerate(ndimage.find_objects(lab), start=1):
            m = lab == k
            d, rnd, c = _shape(m[sl], cell)
            if not hole_d[0] <= d <= hole_d[1] or rnd < min_round:
                continue
            if np.median(deep[m]) < min_depth:  # a shallow dimple in a rough surface
                continue
            ring, valid = _ring(m, h, max(1.0 / cell, 2))
            if valid < ring_valid or len(ring) < 8:
                continue
            z = float(np.median(ring))
            sd = float(1.4826 * np.median(np.abs(ring - z)))
            if sd > ring_sd:
                continue
            inside = h[m]
            bottom = float(np.nanmin(inside)) if np.isfinite(inside).any() else None
            cc = (c[0] + sl[1].start, c[1] + sl[0].start)
            holes.append({"kind": "hole", "xy": to_xy(cc), "z": round(z, 2), "diameter": round(d, 2),
                          "depth_seen": None if bottom is None else round(z - bottom, 2),
                          "no_data": round(float(np.isnan(inside).mean()), 2), "roundness": round(rnd, 2),
                          "ring_sd": round(sd, 2), "object": int(olab[int(cc[1]), int(cc[0])]) or None})
    # the same hole found at several scales: keep the roundest
    holes.sort(key=lambda f: -f["roundness"])
    for f in holes:
        if all(np.hypot(*np.subtract(f["xy"], g["xy"])) > max(1.0, g["diameter"] / 2) for g in feats):
            feats.append(f)

    # bosses: opening removes bumps narrower than the disk
    opened = cv2.morphologyEx(low, cv2.MORPH_OPEN, disk(boss_d[1] / 2 / cell + 1))
    bump = ((low - opened) > rise) & np.isfinite(h)
    bump = ndimage.binary_fill_holes(cv2.morphologyEx(bump.astype(np.uint8), cv2.MORPH_CLOSE, disk(1)).astype(bool))
    lab, n = ndimage.label(bump)
    for k, sl in enumerate(ndimage.find_objects(lab), start=1):
        m = lab == k
        d, rnd, c = _shape(m[sl], cell)
        if not boss_d[0] <= d <= boss_d[1] or rnd < min_round:
            continue
        ring, valid = _ring(m, h, max(1.0 / cell, 2))
        if valid < ring_valid or len(ring) < 8:
            continue
        base = float(np.median(ring))
        sd = float(1.4826 * np.median(np.abs(ring - base)))
        if sd > ring_sd:
            continue
        top = float(np.nanpercentile(h[m], 90))
        cc = (c[0] + sl[1].start, c[1] + sl[0].start)
        feats.append({"kind": "boss", "xy": to_xy(cc), "z": round(top, 2), "base_z": round(base, 2),
                      "diameter": round(d, 2), "rise": round(top - base, 2), "roundness": round(rnd, 2),
                      "ring_sd": round(sd, 2), "object": int(olab[int(cc[1]), int(cc[0])]) or None})

    if min_z is not None:
        feats = [f for f in feats if f.get("base_z", f["z"]) >= min_z]
    # link holes that sit inside a boss
    for f in feats:
        if f["kind"] == "hole":
            for b in feats:
                if b["kind"] == "boss" and np.hypot(*np.subtract(f["xy"], b["xy"])) < b["diameter"] / 2:
                    f["in_boss"] = True
                    b["has_hole"] = True
    feats.sort(key=lambda f: (f["object"] or 0, f["kind"], -f["xy"][1], f["xy"][0]))
    count = {}
    for f in feats:
        count[f["kind"]] = count.get(f["kind"], 0) + 1
        f["id"] = "%s%02d" % (f["kind"][0].upper(), count[f["kind"]])
    return feats, objects


def draw(h, origin, cell, feats, path, scale=2):
    """Shaded relief (north up) with the features ringed: holes blue, bosses red."""
    hv = np.nan_to_num(h, nan=0)
    gy, gx = np.gradient(hv)
    shade = np.clip(128 + 60 * (gx - gy) / cell, 0, 255)
    col = np.clip(np.nan_to_num(h, nan=-5) / max(np.nanpercentile(h, 99.5), 1) * 255, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(col, cv2.COLORMAP_TURBO).astype(np.float32) * (shade[..., None] / 255 * 0.6 + 0.4)
    img[np.isnan(h)] = 0
    img = cv2.resize(img.astype(np.uint8), None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)[::-1].copy()
    H = img.shape[0]
    for f in feats:
        u = int((f["xy"][0] - origin[0]) / cell * scale)
        v = H - int((f["xy"][1] - origin[1]) / cell * scale)
        c = (255, 160, 0) if f["kind"] == "hole" else (0, 0, 255)
        cv2.circle(img, (u, v), max(int(f["diameter"] / 2 / cell * scale), 3), c, 1)
        cv2.putText(img, f["id"], (u + 6, v - 6), 0, 0.35, c, 1)
    cv2.imwrite(path, img)


def _outline_circle(mask, origin, cell, n, close=2):
    """Circle fitted to the outline of the component of `mask` at the window centre.
    Returns (cx, cy, r, rms, points, touches_edge) or None."""
    from carvera_scan.process.spot import fit_circle
    m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, disk(1))
    m = ndimage.binary_fill_holes(cv2.morphologyEx(m, cv2.MORPH_CLOSE, disk(close))).astype(np.uint8)
    lab, _ = ndimage.label(m)
    ci = lab[n // 2 - 3:n // 2 + 4, n // 2 - 3:n // 2 + 4]
    ids = ci[ci > 0]
    if not len(ids):
        return None
    m = (lab == np.bincount(ids).argmax()).astype(np.uint8)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pts = max(cs, key=len)[:, 0, :].astype(float)
    edge = np.hypot(pts[:, 0] - (n - 1) / 2, pts[:, 1] - (n - 1) / 2) > (n - 1) / 2 - 2.5  # the round window's edge
    touches = edge.mean() > 0.02
    pts = pts[~edge]
    if len(pts) < 10:
        return None
    P = np.c_[origin[0] + (pts[:, 0] + .5) * cell, origin[1] + (pts[:, 1] + .5) * cell]
    fit = fit_circle(P)
    if fit is None:
        return None
    cx, cy, r, rms = fit
    d = np.abs(np.hypot(P[:, 0] - cx, P[:, 1] - cy) - r)
    keep = d < max(3 * rms, 2 * cell)
    if keep.sum() >= 10 and keep.sum() < len(P):
        cx, cy, r, rms = fit_circle(P[keep])
    return cx, cy, r, rms, int(keep.sum()), touches


def refine(xy, z, seed, cell=0.1, min_count=2, window=7.0):
    """Circle fits to one hole or boss from dense points (map frame, heights above the bed).
    seed: a feature from find() (kind, xy, z, diameter[, base_z]).

    hole: the bore = cells below the rim (z - 0.8) or empty; and, if the rim-height
    patch around it closes into a small disc (a threaded insert's flange), the flange.
    boss: the top = cells above mid-height between its top and its base."""
    r0 = seed["diameter"] / 2
    R = max(window, 1.6 * r0 + 2.0)
    Rb = 1.4 * r0 + 1.5  # the bore's own, smaller window: keeps clear of shadows beside nearby walls
    c0 = np.asarray(seed["xy"], float)
    k = np.hypot(*(xy - c0).T) < R + 0.5
    if k.sum() < 200:
        return None
    n = int(2 * R / cell) + 1
    origin = (c0[0] - R, c0[1] - R)
    g, _, _, _ = grid_mode(xy[k], z[k], cell, min_count=min_count, origin=origin, shape=(n, n))
    yy, xx = np.mgrid[0:n, 0:n]
    inside = np.hypot((xx + .5) * cell - R, (yy + .5) * cell - R) < R
    gv = np.nan_to_num(g, nan=-99.0)
    out = {"id": seed.get("id"), "kind": seed["kind"], "z": seed["z"]}
    if seed["kind"] == "hole":
        m = int(round((R - Rb) / cell))
        nb = n - 2 * m
        ob = (origin[0] + m * cell, origin[1] + m * cell)
        bore = _outline_circle(((gv < seed["z"] - 0.8) & inside)[m:n - m, m:n - m] & (
            np.hypot(*(np.mgrid[0:nb, 0:nb] - (nb - 1) / 2)) < (nb - 1) / 2), ob, cell, nb)
        if bore is None:
            return None
        cx, cy, r, rms, npt, touch = bore
        out.update({"xy": [round(cx, 3), round(cy, 3)], "diameter": round(2 * r, 3), "rms": round(rms, 3),
                    "outline_points": npt, "ok": bool(rms < 0.25 and not touch)})
        rim = _outline_circle(((np.abs(gv - seed["z"]) < 0.5) | (gv < seed["z"] - 0.8)) & inside, origin, cell, n, close=3)
        if rim is not None and not rim[5] and rim[2] > r + 0.5:
            out["flange"] = {"xy": [round(rim[0], 3), round(rim[1], 3)], "diameter": round(2 * rim[2], 3),
                             "rms": round(rim[3], 3), "outline_points": rim[4]}
    else:
        level = (seed["z"] + seed.get("base_z", seed["z"] - 1)) / 2
        top = _outline_circle((gv > level) & inside, origin, cell, n)
        if top is None:
            return None
        cx, cy, r, rms, npt, touch = top
        out.update({"xy": [round(cx, 3), round(cy, 3)], "diameter": round(2 * r, 3), "rms": round(rms, 3),
                    "outline_points": npt, "ok": bool(rms < 0.25 and not touch)})
    return out


def refine_main(argv):
    ap = argparse.ArgumentParser(prog="features refine")
    ap.add_argument("points")
    ap.add_argument("--seeds", required=True, help="features.json (find) or spot results")
    ap.add_argument("--cell", type=float, default=0.1)
    ap.add_argument("--cam", choices=["A", "B"], help="only this camera's points (default both)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    d = np.load(a.points)
    xy, z = d["xy"].astype(np.float64), d["height"].astype(np.float64)
    if a.cam:
        k = d["cam"] == "AB".index(a.cam)
        xy, z = xy[k], z[k]
    bp = json.load(open(a.seeds)).get("bed_plane")
    if bp is None:  # level with the fine scan's own grid
        bp, _, _ = bed_plane(d["grid"].astype(np.float64), d["origin"].astype(float), float(d["cell"]))
    z = z - (bp[0] * xy[:, 0] + bp[1] * xy[:, 1] + bp[2])
    seeds = json.load(open(a.seeds))
    seeds = seeds["features"] if isinstance(seeds, dict) else seeds
    out = []
    for f in seeds:
        if "xy" not in f:
            continue
        r = refine(xy, z, f, a.cell)
        out.append(r or {"id": f.get("id"), "kind": f["kind"], "ok": False})
        if r:
            fl = r.get("flange")
            print("%-5s %-4s (%8.2f, %8.2f) d %.2f rms %.3f n %3d%s%s" % (
                r["id"], r["kind"], *r["xy"], r["diameter"], r["rms"], r["outline_points"], "" if r["ok"] else " (poor)",
                "" if not fl else "  flange (%.2f, %.2f) d %.2f rms %.3f" % (*fl["xy"], fl["diameter"], fl["rms"])))
    json.dump({"source": a.points, "cell": a.cell, "features": out}, open(a.out, "w"), indent=1)


def main():
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "refine":
        return refine_main(sys.argv[2:])
    ap = argparse.ArgumentParser()
    ap.add_argument("points", help="pipeline points.npz (uses its grid)")
    ap.add_argument("--crop", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"), help="map-frame window")
    ap.add_argument("--regrid", type=float, default=0.25, help="grid the points at this cell (mm); 0 = use the file's grid")
    ap.add_argument("--min-z", type=float, help="drop features below this height (e.g. 3: skip the bed's holes)")
    ap.add_argument("--hole-d", type=float, nargs=2, default=(1.0, 10.0))
    ap.add_argument("--boss-d", type=float, nargs=2, default=(2.5, 14.0))
    ap.add_argument("--depth", type=float, default=1.0)
    ap.add_argument("--rise", type=float, default=0.6)
    ap.add_argument("--out", required=True)
    ap.add_argument("--png", help="relief with the features ringed")
    a = ap.parse_args()
    d = np.load(a.points)
    g, origin, cell = d["grid"].astype(np.float64), d["origin"].astype(float), float(d["cell"])
    bp, GX, GY = bed_plane(g, origin, cell)
    h = g - (bp[0] * GX + bp[1] * GY + bp[2])
    if a.regrid:
        xy, z = d["xy"].astype(np.float64), d["height"].astype(np.float64)
        if a.crop:
            x0, x1, y0, y1 = a.crop
            k = (xy[:, 0] >= x0) & (xy[:, 0] < x1) & (xy[:, 1] >= y0) & (xy[:, 1] < y1)
            xy, z = xy[k], z[k]
            origin, shape = (x0, y0), (int((y1 - y0) / a.regrid), int((x1 - x0) / a.regrid))
        else:
            origin = tuple(np.floor(xy.min(0)))
            shape = tuple((np.ceil((xy.max(0) - origin) / a.regrid)[::-1]).astype(int) + 1)
        z = z - (bp[0] * xy[:, 0] + bp[1] * xy[:, 1] + bp[2])
        h, *_ = grid_mode(xy, z, a.regrid, origin=origin, shape=shape)
        h, cell, a.crop = h.astype(np.float64), a.regrid, None
    if a.crop:
        x0, x1, y0, y1 = a.crop
        j0, j1 = int((x0 - origin[0]) / cell), int((x1 - origin[0]) / cell)
        i0, i1 = int((y0 - origin[1]) / cell), int((y1 - origin[1]) / cell)
        h = h[max(i0, 0):i1, max(j0, 0):j1]
        origin = (origin[0] + max(j0, 0) * cell, origin[1] + max(i0, 0) * cell)
    feats, objects = find(h, origin, cell, tuple(a.hole_d), tuple(a.boss_d), a.depth, a.rise, min_z=a.min_z)
    out = {"source": a.points, "bed_plane": [float(v) for v in bp], "cell": cell, "objects": objects, "features": feats}
    json.dump(out, open(a.out, "w"), indent=1)
    print("%d objects, %d holes, %d bosses -> %s" % (len(objects), sum(f["kind"] == "hole" for f in feats),
                                                    sum(f["kind"] == "boss" for f in feats), a.out))
    if a.png:
        draw(h, origin, cell, feats, a.png)


if __name__ == "__main__":
    main()
