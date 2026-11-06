"""3D map of a cross-laser scan with the lit photos draped on it.

    python -m carvera_scan.view.view3d projects/blocks-123/out/bench/blocks_points.npz --calib config/calib/2026-11-05-cross45-deep.json \
        --photos data/2026-11-05/photos1 data/2026-11-05/survey3 \
        --heights projects/blocks-123/out/bench/heights.json --objects projects/blocks-123/objects.json --out projects/blocks-123/out/view3d

Surface cleaning (specks, hole filling, neighbour support) and cloud thinning use the
pipeline.Params defaults (override with --set); camera constants and the
map -> machine offset come from the calibration set.

Writes OUT/: index.html (three.js viewer, from projects/viewer/) + surface.bin + cloud.bin + meta.json (
serve the folder over HTTP), surface.ply / cloud.ply (machine coordinates, Z = height
above the spoilboard; open in CloudCompare or MeshLab), ortho.png (top-down photo
mosaic at true height), relief.png (shaded height map with the measured heights).

Colour: each surface cell / cloud point is projected into every photo with the
linear camera model the scan used (image = c + M (P - head) H / (H - h)); the
photo whose axis is closest wins, unless a taller surface in the height grid
blocks the line of sight to that camera, then the next. Photos are gain-matched
by their median. Grey only: the cameras are mono.
"""
import argparse
import json
import os

import cv2
import numpy as np

from carvera_scan import calib as calib_mod
from carvera_scan.process import pipeline
from carvera_scan.process.grid import bed_plane, remove_specks, support_filter

TO_MACHINE = np.zeros(2)  # map (camera A) -> machine X, Y; set from the calibration set in main()

# The viewer page lives in the repo, not the package: projects/viewer/index.html.
VIEWER = os.path.join(os.path.dirname(__file__), "..", "..", "..", "projects", "viewer", "index.html")


def load_photos(dirs, cal, shift11):
    photos = []
    for d in dirs:
        st = json.load(open(os.path.join(d, "stops.json")))
        for cam in st["cams"]:
            cc = calib_mod.camera(cal, calib_mod.role_of_port(cal, cam))
            M = np.array(cc["axis_matrix"], float)
            sh = np.array(shift11) if calib_mod.role_of_port(cal, cam) == "B" else np.zeros(2)
            for s in st["stops"]:
                img = cv2.imread(os.path.join(d, cam, "%d.png" % s["i"]), cv2.IMREAD_GRAYSCALE)
                if img is None:
                    continue
                # the laser cross is on in every photo (own switch): mask its lines
                th = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)))
                laser = cv2.dilate((th > 35).astype(np.uint8), np.ones((7, 7), np.uint8))
                laser = cv2.morphologyEx(laser, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                photos.append({"img": img.astype(np.float32), "laser": laser.astype(bool), "gain": 1.0, "M": M, "c": np.array(cc["centre"]),
                               "H": cc["H"] + (s["mpos"][2] - cal["z0"]), "head": np.array(s["mpos"][:2]), "sh": sh,
                               "name": "%s/%s/%d" % (os.path.basename(d), cam, s["i"])})
    allmed = np.median([np.median(p["img"]) for p in photos])
    for p in photos:
        p["gain"] = allmed / max(np.median(p["img"]), 1.0)
    return photos


def project(p, P):
    """Pixel (u, v) of map-frame points P (N x 3, height in P[:, 2]) in photo p."""
    rel = (P[:, :2] - p["sh"]) - p["head"]  # camera 1.1 photos live in its own frame
    k = p["H"] / np.maximum(p["H"] - P[:, 2], 1.0)
    uv = (rel @ p["M"].T) * k[:, None] + p["c"]
    return uv


class Grid:
    def __init__(self, h, origin, cell):
        self.h, self.x0, self.y0, self.c = h, origin[0], origin[1], cell

    def at(self, x, y):
        i = np.clip(((y - self.y0) / self.c).astype(int), 0, self.h.shape[0] - 1)
        j = np.clip(((x - self.x0) / self.c).astype(int), 0, self.h.shape[1] - 1)
        return self.h[i, j]


def colorize(P, photos, grid, chunk=150000, top=6):
    gray = np.full(len(P), np.nan, np.float32)
    for a in range(0, len(P), chunk):
        Q = P[a:a + chunk]
        # rank photos by horizontal distance of the point from the photo's camera axis
        dist = np.stack([np.hypot(*(((Q[:, :2] - p["sh"]) - p["head"]).T)) for p in photos], 1)
        order = np.argsort(dist, 1)[:, :top]
        g = np.full(len(Q), np.nan, np.float32)
        for r in range(top):
            todo = np.isnan(g)
            if not todo.any():
                break
            for pi in np.unique(order[todo, r]):
                m = todo & (order[:, r] == pi)
                p = photos[pi]
                q = Q[m]
                uv = project(p, q)
                ok = (uv[:, 0] > 5) & (uv[:, 0] < p["img"].shape[1] - 6) & (uv[:, 1] > 5) & (uv[:, 1] < p["img"].shape[0] - 6)
                # line of sight: march toward the camera centre (head xy, height H) through the height grid
                cam = np.r_[p["head"] + p["sh"], p["H"]]
                vis = ok.copy()
                iu0, iv0 = np.clip(uv[:, 0].round().astype(int), 0, p["img"].shape[1] - 1), np.clip(uv[:, 1].round().astype(int), 0, p["img"].shape[0] - 1)
                vis &= ~p["laser"][iv0, iu0]
                for t in (0.03, 0.06, 0.1, 0.15, 0.22, 0.3, 0.4, 0.5):
                    s = q + t * (cam - q)
                    gh = grid.at(s[:, 0], s[:, 1])
                    vis &= ~(np.nan_to_num(gh, nan=-99) > s[:, 2] + 1.0)
                idx = np.flatnonzero(m)[vis]
                if not len(idx):
                    continue
                uu, vv = uv[vis, 0], uv[vis, 1]
                iu, iv = np.floor(uu).astype(int), np.floor(vv).astype(int)
                fu, fv = uu - iu, vv - iv
                im = p["img"]
                val = (im[iv, iu] * (1 - fu) * (1 - fv) + im[iv, iu + 1] * fu * (1 - fv) +
                       im[iv + 1, iu] * (1 - fu) * fv + im[iv + 1, iu + 1] * fu * fv)
                g[idx] = val * p["gain"]
        gray[a:a + chunk] = g
    return gray


def fill_pits(h, max_hole_mm=15.0, cell=0.5, max_iter=40):
    """Fill NaN holes no wider than max_hole_mm that are enclosed by data, from their
    lowest neighbours (block holes become pits, not caps). Larger gaps (edges of the
    coverage, shadows) stay empty."""
    from scipy import ndimage
    h = h.copy()
    miss = ~np.isfinite(h)
    lab, n = ndimage.label(miss)
    if n:
        sizes = np.bincount(lab.ravel())
        # a hole touching the border is not enclosed
        border = np.unique(np.r_[lab[0], lab[-1], lab[:, 0], lab[:, -1]])
        r = max_hole_mm / cell / 2
        small = (sizes <= np.pi * r * r) & ~np.isin(np.arange(len(sizes)), border)
        small[0] = False
        target = small[lab]
    else:
        target = np.zeros(h.shape, bool)
    for _ in range(max_iter):
        todo = target & ~np.isfinite(h)
        if not todo.any():
            break
        big = np.where(np.isfinite(h), h, 1e9).astype(np.float32)
        mn = cv2.erode(big, np.ones((3, 3), np.uint8))
        put = todo & (mn < 1e8)
        h[put] = mn[put]
    return h


def despeckle(h, tol=2.0, passes=2):
    """Replace cells that stand out from their 5x5 median by > tol mm (reflection spikes);
    the median keeps real steps."""
    h = h.copy()
    for _ in range(passes):
        f = np.nan_to_num(h, nan=0.0).astype(np.float32)
        med = cv2.medianBlur(f, 5)
        bad = np.isfinite(h) & (np.abs(h - med) > tol)
        h[bad] = med[bad]
    # wider clusters that stick up (reflections in block holes, at back edges): 7x7 median, upward only
    from scipy import ndimage
    for _ in range(2):
        med = ndimage.median_filter(np.nan_to_num(h, nan=0.0), size=7)
        bad = np.isfinite(h) & (h > med + 3.0)
        h[bad] = med[bad]
    return h


def write_ply_points(path, xyz, gray, height):
    n = len(xyz)
    hdr = ("ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty float height\nend_header\n" % n)
    rec = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1"), ("h", "<f4")])
    rec["x"], rec["y"], rec["z"] = xyz.T
    g8 = np.clip(np.nan_to_num(gray, nan=128), 0, 255).astype(np.uint8)
    rec["r"] = rec["g"] = rec["b"] = g8
    rec["h"] = height
    with open(path, "wb") as f:
        f.write(hdr.encode()); f.write(rec.tobytes())


def write_ply_mesh(path, verts, gray, faces):
    hdr = ("ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\nelement face %d\nproperty list uchar int vertex_indices\nend_header\n"
           % (len(verts), len(faces)))
    v = np.zeros(len(verts), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
    v["x"], v["y"], v["z"] = verts.T
    g8 = np.clip(np.nan_to_num(gray, nan=128), 0, 255).astype(np.uint8)
    v["r"] = v["g"] = v["b"] = g8
    f = np.zeros(len(faces), dtype=[("n", "u1"), ("a", "<i4"), ("b", "<i4"), ("c", "<i4")])
    f["n"] = 3
    f["a"], f["b"], f["c"] = faces.T
    with open(path, "wb") as fh:
        fh.write(hdr.encode()); fh.write(v.tobytes()); fh.write(f.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("points")
    ap.add_argument("--photos", nargs="+", required=True)
    ap.add_argument("--calib", required=True, help="calibration set (config/calib/*.json)")
    ap.add_argument("--heights")
    ap.add_argument("--objects")
    ap.add_argument("--crop", type=float, nargs=4, default=(-300, 45, -195, 75), help="map-frame x0 x1 y0 y1")
    ap.add_argument("--max-cloud", type=int, default=2_000_000)
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE",
                    help="override pipeline.Params (specks_min_cells, support, fill_max_hole_mm, cloud_voxel_mm, cloud_ror_min, ...)")
    ap.add_argument("--despeckle", action="store_true",
                    help="old cosmetic median despeckle (off: the pipeline's cleaning replaces it)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    from carvera_scan.process.bench import parse_set
    prm = pipeline.Params(**parse_set(a.set))
    cal = calib_mod.load(a.calib)
    TO_MACHINE[:] = cal["map_to_machine"]
    d = np.load(a.points)
    xy, H = d["xy"].astype(np.float64), d["height"].astype(np.float32)
    grid, (gx0, gy0), cell = d["grid"].astype(np.float32), d["origin"], float(d["cell"])
    shift11 = d["shift"]

    # bed plane from bare spoilboard cells -> heights above the bed
    bp, GX, GY = bed_plane(grid, (gx0, gy0), cell)
    print("bed plane %+.3f %+.5f x %+.5f y" % (bp[2], bp[0], bp[1]))
    grid = grid - (bp[0] * GX + bp[1] * GY + bp[2])
    H = H - (bp[0] * xy[:, 0] + bp[1] * xy[:, 1] + bp[2])

    x0, x1, y0, y1 = a.crop
    j0, j1 = int((x0 - gx0) / cell), int((x1 - gx0) / cell)
    i0, i1 = int((y0 - gy0) / cell), int((y1 - gy0) / cell)
    hs = grid[i0:i1, j0:j1]
    if prm.specks_min_cells:
        hs, nsp = remove_specks(hs, prm.specks_min_cells)
        print("specks: %d cells removed" % nsp)
    hs = fill_pits(hs, max_hole_mm=prm.fill_max_hole_mm, cell=cell)
    if a.despeckle:
        hs = despeckle(hs)
    if prm.support:
        hs, nchg = support_filter(hs, prm.support_tol_mm, prm.support_min)
        print("support filter: %d cells replaced" % nchg)
    ny, nx = hs.shape
    sx0, sy0 = gx0 + j0 * cell, gy0 + i0 * cell
    G = Grid(grid, (gx0, gy0), cell)

    photos = load_photos(a.photos, cal, shift11)
    print("%d photos" % len(photos))

    # surface colours (cell centres at their own height)
    yy, xx = np.mgrid[0:ny, 0:nx]
    SX, SY = sx0 + (xx + .5) * cell, sy0 + (yy + .5) * cell
    valid = np.isfinite(hs)
    P = np.c_[SX[valid], SY[valid], hs[valid]]
    sg = np.full(hs.shape, np.nan, np.float32)
    sg[valid] = colorize(P, photos, G)
    print("surface %d x %d, %d cells, %.0f%% coloured" % (nx, ny, valid.sum(), 100 * np.isfinite(sg[valid]).mean()))

    # cloud: 3-D voxel medians inside the crop
    inb = (xy[:, 0] > x0) & (xy[:, 0] < x1) & (xy[:, 1] > y0) & (xy[:, 1] < y1) & (H > -3) & (H < 90)
    q = np.c_[xy[inb], H[inb]]
    vk = np.floor(q / prm.cloud_voxel_mm).astype(np.int64)
    vk -= vk.min(0)
    key = (vk[:, 0] * 4096 + vk[:, 1]) * 1024 + vk[:, 2]
    order = np.argsort(key, kind="stable")
    key, q = key[order], q[order]
    starts = np.flatnonzero(np.r_[True, np.diff(key) != 0])
    counts = np.diff(np.r_[starts, len(key)])
    keep = counts >= 3
    sums = np.add.reduceat(q, starts, axis=0)
    C = (sums / counts[:, None])[keep]
    if len(C) > a.max_cloud:
        C = C[np.random.default_rng(0).choice(len(C), a.max_cloud, replace=False)]
    # nothing here overhangs: points well above the nearby surface are reflections
    top = cv2.dilate(np.nan_to_num(hs, nan=-50).astype(np.float32), np.ones((5, 5), np.uint8))
    ci = np.clip(((C[:, 1] - sy0) / cell).astype(int), 0, ny - 1)
    cj = np.clip(((C[:, 0] - sx0) / cell).astype(int), 0, nx - 1)
    under = (C[:, 2] <= top[ci, cj] + 2.0) & (top[ci, cj] > -40)
    print("cloud: dropped %d points above the surface (reflections)" % (~under).sum())
    C = C[under]
    if prm.cloud_ror_min:
        import open3d as o3d
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(C.astype(np.float64)))
        _, ind = pc.remove_radius_outlier(nb_points=prm.cloud_ror_min, radius=prm.cloud_ror_radius_mm)
        print("cloud: radius outlier removal kept %d of %d" % (len(ind), len(C)))
        C = C[np.asarray(ind)]
    cg = colorize(C, photos, G)
    print("cloud: %d points -> %d voxels (%.1f mm, >= 3 points each), %.0f%% coloured" % (inb.sum(), len(C), prm.cloud_voxel_mm, 100 * np.isfinite(cg).mean()))

    # outputs for the viewer (map frame, centred) and PLY (machine frame)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hs.astype("<f4").tofile(os.path.join(a.out, "surface.bin"))
    np.clip(np.nan_to_num(sg, nan=110), 0, 255).astype(np.uint8).tofile(os.path.join(a.out, "surface_gray.bin"))
    cloud = np.c_[C[:, 0] - cx, C[:, 1] - cy, C[:, 2]].astype("<f4")
    cloud.tofile(os.path.join(a.out, "cloud.bin"))
    np.clip(np.nan_to_num(cg, nan=110), 0, 255).astype(np.uint8).tofile(os.path.join(a.out, "cloud_gray.bin"))

    labels = []
    if a.heights and a.objects:
        hrows = {r["name"]: r for r in json.load(open(a.heights))}
        for o in json.load(open(a.objects)):
            r = hrows[o["name"]]
            bx0, bx1, by0, by1 = o["box"]
            top = float(np.nanmedian(hs[max(int((by0 - sy0) / cell), 0):int((by1 - sy0) / cell), max(int((bx0 - sx0) / cell), 0):int((bx1 - sx0) / cell)]))
            labels.append({"name": o["name"], "x": (bx0 + bx1) / 2 - cx, "y": (by0 + by1) / 2 - cy, "z": top,
                           "label": r["label"], "nominal_mm": r["nominal_mm"], "measured_mm": r["measured_mm"],
                           "measured_in": r["measured_in"], "error_mm": r["error_mm"], "on": r.get("on"),
                           "dx": o.get("lab", [0, 0])[0], "dy": o.get("lab", [0, 0])[1]})
    meta = {"nx": nx, "ny": ny, "cell": cell, "x0": sx0 - cx, "y0": sy0 - cy, "cloud_n": len(cloud),
            "to_machine": [float(cx + TO_MACHINE[0]), float(cy + TO_MACHINE[1])], "labels": labels,
            "note": "viewer coordinates = map frame minus its centre; add to_machine for machine X, Y"}
    json.dump(meta, open(os.path.join(a.out, "meta.json"), "w"), indent=1)
    import shutil
    shutil.copy(VIEWER, os.path.join(a.out, "index.html"))

    # PLY (machine frame)
    write_ply_points(os.path.join(a.out, "cloud.ply"), np.c_[C[:, :2] + TO_MACHINE, C[:, 2]], cg, C[:, 2])
    vid = -np.ones(hs.shape, np.int64)
    vid[valid] = np.arange(valid.sum())
    verts = np.c_[SX[valid] + TO_MACHINE[0], SY[valid] + TO_MACHINE[1], hs[valid]]
    a_, b_, c_, d_ = vid[:-1, :-1], vid[:-1, 1:], vid[1:, :-1], vid[1:, 1:]
    f1 = np.stack([a_, b_, d_], -1).reshape(-1, 3)
    f2 = np.stack([a_, d_, c_], -1).reshape(-1, 3)
    faces = np.r_[f1[(f1 >= 0).all(1)], f2[(f2 >= 0).all(1)]]
    write_ply_mesh(os.path.join(a.out, "surface.ply"), verts, sg[valid], faces)
    print("PLY: cloud %d points, surface %d vertices %d faces" % (len(C), len(verts), len(faces)))

    # static images
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ext = [sx0 + TO_MACHINE[0], sx0 + nx * cell + TO_MACHINE[0], sy0 + TO_MACHINE[1], sy0 + ny * cell + TO_MACHINE[1]]
    cv2.imwrite(os.path.join(a.out, "ortho.png"), np.flipud(np.clip(np.nan_to_num(sg, nan=0), 0, 255)).astype(np.uint8))
    hz = np.nan_to_num(hs, nan=0.0)
    gyy, gxx = np.gradient(hz, cell)
    az, el = np.radians(315), np.radians(40)
    slope = np.arctan(np.hypot(gxx, gyy))
    aspect = np.arctan2(-gxx, gyy)
    shade = np.sin(el) * np.cos(slope) + np.cos(el) * np.sin(slope) * np.cos(az - aspect)
    cmap = plt.get_cmap("turbo")
    rgb = cmap(np.clip(np.log1p(np.clip(hz, 0, None)) / np.log1p(80), 0, 1))[..., :3] * (0.45 + 0.55 * np.clip(shade, 0, 1))[..., None]
    rgb[~np.isfinite(hs)] = 0.15
    fig, ax = plt.subplots(figsize=(16, 12))
    ax.imshow(rgb, origin="lower", extent=ext)
    for lb in labels:
        X, Y = lb["x"] + cx + TO_MACHINE[0], lb["y"] + cy + TO_MACHINE[1]
        ax.annotate("%s: %.2f mm (%.4f\")\nlabel %.2f mm, %+.2f mm" % (lb["label"], lb["measured_mm"], lb["measured_in"], lb["nominal_mm"], lb["error_mm"]),
                    (X, Y), xytext=(X + lb["dx"], Y + lb["dy"]), color="w", fontsize=8.5, ha="center", va="center", weight="bold",
                    bbox=dict(fc=(0, 0, 0, .7), ec="none", pad=2),
                    arrowprops=dict(arrowstyle="-", color="w", lw=1) if (lb["dx"] or lb["dy"]) else None)
    ax.set_xlabel("machine X, mm"); ax.set_ylabel("machine Y, mm")
    ax.set_title("1-2-3 blocks and setup bars, 2026-11-05: height above the spoilboard (log colour, hillshade), measured vs label")
    plt.tight_layout(); plt.savefig(os.path.join(a.out, "relief.png"), dpi=90)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
