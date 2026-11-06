"""The standard processing chain for cross-laser sweeps, with its default settings.

Everything that turns captures into a cleaned point set, a height map and a surface
goes through here: the benchmark (bench.py), the viewer builder (view3d.py) and,
later, `cscan process`. Change a default here and every project gets it; check the
change with `python -m carvera_scan.process.bench` (regression on the 2026-11-05 data) first.

    from carvera_scan.process.pipeline import Params, run
    P, grid, origin, info = run([("data/…/scan_1.2", "1.2"), ("data/…/scan_1.1", "1.1")], "config/calib/….json")

From the shell (on Buzzkill), captures -> OUT/points.npz (the view3d / measure input) + OUT/info.json:

    python -m carvera_scan.process.pipeline data/2026-11-05/duo1_1.2:1.2 data/2026-11-05/duo1_1.1:1.1 \
        --calib config/calib/2026-11-05-cross45-deep.json --out projects/cutebot-duo/out/scan [--set cell_mm=0.25]

Chain:
  1. scan_points: arms found in every frame (cross.extract, cached), heights from the
     calibration's laser model with the bed row taken from the scan itself, range gate;
     X, Y from head position + image offset (calibration's axis matrix, centre, height).
  2. merge: camera B's points moved into camera A's map frame (grid-matched shift
     from the calibration's guess, optionally ICP); sources labelled camera x arm.
  3. clean: keep points that at least two camera x arm sources agree on (reflections
     appear in one); optional per-source height self-calibration.
  4. grid: per cell the densest height band (walls snap to top or bed), small islands
     removed, cells without neighbour support replaced.
  5. surface (viewer): enclosed small holes filled from their lowest neighbours.
"""
import argparse
import hashlib
import json
import os
from dataclasses import asdict, dataclass, field, fields

import numpy as np

from carvera_scan import calib as calib_mod
from carvera_scan.process import clean as clean_mod
from carvera_scan.process import cross
from carvera_scan.process.blocks_merge import best_shift
from carvera_scan.process.grid import grid_mode, remove_specks, support_filter


@dataclass
class Params:
    # --- extraction (image pass; cached per value) ---
    gate: bool = True              # search for, and keep, only heights from -3 mm to the calibrated depth + 3 mm
    ambig: float = 0.6             # drop a column whose 2nd ridge reaches this x the 1st (two candidate lines); 0 = off
    # --- heights ---
    bedref: str = "scan"           # bed row per arm column: "scan" (lowest strong row band in this scan) or "zsweep"
    # --- merge ---
    register: str = "grid"         # camera B -> A: "grid" (2-D shift by height-grid matching), "icp", or
                                   # "none" (the calibration's b_to_a_shift as is: e.g. --b-shift from a bigger scan)
    register_span_mm: float = 8.0  # search +- this around the calibration's B -> A shift
    # --- cleaning ---
    consistency: int = 2           # a point needs this many camera x arm sources within ~1 mm (0 = off)
    consistency_key: str = "src"   # "src" = camera x arm (independent geometry); "view" = scan x arm (older, weaker)
    consistency_voxel_mm: float = 1.0
    selfcal: bool = False          # per camera x arm height offset/scale from overlaps (tried: mixed result)
    # --- grid ---
    cell_mm: float = 0.5
    grid: str = "mode"             # "mode" (densest 1 mm band, mean-shift refined) or "median"
    specks_min_cells: int = 20     # drop islands smaller than this (20 cells = 5 mm^2 at 0.5 mm); 0 = off
    support: bool = True           # a cell needs >= support_min of 8 neighbours within support_tol_mm
    support_tol_mm: float = 1.5
    support_min: int = 3
    # --- surface for the viewer ---
    fill_max_hole_mm: float = 15.0  # fill enclosed holes up to this size (no data inside, e.g. drilled holes)
    cloud_voxel_mm: float = 0.5
    cloud_ror_min: int = 6          # radius outlier removal: points needed within cloud_ror_radius_mm (0 = off)
    cloud_ror_radius_mm: float = 1.5

    @classmethod
    def from_dict(cls, d):
        names = {f.name for f in fields(cls)}
        unknown = set(d) - names
        if unknown:
            raise KeyError("unknown processing parameters: %s" % sorted(unknown))
        return cls(**d)

    def extraction(self):
        return {"gate": self.gate, "ambig": self.ambig or None}


def scan_points(scan_dir, port, cal, p, cache=None):
    """Points of one sweep from the camera on `port`: xy (camera's own map frame), height, arm, frame."""
    cam = calib_mod.camera(cal, calib_mod.role_of_port(cal, port))
    laser = cam["laser"]
    E = None
    if cache:
        key = hashlib.md5(json.dumps([p.extraction(), cal["name"], port, laser["arms"]], sort_keys=True,
                                     default=str).encode()).hexdigest()[:10]
        path = os.path.join(cache, "E_%s_%s.npz" % (os.path.basename(scan_dir.rstrip("/")), key))
        if os.path.exists(path):
            E = dict(np.load(path))
    if E is None:
        E = cross.extract(scan_dir, laser, p.extraction())
        if cache:
            os.makedirs(cache, exist_ok=True)
            np.savez_compressed(path, **E)
    h, info = cross.compute_heights(E, laser, p.bedref, p.gate)
    for b in info:
        b["port"] = port
    xy, _ = cross.place_cal(E, h, scan_dir, cam["axis_matrix"], cam["centre"], cam["H"], laser["z0"])
    good = np.isfinite(xy).all(1) & np.isfinite(h)
    return {"xy": xy[good], "height": h[good], "arm": E["arm"][good], "frame": E["frame"][good], "bed_info": info}


def merge(scans, cal, p, cache=None, extra_shift=(0.0, 0.0)):
    """scans: [(scan_dir, port), ...]. Returns (P, info). P has xy (camera A's map frame,
    plus extra_shift), height, src (role index x 2 + arm), view (scan index x 2 + arm), scan."""
    roles = {r: i for i, r in enumerate(sorted(cal["cameras"]))}  # A -> 0, B -> 1
    pts = []
    for k, (d, port) in enumerate(scans):
        role = calib_mod.role_of_port(cal, port)
        pts.append((scan_points(d, port, cal, p, cache), roles[role], k))
    A = [q for q, r, _ in pts if r == 0]
    B = [q for q, r, _ in pts if r == 1]
    info = {"bed_rows": [q["bed_info"] for q, _, _ in pts]}
    extra = np.asarray(extra_shift, float)
    fB = None
    if B and p.register == "none":
        sh = np.asarray(cal["b_to_a_shift"], float)
        info["grid_shift"] = sh.tolist()
        fB = lambda xy, s=sh: xy + s + extra  # noqa: E731
    elif B:
        sh, e = best_shift(A[0], B[0], tuple(cal["b_to_a_shift"]), span=p.register_span_mm)
        info["grid_shift"] = [float(sh[0]), float(sh[1])]
        if p.register == "icp":
            reg = clean_mod.register_icp(np.concatenate([np.c_[q["xy"], q["height"]] for q in A]),
                                         np.concatenate([np.c_[q["xy"], q["height"]] for q in B]), sh)
            info["icp"] = {k: v for k, v in reg.items() if k != "T"}
            fB = lambda xy, r=reg: clean_mod.apply_xy_yaw(xy, r) + extra  # noqa: E731
        else:
            fB = lambda xy, s=sh: xy + s + extra  # noqa: E731
    xy, h, src, view, sc = [], [], [], [], []
    for q, r, k in pts:
        xy.append(q["xy"] + extra if r == 0 else fB(q["xy"]))
        h.append(q["height"])
        src.append(r * 2 + q["arm"].astype(np.int64))
        view.append(k * 2 + q["arm"].astype(np.int64))
        sc.append(np.full(len(q["height"]), k))
    P = {"xy": np.concatenate(xy), "height": np.concatenate(h).astype(np.float64), "src": np.concatenate(src),
         "view": np.concatenate(view), "scan": np.concatenate(sc)}
    return P, info


def clean(P, p):
    info = {}
    if p.selfcal:
        from carvera_scan.process import bench  # selfcal lives with the benchmark until it earns a default
        P, info["selfcal"] = bench.selfcal(P)
    if p.consistency:
        key = P["src"] if p.consistency_key == "src" else P["view"]
        keep = clean_mod.consistency(np.c_[P["xy"], P["height"]], key, voxel=p.consistency_voxel_mm,
                                     min_sources=p.consistency)
        info["consistency_dropped"] = int((~keep).sum())
        P = {k: v[keep] for k, v in P.items()}
    return P, info


def median_grid(xy, h, cell, origin, shape, min_count=4):
    x0, y0 = origin
    ix = ((xy[:, 0] - x0) / cell).astype(np.int64); iy = ((xy[:, 1] - y0) / cell).astype(np.int64)
    ok = (ix >= 0) & (iy >= 0) & (ix < shape[1]) & (iy < shape[0])
    key = iy[ok] * shape[1] + ix[ok]; hh = h[ok]
    o = np.argsort(key, kind="stable"); key, hh = key[o], hh[o]
    st = np.flatnonzero(np.r_[True, np.diff(key) != 0]); cnt = np.diff(np.r_[st, len(key)])
    g = np.full(shape[0] * shape[1], np.nan, np.float32)
    good = cnt >= min_count
    g[key[st][good]] = [np.median(hh[a:a + n]) for a, n in zip(st[good], cnt[good])]
    return g.reshape(shape)


def grid(P, p, origin=None, shape=None):
    """Height map of P. origin/shape default to P's extent. Returns (grid, origin, info)."""
    c = p.cell_mm
    if origin is None:
        origin = tuple(np.floor(P["xy"].min(0)))
    if shape is None:
        span = P["xy"].max(0) - np.asarray(origin)
        shape = (int(span[1] / c) + 1, int(span[0] / c) + 1)
    if p.grid == "mode":
        g, _, _, _ = grid_mode(P["xy"], P["height"], c, origin=origin, shape=shape)
        g = g.astype(np.float32)
    else:
        g = median_grid(P["xy"], P["height"], c, origin, shape)
    info = {}
    if p.specks_min_cells:
        g, info["specks_removed"] = remove_specks(g, p.specks_min_cells)
    if p.support:
        g, info["support_changed"] = support_filter(g, p.support_tol_mm, p.support_min)
    return g, origin, info


def run(scans, calibration, params=None, cache=None, origin=None, shape=None, extra_shift=(0.0, 0.0)):
    """The whole chain. Returns (cleaned points, grid, info)."""
    cal = calib_mod.load(calibration) if isinstance(calibration, str) else calibration
    p = params or Params()
    P, info = merge(scans, cal, p, cache, extra_shift)
    info["points_merged"] = int(len(P["height"]))
    P, ci = clean(P, p)
    info.update(ci)
    g, origin, gi = grid(P, p, origin, shape)
    info.update(gi)
    info["params"] = asdict(p)
    info["calibration"] = cal.get("name")
    return P, g, origin, info


def save_points(path, P, g, origin, cell, info):
    """The points file view3d and the measure modules read: cleaned points (map frame of
    camera A), camera/arm/scan labels, the height grid and camera B's registered shift."""
    np.savez_compressed(path, xy=P["xy"].astype(np.float32), height=P["height"].astype(np.float32),
                        cam=(P["src"] // 2).astype(np.int8), arm=(P["src"] % 2).astype(np.int8),
                        scan=P["scan"].astype(np.int8), grid=g, origin=origin, cell=cell,
                        shift=np.array(info.get("grid_shift", [0, 0])))


def main():
    from carvera_scan.process.bench import parse_set
    ap = argparse.ArgumentParser(description="captures -> cleaned points + height grid")
    ap.add_argument("scans", nargs="+", metavar="DIR:PORT", help="sweep capture and its camera's USB port")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--out", required=True, help="directory: points.npz, info.json")
    ap.add_argument("--cache", help="extraction cache (default OUT/../cache)")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="override Params defaults")
    ap.add_argument("--b-shift", type=float, nargs=2, metavar=("DX", "DY"),
                    help="camera B -> A shift to use as is (e.g. the grid_shift of a bigger scan in the same session)")
    a = ap.parse_args()
    scans = [tuple(s.rsplit(":", 1)) for s in a.scans]
    p = Params(**parse_set(a.set))
    cal = calib_mod.load(a.calib)
    if a.b_shift:
        cal["b_to_a_shift"], p.register = list(a.b_shift), "none"
    os.makedirs(a.out, exist_ok=True)
    cache = a.cache or os.path.join(os.path.dirname(a.out.rstrip("/")), "cache")
    P, g, origin, info = run(scans, cal, p, cache)
    save_points(os.path.join(a.out, "points.npz"), P, g, origin, p.cell_mm, info)
    info["scans"] = a.scans
    json.dump(info, open(os.path.join(a.out, "info.json"), "w"), indent=1, default=str)
    print("%d points, grid %s at %s, B shift %s -> %s" % (len(P["height"]), g.shape, origin, info.get("grid_shift"), a.out))


if __name__ == "__main__":
    main()
