"""Still frames of the laser cross: where the cross centre lands, and the centre and
size of a hole (or a boss) from the gaps it cuts in the two arms.

The cross is two laser planes. At height h each arm is a known line in the image
(the calibration's laser model run backwards), so the cross centre at height h is a
known pixel, and through the camera model a known offset from the head. Put the head
at target - offset(h) and the cross centre sits on the target.

Over a hole the arm leaves the top surface at the rim and comes back at the far rim:
its line on the surface has a gap, and the gap's two ends are points on the rim
circle. Each arm gives a chord; the two arms cross at about 90 deg, so one frame
gives two chords (four rim points), which fix the centre and radius. Shots with the
cross a little off centre give more chords; all rim points from all shots and both
cameras go into one circle fit. A boss works the same way with "raised" for "gap".

    from carvera_scan.process import spot
    geo = spot.Geometry(cal, info)                  # calibration + the scan's bed lines and B shift
    head = geo.head_for("A", target_xy, h)          # head XY that puts the cross centre on target at height h
    prof = geo.profile(img, "A", head_xy)           # per arm: image points, height, map XY
    rim = geo.chord_ends(prof, "A", head_xy, target_xy, h, kind="hole")
    fit = fit_circle(rim)

    python -m carvera_scan.process.spot plan FEATURES.json --calib C --info OUT/info.json --out PLAN.json [--offsets 0.35]
    python -m carvera_scan.process.spot measure CAPTURE --plan PLAN.json --calib C --info OUT/info.json --out RESULT.json

    python -m carvera_scan.process.spot passes FEATURES.json --calib C --info OUT/info.json --out PASSES.json

`passes` writes fine-scan passes for acquire.line_scan --passes: per object, one pass along
Y with the cross centre down the middle of its features, long enough that both arms
(at about +-45 deg) sweep every feature.
`plan` writes the head positions (one per shot) that `acquire.survey --at` visits;
`measure` reads the frames that survey wrote and fits each feature.
"""
import argparse
import json
import os

import cv2
import numpy as np

from carvera_scan import calib as calib_mod
from carvera_scan.process import cross


def fit_circle(pts):
    """Least-squares circle (algebraic, then geometric refinement). Returns (cx, cy, r, rms) or None."""
    pts = np.asarray(pts, float)
    if len(pts) < 3:
        return None
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    sol, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    cx, cy = sol[0], sol[1]
    r = np.sqrt(max(sol[2] + cx * cx + cy * cy, 0))
    for _ in range(10):  # Gauss-Newton on the geometric distance
        d = np.hypot(x - cx, y - cy)
        J = np.c_[-(x - cx) / d, -(y - cy) / d, -np.ones(len(x))]
        res = d - r
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        cx, cy, r = cx + step[0], cy + step[1], r + step[2]
        if np.abs(step).max() < 1e-6:
            break
    res = np.hypot(x - cx, y - cy) - r
    return float(cx), float(cy), float(abs(r)), float(np.sqrt(np.mean(res ** 2)))


def line_fit(p):
    """Point on and unit direction of the best line through p (N x 2)."""
    m = p.mean(0)
    _, _, vt = np.linalg.svd(p - m)
    return m, vt[0]


def intersect(m1, d1, m2, d2):
    t = np.linalg.solve(np.c_[d1, -d2], m2 - m1)
    return m1 + t[0] * d1


class Geometry:
    """Camera + laser geometry for still frames at the scan's head Z.

    cal: calibration set (dict or path). info: the pipeline's info.json (dict or path);
    from it come each camera's bed lines (bed_rows, per scan and arm; the median line per
    camera is used) and camera B's registered shift into camera A's map frame."""

    def __init__(self, cal, info=None, z=None):
        self.cal = calib_mod.load(cal) if isinstance(cal, str) else cal
        info = json.load(open(info)) if isinstance(info, str) else (info or {})
        self.shift = {"A": np.zeros(2), "B": np.asarray(info.get("grid_shift", self.cal["b_to_a_shift"]), float)}
        self.z = self.cal["z0"] if z is None else z
        self.arms = {}
        for role, cam in self.cal["cameras"].items():
            arms = [dict(a) for a in cam["laser"]["arms"]]
            lines = {}
            for scan in info.get("bed_rows", []):
                for b in scan:
                    if b.get("port") == cam["port"] and "bed_line" in b:
                        lines.setdefault(b["arm"], []).append(b["bed_line"])
            for ai, a in enumerate(arms):
                if lines.get(ai):
                    # evaluate each scan's line and fit the median: robust to one odd scan
                    c = np.arange(0, cross.SIDE, 8.0)
                    rows = np.median([np.polyval(k, c) for k in lines[ai]], axis=0)
                    a["bed_line"] = np.polyfit(c, rows, 2).tolist()
            self.arms[role] = arms

    def cam(self, role):
        return self.cal["cameras"][role]

    def Hc(self, role):
        return self.cam(role)["H"] + (self.z - self.cal["z0"])

    def to_map(self, role, uv, h, head):
        """Map-frame (camera A) XY of image points uv at heights h, head at `head`."""
        cam = self.cam(role)
        Minv = np.linalg.inv(np.asarray(cam["axis_matrix"], float))
        rel = (np.atleast_2d(uv) - cam["centre"]) @ Minv.T
        Hc = self.Hc(role)
        k = (Hc - np.atleast_1d(h)) / Hc
        return np.asarray(head, float) + rel * k[:, None] + self.shift[role]

    def arm_line_uv(self, role, ai, h):
        """Image points along arm ai where it falls on a surface at height h."""
        a = self.arms[role][ai]
        cols = np.arange(0, cross.SIDE, 4.0)
        sc = (cols - cross.SIDE / 2) / cross.SIDE
        d = cross.d_at(a, h, sc)
        bed = np.polyval(a["bed_line"], cols)
        rows = bed - d if a["sign"] < 0 else bed + d
        Mi = cv2.invertAffineTransform(cross.turn(a["theta"]))
        uv = np.c_[cols, rows, np.ones(len(cols))] @ Mi.T
        ok = (uv[:, 0] >= 0) & (uv[:, 0] < cross.W) & (uv[:, 1] >= 0) & (uv[:, 1] < cross.H)
        return uv[ok]

    def centre_uv(self, role, h):
        """Pixel of the cross centre on a surface at height h."""
        m1, d1 = line_fit(self.arm_line_uv(role, 0, h))
        m2, d2 = line_fit(self.arm_line_uv(role, 1, h))
        return intersect(m1, d1, m2, d2)

    def centre_offset(self, role, h):
        """Map-frame position of the cross centre at height h, relative to the head."""
        return self.to_map(role, self.centre_uv(role, h), h, (0.0, 0.0))[0]

    def head_for(self, role, target, h):
        return np.asarray(target, float) - self.centre_offset(role, h)

    def arm_dir(self, role, ai, h):
        """Unit direction of arm ai in the map frame."""
        uv = self.arm_line_uv(role, ai, h)
        return line_fit(self.to_map(role, uv, np.full(len(uv), h), (0, 0)))[1]

    def profile(self, img, role, ambig=0.6, min_len=4):
        """Per arm: the arm's ridge in this frame (u, v, turned col, height). Short runs are
        kept (min_len px) so thin rims survive."""
        out = []
        for a in self.arms[role]:
            band = cross.arm_band(a, below=8.0, gate=True)
            pts, cl, rl = cross.arm_points(img, a["theta"], band, ambig=ambig, min_len=min_len)
            h = cross.arm_height(a, cl.astype(float), rl.astype(float))
            out.append({"uv": pts[:, :2], "col": cl.astype(int), "h": h})
        return out

    def chord_ends(self, prof, role, head, target, h, kind="hole", r_max=6.0, thr=1.2):
        """Rim points from each arm's chord around the target (map frame, at height h).

        Along each arm, columns within r_max mm of the target are "surface" if their
        height is within thr of h (hole: the top surface around it) or above h - thr
        (boss: h is then the boss top). The run of non-surface columns (missing counts)
        that holds, or is nearest to, the target is the chord; its ends (half a column
        out from the last surface sample) are rim points. A chord needs surface on both
        sides. Returns [(arm, end_xy), ...] and per-arm notes."""
        ends, notes = [], []
        for ai, p in enumerate(prof):
            if len(p["col"]) < 10:
                notes.append({"arm": ai, "why": "no line"}); continue
            xy = self.to_map(role, p["uv"], np.full(len(p["col"]), h), head)
            t = self.arm_dir(role, ai, h)
            s = (xy - target) @ t
            perp = np.abs((xy - target) @ np.array([-t[1], t[0]]))
            near = (np.abs(s) < r_max + 3) & (perp < 3)
            if near.sum() < 6:
                notes.append({"arm": ai, "why": "arm not at target"}); continue
            col, hh, ss, uv = p["col"][near], p["h"][near], s[near], p["uv"][near]
            o = np.argsort(col)
            col, hh, ss, uv = col[o], hh[o], ss[o], uv[o]
            # every column in range: surface (1), off (0: lower/higher or missing)
            c0, c1 = col.min(), col.max()
            allc = np.arange(c0, c1 + 1)
            surf = np.zeros(len(allc), bool)
            ok = np.abs(hh - h) < thr if kind == "hole" else hh > h - thr
            surf[col[ok] - c0] = True
            # mm per column and the target's column (s = 0)
            k = np.polyfit(col, ss, 1)
            ct = -k[1] / k[0]
            # runs of off-surface columns
            off = ~surf
            edges = np.flatnonzero(np.diff(np.r_[0, off.astype(int), 0]))
            runs = list(zip(edges[::2], edges[1::2]))  # [start, stop) in allc index
            if kind == "boss":  # the chord is the surface run itself
                on = surf
                edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
                runs = list(zip(edges[::2], edges[1::2]))
            best = None
            for a_, b_ in runs:
                if a_ == 0 or b_ == len(allc):
                    continue  # not bounded on both sides
                lo, hi = allc[a_] - 0.5, allc[b_ - 1] + 0.5
                if kind == "hole":
                    lo, hi = lo, hi  # gap ends: half a column past the last surface sample
                dist = 0.0 if lo <= ct <= hi else min(abs(lo - ct), abs(hi - ct))
                length = abs(k[0]) * (hi - lo)
                if length < 0.6 or length > 2 * r_max:
                    continue
                if best is None or dist < best[0]:
                    best = (dist, lo, hi, length)
            if best is None or abs(k[0]) * best[0] > 1.5:
                notes.append({"arm": ai, "why": "no %s chord at target" % kind}); continue
            _, lo, hi, length = best
            # column -> image uv by interpolating the arm's samples; then -> map at height h
            for c in (lo, hi):
                u = np.interp(c, col, uv[:, 0]); v = np.interp(c, col, uv[:, 1])
                ends.append((ai, self.to_map(role, np.array([[u, v]]), np.array([h]), head)[0]))
            mid_s = np.polyval(k, (lo + hi) / 2)
            notes.append({"arm": ai, "chord_mm": round(float(length), 3), "mid_off_mm": round(float(mid_s), 3)})
        return ends, notes


def centre_from_two_chords(ends):
    """Centre from one frame's two chords: the perpendicular bisectors cross at the centre."""
    by = {}
    for ai, p in ends:
        by.setdefault(ai, []).append(p)
    if len(by) < 2 or any(len(v) != 2 for v in by.values()):
        return None
    (a1, b1), (a2, b2) = by[0], by[1]
    m1, m2 = (a1 + b1) / 2, (a2 + b2) / 2
    t1, t2 = (b1 - a1), (b2 - a2)
    n1, n2 = np.array([-t1[1], t1[0]]), np.array([-t2[1], t2[0]])
    try:
        return intersect(m1, n1, m2, n2)
    except np.linalg.LinAlgError:
        return None


# ---------------------------------------------------------------- command line
def plan(a):
    geo = Geometry(a.calib, a.info)
    feats = json.load(open(a.features))
    feats = feats["features"] if isinstance(feats, dict) else feats
    shots = []
    for f in feats:
        if f["kind"] not in a.kinds or "xy" not in f or not f.get("ok", True):
            continue
        h, r = f["z"], f.get("diameter", 4.0) / 2
        tgt = np.asarray(f["xy"], float)
        offs = [(0.0, 0.0)]
        if a.offsets:
            d0, d1 = geo.arm_dir("A", 0, h), geo.arm_dir("A", 1, h)
            for d in (d0, -d0, d1, -d1):
                offs.append(tuple(a.offsets * r * d))
        for o in offs:
            aim = tgt + np.asarray(o)
            head = geo.head_for("A", aim, h)
            if not (-369.0 <= head[0] <= -2.5 and -247.5 <= head[1] <= -2.5):
                print("skip %s: head %s outside travel" % (f["id"], head.round(1)))
                break
            shots.append({"id": f["id"], "kind": f["kind"], "z": h, "target": aim.round(3).tolist(),
                          "head": head.round(3).tolist(), "r_guess": r})
    json.dump({"shots": shots, "calib": geo.cal["name"]}, open(a.out, "w"), indent=1)
    json.dump([s["head"] for s in shots], open(os.path.splitext(a.out)[0] + "-at.json", "w"))
    print("%d shots for %d features -> %s (+ -at.json for acquire.survey --at)"
          % (len(shots), len({s["id"] for s in shots}), a.out))


def measure(a):
    geo = Geometry(a.calib, a.info)
    pl = json.load(open(a.plan))["shots"]
    st = json.load(open(os.path.join(a.capture, "stops.json")))
    roles = {c: calib_mod.role_of_port(geo.cal, c) for c in st["cams"]}
    by_id = {}
    for s, shot in zip(st["stops"], pl):
        head = np.asarray(s["mpos"][:2], float)
        for cam, role in roles.items():
            img = cv2.imread(os.path.join(a.capture, cam, "%d.png" % s["i"]), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            prof = geo.profile(img, role)
            r_max = max(3.0, 1.6 * shot["r_guess"] + 1.0)
            ends, notes = geo.chord_ends(prof, role, head, np.asarray(shot["target"]), shot["z"], shot["kind"], r_max=r_max)
            by_id.setdefault(shot["id"], {"kind": shot["kind"], "z": shot["z"], "rim": [], "shots": [],
                                          "r_guess": shot["r_guess"]})
            e = by_id[shot["id"]]
            e["rim"] += [(role, p) for _, p in ends]
            c2 = centre_from_two_chords(ends)
            e["shots"].append({"i": s["i"], "cam": cam, "target": shot["target"], "notes": notes,
                               "centre_2chord": None if c2 is None else np.round(c2, 3).tolist()})
    out = []
    for fid, e in by_id.items():
        rim = np.array([p for _, p in e["rim"]]) if e["rim"] else np.zeros((0, 2))
        fit = fit_circle(rim) if len(rim) >= 3 else None
        row = {"id": fid, "kind": e["kind"], "z": e["z"], "rim_points": len(rim), "shots": e["shots"]}
        if fit:
            cx, cy, r, rms = fit
            # drop rim points > 3 x rms (or 0.3 mm) off the circle and refit once
            d = np.abs(np.hypot(rim[:, 0] - cx, rim[:, 1] - cy) - r)
            keep = d < max(3 * rms, 0.3)
            if keep.sum() >= 4 and keep.sum() < len(rim):
                cx, cy, r, rms = fit_circle(rim[keep])
            per = {}
            for role in ("A", "B"):
                pts = np.array([p for ro, p in e["rim"] if ro == role])
                f2 = fit_circle(pts) if len(pts) >= 4 else None
                if f2:
                    per[role] = [round(f2[0], 3), round(f2[1], 3), round(2 * f2[2], 3)]
            row.update({"xy": [round(cx, 3), round(cy, 3)], "diameter": round(2 * r, 3), "rms": round(rms, 3),
                        "used": int(keep.sum()), "per_camera": per})
            # a circle much bigger than the search, or a poor fit, is not this feature
            r_lim = max(3.0, 1.6 * e["r_guess"] + 1.0)
            row["ok"] = bool(r <= r_lim and rms <= 0.5 and len(per) >= 1)
        out.append(row)
    json.dump({"capture": a.capture, "calib": geo.cal["name"], "features": out}, open(a.out, "w"), indent=1)
    ok = [r for r in out if r.get("ok")]
    print("%d of %d features fitted -> %s" % (len(ok), len(out), a.out))
    for r in ok:
        pc = r["per_camera"]
        ab = ("A-B %.2f mm" % np.hypot(pc["A"][0] - pc["B"][0], pc["A"][1] - pc["B"][1])) if len(pc) == 2 else ""
        print("  %-8s %-5s (%8.2f, %8.2f) d %.2f rms %.3f n %d %s" % (r["id"], r["kind"], *r["xy"], r["diameter"], r["rms"], r["used"], ab))


def passes(a):
    geo = Geometry(a.calib, a.info)
    d = json.load(open(a.features))
    feats = d["features"] if isinstance(d, dict) else d
    feats = [f for f in feats if f["kind"] in a.kinds and "xy" in f and f.get("ok", True)]
    groups = {}
    for f in feats:
        groups.setdefault(f.get("object"), []).append(f)
    out = []
    for obj, fs in sorted(groups.items(), key=lambda kv: str(kv[0])):
        xy = np.array([f["xy"] for f in fs], float)
        h = float(np.median([f["z"] for f in fs]))
        off = geo.centre_offset("A", h)
        cx = (xy[:, 0].min() + xy[:, 0].max()) / 2
        reach = np.abs(xy[:, 0] - cx).max()  # an arm at 45 deg passes a feature dx away when the centre is dx away in Y
        y0, y1 = xy[:, 1].min() - reach - a.margin, xy[:, 1].max() + reach + a.margin
        k = len(out)
        hy = [y0 - off[1], y1 - off[1]] if k % 2 == 0 else [y1 - off[1], y0 - off[1]]
        out.append({"along": "y", "at": round(cx - off[0], 3), "from": round(hy[0], 3), "to": round(hy[1], 3),
                    "object": obj, "features": [f["id"] for f in fs]})
        print("object %s: %d features, head X %.1f, Y %.1f .. %.1f" % (obj, len(fs), cx - off[0], *hy))
    json.dump(out, open(a.out, "w"), indent=1)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan", help="head positions that put the cross centre on each feature")
    p.add_argument("features", help="features.json (measure.features) or a list of {id, kind, xy, z, diameter}")
    p.add_argument("--calib", required=True)
    p.add_argument("--info", help="pipeline info.json of the scan (bed lines, B shift)")
    p.add_argument("--kinds", nargs="+", default=["hole"])
    p.add_argument("--offsets", type=float, default=0.0, help="also shoot +-this x radius along each arm (0 = centre only)")
    p.add_argument("--out", required=True)
    m = sub.add_parser("measure", help="fit each feature from the frames of a survey --at capture")
    m.add_argument("capture")
    m.add_argument("--plan", required=True)
    m.add_argument("--calib", required=True)
    m.add_argument("--info")
    m.add_argument("--out", required=True)
    q = sub.add_parser("passes", help="fine line-scan passes over each object's features")
    q.add_argument("features")
    q.add_argument("--calib", required=True)
    q.add_argument("--info")
    q.add_argument("--kinds", nargs="+", default=["hole", "boss"])
    q.add_argument("--margin", type=float, default=8.0)
    q.add_argument("--out", required=True)
    a = ap.parse_args()
    {"plan": plan, "measure": measure, "passes": passes}[a.cmd](a)


if __name__ == "__main__":
    main()
