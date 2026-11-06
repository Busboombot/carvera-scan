"""Compare M466 probe touches (`carvera_scan.acquire.probe_z`) with the laser map.

    python -m carvera_scan.measure.probe_compare data/2026-11-05/probe1/probe1.jsonl data/2026-11-05/flat2.npz \
        --offset -71.7 -68.1

--offset: machine (spindle) X, Y = laser-map X, Y + offset (from the anchor
corner). Spots named B* are bare bed, the rest are on the sheet. Both methods
are reduced to height above their own bed plane (probe: plane through the B
touches; laser: plane fitted to bare spoilboard around the sheet), so neither
needs an absolute Z reference.
"""
import argparse
import json

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("probes")
    ap.add_argument("flat")
    ap.add_argument("--offset", type=float, nargs=2, required=True)
    ap.add_argument("--radius", type=float, default=3.0, help="mm around each spot for the laser median")
    a = ap.parse_args()
    recs = {}
    for ln in open(a.probes):
        r = json.loads(ln)
        recs.setdefault(r["spot"], []).append(r)
    g = np.load(a.flat)
    h, bedp = g["height"], g["bed_plane"]
    X0, Y0 = g["origin"]
    c = float(g["cell"])
    yy, xx = np.mgrid[0:h.shape[0], 0:h.shape[1]]
    X, Y = X0 + (xx + .5) * c, Y0 + (yy + .5) * c

    def laser(xm, ym):
        mx, my = xm - a.offset[0], ym - a.offset[1]
        m = np.isfinite(h) & ((X - mx) ** 2 + (Y - my) ** 2 <= a.radius ** 2)
        v = h[m] - (bedp[0] * X[m] + bedp[1] * Y[m] + bedp[2])
        return (mx, my), float(np.median(v)), float(np.std(v)), int(m.sum())

    spots = {k: (v[0]["x"], v[0]["y"], np.mean([r["z_machine"] for r in v]), [r["z_machine"] for r in v]) for k, v in recs.items()}
    bed = [k for k in spots if k.startswith("B")]
    A = np.array([[1, spots[k][0], spots[k][1]] for k in bed])
    z = np.array([spots[k][2] for k in bed])
    coef, *_ = np.linalg.lstsq(A, z, rcond=None)
    print("probe bed plane: z = %.3f %+.5f X %+.5f Y (tilt %.3f mm/100 mm X, %.3f mm/100 mm Y); bed residuals %s mm" % (
        *coef, coef[1] * 100, coef[2] * 100, np.round(z - A @ coef, 3)))
    rows = []
    print("%-4s %9s %9s  %9s  %9s %9s  %7s %7s %7s" % ("spot", "X mach", "Y mach", "touch Z", "map X", "map Y", "probe h", "laser h", "diff"))
    for k, (x, y, zm, allz) in sorted(spots.items()):
        hp = zm - (coef[0] + coef[1] * x + coef[2] * y)
        (mx, my), hl, sd, n = laser(x, y)
        rows.append({"spot": k, "x": x, "y": y, "touch_z": allz, "map_xy": [mx, my], "probe_h": hp, "laser_h": hl,
                     "laser_sd": sd, "laser_n": n, "diff": hl - hp})
        print("%-4s %9.2f %9.2f  %9.3f  %9.1f %9.1f  %7.3f %7.3f %+7.3f   (laser sd %.2f, n %d%s)" % (
            k, x, y, zm, mx, my, hp, hl, hl - hp, sd, n, "" if len(allz) == 1 else ", touches %s" % allz))
    sh = [r for r in rows if not r["spot"].startswith("B")]
    d = np.array([r["diff"] for r in sh])
    pr = np.array([r["probe_h"] for r in sh])
    la = np.array([r["laser_h"] for r in sh])
    print("sheet: probe thickness mean %.3f (range %.3f); laser mean %.3f (range %.3f); laser - probe mean %+.3f, sd %.3f" % (
        pr.mean(), np.ptp(pr), la.mean(), np.ptp(la), d.mean(), d.std()))
    print("after removing the mean offset, laser vs probe per spot: %s mm" % np.round(d - d.mean(), 3))
    json.dump(rows, open(a.probes.replace(".jsonl", "-compare.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
