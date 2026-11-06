"""Top-down camera images of one region at given heights, from both cameras'
line_scan.py sweeps, in camera A's frame (fit_shapes.Mosaic per camera).

    python projects/cutebot-pro/robot_ortho.py --out data/2026-11-04/robot1 \
        --a data/2026-11-04/robot1_2.2 --a-zcal data/2026-11-04/zcal2-2.2.json --a-axis data/2026-11-04/axis3/2.2 \
        --b data/2026-11-04/robot1_2.3 --b-zcal data/2026-11-04/zcal2-2.3.json --b-axis data/2026-11-04/axis3/2.3 \
        --offset 4.79 -118.63 --view lower -140 -230 -10 -160 20.1 0.1 --view upper -105 -160 -48 -88 36.1 0.1

--offset is camera B's frame relative to A (B + offset = A), e.g. from bed holes.
Each --view is NAME X0 Y0 X1 Y1 HEIGHT RES (mm). Writes OUT/ortho_NAME_{A,B}.npy
(row 0 = Y0, so north is down; flip for display) and a PNG with north up.
"""
import argparse
import os

import cv2
import numpy as np

from carvera_scan.measure.shapes import Mosaic


def to_png(img, path, x0, y0, x1, y1, scale=2):
    v = img[np.isfinite(img)]
    lo, hi = np.percentile(v, [1, 99.7])
    g = np.clip((np.nan_to_num(img, nan=lo) - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)[::-1]
    g = cv2.cvtColor(cv2.resize(g, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC), cv2.COLOR_GRAY2BGR)
    sc = g.shape[1] / (x1 - x0)
    for yy in range(int(np.ceil(y0 / 5) * 5), int(y1) + 1, 5):
        r = int((y1 - yy) * sc)
        cv2.line(g, (0, r), (8 if yy % 10 else 16, r), (0, 255, 255), 1)
        if yy % 10 == 0:
            cv2.putText(g, str(yy), (18, r + 4), 0, .4, (0, 255, 255), 1)
    for xx in range(int(np.ceil(x0 / 5) * 5), int(x1) + 1, 5):
        c = int((xx - x0) * sc)
        cv2.line(g, (c, 0), (c, 8 if xx % 10 else 16), (0, 255, 255), 1)
        if xx % 10 == 0:
            cv2.putText(g, str(xx), (c - 12, 28), 0, .4, (0, 255, 255), 1)
    cv2.imwrite(path, g)


def main():
    ap = argparse.ArgumentParser()
    for k in "ab":
        ap.add_argument("--" + k, required=True)
        ap.add_argument("--%s-zcal" % k, required=True)
        ap.add_argument("--%s-axis" % k, required=True)
    ap.add_argument("--offset", type=float, nargs=2, required=True)
    ap.add_argument("--latency", type=float, default=0.004)
    ap.add_argument("--step", type=int, default=2, help="use every Nth frame")
    ap.add_argument("--view", nargs=7, action="append", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    cams = {"A": (Mosaic(a.a, a.a_zcal, a.a_axis, a.latency, a.step), np.zeros(2)),
            "B": (Mosaic(a.b, a.b_zcal, a.b_axis, a.latency, a.step), np.array(a.offset))}
    for v in a.view:
        name, (x0, y0, x1, y1, h, res) = v[0], map(float, v[1:])
        for k, (m, o) in cams.items():
            img, _ = m.image(x0 - o[0], y0 - o[1], x1 - o[0], y1 - o[1], h, res)
            np.save(os.path.join(a.out, "ortho_%s_%s.npy" % (name, k)), img)
            to_png(img, os.path.join(a.out, "ortho_%s_%s.png" % (name, k)), x0, y0, x1, y1, scale=max(1, round(0.2 / res)))
            print(name, k, img.shape, flush=True)


if __name__ == "__main__":
    main()
