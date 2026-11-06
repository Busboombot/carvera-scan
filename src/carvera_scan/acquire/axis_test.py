"""Move the head a known step in X and in Y and save a frame at each stop.

    python -m carvera_scan.acquire.axis_test OUTDIR --step 20

Frames: 0.png at the start, 1.png after the X step, 2.png after the Y step,
then back to the start. Z never changes. Steps are negative unless that
would cross the soft limit.
"""
import argparse
import json
import os

import cv2

from carvera_scan.arducam import Arducam
from carvera_scan.machine import Carvera, LIMITS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--step", type=float, default=20)
    ap.add_argument("--feed", type=int, default=1500)
    ap.add_argument("--cam", nargs="+", default=[None], help="USB ports; frames go to OUTDIR/<port>/")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    m = Carvera(log=lambda s: None)
    st = m.status()
    if st["state"] != "Idle":
        raise SystemExit("machine not Idle: " + st["raw"])
    x0, y0, z0 = st["MPos"][:3]
    dx = -a.step if x0 - a.step >= LIMITS["x"][0] else a.step
    dy = -a.step if y0 - a.step >= LIMITS["y"][0] else a.step
    stops = [(x0, y0), (x0 + dx, y0), (x0, y0 + dy), (x0, y0)]
    rec = []
    dirs = {c: os.path.join(a.outdir, c) if c else a.outdir for c in a.cam}
    try:
        for i, (x, y) in enumerate(stops):
            st = m.move(x, y, None, feed=a.feed)
            # one camera at a time: two can't stream together on one USB 2 hub
            for c, d in dirs.items():
                cam = Arducam(port=c)
                img = cam.grab(settle=0.6)
                cam.close()
                if i < 3:
                    os.makedirs(d, exist_ok=True)
                    cv2.imwrite(os.path.join(d, "%d.png" % i), img)
            rec.append({"i": i, "cmd": [x, y, z0], "mpos": st["MPos"][:3]})
            print(rec[-1], flush=True)
    finally:
        m.close()
    for d in dirs.values():
        json.dump({"dx": dx, "dy": dy, "stops": rec}, open(os.path.join(d, "moves.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
