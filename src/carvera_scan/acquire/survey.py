"""Photograph the bed from a grid of head positions, Z unchanged, with each
camera in turn (two can't stream together on one USB 2 link).

    python -m carvera_scan.acquire.survey OUTDIR --x -30 -130 -230 -330 --y -50 -150 -245 --cam 2.2 2.3

Frames go to OUTDIR/<cam>/<i>.png; OUTDIR/stops.json lists the head
positions. The work light is on during the survey.
"""
import argparse
import json
import os

import cv2

from carvera_scan.arducam import Arducam
from carvera_scan.machine import Carvera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--x", type=float, nargs="+", required=True)
    ap.add_argument("--y", type=float, nargs="+", required=True)
    ap.add_argument("--cam", nargs="+", default=["2.2", "2.3"])
    ap.add_argument("--exposure", type=int, default=None, help="100 us units; default auto")
    ap.add_argument("--feed", type=int, default=3000)
    ap.add_argument("--light", type=int, default=1, help="1 = work light on, 0 = off")
    a = ap.parse_args()
    for c in a.cam:
        os.makedirs(os.path.join(a.outdir, c), exist_ok=True)
    m = Carvera(log=lambda s: None)
    st = m.status()
    if st["state"] != "Idle":
        raise SystemExit("machine not Idle: " + st["raw"])
    m.set_light(bool(a.light))
    stops = []
    i = 0
    for j, x in enumerate(a.x):
        ys = a.y if j % 2 == 0 else a.y[::-1]
        for y in ys:
            st = m.move(x, y, None, feed=a.feed)
            for c in a.cam:
                cam = Arducam(port=c)
                cam.exposure(a.exposure)
                cv2.imwrite(os.path.join(a.outdir, c, "%d.png" % i), cam.grab(settle=1.0))
                cam.close()
            stops.append({"i": i, "mpos": st["MPos"][:3]})
            print(stops[-1], flush=True)
            i += 1
    json.dump({"stops": stops, "cams": a.cam}, open(os.path.join(a.outdir, "stops.json"), "w"), indent=1)
    m.close()


if __name__ == "__main__":
    main()
