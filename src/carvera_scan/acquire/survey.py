"""Photograph the bed from a list of head positions, Z unchanged, with each camera in
turn (two can't stream together on one USB 2 link): each camera is opened once and
visits every position.

    python -m carvera_scan.acquire.survey OUTDIR --x -30 -130 -230 -330 --y -50 -150 -245 --cam 1.2 1.1
    python -m carvera_scan.acquire.survey OUTDIR --at spots-at.json --cam 1.2 1.1 --light 0 --exposure 15

--x/--y: a grid, visited snaking along Y. --at: a JSON list of [x, y] head positions
(e.g. from process.spot plan), visited in that order. Laser frames for process.spot
want the light off and the line-scan exposure (--light 0 --exposure 15).

Frames go to OUTDIR/<cam>/<i>.png; OUTDIR/stops.json lists the head positions (as
reported after the first camera's move). The light is restored on at the end.
"""
import argparse
import json
import os

import cv2

from carvera_scan.arducam import Arducam
from carvera_scan.machine import LIMITS, Carvera, light_on

EDGE = 2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--x", type=float, nargs="+")
    ap.add_argument("--y", type=float, nargs="+")
    ap.add_argument("--at", help="JSON file: list of [x, y] head positions")
    ap.add_argument("--cam", nargs="+", default=["1.2", "1.1"])
    ap.add_argument("--exposure", type=int, default=None, help="100 us units; default auto")
    ap.add_argument("--settle", type=float, default=1.0, help="s of frames dropped after each move")
    ap.add_argument("--feed", type=int, default=3000)
    ap.add_argument("--light", type=int, default=1, help="1 = work light on, 0 = off")
    a = ap.parse_args()
    if a.at:
        pos = [tuple(map(float, p[:2])) for p in json.load(open(a.at))]
    elif a.x and a.y:
        pos = [(x, y) for j, x in enumerate(a.x) for y in (a.y if j % 2 == 0 else a.y[::-1])]
    else:
        raise SystemExit("give --x and --y, or --at")
    for x, y in pos:  # check every position before the first move
        if not (LIMITS["x"][0] + EDGE <= x <= LIMITS["x"][1] - EDGE and LIMITS["y"][0] + EDGE <= y <= LIMITS["y"][1] - EDGE):
            raise SystemExit("position (%g, %g) is within %g mm of the travel limits" % (x, y, EDGE))
    for c in a.cam:
        os.makedirs(os.path.join(a.outdir, c), exist_ok=True)
    m = Carvera(log=lambda s: None)
    st = m.status()
    if st["state"] != "Idle":
        raise SystemExit("machine not Idle: " + st["raw"])
    stops = []
    try:
        if not m.set_light(bool(a.light)):
            raise SystemExit("could not set the work light")
        for n, c in enumerate(a.cam):
            cam = Arducam(port=c)
            cam.exposure(a.exposure)
            order = range(len(pos)) if n % 2 == 0 else range(len(pos) - 1, -1, -1)  # no long move back
            for i in order:
                st = m.move(*pos[i], None, feed=a.feed)
                cv2.imwrite(os.path.join(a.outdir, c, "%d.png" % i), cam.grab(settle=a.settle))
                if n == 0:
                    stops.append({"i": i, "mpos": st["MPos"][:3]})
                print(c, i, st["MPos"][:3], flush=True)
            cam.exposure(None)
            cam.close()
    finally:
        stops.sort(key=lambda s: s["i"])
        json.dump({"stops": stops, "cams": a.cam, "light": a.light, "exposure": a.exposure},
                  open(os.path.join(a.outdir, "stops.json"), "w"), indent=1)
        m.close()
        if not a.light:
            light_on("127.0.0.1")


if __name__ == "__main__":
    main()
