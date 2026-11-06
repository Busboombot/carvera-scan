"""Height calibration for the line laser: lower the head over bare bed and
record the line at each Z (the bed rising toward a fixed rig looks the same
as a surface that height).

    python -m carvera_scan.acquire.z_sweep OUTDIR --z -1 -80 4 --zmin -80 --exposure 15
    python -m carvera_scan.acquire.z_sweep OUTDIR --z -2 -80 3 --zmin -80 --cam 2.2 2.3   # both cameras, one at a time

X and Y never change. Z steps from the first value toward the second, then
the head goes back to the starting Z. Work light off during the sweep (M822),
back on at the end. Frames OUTDIR/zNNN.png (NNN = 10 x depth below the start),
OUTDIR/log.jsonl with commanded and reported positions; with --cam, the same
in OUTDIR/<port>/ for each camera.
"""
import argparse
import json
import os

import cv2

from carvera_scan.arducam import Arducam
from carvera_scan.machine import Carvera, axis, light_on


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--z", type=float, nargs=3, required=True, help="start stop step")
    ap.add_argument("--zmin", type=float, required=True, help="never go below this Z")
    ap.add_argument("--exposure", type=int, default=15)
    ap.add_argument("--feed", type=int, default=600)
    ap.add_argument("--cam", nargs="+", default=[None], help="USB ports of the cameras")
    a = ap.parse_args()
    zs = axis(*a.z)
    if min(zs) < a.zmin:
        raise SystemExit("Z %g below --zmin %g" % (min(zs), a.zmin))
    os.makedirs(a.outdir, exist_ok=True)
    m = Carvera(log=lambda s: None)
    st = m.status()
    if st["state"] != "Idle":
        raise SystemExit("machine not Idle: " + st["raw"])
    x, y, z0 = st["MPos"][:3]
    print("X %.2f Y %.2f, Z %s, back to %.2f" % (x, y, zs, z0), flush=True)
    dirs = {c: os.path.join(a.outdir, c) if c else a.outdir for c in a.cam}
    logs = {}
    try:
        if not m.set_light(False):
            raise SystemExit("could not turn the work light off")
        for c, d in dirs.items():
            os.makedirs(d, exist_ok=True)
            logs[c] = open(os.path.join(d, "log.jsonl"), "w")
        for z in zs:
            st = m.move(None, None, z, feed=a.feed, zmin=a.zmin)
            name = "z%03d.png" % round(10 * (zs[0] - z))
            # one camera at a time: two can't stream together on one USB 2 hub
            for c, d in dirs.items():
                cam = Arducam(port=c)
                cam.exposure(a.exposure)
                img = cam.grab(settle=0.5)
                cam.exposure(None)
                cam.close()
                cv2.imwrite(os.path.join(d, name), img)
                logs[c].write(json.dumps({"file": name, "cmd": [x, y, z], "mpos": st["MPos"][:3]}) + "\n")
            print("Z %7.2f  %s" % (st["MPos"][2], name), flush=True)
    finally:
        for f in logs.values():
            f.close()
        m.move(None, None, z0, feed=a.feed)
        print("back at", m.status()["MPos"][:3], flush=True)
        m.close()
        light_on("127.0.0.1")


if __name__ == "__main__":
    main()
