"""Move the head in X/Y (Z unchanged) and save one Arducam frame.

    python -m carvera_scan.acquire.look X Y OUT.png [--settle S]
"""
import argparse

import cv2

from carvera_scan.arducam import Arducam
from carvera_scan.machine import Carvera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("out")
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--feed", type=int, default=1500)
    a = ap.parse_args()
    m = Carvera(log=lambda s: None)
    if m.status()["state"] != "Idle":
        raise SystemExit("machine not Idle")
    st = m.move(a.x, a.y, None, feed=a.feed)
    cam = Arducam()
    cv2.imwrite(a.out, cam.grab(settle=a.settle))
    cam.close()
    m.close()
    print(st["raw"])


if __name__ == "__main__":
    main()
