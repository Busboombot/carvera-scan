"""Calibration sets: everything rig-specific that processing needs, in one JSON file.

    python -m carvera_scan.calib build --name 2026-11-05-cross45 --zcal data/2026-11-05/zcal45deep \
        --axis data/2026-11-05/axis45 --cameras A=1.2 B=1.1 \
        --centre A=622.1,437.8 B=649.6,430.1 --height A=161.3 B=162.6 \
        --b-to-a -3.5 53.5 --map-to-machine -70.7 -67.1 --out config/calib/2026-11-05-cross45.json

A calibration set holds, per camera role (A = reference frame, B = second camera):
USB port, principal point ("centre", px) and optical-centre height above the bed at
the Z sweep's first Z ("H", mm), the image<->machine axis matrix (px per mm of X, Y),
and the per-arm laser model from the Z sweep (cross.py zcal). Plus: the approximate
B -> A map shift (refined per scan by registration), the map(A) -> machine offset
(camera to spindle), and where each piece came from. Processing reads only this file,
so a new rig or a new calibration is a new file, not a code change.
"""
import argparse
import json
import os
import time


def load(path):
    c = json.load(open(path))
    c["_path"] = path
    return c


def camera(cal, role):
    return cal["cameras"][role]


def role_of_port(cal, port):
    for r, cam in cal["cameras"].items():
        if cam["port"] == port:
            return r
    raise KeyError("no camera on USB port %s in %s" % (port, cal.get("name")))


def build(a):
    roles = dict(kv.split("=") for kv in a.cameras)
    centres = {k: [float(x) for x in v.split(",")] for k, v in (kv.split("=") for kv in a.centre)}
    heights = {k: float(v) for k, v in (kv.split("=") for kv in a.height)}
    cams = {}
    for role, port in roles.items():
        z = json.load(open(os.path.join(a.zcal, port + ".json")))
        M = json.load(open(os.path.join(a.axis, port, "matrix.json")))
        cams[role] = {"port": port, "centre": centres[role], "H": heights[role], "axis_matrix": M["matrix"],
                      "laser": z, "sources": {"zcal": os.path.join(a.zcal, port + ".json"),
                                              "axis": os.path.join(a.axis, port, "matrix.json"),
                                              "centre_H": a.centre_source}}
    out = {"name": a.name, "created": time.strftime("%Y-%m-%d %H:%M"), "laser_type": a.laser,
           "z0": next(iter(cams.values()))["laser"]["z0"], "cameras": cams,
           "b_to_a_shift": a.b_to_a, "map_to_machine": a.map_to_machine,
           "notes": a.notes}
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=1)
    print("wrote", a.out)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--name", required=True)
    b.add_argument("--zcal", required=True, help="dir with <port>.json from cross.py zcal")
    b.add_argument("--axis", required=True, help="dir with <port>/matrix.json")
    b.add_argument("--cameras", nargs="+", required=True, help="ROLE=PORT, A is the reference frame")
    b.add_argument("--centre", nargs="+", required=True, help="ROLE=u,v principal point (px)")
    b.add_argument("--height", nargs="+", required=True, help="ROLE=H camera height above the bed at the Z sweep's first Z (mm)")
    b.add_argument("--centre-source", default="zcal5 hole-zoom fit (straight-line rig, 2026-11-05)")
    b.add_argument("--b-to-a", type=float, nargs=2, required=True, help="approximate B -> A map shift (mm)")
    b.add_argument("--map-to-machine", type=float, nargs=2, required=True, help="map(A) -> machine X, Y offset (mm)")
    b.add_argument("--laser", default="cross")
    b.add_argument("--notes", default="")
    b.add_argument("--out", required=True)
    a = ap.parse_args()
    if a.cmd == "build":
        build(a)


if __name__ == "__main__":
    main()
