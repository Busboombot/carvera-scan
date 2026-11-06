"""OpenSCAD model of the plate from fitted shapes, plus a browser viewer.

    python projects/plate-model/cad_model.py data/2026-11-04/plate2-shapes.json data/2026-11-04/plate2-grid1.npz \
        --out data/2026-11-04/model

Writes OUT/plate.scad (the whole plate: spoilboard with the holes the scan
found, boxes as cubes at their fitted centre, size, angle and height, other
objects as terraces of their measured contours), OUT/parts/*.scad and *.stl
(each part rendered by OpenSCAD), OUT/points.json (the height map, for an
overlay) and OUT/index.html (three.js viewer of the STLs).
Coordinates: machine X, Y of the head (plus the camera's offset), Z up from
the spoilboard plane.
"""
import argparse
import json
import os
import subprocess

import cv2
import numpy as np
from scipy import ndimage

from carvera_scan.measure.shapes import level

NOMINAL = {25.4: '1"', 50.8: '2"', 76.2: '3"'}
COLORS = ["#e4572e", "#29335c", "#f3a712", "#669bbc", "#a8c686", "#8e5572", "#2a9d8f", "#bc6c25", "#5f0f40"]


def contour_polys(mask, x0, y0, cell):
    """OpenSCAD polygon(points, paths) for a mask, holes included."""
    cs, hier = cv2.findContours(mask.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return []
    out = []
    for i, c in enumerate(cs):
        if hier[0][i][3] != -1 or cv2.contourArea(c) < 4:
            continue
        rings = [c] + [cs[j] for j in range(len(cs)) if hier[0][j][3] == i and cv2.contourArea(cs[j]) >= 4]
        pts, paths = [], []
        for r in rings:
            r = cv2.approxPolyDP(r, 0.7, True)[:, 0, :]
            paths.append(list(range(len(pts), len(pts) + len(r))))
            pts += [[round(float(x0 + (x + 0.5) * cell), 2), round(float(y0 + (y + 0.5) * cell), 2)] for x, y in r]
        out.append((pts, paths))
    return out


def scad_poly(pts, paths):
    return "polygon(points=%s, paths=%s);" % (json.dumps(pts), json.dumps(paths))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("shapes")
    ap.add_argument("grid")
    ap.add_argument("--out", required=True)
    ap.add_argument("--terrace", type=float, default=1.5, help="mm between contour levels")
    ap.add_argument("--plate", type=float, default=6.0, help="spoilboard thickness drawn below Z 0")
    ap.add_argument("--min-height", type=float, default=4.0,
                    help="leave out lower objects (the black PCB ruler and screw heads only show as fragments)")
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, "parts"), exist_ok=True)
    S = json.load(open(a.shapes))
    G = np.load(a.grid)
    cell = float(G["cell"])
    x0, y0 = G["origin"]
    h, _, floor = level(G["height"], cell)
    ny, nx = h.shape
    X1, Y1 = x0 + nx * cell, y0 + ny * cell

    parts = []  # (name, label, scad body, color)
    objmask = np.zeros(h.shape, bool)
    yy, xx = np.mgrid[:ny, :nx]
    cxg, cyg = x0 + (xx + 0.5) * cell, y0 + (yy + 0.5) * cell
    for k, o in enumerate(o for o in S["objects"] if o["height"] >= a.min_height):
        col = COLORS[k % len(COLORS)]
        if o["type"] == "box":
            w, l = o["size"]
            near = min(NOMINAL, key=lambda n: abs(n - o["height"]))
            is_block = abs(near - o["height"]) < 2.5 and sorted([round(w / 25.4), round(l / 25.4)]) in ([1, 2], [1, 3], [2, 3])
            label = ("1-2-3 block, %s up: " % NOMINAL[near] if is_block else "box: ") + \
                "%.2f x %.2f x %.2f mm, %.2f deg" % (w, l, o["height"], o["angle_deg"])
            body = ("translate([%.3f, %.3f, 0]) rotate([0, 0, %.3f]) translate([%.3f, %.3f, 0]) cube([%.3f, %.3f, %.3f]);"
                    % (*o["centre"], o["angle_deg"], -w / 2, -l / 2, w, l, o["height"]))
            # footprint for excluding bed holes
            ang = np.radians(o["angle_deg"])
            rx = (cxg - o["centre"][0]) * np.cos(ang) + (cyg - o["centre"][1]) * np.sin(ang)
            ry = -(cxg - o["centre"][0]) * np.sin(ang) + (cyg - o["centre"][1]) * np.cos(ang)
            objmask |= (np.abs(rx) < w / 2 + 3) & (np.abs(ry) < l / 2 + 3)
        else:
            poly = np.array(o["polygon"])
            region = cv2.fillPoly(np.zeros(h.shape, np.uint8),
                                  [np.round((poly - [x0, y0]) / cell - 0.5).astype(np.int32)], 1).astype(bool)
            hh = np.where(region, np.nan_to_num(h, nan=0), 0)
            hh = ndimage.median_filter(hh, 3)
            layers = []
            lv = 0.0
            while lv < o["height"] - 0.1:
                top = min(lv + a.terrace, o["height"])
                m = region & (hh >= (lv + top) / 2)
                for pts, paths in contour_polys(m, x0, y0, cell):
                    layers.append("translate([0, 0, %.2f]) linear_extrude(%.2f) %s" % (lv, top - lv, scad_poly(pts, paths)))
                lv = top
            if not layers:
                continue
            body = "union() {\n  " + "\n  ".join(layers) + "\n}"
            label = "object: %.0f mm tall, %d mm^2 footprint" % (o["height"], o["footprint_mm2"])
            objmask |= ndimage.binary_dilation(region, iterations=3)
        parts.append(("obj%d" % o["id"], label, body, col))

    # spoilboard holes: small round gaps in the floor away from objects
    gap = ~np.isfinite(h) | (h < -1.5)
    gap &= ~objmask
    lab, n = ndimage.label(gap)
    holes = []
    for i, sl in enumerate(ndimage.find_objects(lab), 1):
        m = lab[sl] == i
        area = m.sum() * cell ** 2
        hy, hx = m.shape
        if 40 < area < 200 and 0.6 < hx / hy < 1.6 and m.sum() / (hx * hy) > 0.6 and \
                sl[0].start > 0 and sl[1].start > 0 and sl[0].stop < ny and sl[1].stop < nx:
            cy_, cx_ = ndimage.center_of_mass(m)
            holes.append((x0 + (sl[1].start + cx_ + 0.5) * cell, y0 + (sl[0].start + cy_ + 0.5) * cell,
                          2 * np.sqrt(area / np.pi)))
    hole_d = float(np.median([d for *_, d in holes])) if holes else 0
    plate = ("difference() {\n  translate([%.2f, %.2f, %.2f]) cube([%.2f, %.2f, %.2f]);\n" % (x0, y0, -a.plate, X1 - x0, Y1 - y0, a.plate)
             + "".join("  translate([%.2f, %.2f, %.2f]) cylinder(d=%.2f, h=%.2f, $fn=32);\n"
                       % (x, y, -a.plate - 1, hole_d, a.plate + 2) for x, y, _ in holes) + "}")
    parts.insert(0, ("plate", "spoilboard (scanned area) with %d holes, %.1f mm" % (len(holes), hole_d), plate, "#c8b99a"))

    with open(os.path.join(a.out, "plate.scad"), "w") as f:
        f.write("// Carvera plate from the line-laser scan; fitted shapes. mm, machine X/Y, Z up from the spoilboard.\n")
        for name, label, body, col in parts:
            f.write("\n// %s: %s\ncolor(\"%s\") %s\n" % (name, label, col, body))
    manifest = []
    for name, label, body, col in parts:
        scad = os.path.join(a.out, "parts", name + ".scad")
        stl = os.path.join(a.out, "parts", name + ".stl")
        open(scad, "w").write(body + "\n")
        r = subprocess.run(["openscad", "-o", stl, scad], capture_output=True, text=True)
        if r.returncode:
            print("openscad failed for", name, r.stderr[-400:])
            continue
        manifest.append({"name": name, "label": label, "color": col, "stl": "parts/%s.stl" % name})
        print("%-6s %s" % (name, label))

    # height map overlay (every cell, rounded)
    ok = np.isfinite(h)
    pts = np.c_[cxg[ok], cyg[ok], h[ok]].round(2)
    json.dump({"parts": manifest, "points": pts.ravel().tolist(), "bounds": [x0, y0, X1, Y1]},
              open(os.path.join(a.out, "model.json"), "w"))
    open(os.path.join(a.out, "index.html"), "w").write(VIEWER)
    print("wrote", a.out)


VIEWER = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Plate model</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
:root{--bg:#f6f5f2;--fg:#1d1d1f;--muted:#666;--panel:rgba(255,255,255,.92);--line:#ddd}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#151618;--fg:#e8e8ea;--muted:#9a9a9f;--panel:rgba(30,31,34,.92);--line:#333}}
:root[data-theme="dark"]{--bg:#151618;--fg:#e8e8ea;--muted:#9a9a9f;--panel:rgba(30,31,34,.92);--line:#333}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
#v{position:fixed;inset:0}
#p{position:fixed;top:12px;left:12px;max-width:min(420px,calc(100vw - 24px));background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px 12px;max-height:calc(100vh - 24px);overflow:auto}
#p h1{font-size:15px;margin:0 0 6px}#p .m{color:var(--muted);font-size:12px;margin-bottom:8px}
#p label{display:flex;gap:8px;align-items:flex-start;margin:4px 0;cursor:pointer}
.sw{width:12px;height:12px;border-radius:3px;flex:none;margin-top:3px}
</style></head><body><div id="v"></div>
<div id="p"><h1>Plate model (fitted)</h1><div class="m">OpenSCAD parts from the line-laser scan. Drag to orbit, scroll to zoom, right-drag to pan.</div>
<div id="list"></div><label><input type="checkbox" id="pts"> measured height map (points)</label></div>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";
import {STLLoader} from "three/addons/loaders/STLLoader.js";
const d = await (await fetch("model.json")).json();
const el = document.getElementById("v");
const r = new THREE.WebGLRenderer({antialias:true, alpha:true}); r.setPixelRatio(devicePixelRatio); el.appendChild(r.domElement);
const scene = new THREE.Scene();
const cam = new THREE.PerspectiveCamera(40, 1, 1, 5000);
const [x0,y0,x1,y1] = d.bounds, cx=(x0+x1)/2, cy=(y0+y1)/2, span=Math.max(x1-x0,y1-y0);
cam.up.set(0,0,1); cam.position.set(cx+span*0.2, cy-span*1.15, span*0.95);
const ctl = new OrbitControls(cam, r.domElement); ctl.target.set(cx,cy,10); ctl.update();
scene.add(new THREE.HemisphereLight(0xffffff, 0x666666, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(cx+200, cy-300, 500); scene.add(sun);
const list = document.getElementById("list"), loader = new STLLoader();
const geoms = await Promise.all(d.parts.map(p => loader.loadAsync(p.stl)));
for (const [i, p] of d.parts.entries()) {
  const g = geoms[i]; g.computeVertexNormals();
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({color:p.color, roughness:.7, metalness:.05, transparent:true, opacity:p.name==="plate"?1:0.92}));
  scene.add(m);
  const row = document.createElement("label");
  row.innerHTML = `<input type="checkbox" checked><span class="sw" style="background:${p.color}"></span><span>${p.label}</span>`;
  row.querySelector("input").onchange = e => { m.visible = e.target.checked; };
  list.appendChild(row);
}
const pg = new THREE.BufferGeometry(); pg.setAttribute("position", new THREE.Float32BufferAttribute(d.points, 3));
const zs = d.points.filter((_,i)=>i%3===2), zmax = Math.max(...zs.slice(0,200000));
const colors = new Float32Array(d.points.length); const c = new THREE.Color();
for (let i=0;i<d.points.length/3;i++){ c.setHSL(0.66-0.66*Math.min(1,Math.max(0,d.points[3*i+2]/80)),0.85,0.5); colors.set([c.r,c.g,c.b],3*i); }
pg.setAttribute("color", new THREE.BufferAttribute(colors,3));
const cloud = new THREE.Points(pg, new THREE.PointsMaterial({size:1.2, vertexColors:true})); cloud.visible=false; scene.add(cloud);
document.getElementById("pts").onchange = e => { cloud.visible = e.target.checked; };
function size(){ const w=el.clientWidth,h=el.clientHeight; r.setSize(w,h); cam.aspect=w/h; cam.updateProjectionMatrix(); }
addEventListener("resize", size); size();
r.setAnimationLoop(()=>{ ctl.update(); r.render(scene,cam); });
</script></body></html>
"""

if __name__ == "__main__":
    main()
