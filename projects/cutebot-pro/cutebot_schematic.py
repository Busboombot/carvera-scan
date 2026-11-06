"""Schematic of the Cutebot Pro scan (top, side, front) for designing a shell,
from robot_features.py's features.json and robot_laser.py's merged grid.

    python projects/cutebot-pro/cutebot_schematic.py data/2026-11-04/robot1 --out projects/cutebot-pro/out/cutebot-pro

Writes OUT-schematic.png and OUT-outline.json (outlines in the robot frame).
Labels: B1-B4 lower-deck bosses; L1-L7 / R1-R7 the rows beside the battery,
rear to front.
"""
import argparse
import json
import os

import cv2
import matplotlib
import numpy as np
from scipy import ndimage

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

BOSS_LABEL = {"side_left": "B1", "front_left": "B2", "front_right": "B3", "side_right": "B4"}


def frame(f):
    th = np.radians(f["rotation_deg"])
    left = np.array([np.cos(th), np.sin(th)])
    return np.array(f["machine_origin"]), -left, np.array([left[1], -left[0]])


def outline(mask, x0, y0, cell, O, right, fwd, eps=0.8):
    """Largest region's outer contour, simplified, in the robot frame."""
    lab, n = ndimage.label(mask)
    if n == 0:
        return []
    m = (lab == np.argmax(np.bincount(lab.ravel())[1:]) + 1).astype(np.uint8)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = cv2.approxPolyDP(max(cs, key=cv2.contourArea), eps / cell, True)[:, 0].astype(float)
    P = np.c_[x0 + (c[:, 0] + .5) * cell, y0 + (c[:, 1] + .5) * cell] - O
    return np.round(np.c_[P @ right, P @ fwd], 1).tolist()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    f = json.load(open(os.path.join(a.dir, "features.json")))
    O, right, fwd = frame(f)
    g = np.load(os.path.join(a.dir, "merged-grid.npz"))
    h, (x0, y0), cell = g["height"], g["origin"], float(g["cell"])
    hz = np.nan_to_num(h, nan=0)

    body = ndimage.binary_closing(ndimage.binary_opening(hz > 3, iterations=2), iterations=6)
    lo, hi = f["lower_deck_z"], f["upper_deck_z"]
    deck = ndimage.binary_closing(ndimage.binary_opening(np.abs(hz - lo) < 3, iterations=2), iterations=4)
    out = {"frame": f["frame"],
           "footprint": outline(body, x0, y0, cell, O, right, fwd),
           "lower_deck": outline(deck, x0, y0, cell, O, right, fwd),
           "note": "Outer outlines from the laser height map (0.5 mm cells, closed over gaps). "
                   "footprint = everything above 3 mm (wheels included); lower_deck = cells within 3 mm "
                   "of the lower deck height. Edges good to ~1-2 mm."}
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    json.dump(out, open(a.out + "-outline.json", "w"), indent=1)

    fp, dk = np.array(out["footprint"]), np.array(out["lower_deck"])
    b, mb = f["battery_box"], f["microbit"]
    wl, wr = f["wheel_left"], f["wheel_right"]
    rows = f["upper_rows"]
    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1], height_ratios=[1, 1])
    colors = {"boss": "#d32f2f", "hole": "#1e88e5", "screw": "#757575"}

    # top view: +x right, +y forward (up the page)
    ax = fig.add_subplot(gs[:, 0])
    ax.fill(*np.r_[fp, fp[:1]].T, color="#eeeeee", ec="#999", lw=1, label="footprint (all > 3 mm)")
    ax.plot(*np.r_[dk, dk[:1]].T, color="#26a69a", lw=1.2, label="lower deck %.1f" % lo)
    for w, nm in ((wl, "left"), (wr, "right")):
        ax.add_patch(Rectangle((w["x"][0], w["axle_y"] - w["dia"] / 2), w["x"][1] - w["x"][0], w["dia"],
                               fc="#bdbdbd", ec="#424242", alpha=.6))
        ax.text(np.mean(w["x"]), w["axle_y"], "wheel\n%.0f dia\ntop %.1f" % (w["dia"], w["top_z"]), ha="center",
                va="center", fontsize=8)
    ax.add_patch(Rectangle((b["x"][0], b["y"][0]), b["x"][1] - b["x"][0], b["y"][1] - b["y"][0],
                           fc="#ffe082", ec="#f9a825", alpha=.7))
    ax.text(np.mean(b["x"]), b["y"][0] + 3, "battery\ntop %.1f" % b["top_z"], ha="center", fontsize=8)
    ax.add_patch(Rectangle((mb["x_centre"] - mb["width"] / 2, mb["y_board"] - .8), mb["width"], 1.6,
                           fc="#66bb6a", ec="#2e7d32"))
    ax.text(mb["x_centre"], mb["y_board"] + 2, "micro:bit (vertical) top %.0f" % mb["top_z"], ha="center", fontsize=8)
    for k, v in f["lower_bosses"].items():
        x, y = v["xy"]
        ax.add_patch(Circle((x, y), v["dia"] / 2, fc=colors["boss"], ec="k", lw=.5))
        ax.annotate("%s (%g, %g)" % (BOSS_LABEL[k], x, y), (x, y), xytext=(0, -12 if y > 75 else 9),
                    textcoords="offset points", ha="center", fontsize=8.5, weight="bold")
    for side, tag, dx in (("left", "L", -1), ("right", "R", 1)):
        for i, r in enumerate(rows[side]):
            x, y = r["xy"]
            ax.add_patch(Circle((x, y), 2.5, fc=colors[r["kind"]], ec="k", lw=.5))
            ax.annotate("%s%d" % (tag, i + 1), (x, y), xytext=(dx * 9, -3), textcoords="offset points",
                        ha="center", fontsize=8)
    ax.plot(0, 0, "k+", ms=14)
    ax.annotate("", (12, 0), (0, 0), arrowprops=dict(arrowstyle="->"))
    ax.annotate("", (0, 12), (0, 0), arrowprops=dict(arrowstyle="->"))
    ax.text(13, -1, "+x", fontsize=9)
    ax.text(-1, 13, "+y fwd", fontsize=9, ha="right")
    for c, lab in ((colors["boss"], "boss (threaded insert)"), (colors["hole"], "leg-connector hole"),
                   (colors["screw"], "screw head")):
        ax.plot([], [], "o", color=c, label=lab)
    ax.set_aspect("equal")
    ax.set_xticks(np.arange(-56, 57, 8))
    ax.set_yticks(np.arange(-40, 97, 8))
    ax.tick_params(labelsize=8)
    ax.grid(alpha=.35)
    ax.set_xlabel("x (mm, + = robot's right); origin on the grid, ~0.4 mm from the axle")
    ax.set_ylabel("y (mm, + = forward)")
    ax.set_title("Top view: gridlines every 8 mm (LEGO); every boss and hole is on a grid point")
    ax.legend(loc="lower left", fontsize=8)

    # side view from the robot's right: y horizontal (forward to the right), z up
    ax = fig.add_subplot(gs[0, 1])
    ylo, yhi = fp[:, 1].min(), fp[:, 1].max()
    ax.add_patch(Circle((wr["axle_y"], wr["axle_z"]), wr["dia"] / 2, fc="#bdbdbd", ec="#424242", alpha=.6))
    ax.add_patch(Rectangle((b["y"][0], hi), b["y"][1] - b["y"][0], b["top_z"] - hi, fc="#ffe082", ec="#f9a825"))
    ax.add_patch(Rectangle((mb["y_board"] - .8, mb["top_z"] - 42), 1.6, 42, fc="#66bb6a", ec="#2e7d32"))
    ax.plot([dk[:, 1].min(), dk[:, 1].max()], [lo, lo], color="#26a69a", lw=3)
    ry = [r["xy"][1] for r in rows["right"]]
    ax.plot([min(ry) - 5, max(ry) + 5], [hi, hi], color="#7e57c2", lw=3)
    for k, v in f["lower_bosses"].items():
        ax.plot(v["xy"][1], lo, "v", color=colors["boss"], ms=8)
    for r in rows["right"]:
        ax.plot(r["xy"][1], hi, "v", color=colors[r["kind"]], ms=6)
    for z, lab in ((lo, "lower deck %.1f" % lo), (hi, "upper deck %.1f" % hi), (wr["top_z"], "wheel top %.1f" % wr["top_z"]),
                   (b["top_z"], "battery top %.1f" % b["top_z"]), (mb["top_z"], "micro:bit top %.0f" % mb["top_z"])):
        ax.axhline(z, color="#999", lw=.6, ls="--")
        ax.text(ylo - 2, z + .8, lab, fontsize=8, ha="left")
    ax.axhline(0, color="k", lw=1)
    ax.set_xlim(ylo - 5, yhi + 5)
    ax.set_ylim(-3, 75)
    ax.set_aspect("equal")
    ax.grid(alpha=.3)
    ax.set_xlabel("y (mm, forward →)")
    ax.set_ylabel("z (mm above the bed)")
    ax.set_title("Side view from the robot's right")

    # front view: looking backward at the front, so the robot's right (+x) is on the left
    ax = fig.add_subplot(gs[1, 1])
    for w in (wl, wr):
        ax.add_patch(Rectangle((w["x"][0], 0), w["x"][1] - w["x"][0], w["top_z"], fc="#bdbdbd", ec="#424242", alpha=.6))
    ax.add_patch(Rectangle((b["x"][0], hi), b["x"][1] - b["x"][0], b["top_z"] - hi, fc="#ffe082", ec="#f9a825"))
    ax.add_patch(Rectangle((mb["x_centre"] - mb["width"] / 2, mb["top_z"] - 42), mb["width"], 42,
                           fc="#66bb6a", ec="#2e7d32", alpha=.5))
    ax.plot([dk[:, 0].min(), dk[:, 0].max()], [lo, lo], color="#26a69a", lw=3)
    rx = [rows["left"][0]["xy"][0], rows["right"][0]["xy"][0]]
    ax.plot([min(rx) - 4, max(rx) + 4], [hi, hi], color="#7e57c2", lw=3)
    for k, v in f["lower_bosses"].items():
        ax.plot(v["xy"][0], lo, "v", color=colors["boss"], ms=8)
        ax.text(v["xy"][0], lo - 4, BOSS_LABEL[k], ha="center", fontsize=8)
    for x in rx:
        ax.plot(x, hi, "v", color=colors["boss"], ms=6)
    ax.axhline(0, color="k", lw=1)
    ax.invert_xaxis()
    ax.set_ylim(-6, 75)
    ax.set_aspect("equal")
    ax.grid(alpha=.3)
    ax.set_xlabel("x (mm; robot's right on the left, as seen from the front)")
    ax.set_ylabel("z (mm above the bed)")
    ax.set_title("Front view")
    ax.text(0, 72, "micro:bit (behind the front bosses)", ha="center", fontsize=8)

    fig.suptitle("Elecfreaks Cutebot Pro, as scanned 2026-11-04 (mm)", fontsize=14)
    fig.tight_layout()
    fig.savefig(a.out + "-schematic.png", dpi=110)
    print("wrote", a.out + "-schematic.png", a.out + "-outline.json",
          "footprint x %.1f..%.1f y %.1f..%.1f" % (fp[:, 0].min(), fp[:, 0].max(), fp[:, 1].min(), fp[:, 1].max()),
          "lower deck x %.1f..%.1f y %.1f..%.1f" % (dk[:, 0].min(), dk[:, 0].max(), dk[:, 1].min(), dk[:, 1].max()))


if __name__ == "__main__":
    main()
