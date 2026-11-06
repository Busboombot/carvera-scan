"""Synthetic checks of the feature geometry: circle fit, two-chord centre, grid fit, hole finder."""
import numpy as np

from carvera_scan.measure import features, lattice
from carvera_scan.process.spot import centre_from_two_chords, fit_circle


def test_fit_circle_exact_and_noisy():
    t = np.linspace(0, 2 * np.pi, 40, endpoint=False)
    pts = np.c_[3 + 2.4 * np.cos(t), -5 + 2.4 * np.sin(t)]
    cx, cy, r, rms = fit_circle(pts)
    assert abs(cx - 3) < 1e-6 and abs(cy + 5) < 1e-6 and abs(r - 2.4) < 1e-6 and rms < 1e-6
    rng = np.random.default_rng(0)
    cx, cy, r, rms = fit_circle(pts + rng.normal(0, 0.05, pts.shape))
    assert np.hypot(cx - 3, cy + 5) < 0.03 and abs(r - 2.4) < 0.03


def test_two_chords_give_the_centre():
    """Two chords at +-45 deg through points off the centre: perpendicular bisectors meet at it."""
    c, r = np.array([10.0, 20.0]), 2.4

    def chord(direction, through):
        d = np.asarray(direction, float) / np.linalg.norm(direction)
        p = np.asarray(through, float) - c
        b = p @ d
        s = np.sqrt(r * r - (p @ p - b * b))
        return [c + p - (b + s) * d, c + p - (b - s) * d]

    a = chord((1, 1), c + (0.5, -0.3))
    b = chord((-1, 1), c + (0.5, -0.3))
    ends = [(0, a[0]), (0, a[1]), (1, b[0]), (1, b[1])]
    assert np.allclose(centre_from_two_chords(ends), c, atol=1e-9)


def test_lattice_fit():
    rng = np.random.default_rng(1)
    idx = np.array([(i, j) for i in (-2, 2) for j in range(-3, 4)] + [(-4, 8), (4, 8), (-1, 10), (1, 10)])
    th = np.radians(-0.7)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    P = np.array([-200.0, -50.0]) + 8 * idx @ R.T + rng.normal(0, 0.1, (len(idx), 2))
    f = lattice.fit(P, 8.0)
    assert 0.05 < f["rms"] < 0.2
    assert abs(f["angle_deg"] + 0.7) < 0.2 or abs(f["angle_deg"] - 89.3) < 0.2
    assert abs(f["free_pitch"] - 8) < 0.02


def test_find_hole_and_boss():
    cell = 0.25
    h = np.full((200, 200), 20.0)
    yy, xx = np.mgrid[0:200, 0:200] * cell
    h[np.hypot(xx - 15, yy - 25) < 2.4] = np.nan          # a 4.8 mm hole, no data inside
    h[np.hypot(xx - 35, yy - 25) < 3.0] = 21.0            # a 6 mm boss, 1 mm proud
    feats, objs = features.find(h, (0.0, 0.0), cell)
    holes = [f for f in feats if f["kind"] == "hole"]
    bosses = [f for f in feats if f["kind"] == "boss"]
    assert len(holes) == 1 and len(bosses) == 1
    assert np.hypot(holes[0]["xy"][0] - 15, holes[0]["xy"][1] - 25) < 0.2
    assert abs(holes[0]["diameter"] - 4.8) < 0.4 and abs(holes[0]["z"] - 20) < 0.1
    assert np.hypot(bosses[0]["xy"][0] - 35, bosses[0]["xy"][1] - 25) < 0.2
