"""Line-laser sweep: the Arducam and a line laser ride on the head. The head
moves in Y without stopping while the camera records at full rate.

    python -m carvera_scan.acquire.line_scan OUTDIR --x -110 --y -230 -40 --pitch 0.25
    python -m carvera_scan.acquire.line_scan OUTDIR --x -110 -110 --y -230 -40     # there and back: timing check
    python -m carvera_scan.acquire.line_scan OUTDIR --x -40 -150 -260 --y -230 -40  # three passes, snaking
    python -m carvera_scan.acquire.line_scan OUTDIR --x -110 --y -150 -150 --test 10 20 40 80
    python -m carvera_scan.acquire.line_scan OUTDIR --cam 2.3 --x ...                # pick the camera by USB port
    python -m carvera_scan.acquire.line_scan OUTDIR --along x --y -20 -60 --x -40 -220  # passes along X, one per Y

Each X in --x is one pass: a single G1 from one end of --y to the other (plus
--lead mm each side to get up to speed), alternating direction. The feed is
--pitch mm per frame at the camera's frame rate. During a pass a thread polls
status as fast as the machine answers; the analysis fits the head position
against time and places each frame by its timestamp.

The laser has its own switch (it is not under machine control); it must be on
before the run. The work light is off during the sweep (M822) and back on
(M821) at the end, unless --keep-light. Z never changes.

Output: OUTDIR/frames.mjpg (the camera's JPEGs back to back, playable with
`ffplay -f mjpeg`) and OUTDIR/scan.json (settings; per frame: timestamp, byte
offset and length, pass; per pass: X, Y range, status samples
[t_sent, t_received, x, y, z]). --test grabs one frame per exposure at the
first stop and exits.
"""
import argparse
import json
import os
import threading
import time

import cv2

from carvera_scan.arducam import Arducam
from carvera_scan.machine import LIMITS, Carvera, light_on

FPS = 120
EDGE = 2.0  # mm kept from each end of travel


class Recorder(threading.Thread):
    """Reads frames continuously; stores the ones that arrive while `pass_` is set."""

    def __init__(self, cam, path):
        super().__init__(daemon=True)
        self.cam, self.f = cam, open(path, "wb")
        self.pass_, self.running, self.error = None, True, None
        self.frames = {"t": [], "off": [], "len": [], "pass": []}

    def run(self):
        try:
            while self.running:
                jpeg, t = self.cam.read_raw()
                p = self.pass_
                if p is None:
                    continue
                fr = self.frames
                fr["t"].append(round(t, 6))
                fr["off"].append(self.f.tell())
                fr["len"].append(len(jpeg))
                fr["pass"].append(p)
                self.f.write(jpeg)
        except Exception as e:  # reported by the main thread
            self.error = e

    def stop(self):
        self.running = False
        self.join(timeout=2)
        self.f.close()


def poll(m, out, stop):
    """Status as fast as it comes back: [t_sent, t_received, x, y, z, state]."""
    while not stop.is_set():
        t0 = time.monotonic()
        try:
            st = m.status(timeout=0.5)
        except TimeoutError:
            continue
        out.append([round(t0, 6), round(time.monotonic(), 6)] + st["MPos"][:3] + [st["state"]])
        time.sleep(0.003)


def sweep(m, rec, k, x, y_from, y_to, feed, axis="y"):
    """One pass: a single G1 along `axis` (Y by default; then x is the fixed X and
    y_from/y_to the travel; for axis "x", x is the fixed Y and y_from/y_to are X),
    recorded from send until Idle at the end."""
    col = 3 if axis == "y" else 2
    samples, stop = [], threading.Event()
    th = threading.Thread(target=poll, args=(m, samples, stop), daemon=True)
    th.start()
    line = "G90 G53 G1 %s%.3f F%d" % (axis.upper(), y_to, feed)
    expect = abs(y_to - y_from) / feed * 60
    rec.pass_ = k
    t_send = time.monotonic()
    tries = 1
    try:
        m.send(line, wait=0)
        while True:
            time.sleep(0.05)
            if rec.error:
                raise RuntimeError("camera: %s" % rec.error)
            if not samples:
                continue
            s = samples[-1]
            if s[5].startswith("Alarm"):
                raise RuntimeError("machine alarm during pass %d" % k)
            if s[5] == "Idle" and abs(s[col] - y_to) <= 0.01:
                break
            # A move is sometimes lost over Wi-Fi; it is absolute, so resend.
            if s[5] == "Idle" and abs(s[col] - y_from) <= 0.01 and time.monotonic() - t_send > 2:
                if tries == 3:
                    raise RuntimeError("pass %d: move lost 3 times" % k)
                print("  resending lost move", flush=True)
                m.send(line, wait=0)
                t_send, tries = time.monotonic(), tries + 1
            if time.monotonic() - t_send > expect + 30:
                raise TimeoutError("pass %d did not finish" % k)
    except KeyboardInterrupt:
        m.realtime("!")
        print("FEED HOLD sent", flush=True)
        raise
    finally:
        rec.pass_ = None
        stop.set()
        th.join()
    rec_ = {"axis": axis, "from": y_from, "to": y_to, "feed": feed, "t_send": round(t_send, 6),
            "status": [s[:5] for s in samples]}
    rec_["x" if axis == "y" else "y"] = x
    return rec_


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--x", type=float, nargs="+", required=True, help="one pass per X (--along x: start stop)")
    ap.add_argument("--y", type=float, nargs="+", required=True, help="start stop (--along x: one pass per Y)")
    ap.add_argument("--along", choices=["y", "x"], default="y", help="direction of travel during a pass")
    ap.add_argument("--pitch", type=float, default=0.25, help="mm of travel per frame")
    ap.add_argument("--lead", type=float, default=5.0, help="mm run-up at each end")
    ap.add_argument("--exposure", type=int, default=15, help="100 us units")
    ap.add_argument("--test", type=int, nargs="+", help="exposures to try at the first stop")
    ap.add_argument("--feed", type=int, default=1500, help="feed for moves between passes")
    ap.add_argument("--max-feed", type=int, default=6000)
    ap.add_argument("--keep-light", action="store_true")
    ap.add_argument("--cam", help="USB port of the camera (e.g. 2.3) when two are connected")
    a = ap.parse_args()

    feed = min(int(round(a.pitch * FPS * 60)), a.max_feed)
    # travel axis ("ax"): its start/stop; fixed axis ("fx"): one pass per value
    ax, fx = a.along, "x" if a.along == "y" else "y"
    travel, fixed = (a.y, a.x) if ax == "y" else (a.x, a.y)
    if len(travel) != 2:
        raise SystemExit("--%s takes start stop" % ax)
    lo, hi = sorted(travel)
    # Stay clear of the travel ends: Y 0 is the home switch, and a G1 that
    # reaches it trips a hard limit (HALT 21, seen 2026-11-04).
    ylo, yhi = max(lo - a.lead, LIMITS[ax][0] + EDGE), min(hi + a.lead, LIMITS[ax][1] - EDGE)
    for x in fixed:
        if not LIMITS[fx][0] + EDGE <= x <= LIMITS[fx][1] - EDGE:
            raise SystemExit("%s %g outside %s" % (fx.upper(), x, LIMITS[fx]))
    ends = [(ylo, yhi) if (k % 2 == 0) == (travel[0] <= travel[1]) else (yhi, ylo) for k in range(len(fixed))]

    os.makedirs(a.outdir, exist_ok=True)
    m = Carvera(log=lambda s: None)
    st = m.status()
    if st["state"] != "Idle":
        raise SystemExit("machine not Idle: " + st["raw"])
    z = st["MPos"][2]
    span = yhi - ylo
    print("%d passes along %s at Z %.1f, %s %.1f .. %.1f, F%d (%.2f mm/frame), ~%.0f s each"
          % (len(fixed), ax.upper(), z, ax.upper(), ylo, yhi, feed, feed / 60 / FPS, span / feed * 60), flush=True)
    cam = Arducam(raw=True, port=a.cam)
    rec, passes, t0 = None, [], time.time()
    try:
        if not a.keep_light and not m.set_light(False):
            raise SystemExit("could not turn the work light off")
        def at(f, t):  # head X, Y for fixed value f and travel position t
            return (f, t) if ax == "y" else (t, f)
        m.move(*at(fixed[0], ends[0][0]), None, feed=a.feed)
        if a.test:
            for e in a.test:
                cam.exposure(e)
                cv2.imwrite(os.path.join(a.outdir, "exp%03d.png" % e), cam.grab(settle=0.5))
            return
        cam.exposure(a.exposure)
        cam.grab(settle=0.5)
        rec = Recorder(cam, os.path.join(a.outdir, "frames.mjpg"))
        rec.start()
        for k, (x, (y_from, y_to)) in enumerate(zip(fixed, ends)):
            m.move(*at(x, y_from), None, feed=a.feed)
            n0 = len(rec.frames["t"])
            passes.append(sweep(m, rec, k, x, y_from, y_to, feed, axis=ax))
            ts = rec.frames["t"][n0:]
            gaps = [b - c for b, c in zip(ts[1:], ts)]
            print("pass %d %s %.1f: %d frames in %.2f s (max gap %.0f ms), %d status samples"
                  % (k, fx.upper(), x, len(ts), ts[-1] - ts[0], 1e3 * max(gaps), len(passes[-1]["status"])), flush=True)
    finally:
        if rec:
            rec.stop()
            json.dump({"cam": a.cam, "z": z, "pitch": a.pitch, "fps": FPS, "feed": feed, "exposure": a.exposure,
                       "lead": a.lead, "y": a.y, "along": ax, "frames": rec.frames, "passes": passes},
                      open(os.path.join(a.outdir, "scan.json"), "w"))
        try:
            cam.exposure(None)
        finally:
            cam.close()
            m.close()
            if not a.keep_light:
                light_on("127.0.0.1")
        print("done (%.0fs)" % (time.time() - t0), flush=True)


if __name__ == "__main__":
    main()
