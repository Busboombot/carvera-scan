"""Z touches with the Carvera's wireless 3D probe, using only firmware probe
cycles (no hand-built G38 moves).

    python -m carvera_scan.acquire.probe_z load                      # M6 T0: ATC fetches the probe, measures it on the TLO sensor, checks it triggers
    python -m carvera_scan.acquire.probe_z touch OUT.jsonl --spot NAME X Y DIST [--spot ...] [--top Z]
    python -m carvera_scan.acquire.probe_z unload                    # M6 T-1: probe back to the rack

touch: for each spot (machine X, Y of the spindle): go to --top Z (default
-1.3), then X Y, then `M466 Z-DIST` (Community firmware single-axis
double-tap probe: G38.2 down at F300 until contact or DIST, back 1.5 mm, slow
second touch, back 1.5 mm; S0 = work offsets untouched). The reply's
"Final Positon Z" (machine coordinates) is recorded, then the head goes back
to --top. DIST bounds the descent: without contact the cycle ends in a probe
alarm (halt) instead of pressing on. The first unexpected reply stops the run.
"""
import argparse
import json
import re
import time

from carvera_scan.machine import Carvera

TOP = -1.3


def wait_line(m, start, pattern, timeout):
    """Wait for a console line matching `pattern` (or an error) after index `start`."""
    end = time.time() + timeout
    while time.time() < end:
        with m.cond:
            new = m.lines[start:]
        for ln in new:
            if re.search(r"ERROR|error|ALARM|Alarm|HALT|halt", ln):
                raise RuntimeError("machine: " + ln)
            mm = re.search(pattern, ln)
            if mm:
                return mm, new
        time.sleep(0.2)
    raise TimeoutError("no %r after %ss" % (pattern, timeout))


def tool(m):
    return int(m.status()["T"][0])


def load(m):
    st = m.status()
    if st["state"] != "Idle":
        raise SystemExit("not Idle: " + st["raw"])
    if tool(m) == 0:
        print("probe already loaded")
        return
    with m.cond:
        start = len(m.lines)
    m.send("M6 T0", wait=1.0)
    end = time.time() + 300
    while time.time() < end:
        time.sleep(1.0)
        st = m.status()
        if st["state"].startswith("Alarm"):
            raise RuntimeError("alarm during tool change: " + st["raw"])
        if st["state"] == "Idle" and int(st["T"][0]) == 0:
            time.sleep(2.0)
            st = m.status()
            if st["state"] == "Idle":
                break
    with m.cond:
        print("\n".join(m.lines[start:]))
    print(st["raw"])
    m.send("$#", wait=1.0)


def touch(m, out, spots, top):
    if tool(m) != 0:
        raise SystemExit("probe (tool 0) not loaded")
    f = open(out, "a")
    for name, x, y, dist in spots:
        m.move(None, None, top, feed=1000)
        m.move(x, y, None, feed=2000)
        with m.cond:
            start = len(m.lines)
        t0 = time.time()
        m.send("M466 Z-%.3f" % dist, wait=0.2)
        mm, new = wait_line(m, start, r"Final Posit\w*n Z:\s*(-?[\d.]+)", timeout=60 + dist / 5.0 * 2)
        zm = float(mm.group(1))
        # the work-coordinate line follows the machine one
        try:
            mw, new = wait_line(m, start + new.index(mm.string) + 1, r"Final Posit\w*n Z:\s*(-?[\d.]+)", 5)
            zw = float(mw.group(1))
        except (TimeoutError, ValueError):
            zw = None
        st = m.wait_idle(timeout=60)
        rec = {"spot": name, "x": x, "y": y, "dist": dist, "z_machine": zm, "z_work": zw,
               "after": st["MPos"][:3], "s": round(time.time() - t0, 1)}
        print(json.dumps(rec), flush=True)
        f.write(json.dumps(rec) + "\n")
        f.flush()
        m.move(None, None, top, feed=1000)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["load", "touch", "unload"])
    ap.add_argument("out", nargs="?")
    ap.add_argument("--spot", nargs=4, action="append", default=[], metavar=("NAME", "X", "Y", "DIST"))
    ap.add_argument("--top", type=float, default=TOP)
    a = ap.parse_args()
    m = Carvera(log=lambda s: None)
    try:
        if a.action == "load":
            load(m)
        elif a.action == "unload":
            m.move(None, None, a.top, feed=1000)
            m.send("M6 T-1", wait=1.0)
            time.sleep(3)
            print(m.wait_idle(timeout=300)["raw"])
        else:
            spots = [(n, float(x), float(y), float(d)) for n, x, y, d in a.spot]
            for n, x, y, d in spots:
                if not (-371.4 <= x <= 0 and -250 <= y <= 0 and 0 < d <= 130):
                    raise SystemExit("bad spot %s" % n)
            touch(m, a.out, spots, a.top)
    finally:
        m.close()


if __name__ == "__main__":
    main()
