"""Carvera client over the Makera framed protocol, through cmux on 127.0.0.1:2222.

    python -m carvera_scan.machine status
    python -m carvera_scan.machine cmd M821            # any console/G-code line
    python -m carvera_scan.machine move X Y Z [--feed F] [--zmin Z]   # machine coords, G53

Moves are checked against the soft endstops and --zmin before they are sent.
Ctrl-C during a wait sends feed hold ('!').
"""
import argparse
import re
import socket
import threading
import time

HEADER, FOOTER = b"\x86\x68", b"\x55\xaa"
CTRL_SINGLE, CTRL_MULTI = 0xA1, 0xA2
# soft_endstop.* and travel tops from the machine's config.txt
LIMITS = {"x": (-371.4, 0.0), "y": (-250.0, 0.0), "z": (-135.0, 0.0)}


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def frame(ptype, payload):
    body = (len(payload) + 3).to_bytes(2, "big") + bytes([ptype]) + payload
    return HEADER + body + crc16(body).to_bytes(2, "big") + FOOTER


class MoveLost(RuntimeError):
    pass


def parse_status(line):
    """'<Idle|MPos:x,y,z,a,b|WPos:...|T:1,...>' -> dict"""
    parts = line.strip("<>").split("|")
    st = {"state": parts[0], "raw": line}
    for p in parts[1:]:
        k, _, v = p.partition(":")
        st[k] = [float(x) if re.fullmatch(r"-?[\d.]+", x.strip()) else x.strip() for x in v.split(",")]
    return st


class Carvera:
    def __init__(self, host="127.0.0.1", port=2222, log=print):
        self.host, self.port, self.log = host, port, log
        self.lines, self.status_line = [], None
        self.cond = threading.Condition()
        self.wlock = threading.Lock()  # status polling and moves may share the socket
        self.connect()

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        self.sock.settimeout(0.5)
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def close(self):
        self.alive = False
        self.sock.close()

    def _reader(self):
        buf, text = b"", ""
        while self.alive:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while True:
                i = buf.find(HEADER)
                if i < 0 or len(buf) < i + 4:
                    break
                n = int.from_bytes(buf[i + 2:i + 4], "big")
                if len(buf) < i + 4 + n + 2:
                    break
                body = buf[i + 2:i + 4 + n]
                buf = buf[i + 4 + n + 2:]
                if crc16(body[:-2]) != int.from_bytes(body[-2:], "big"):
                    continue
                text += body[3:-2].decode(errors="replace")
                # status replies arrive whole; console output may be split
                while "\n" in text or (text.startswith("<") and ">" in text):
                    if text.startswith("<") and ">" in text:
                        line, text = text[:text.index(">") + 1], text[text.index(">") + 1:].lstrip("\r\n")
                    else:
                        line, text = text.split("\n", 1)
                    self._got(line.strip())
        self.alive = False

    def _got(self, line):
        if not line:
            return
        with self.cond:
            if line.startswith("<"):
                self.status_line = line
            else:
                self.lines.append(line)
                self.log("  < " + line)
            self.cond.notify_all()

    def send(self, line, wait=1.0):
        """Send one line; return console lines that arrive within `wait` s."""
        with self.cond:
            start = len(self.lines)
        self.log("  > " + line)
        with self.wlock:
            self.sock.sendall(frame(CTRL_MULTI, line.encode()))
        time.sleep(wait)
        with self.cond:
            return self.lines[start:]

    def diagnose(self):
        """The diagnose reply as {key: [values]} (E: limit/cover inputs, G: light, ...)."""
        with self.cond:
            start = len(self.lines)
        self.send("diagnose", wait=1.0)
        with self.cond:
            new = self.lines[start:]
        for line in new:
            if line.startswith("{") and "|E:" in line:
                out = {}
                for part in line.strip("{}").split("|"):
                    k, _, v = part.partition(":")
                    out[k] = v.split(",")
                return out
        raise TimeoutError("no diagnose reply")

    def set_light(self, on, tries=4):
        """M821/M822 and confirm from diagnose (commands are sometimes lost over Wi-Fi)."""
        for _ in range(tries):
            self.send("M821" if on else "M822", wait=0.5)
            try:
                if int(self.diagnose()["G"][0]) == int(on):
                    return True
            except (TimeoutError, KeyError, ValueError):
                pass
        return False

    def cover_closed(self):
        """True if diagnose reports the cover input closed (E field index 5 == 1)."""
        with self.cond:
            start = len(self.lines)
        self.send("diagnose", wait=1.0)
        with self.cond:
            new = self.lines[start:]
        for line in new:
            m = re.search(r"\|E:([\d,]+)", line)
            if m:
                return int(m.group(1).split(",")[5]) == 1
        raise TimeoutError("no diagnose reply")

    def realtime(self, ch):
        with self.wlock:
            self.sock.sendall(frame(CTRL_SINGLE, ch.encode() if isinstance(ch, str) else bytes([ch])))

    def status(self, timeout=2.0, tries=3):
        # Over Wi-Fi a status reply is occasionally lost; ask again before failing.
        for _ in range(tries):
            with self.cond:
                self.status_line = None
                self.realtime("?")
                self.cond.wait_for(lambda: self.status_line is not None, timeout)
                line = self.status_line
            if line is not None:
                return parse_status(line)
        raise TimeoutError("no status reply after %d tries" % tries)

    def wait_idle(self, timeout=300, settle=0.3):
        """Poll until Idle. Returns the final status. Ctrl-C -> feed hold."""
        end = time.time() + timeout
        time.sleep(settle)
        try:
            while time.time() < end:
                st = self.status()
                if st["state"] == "Idle":
                    return st
                if st["state"].startswith("Alarm"):
                    raise RuntimeError("machine alarm: " + st["raw"])
                time.sleep(0.25)
        except KeyboardInterrupt:
            self.realtime("!")
            self.log("FEED HOLD sent")
            raise
        raise TimeoutError("not idle after %ss" % timeout)

    def move(self, x=None, y=None, z=None, feed=1500, zmin=None):
        """Linear move in machine coordinates (G53), then wait for Idle."""
        target = {"x": x, "y": y, "z": z}
        for ax, v in target.items():
            if v is None:
                continue
            lo, hi = LIMITS[ax]
            if ax == "z" and zmin is not None:
                lo = max(lo, zmin)
            if not lo <= v <= hi:
                raise ValueError("%s=%g outside [%g, %g]" % (ax, v, lo, hi))
        words = "".join("%s%.3f" % (ax.upper(), v) for ax, v in target.items() if v is not None)
        line = "G90 G53 G1 %s F%d" % (words, feed)
        # The machine occasionally drops a command (seen over Wi-Fi). The move
        # is absolute, so sending it again is safe.
        for attempt in range(3):
            self.send(line, wait=0.2)
            try:
                return self.wait_at(target)
            except MoveLost as e:
                if attempt == 2:
                    raise
                print("retrying lost move (%s)" % e, flush=True)

    def wait_at(self, target, timeout=300, tol=0.01, stuck=3.0):
        """Wait until Idle at `target` ({'x': .., 'y': .., 'z': ..}, None = any).

        "Idle" alone isn't proof: the machine reports Idle before a queued
        move starts, and cmux may hold a frame during a file transfer. If it
        sits Idle away from the target for `stuck` seconds, the move was
        lost: raise rather than carry on.
        """
        end = time.time() + timeout
        idle_since = None
        try:
            while time.time() < end:
                st = self.status()
                if st["state"].startswith("Alarm"):
                    raise RuntimeError("machine alarm: " + st["raw"])
                pos = dict(zip("xyz", st["MPos"][:3]))
                there = all(v is None or abs(pos[ax] - v) <= tol for ax, v in target.items())
                if st["state"] == "Idle":
                    if there:
                        return st
                    idle_since = idle_since or time.time()
                    if time.time() - idle_since > stuck:
                        raise MoveLost("Idle at %s, wanted %s" % (pos, target))
                else:
                    idle_since = None
                time.sleep(0.2)
        except KeyboardInterrupt:
            self.realtime("!")
            self.log("FEED HOLD sent")
            raise
        raise TimeoutError("not at %s after %ss" % (target, timeout))


def light_on(host="127.0.0.1"):
    """Turn the work light back on from a fresh connection (used after a scan,
    when the scan's own link may have died). Never raises."""
    try:
        m = Carvera(host, log=lambda s: None)
        if not m.set_light(True):
            print("light did not confirm on", flush=True)
        m.close()
    except Exception as e:
        print("could not turn the light back on: %s" % e, flush=True)


def axis(start, stop, step):
    """start, start+step, ... up to stop (inclusive), stepping toward stop."""
    import numpy as np
    step = abs(step) * (1 if stop >= start else -1)
    return list(np.arange(start, stop + step / 2, step).round(3))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["status", "cmd", "move"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--feed", type=int, default=1500)
    ap.add_argument("--zmin", type=float)
    ap.add_argument("--wait", type=float, default=2.0)
    a = ap.parse_args()
    m = Carvera(a.host)
    if a.action == "status":
        print(m.status()["raw"])
    elif a.action == "cmd":
        m.send(" ".join(a.args), wait=a.wait)
    else:
        x, y, z = (None if v == "-" else float(v) for v in a.args)
        print(m.move(x, y, z, feed=a.feed, zmin=a.zmin)["raw"])
    m.close()
