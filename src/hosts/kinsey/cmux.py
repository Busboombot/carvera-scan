"""cmux: share one Carvera connection between several clients.

The Carvera accepts one TCP client at a time. cmux holds that connection and
listens on its own port 2222; any number of controllers and scripts connect
to cmux instead.

- Client -> machine: bytes are reassembled into whole Makera frames
  (0x8668 ... CRC-16 ... 0x55AA) and each frame is written upstream in one
  piece, so two clients can never interleave mid-frame. Bytes that are not
  part of a frame (the controller's plain-text `echo echo` protocol probe)
  are dropped; the silence tells the controller to use the framed protocol.
- Machine -> clients: every frame goes to every client, except file-transfer
  frames, which go only to the client doing the transfer.
- File transfers (`upload`, `download`, or any 0xB0-0xB6 frame) lock the
  machine to that client; other clients' frames wait until the transfer has
  been quiet for LOCK_IDLE seconds, and their `?` polls are dropped.
- When nothing has been sent upstream for KEEPALIVE seconds, cmux sends `?`
  so the machine doesn't drop the idle link.
- Announces itself on UDP 3333 like the machine does, as `<name>-mux`.

    python3 cmux.py [--machine 192.168.6.32] [--port 2222] [--name Carvera_03628]
"""
import argparse
import asyncio
import logging
import socket
import time

HEADER, FOOTER = b"\x86\x68", b"\x55\xaa"
CTRL_SINGLE, CTRL_MULTI = 0xA1, 0xA2
NORMAL_INFO = 0x90
FILE_TYPES = range(0xB0, 0xB7)
MAX_LEN = 8200
KEEPALIVE = 2.0
LOCK_IDLE = 3.0

log = logging.getLogger("cmux")


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


class Deframer:
    """Split a byte stream into whole frames. Yields (ptype, payload, raw)."""

    def __init__(self):
        self.buf = b""
        self.dropped = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(HEADER)
            if i < 0:
                keep = 1 if self.buf.endswith(HEADER[:1]) else 0
                self.dropped += len(self.buf) - keep
                self.buf = self.buf[len(self.buf) - keep:]
                break
            self.dropped += i
            self.buf = self.buf[i:]
            if len(self.buf) < 4:
                break
            n = int.from_bytes(self.buf[2:4], "big")
            if not 3 <= n <= MAX_LEN:
                self.buf = self.buf[2:]  # false header
                continue
            total = 4 + n + 2
            if len(self.buf) < total:
                break
            raw, body = self.buf[:total], self.buf[2:4 + n]
            if raw[-2:] != FOOTER or crc16(body[:-2]) != int.from_bytes(body[-2:], "big"):
                self.buf = self.buf[2:]
                continue
            self.buf = self.buf[total:]
            out.append((body[2], body[3:-2], raw))
        return out


class Client:
    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer
        self.peer = "%s:%s" % writer.get_extra_info("peername")[:2]
        self.deframer = Deframer()


class Mux:
    def __init__(self, machine, port, name):
        self.machine, self.port, self.name = machine, port, name
        self.clients = []
        self.up_writer = None
        self.last_up = 0.0
        self.lock_owner, self.lock_time = None, 0.0
        self.held = []  # (client, raw) waiting for a file-transfer lock to end
        self.drop_warned = False

    # --- upstream -----------------------------------------------------------

    async def upstream(self):
        delay = 1
        while True:
            try:
                reader, writer = await asyncio.wait_for(
                    asyncio.open_connection(self.machine, 2222), 10)
            except (OSError, asyncio.TimeoutError) as e:
                log.warning("machine %s: %s; retry in %ss", self.machine, e, delay)
                await asyncio.sleep(delay)
                delay = min(delay * 2, 30)
                continue
            log.info("connected to machine %s:2222", self.machine)
            since = time.monotonic()
            self.drop_warned = False
            self.up_writer, self.last_up = writer, 0.0
            deframer = Deframer()
            try:
                while data := await reader.read(65536):
                    for ptype, payload, raw in deframer.feed(data):
                        self.from_machine(ptype, raw)
            except OSError as e:
                log.warning("machine link error: %s", e)
            self.up_writer = None
            writer.close()
            # A reset right after connecting means another client holds the
            # machine (it takes one at a time): back off instead of hammering.
            delay = 1 if time.monotonic() - since > 10 else min(delay * 2, 30)
            log.warning("machine connection closed; reconnecting in %ss", delay)
            await asyncio.sleep(delay)

    def send_up(self, raw):
        if self.up_writer is None:
            return False
        self.up_writer.write(raw)
        self.last_up = time.monotonic()
        return True

    def from_machine(self, ptype, raw):
        if ptype in FILE_TYPES and self.lock_owner is not None:
            self.lock_time = time.monotonic()
            targets = [self.lock_owner]
        else:
            targets = self.clients
        for c in list(targets):
            try:
                c.writer.write(raw)
            except OSError:
                pass

    # --- clients ------------------------------------------------------------

    async def serve_client(self, reader, writer):
        c = Client(reader, writer)
        self.clients.append(c)
        log.info("client %s connected (%d clients)", c.peer, len(self.clients))
        try:
            while data := await reader.read(65536):
                for ptype, payload, raw in c.deframer.feed(data):
                    self.from_client(c, ptype, payload, raw)
                await asyncio.gather(*(x.writer.drain() for x in self.clients),
                                     return_exceptions=True)
        except OSError:
            pass
        finally:
            self.clients.remove(c)
            if self.lock_owner is c:
                self.release_lock()
            writer.close()
            log.info("client %s gone (%d clients, %d stray bytes dropped)",
                     c.peer, len(self.clients), c.deframer.dropped)

    def from_client(self, c, ptype, payload, raw):
        starts_transfer = ptype in FILE_TYPES or (
            ptype == CTRL_MULTI and payload.split(b" ", 1)[0] in (b"upload", b"download"))
        if self.lock_owner is not None and self.lock_owner is not c:
            if not (ptype == CTRL_SINGLE and payload == b"?"):
                self.held.append((c, raw))
                c.writer.write(frame(NORMAL_INFO, b"cmux: file transfer in progress, command held\n"))
                log.info("%s: held during file transfer", c.peer)
            return
        if starts_transfer:
            if self.lock_owner is None:
                log.info("file transfer lock -> %s", c.peer)
            self.lock_owner, self.lock_time = c, time.monotonic()
        if ptype == CTRL_MULTI:
            log.info("%s > %s", c.peer, payload.decode(errors="replace"))
        elif ptype == CTRL_SINGLE and payload != b"?":
            log.info("%s > realtime %r", c.peer, payload)
        if not self.send_up(raw) and not self.drop_warned:
            log.warning("dropping client frames: machine not connected")
            self.drop_warned = True

    def release_lock(self):
        log.info("file transfer lock released")
        self.lock_owner = None
        held, self.held = self.held, []
        for c, raw in held:
            if c in self.clients:
                self.send_up(raw)

    # --- housekeeping -------------------------------------------------------

    async def ticker(self):
        poll = frame(CTRL_SINGLE, b"?")
        while True:
            await asyncio.sleep(0.5)
            now = time.monotonic()
            if self.lock_owner is not None:
                if now - self.lock_time > LOCK_IDLE:
                    self.release_lock()
            elif self.up_writer is not None and now - self.last_up > KEEPALIVE:
                self.send_up(poll)

    async def announce(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while True:
            try:
                probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                probe.connect((self.machine, 2222))
                ip = probe.getsockname()[0]
                probe.close()
                msg = "%s-mux,%s,%d,0" % (self.name, ip, self.port)
                s.sendto(msg.encode(), ("255.255.255.255", 3333))
            except OSError:
                pass
            await asyncio.sleep(2)

    async def run(self):
        server = await asyncio.start_server(self.serve_client, "0.0.0.0", self.port)
        log.info("listening on :%d for machine %s", self.port, self.machine)
        async with server:
            await asyncio.gather(server.serve_forever(), self.upstream(),
                                 self.ticker(), self.announce())


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--machine", default="192.168.6.32")
    ap.add_argument("--port", type=int, default=2222)
    ap.add_argument("--name", default="Carvera_03628")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    asyncio.run(Mux(a.machine, a.port, a.name).run())
