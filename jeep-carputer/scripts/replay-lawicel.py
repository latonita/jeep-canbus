#!/usr/bin/env python3
"""Replay a LAWICEL/SLCAN capture into the carputer's link UART.

Stands in for the ATmega328 gateway: answers the AT handshake, then streams
recorded frames in the adapter's own wire format. Lets the dashboard be worked
on at a desk, against real traffic, with no car and no radio.

    python scripts/replay-lawicel.py session.log --port COM5
    python scripts/replay-lawicel.py session.log --port COM5 --loop --rate 150

WIRING. The link is the ESP32's UART0 on GPIO43 (TX) / GPIO44 (RX), 115200 8N1,
3.3 V -- see LINK_UART_* in src/Board.h. A USB-TTL adapter's TX goes to GPIO44,
grounds common, and the 328 must be disconnected or both will drive the line.
This is NOT the USB-serial console; that is a separate interface and the log
still comes out of it.

TWO FORMATS, TWO CHARACTERS APART.

  LAWICEL   t 014 7 0FBF4FFF1166FF
              |   |  |
              |   |  data, 2 hex per byte
              |   DLC nibble
              11-bit id

  Adapter     014  0FBF4FFF1166FF        (ATH1 + ATS0 + ATL0)
              id   data

The adapter emits no 't' and no DLC -- the length is implied by how many hex
digits follow. Lines are CR-terminated, pure hex, no spaces, always an ODD
number of characters between 3 and 19. That is exactly what core/LinkLine.cpp
tests, and anything failing it is read as a command reply and dropped, which is
what would happen to raw 't...' lines.

TIMESTAMPS. LAWICEL appends four hex digits of millisecond timestamp when the
adapter was put in Z1 mode. Detected here by length rather than by being told:
a frame carrying four more hex digits than its DLC accounts for has one. When
present they drive the pacing, so the replay runs at the speed it was recorded.
When absent --rate stands in.

WHAT THIS CANNOT DO. It replays broadcasts. The poll plan (fuel level, warning
lamps, tyre pressures) is request/response against modules that are not here, so
those requests go unanswered and their readouts stay dashed. That is honest
rather than broken: the TPMS values are not in a broadcast capture to begin
with.
"""
import argparse
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  python -m pip install pyserial")

# A LAWICEL standard data frame: 't' + 3 id nibbles + 1 DLC nibble + payload.
LAWICEL = re.compile(r"^t([0-9A-Fa-f]{3})([0-9A-Fa-f])([0-9A-Fa-f]*)\s*$")

# The adapter's identity. Deliberately marked, so a session spent chasing a
# problem that only exists in the replay is a short one -- the SETTINGS page
# shows this as the adapter version.
FW_VERSION = "2.1R"
IDENT = "latonita-jeep v%s (replay)" % FW_VERSION

STATUS = ("FW=%s MODE=01 AUX=0 BT=0 BENCH=0 CAN=OK RX=0 TX=0 ER=0 "
          "SH=7E0 CF=000 CM=000 F=VTB UP=%%u RAM=1333 RST=01 REI=0 SM=1 SD=0"
          % FW_VERSION)


def parse(path):
    """-> [(wire_line, timestamp_ms or None)], skipping what we cannot read."""
    out, skipped = [], 0
    with open(path, "r", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            m = LAWICEL.match(line)
            if not m:
                skipped += 1          # 'T' extended, 'r' RTR, status lines, junk
                continue
            cid, dlc, payload = m.group(1).upper(), int(m.group(2), 16), m.group(3)
            want = dlc * 2
            stamp = None
            if len(payload) == want + 4:
                # Trailing millisecond timestamp, LAWICEL Z1 mode. Wraps at
                # 60000 (0xEA60), which unwrap() below puts back together.
                stamp = int(payload[want:], 16)
                payload = payload[:want]
            elif len(payload) != want:
                skipped += 1
                continue
            if dlc > 8:
                skipped += 1
                continue
            out.append((cid + payload.upper(), stamp))
    return out, skipped


def unwrap(frames):
    """Turn wrapping 0..59999 ms stamps into a monotonic millisecond timeline."""
    base, prev, times = 0, None, []
    for _, stamp in frames:
        if stamp is None:
            return None               # one missing stamp and the timeline is a guess
        if prev is not None and stamp < prev:
            base += 60000
        prev = stamp
        times.append(base + stamp)
    return times


class Adapter(object):
    """Answers the AT handshake so At328Link::begin() completes.

    Everything beginning with AT is acknowledged. That is enough: begin() only
    checks for OK or the prompt, and answering '?' to something it wanted would
    fail bring-up for no benefit. Bare hex lines are the carputer's own KWP
    requests and are ignored -- there is no module here to answer them.
    """

    def __init__(self, port):
        self.port = port
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.stop = threading.Event()

    def send(self, text):
        with self.lock:               # never interleave a reply into a frame line
            self.port.write(text.encode("ascii", "replace"))

    def reply(self, body=None):
        self.send((body + "\r" if body else "") + ">")

    def serve(self):
        buf = bytearray()
        while not self.stop.is_set():
            try:
                chunk = self.port.read(64)
            except Exception:
                break
            if not chunk:
                continue
            for byte in chunk:
                if byte in (13, 10):
                    line = buf.decode("ascii", "replace").strip()
                    buf.clear()
                    if line:
                        self.command(line)
                elif len(buf) < 64:
                    buf.append(byte)

    def command(self, line):
        up = line.upper()
        if not up.startswith("AT"):
            return                    # a KWP request; nobody here to answer it
        if up.startswith("AT@ST"):
            self.reply(STATUS % int(time.time() - self.t0))
        elif up in ("ATI", "ATZ"):
            self.reply(IDENT)
        else:
            self.reply("OK")
        print("  <- %-10s" % line, end="\r")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile")
    ap.add_argument("--port", required=True, help="e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=float, default=150.0,
                    help="frames/sec when the capture has no timestamps "
                         "(default 150, the measured rate of this bus)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="replay speed multiplier; >1 compresses time and will "
                         "make the fuel integral wrong, because dt is measured "
                         "on arrival rather than read from the log")
    ap.add_argument("--loop", action="store_true", help="restart at the end")
    args = ap.parse_args()

    frames, skipped = parse(args.logfile)
    if not frames:
        sys.exit("no LAWICEL frames in %s" % args.logfile)
    times = unwrap(frames)

    ids = sorted({f[0][:3] for f in frames})
    span = (times[-1] - times[0]) / 1000.0 if times else len(frames) / args.rate
    print("%s: %d frames, %d ids, %.1f s  (%.0f f/s)%s"
          % (args.logfile, len(frames), len(ids), span,
             len(frames) / span if span else 0,
             "  [%d lines skipped]" % skipped if skipped else ""))
    print("pacing: %s" % ("capture timestamps" if times else
                          "%.0f f/s (no timestamps in log)" % args.rate))

    port = serial.Serial(args.port, args.baud, timeout=0.05)
    adapter = Adapter(port)
    threading.Thread(target=adapter.serve, daemon=True).start()
    print("serving on %s at %d -- Ctrl+C to stop\n" % (args.port, args.baud))

    gap = 1.0 / (args.rate * args.speed)
    try:
        while True:
            start = time.time()
            for i, (wire, _) in enumerate(frames):
                adapter.send(wire + "\r")
                if times:
                    due = start + (times[i] - times[0]) / 1000.0 / args.speed
                    delay = due - time.time()
                    if delay > 0:
                        time.sleep(delay)
                else:
                    time.sleep(gap)
            if not args.loop:
                break
            print("  -- loop --%s" % (" " * 30))
    except KeyboardInterrupt:
        pass
    finally:
        adapter.stop.set()
        port.close()
        print("\nstopped")


if __name__ == "__main__":
    main()
