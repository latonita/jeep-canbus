#!/usr/bin/env python3
"""Bench emulator: the ATmega328 gateway AND the modules behind it.

Stands in for the whole car. Answers the AT handshake, streams recorded
broadcasts, and -- the part a capture replay cannot do -- ANSWERS REQUESTS.
Tyre pressures, fault codes, fuel level, lamps and ECU identity all come back
as real ISO-TP/KWP responses, so the polled half of the dashboard comes alive
at a desk with no car, no radio and no 328.

    python scripts/emulate-car.py --port COM5
    python scripts/emulate-car.py --port COM5 --log playback --loop
    python scripts/emulate-car.py --port COM5 --log drive        # the real one

--log takes a NAME out of LOGS (below) or any path. `drive` is the owner's own
25-minute timestamped drive, which lives outside this repo and is the closest
thing to sitting in the truck; `playback` is the 2015 capture every measurement
in CLAUDE.md was taken on.

The log is OPTIONAL now. Without it there are no broadcasts (speed, rpm, flow
and the trips stay dashed) but every request/response page works -- which is
usually what you are here for. With it, both halves work at once.

Once running, type commands at the prompt to change what the car is doing:

    tyre 2 24          one corner down to 24 psi -- watch TYRES colour it
    dtc add wcm C0077  a stored fault, with its real description
    dtc clear ccn      wipe one module's memory
    ecu off ccn        make a module stop answering -- "quiet", not "clean"
    clear refuse       make service 0x14 answer 7F ... 22
    pending once       answer 7F ... 78 before the real reply
    show               what the car currently looks like
    help               all of it

WIRING. The link is the ESP32's UART0 on GPIO43 (TX) / GPIO44 (RX), 115200 8N1,
3.3 V -- see LINK_UART_* in src/Board.h. A USB-TTL adapter's TX goes to GPIO44,
its RX to GPIO43, grounds common, and the 328 must be disconnected or both will
drive the line. This is NOT the USB-serial console; that is a separate
interface and the ESP_LOG output still comes out of it.

WHY THE TWO AWKWARD MODES ARE HERE. `clear refuse` and `pending once` exist
because CLAUDE.md records both of those paths as UNVERIFIED on the real
vehicle, and neither can be reached from a broadcast capture:

  * The DTC clear sends `14 <hi> <lo>` -- clearDiagnosticInformation with a
    specific code in the group field. Whether these modules honour the
    single-code form is unknown; a refusal is supposed to log and leave the row
    alone, and the button is supposed to end up saying FAILED. `clear refuse`
    is how you find out whether it actually does.
  * `7F xx 78` "response pending" is a KNOWN HOLE: IsoTpClient::transact()
    treats it as a complete reply and stops reading, so the real answer that
    follows is discarded and the next request resets the queue over it. A
    module that always answers pending first is therefore permanently
    unreadable. `pending once` and `pending on` reproduce that on demand,
    which is the only way to test a fix for it without the car.

TWO WIRE FORMATS, TWO CHARACTERS APART.

  LAWICEL   t 014 7 0FBF4FFF1166FF     (what a capture holds, and the shape
              |   |  |                    the carputer now asks for)
              |   |  data, 2 hex per byte
              |   DLC nibble
              11-bit id

  Adapter     014  0FBF4FFF1166FF      (the older AT@SM shape: ATH1+ATS0+ATL0)
              id   data

The adapter shape emits no 't' and no DLC -- length is implied by the hex
digit count. Lines are CR-terminated, pure hex, no spaces, an ODD number of
characters between 3 and 19. That is exactly what core/LinkLine.cpp accepts;
anything failing it is filed as a command reply and dropped.

Both shapes are emulated, at all four levels each, because the gateway holds
ONE level field and the carputer now asks for the LAWICEL side of it:

  AT@SML0..3   off / LAWICEL no stamp / + 4 hex digits / + 8 hex digits
  AT@SM0..3    off / adapter shape    / + 4 hex digits / + 8 hex digits

AT@SML2 is the headline: it is what At328Link sends, so it is the path that
has to be right. The AT@SM levels are kept because they are the fallback the
firmware drops to when a level is refused, not because anything still asks
for them first.

The stamps are real, off this process's own monotonic clock: 4 digits are
ms % 60000 (bit-exact LAWICEL "Z1"), 8 digits the raw ms, uppercase, taken at
the moment the line is written and so never going backwards between two
consecutive frames. Faking them was never an option -- At328Link strips a
FIXED number of characters off every frame line and decodes the rest, so a
wrong width or a stamp that walks backwards corrupts data silently instead of
failing, and BusCycle reads the values as bus timing.

A replayed capture carries stamps of its own, and those are used for PACING
only: the gap between two lines follows the capture (divided by --speed),
while the digits on the wire come from the emulator's clock at send time,
exactly as the real gateway stamps at drain time rather than at bus arrival.
Copying the capture's digits onto a line the emulator emits seconds later
would be the one dishonest thing here.

One level field, so the two commands are mutually exclusive exactly as on the
board: AT@SML2 turns the adapter-shaped stream off, AT@SM1 turns LAWICEL off.
Each query is truthful about ITS OWN format only -- under AT@SML2 a bare
`AT@SM` prints 0 -- and so do the SM= and SML= fields of AT@ST. An
out-of-range digit answers '?', which is also what exercises At328Link's
fallback to AT@SM1. ATZ restores level 0; ATI does not.

TRAP, the same one the gateway documents against itself: AT@SML has to be
matched BEFORE AT@SM, or the prefix test eats AT@SML1 and hands the level
parser "L1", and a command that is right there looks unimplemented.

Request/response lines are NOT part of any of this. They stay adapter-shaped
and are NEVER stamped whatever the stream format is, because LAWICEL has no
request/response notion and those lines belong to the ELM327 surface (ATH,
NO DATA, the ISO-TP flow) that the host parser already depends on.

One deliberate difference from the board: level 0 does not silence the replay
here. A real gateway is mute until a host asks for a stream, but a bench
emulator that goes quiet before the handshake -- or under the ATMA fallback,
which this emulator answers with OK rather than implementing as a mode -- is
indistinguishable from a dead emulator or a dead USB adapter, so level 0
keeps emitting plain unstamped adapter-shaped lines.
"""
import argparse
import os
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  python -m pip install pyserial")


# ---------------------------------------------------------------------------
# The modules, keyed the way the carputer addresses them.
#
# Copied from core/SignalDb.cpp's kEcus, and the `present` column is the point
# of the whole table: a module that DOES NOT ANSWER is not a module with no
# faults. The carputer keeps `answered` and `unreachable` as separate bitmasks
# and HEALTH says "none (7 asked)" rather than a bare "none", so being able to
# switch a module off here is how that distinction gets tested.
#
# The default three present are the ones the firmware probes at bring-up
# (RADIO, CCN, WCM). The CAN-C modules (PCM, TCM, ABS...) are reachable only
# through a gateway hop this project does not do, so they are off and should
# stay off -- they are what makes a real scan report nine quiet modules.
ECUS = {
    #  name       req    resp   present
    "CCN":    (0x6A0, 0x514, True),
    "FCMCGW": (0x620, 0x504, False),
    "RADIO":  (0x6B0, 0x516, True),
    "ORC":    (0x6E0, 0x51C, False),
    "WCM":    (0x600, 0x500, True),
    "SCM":    (0x6A8, 0x515, False),
    "EOM":    (0x6F0, 0x51E, False),
    "PCM":    (0x7E0, 0x7E8, False),
    "TCM":    (0x7E1, 0x7E9, False),
    "ABS":    (0x784, 0x785, False),
    "FDCM":   (0x7B6, 0x7B7, False),
    "ESM":    (0x788, 0x789, False),
}

# KWP services this emulator understands. Anything else gets a negative reply
# rather than silence, because silence is indistinguishable from an absent
# module and would make a typo look like a wiring fault.
SVC_CLEAR_DTC = 0x14
SVC_READ_DTC = 0x18
SVC_READ_IDENT = 0x1A
SVC_READ_LOCAL = 0x21

NRC_SERVICE_NOT_SUPPORTED = 0x11
NRC_SUBFUNCTION_NOT_SUPPORTED = 0x12
NRC_CONDITIONS_NOT_CORRECT = 0x22
NRC_REQUEST_OUT_OF_RANGE = 0x31
NRC_PENDING = 0x78

# DTC status bits, from core/Kwp.cpp.
ST_LAMP = 0x80
ST_ACTIVE = 0x40
ST_STORED = 0x20

FW_VERSION = "2.1R"
IDENT = "latonita-jeep v%s (emulator)" % FW_VERSION
STATUS = ("FW=%s MODE=01 AUX=0 BT=0 BENCH=0 CAN=OK RX=0 TX=0 ER=0 "
          "SH=7E0 CF=000 CM=000 F=VTB UP=%%u RAM=1333 RST=01 REI=0 "
          "SM=%%d SML=%%d SD=0"
          % FW_VERSION)

# The stream level, as the ONE mutually-exclusive field the gateway keeps --
# StreamMode in ../jeep-aux-and-can-gateway/lib/AtServer/AtServer.h. 0 is off,
# 1..3 are the adapter-shaped levels and 4..6 the LAWICEL ones, so writing
# either command's level necessarily clears the other and no invariant like
# "never both on" has to be maintained by hand.
SM_OFF = 0
SM_ELM = 1              # AT@SM1 .. AT@SM3   are SM_ELM + 0..2
SM_LAW = 4              # AT@SML1 .. AT@SML3 are SM_LAW + 0..2

# Within either format the offset from its level 1 IS the stamp width, so a
# width cannot be taught to one format and forgotten in the other.
TS_CHARS = (0, 4, 8)

LAWICEL = re.compile(r"^t([0-9A-Fa-f]{3})([0-9A-Fa-f])([0-9A-Fa-f]*)\s*$")


# ---------------------------------------------------------------------------
# DTC codes, in the form the firmware renders them.
#
# The rule is derived in core/Kwp.cpp from all 1152 (raw, SAE) pairs in the
# extracted CSV and re-verified against every one of them in test_kwp:
#   bits 15..14 -> letter, index into "PCBU"
#   bits 13..12 -> first digit, 0..3
#   bits 11..0  -> the last three characters, as HEX digits
# The tail being hex is why P213A exists and why this cannot use int(,10).
LETTERS = "PCBU"


def sae_to_raw(text):
    s = text.strip().upper()
    if len(s) != 5 or s[0] not in LETTERS:
        raise ValueError("expected a code like P0119 / C0077 / B2102 / U0184")
    first = int(s[1], 16)
    if first > 3:
        # Only two bits hold it, so P4xxx has no encoding. Better to say so
        # than to silently emit a code the dashboard renders as something else.
        raise ValueError("second character must be 0-3 (only two bits carry it)")
    return ((LETTERS.index(s[0]) & 3) << 14) | ((first & 3) << 12) | int(s[2:5], 16)


def raw_to_sae(raw):
    return "%c%X%X%X%X" % (LETTERS[(raw >> 14) & 3], (raw >> 12) & 3,
                           (raw >> 8) & 0xF, (raw >> 4) & 0xF, raw & 0xF)


def status_text(status):
    if status == 0:
        return "no fault"
    if status & ST_ACTIVE:
        return "active, lamp on" if status & ST_LAMP else "active"
    if status & ST_STORED:
        return "stored, lamp on" if status & ST_LAMP else "stored"
    if status & ST_LAMP:
        return "lamp on"
    return "unknown"


# ---------------------------------------------------------------------------
class Car(object):
    """What the emulated vehicle currently is. Mutated from the console."""

    def __init__(self):
        # psi, one per sensor, in the same UNMAPPED index order the firmware
        # uses. 33 psi is 2.28 bar, inside the owner's 2.1-2.3 band; sensor 2
        # starts soft on purpose so the low-corner colouring is visible the
        # moment the page is opened rather than only after someone types.
        self.tyres = [33, 29, 34, 33]
        self.spare = 7            # the recorded value: flat, or unpaired
        self.fuel_pct = 62.0
        self.lamps = 0x00         # 0x213F bytes 2..3
        self.lamps2 = 0x00
        self.fluids = 0x00        # 0x2135 byte 2

        # Faults per module, as {ecu_name: [(raw_code, status)]}.
        #
        # The defaults are the five this car really reported on 2026-09-16, so
        # a bench session starts from the truth and their descriptions resolve
        # out of the compiled-in table. U0184 on three modules is "lost
        # communication with radio" -- exactly the kind of permanent known
        # fault that muting exists for.
        self.faults = {
            "CCN": [(sae_to_raw("U0184"), ST_STORED)],
            "WCM": [(sae_to_raw("C0077"), ST_STORED | ST_LAMP),
                    (sae_to_raw("B2102"), ST_STORED)],
            "SCM": [(sae_to_raw("U0184"), ST_STORED)],
            "EOM": [(sae_to_raw("U0184"), ST_STORED)],
        }
        self.present = {name: spec[2] for name, spec in ECUS.items()}

        # How service 0x14 behaves: "accept" erases, "refuse" answers
        # negatively, "silent" does not answer at all. See the module
        # docstring -- the single-code form is unverified on the real car and
        # all three are plausible.
        self.clear_mode = "accept"

        # "off", "once" or "on" -- whether to precede a reply with 7F xx 78.
        self.pending_mode = "off"

        self.lock = threading.Lock()

    # -- 0x2103 fuel level: scaling [6] is x0.5, value at bit 24 = byte 3 ----
    def fuel_raw(self):
        return max(0, min(255, int(round(self.fuel_pct * 2.0))))


# ---------------------------------------------------------------------------
def isotp_frames(payload, pad=True):
    """Split a KWP payload into ISO 15765-2 frames, ECU -> tester direction.

    NO FLOW CONTROL IS WAITED FOR, and that is correct rather than a shortcut:
    in the real system the 328 auto-sends the FC frame the moment it sees a
    First Frame, so from the carputer's side a multi-frame reply simply
    arrives as FF followed by CFs. It never sends FC itself -- IsoTpClient is
    single-frame-request only and says so.
    """
    n = len(payload)
    out = []
    if n <= 7:
        f = bytearray([n]) + payload
        out.append(f)
    else:
        out.append(bytearray([0x10 | ((n >> 8) & 0x0F), n & 0xFF]) + payload[:6])
        seq, i = 1, 6
        while i < n:
            out.append(bytearray([0x20 | (seq & 0x0F)]) + payload[i:i + 7])
            seq = (seq + 1) & 0x0F
            i += 7
    if pad:
        # Real modules pad to the full 8 bytes. Worth doing: the reassembler
        # has to clamp to the length declared in the PCI rather than trusting
        # the frame size, and padding is what exercises that.
        for f in out:
            while len(f) < 8:
                f.append(0x00)
    return [bytes(f) for f in out]


def isotp_unwrap_request(frame):
    """Frame bytes -> KWP payload, or None if this frame carries no request.

    THE CARPUTER BUILDS ITS OWN PCI. At328Link sends ATCAF0 ("no auto
    formatting -- we build our own PCI bytes"), so what arrives on the wire is
    a complete ISO-TP frame: `02 21 35`, not `21 35`. Missing that is the first
    thing this emulator got wrong, and the symptom was misleading in exactly
    the way a bench tool must not be -- every request came back
    `7F 02 11` "service 0x02 not supported", because the single-frame PCI byte
    was being read as the service number. The dashboard then showed dashes
    everywhere and nothing in the firmware was at fault.

    Single frame is the only shape to expect: IsoTpClient is deliberately
    single-frame-REQUEST only, and says so -- every request this project sends
    fits in one frame (readDataByLocalIdentifier is 3 bytes,
    readDiagnosticTroubleCodes 4). A First Frame here would mean the firmware
    grew multi-frame requests, which is worth a line in the log rather than a
    silent guess.
    """
    if not frame:
        return None
    kind = (frame[0] >> 4) & 0x0F
    if kind == 0x0:                       # single frame
        n = frame[0] & 0x0F
        if n == 0 or n > len(frame) - 1:
            return None
        return frame[1:1 + n]
    if kind == 0x3:                       # flow control -- not ours to answer
        return None
    return None


class Emulator(object):
    def __init__(self, port, car, pad=True, verbose=True):
        self.port = port
        self.car = car
        self.pad = pad
        self.verbose = verbose
        # Re-entrant because send_stream() stamps the line inside the lock it
        # writes under: the stamp has to describe the moment the line reaches
        # the wire, not the moment some thread decided to build it.
        self.wlock = threading.RLock()
        self.t0 = time.time()
        # Stamps come off the monotonic clock, not time.time(): a host clock
        # step mid-replay would look to the carputer like a 60 s rollover.
        self.mono0 = time.monotonic()
        self.stream = SM_OFF    # AT@SM / AT@SML, one field -- see SM_* above
        self.stop = threading.Event()
        self.sh = None          # ATSH: the request id the next payload goes to
        self.cra = None         # ATCRA: the response filter
        self.by_req = {spec[0]: name for name, spec in ECUS.items()}
        self.counts = {"req": 0, "ans": 0, "neg": 0, "quiet": 0}

    # -- wire ---------------------------------------------------------------
    def raw_send(self, text):
        with self.wlock:        # never interleave a reply into a frame line
            try:
                self.port.write(text.encode("ascii", "replace"))
            except Exception:
                pass

    def send_frame(self, can_id, data):
        # A request/response line: always the adapter shape and never stamped,
        # whatever the stream format is. The carputer's ISO-TP reader is on the
        # other end of this and knows nothing about LAWICEL.
        self.raw_send("%03X%s\r" % (can_id, data.hex().upper()))

    # -- the frame stream ---------------------------------------------------
    def lawicel(self):
        return self.stream >= SM_LAW

    def ts_chars(self):
        if self.stream == SM_OFF:
            return 0
        return TS_CHARS[(self.stream - 1) % 3]

    def elm_level(self):
        # Truthful about the adapter shape only: in a LAWICEL mode that stream
        # really is off, and lawicel_level() says the same the other way round.
        return self.stream if self.stream <= SM_ELM + 2 else 0

    def lawicel_level(self):
        return self.stream - (SM_LAW - 1) if self.lawicel() else 0

    def stamp(self):
        width = self.ts_chars()
        if not width:
            return ""
        ms = int((time.monotonic() - self.mono0) * 1000.0)
        return "%0*X" % (width, ms % 60000 if width == 4 else ms % (1 << 32))

    def stream_line(self, cid, dlc, data):
        """One unsolicited frame, in whatever format the LIVE level selects.

        Formatted here rather than at load time because the level changes
        mid-replay: the broadcast thread is already running when the carputer
        sends its AT@SML2. The LAWICEL line is space-free and CR-terminated
        whatever ATS/ATL said, which is a rule of the format and not a
        shortcut -- an slcan parser splits on CR, so a trailing LF arrives as a
        LEADING LF on the next line and eats a whole frame.
        """
        if self.lawicel():
            return "t%s%X%s%s\r" % (cid, dlc, data, self.stamp())
        return "%s%s%s\r" % (cid, data, self.stamp())

    def send_stream(self, cid, dlc, data):
        with self.wlock:
            self.raw_send(self.stream_line(cid, dlc, data))

    def reply(self, body=None):
        self.raw_send((body + "\r" if body else "") + ">")

    def note(self, text):
        if self.verbose:
            # Newline-terminated, not '\r'-overwritten: the console prompt is
            # sharing this terminal and a half-erased line is worse than a
            # scrolling one.
            sys.stdout.write("  %s\n" % text)
            sys.stdout.flush()

    # -- serve --------------------------------------------------------------
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
                        self.on_line(line)
                elif len(buf) < 64:
                    buf.append(byte)

    def on_line(self, line):
        up = line.upper()
        if up.startswith("AT"):
            self.at_command(up)
            return
        # Not AT: a bare hex frame aimed at whatever ATSH last selected. This
        # is the half a capture replay cannot do.
        try:
            frame = bytes.fromhex(up)
        except ValueError:
            self.reply("?")
            return
        payload = isotp_unwrap_request(frame)
        if payload is None:
            self.note("unhandled request frame %s" % up)
            return
        self.on_request(payload)

    def at_command(self, up):
        if up.startswith("AT@ST"):
            self.reply(STATUS % (int(time.time() - self.t0),
                                 self.elm_level(), self.lawicel_level()))
        # AT@SML BEFORE AT@SM: a prefix test on "AT@SM" matches "AT@SML1" too
        # and hands the level parser "L1", so AT@SML1 would answer '?' and the
        # command the carputer actually uses would look unimplemented. Unlike
        # AT@ST vs AT@SM, where either order works, this order is load-bearing.
        elif up.startswith("AT@SML"):
            self.stream_cmd(up[6:], SM_LAW)
        elif up.startswith("AT@SM"):
            self.stream_cmd(up[5:], SM_ELM)
        elif up in ("ATI", "ATZ"):
            self.sh = self.cra = None
            if up == "ATZ":
                # ATZ is resetDefaults on the board and the stream level is one
                # of the defaults, so a fresh session is always silent and
                # adapter-shaped. ATI is an identity query and resets nothing.
                self.stream = SM_OFF
            self.reply(IDENT)
        elif up.startswith("ATSH"):
            try:
                self.sh = int(up[4:], 16)
            except ValueError:
                self.reply("?")
                return
            self.reply("OK")
        elif up.startswith("ATCRA"):
            arg = up[5:]
            # Bare ATCRA clears the filter -- that is what monitorBegin() does
            # before ATMA, so it must not be read as a malformed id.
            try:
                self.cra = int(arg, 16) if arg else None
            except ValueError:
                self.reply("?")
                return
            self.reply("OK")
        else:
            self.reply("OK")

    def stream_cmd(self, arg, base):
        """AT@SM / AT@SML, both the query and the set. `base` is the level-1
        StreamMode of the format being addressed.

        A bare command is a query and prints one digit. Exactly one digit 0..3
        sets the level; anything else answers '?' rather than being clamped, so
        AT@SM4 reads as the typo it is instead of quietly behaving like AT@SM1
        -- and answering '?' is also the path At328Link's fallback needs.
        """
        if not arg:
            self.reply("%d" % (self.lawicel_level() if base == SM_LAW
                               else self.elm_level()))
            return
        if len(arg) != 1 or arg not in "0123":
            self.reply("?")
            return
        level = int(arg)
        # One field, so selecting either format turns the other off for free.
        self.stream = SM_OFF if level == 0 else base + level - 1
        self.reply("OK")

    # -- the modules --------------------------------------------------------
    def on_request(self, payload):
        self.counts["req"] += 1
        if self.sh is None:
            self.note("request %s with no ATSH -- ignored" % payload.hex().upper())
            return
        name = self.by_req.get(self.sh)
        if name is None:
            self.note("request to unknown id %03X -- silent" % self.sh)
            self.counts["quiet"] += 1
            return

        car = self.car
        with car.lock:
            present = car.present.get(name, False)
        if not present:
            # SILENT, not negative. This is what an absent module looks like,
            # and telling it apart from a present-but-clean one is the whole
            # reason the carputer tracks `answered` and `unreachable`
            # separately.
            self.note("%-6s <- %s   (quiet: module off)"
                      % (name, payload.hex().upper()))
            self.counts["quiet"] += 1
            return

        # "Response pending" first, if asked for. The real answer follows
        # immediately after -- which is precisely the shape that currently
        # loses it, since transact() returns on the first complete PDU.
        with car.lock:
            mode = car.pending_mode
            if mode == "once":
                car.pending_mode = "off"
        if mode in ("once", "on") and payload:
            self.respond(name, bytes([0x7F, payload[0], NRC_PENDING]))
            self.note("%-6s -> 7F %02X 78  (response pending)" % (name, payload[0]))
            time.sleep(0.01)

        reply = self.build(name, payload)
        if reply is None:
            self.note("%-6s <- %s   (silent)" % (name, payload.hex().upper()))
            self.counts["quiet"] += 1
            return
        self.respond(name, reply)
        if len(reply) >= 3 and reply[0] == 0x7F:
            self.counts["neg"] += 1
            self.note("%-6s <- %s  ->  7F %02X %02X  (refused)"
                      % (name, payload.hex().upper(), reply[1], reply[2]))
        else:
            self.counts["ans"] += 1
            self.note("%-6s <- %s  ->  %s"
                      % (name, payload.hex().upper(), reply.hex().upper()))

    def respond(self, name, payload):
        resp_id = ECUS[name][1]
        for f in isotp_frames(payload, self.pad):
            self.send_frame(resp_id, f)
            time.sleep(0.002)     # a module does not answer instantly

    def build(self, name, req):
        """The KWP reply, or None for deliberate silence."""
        if not req:
            return None
        svc = req[0]
        car = self.car

        if svc == SVC_READ_IDENT:
            # `1A 87` is an ASCII part number, and it is what probeEcu() uses
            # to tell "module absent" from "module present with nothing to
            # report". Any other option is refused rather than invented.
            if len(req) < 2 or req[1] != 0x87:
                return bytes([0x7F, svc, NRC_SUBFUNCTION_NOT_SUPPORTED])
            text = ("P%s-EMU" % name)[:11].encode("ascii")
            return bytes([0x5A, 0x87]) + text

        if svc == SVC_READ_DTC:
            # `18 00 FF 00` -> `58 <count> [hi lo status] ...`
            with car.lock:
                rows = list(car.faults.get(name, []))
            body = bytearray([0x58, len(rows)])
            for raw, status in rows:
                body += bytes([(raw >> 8) & 0xFF, raw & 0xFF, status])
            return bytes(body)

        if svc == SVC_CLEAR_DTC:
            if len(req) < 3:
                return bytes([0x7F, svc, NRC_SUBFUNCTION_NOT_SUPPORTED])
            with car.lock:
                mode = car.clear_mode
            if mode == "silent":
                return None
            if mode == "refuse":
                return bytes([0x7F, svc, NRC_CONDITIONS_NOT_CORRECT])
            code = (req[1] << 8) | req[2]
            with car.lock:
                rows = car.faults.get(name, [])
                if code == 0xFF00:
                    # The all-groups form: wipe the module. The firmware never
                    # sends this, but answering it correctly is how you find
                    # out if it ever starts to.
                    car.faults[name] = []
                else:
                    car.faults[name] = [r for r in rows if r[0] != code]
            return bytes([0x54])

        if svc == SVC_READ_LOCAL:
            if len(req) < 2:
                return bytes([0x7F, svc, NRC_SUBFUNCTION_NOT_SUPPORTED])
            return self.read_local(name, req[1])

        return bytes([0x7F, svc, NRC_SERVICE_NOT_SUPPORTED])

    def read_local(self, name, pid):
        """Service 21. Layouts taken from core/SignalDb.cpp and TpmsSignals.cpp.

        bitOffset there counts from byte 0 of the FULL response, echo included,
        so the first data byte of `61 xx ...` sits at bit 16. Getting that
        wrong here would read as a firmware bug, which is the worst kind of
        emulator error.
        """
        car = self.car
        with car.lock:
            if name == "WCM":
                if pid == 0x3B:
                    # Road wheels only -- the spare is deliberately excluded.
                    return bytes([0x61, 0x3B] + [t & 0xFF for t in car.tyres])
                if pid == 0x24:
                    # `61 24 07 4C 01 83` in the probe log: pressure at bit 16,
                    # the rest unidentified and reproduced so the layout is
                    # exercised as recorded rather than as assumed.
                    return bytes([0x61, 0x24, car.spare & 0xFF, 0x4C, 0x01, 0x83])
                if 0x20 <= pid <= 0x23:
                    # One sensor each; 0x20 is sensor 1.
                    return bytes([0x61, pid, car.tyres[pid - 0x20] & 0xFF])
            if name == "CCN":
                if pid == 0x03:
                    # Fuel % at bit 24 = byte 3, scaling x0.5.
                    return bytes([0x61, 0x03, 0x00, car.fuel_raw()])
                if pid == 0x3F:
                    return bytes([0x61, 0x3F, car.lamps, car.lamps2])
                if pid == 0x35:
                    return bytes([0x61, 0x35, car.fluids])
        return bytes([0x7F, SVC_READ_LOCAL, NRC_REQUEST_OUT_OF_RANGE])


# ---------------------------------------------------------------------------
# The captures worth replaying, by NAME.
#
# --log takes either a name from here or any path, because the path of the one
# that matters is 95 characters with spaces in it and typing it by hand is how
# a bench session ends up replaying the wrong drive. Resolved against this
# file's directory, not the caller's cwd, so the name works from anywhere.
#
#   playback  the 2015 capture: 841 s, 126k frames, no engine-crank restart.
#             What every measurement in CLAUDE.md was taken on.
#   drive     the owner's own 25 min drive, 2026-09-17: 224,709 frames over
#             1484.9 s, 48 ids, stamped throughout, and it carries the mid-
#             capture stamp RESTART (34742 -> 7709, the crank a few hundred
#             frames in) that unwrap() has to tell from a rollover. Not copied
#             into the repo -- a quarter of a million lines does not belong in
#             a firmware tree, and it is one owner's drive, not a fixture.
LOGS = {
    "playback": os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "playback.log"),
    "drive": r"D:\Repos\car\jeep-can-b-logs\drive log - becici to school"
             r" on top of the steep hill and back - 2026-09-17_074126.log",
}


def resolve_log(name):
    """A name from LOGS, or a path straight through.

    Both halves are checked HERE rather than left to open(): a mistyped name is
    a likely mistake now that names exist, and the two ways it used to surface
    were both bad. A name that is in the table but whose file is missing -- the
    drive log lives outside this repo, so a fresh clone has the name and not
    the file -- would have read as an empty capture; anything else fell through
    to a FileNotFoundError traceback, which buries "you typed drve" under a
    stack.
    """
    if name in LOGS:
        path = LOGS[name]
        if not os.path.exists(path):
            sys.exit("--log %s names %s, which is not there. That capture "
                     "lives outside this repo; pass a path instead."
                     % (name, path))
        return path
    if not os.path.exists(name):
        sys.exit("--log %s is neither a file nor one of the named captures (%s)"
                 % (name, ", ".join(sorted(LOGS))))
    return name


# ---------------------------------------------------------------------------
def parse_log(path):
    """-> [(id_hex, dlc, data_hex, stamp_ms or None)], skipping the unreadable.

    The record stays in FIELDS rather than becoming a finished wire line here,
    because the format a frame goes out in is decided by the live stream level
    at send time: the replay thread is already running when the carputer sends
    its AT@SML2, so a line baked into one shape at load time would be the wrong
    shape for the rest of the run.

    A gateway capture is CR-separated, which plain iteration handles -- Python's
    universal newlines treat a lone CR as a line end -- and its first records
    are usually terminal echo, so the unreadable ones are counted, not fatal.
    A record may also arrive with the '>' prompt still glued to its front,
    which is a real frame and must not be thrown away for its prefix.
    """
    out, skipped = [], 0
    with open(path, "r", errors="replace") as fh:
        for raw in fh:
            line = raw.strip().lstrip(">")
            if not line:
                continue
            m = LAWICEL.match(line)
            if not m:
                skipped += 1
                continue
            cid, dlc, payload = m.group(1).upper(), int(m.group(2), 16), m.group(3)
            want = dlc * 2
            stamp = None
            if len(payload) == want + 4:
                stamp = int(payload[want:], 16)
                payload = payload[:want]
            elif len(payload) != want:
                skipped += 1
                continue
            if dlc > 8:
                skipped += 1
                continue
            out.append((cid, dlc, payload.upper(), stamp))
    return out, skipped


def unwrap(frames):
    """Capture stamps -> a monotonic millisecond timeline for pacing.

    A 4-digit stamp is millis() % 60000, so it goes backwards once a minute and
    the wrap has to be added back. But it ALSO goes backwards when the gateway
    itself restarts mid-capture, and that is not a wrap: the real captures have
    one (34742 -> 7709 a few hundred frames in), and adding 60000 there froze
    the replay for 33 s over a gap that was really milliseconds. A wrap lands
    near the top of the window, a restart does not, so only a drop of more than
    half the window is treated as one; anything else keeps the timeline
    continuous across the restart.
    """
    base, prev, times = 0, None, []
    for frame in frames:
        stamp = frame[3]
        if stamp is None:
            return None
        if prev is not None and stamp < prev:
            base += 60000 if prev - stamp > 30000 else prev - stamp
        prev = stamp
        times.append(base + stamp)
    return times


def broadcast_loop(emu, frames, times, rate, speed, loop, stop):
    # Paced by the capture's own stamps when it has them, by --rate when it does
    # not; --speed divides the schedule, so a 2x replay really does put the
    # frames on the wire twice as fast and the stamps the emulator writes halve
    # with it, because they are read off its clock as each line goes out.
    gap = 1.0 / (rate * speed)
    while not stop.is_set():
        start = time.time()
        for i, (cid, dlc, data, _) in enumerate(frames):
            if stop.is_set():
                return
            emu.send_stream(cid, dlc, data)
            if times:
                due = start + (times[i] - times[0]) / 1000.0 / speed
                delay = due - time.time()
                if delay > 0:
                    time.sleep(delay)
            else:
                time.sleep(gap)
        if not loop:
            return


# ---------------------------------------------------------------------------
class _NullPort(object):
    """Enough of a serial port to collect what the emulator would have sent."""

    def __init__(self):
        self.out = []

    def write(self, data):
        self.out.append(data.decode("ascii", "replace"))

    def read(self, _n):
        return b""

    def close(self):
        pass


# The exact questions the firmware asks: the poll plan, the DTC read, the
# bring-up ident probe, plus a clear and a deliberately bad PID so the
# negative-response path is covered too.
DUMP_SCENARIOS = (
    ("tyres",   "WCM", "213B"),
    ("spare",   "WCM", "2124"),
    ("fuel",    "CCN", "2103"),
    ("lamps",   "CCN", "213F"),
    ("fluids",  "CCN", "2135"),
    ("dtc_wcm", "WCM", "1800FF00"),
    ("dtc_ccn", "CCN", "1800FF00"),
    ("ident",   "WCM", "1A87"),
    ("clear",   "WCM", "14C077"),
    ("badpid",  "WCM", "2199"),
)


def dump_frames(path):
    car = Car()
    emu = Emulator(_NullPort(), car, pad=True, verbose=False)
    with open(path, "w") as fh:
        for name, ecu, req in DUMP_SCENARIOS:
            emu.sh = ECUS[ecu][0]
            emu.port.out = []
            # Through the same unwrap the serial path uses, so the verifier
            # exercises the PCI handling rather than stepping around it. The
            # first version of this dump fed build() directly and therefore
            # could not have caught the bug above -- a verifier that generates
            # its own input in the wrong format verifies nothing.
            body = bytes.fromhex(req)
            wire = bytes([len(body)]) + body
            payload = isotp_unwrap_request(wire)
            if payload is None:
                raise SystemExit("cannot wrap %s" % req)
            emu.on_request(payload)
            fh.write("SCENARIO %s %s %d\n" % (name, req, ECUS[ecu][1]))
            for chunk in emu.port.out:
                wire = chunk.strip()
                if wire and not wire.endswith(">"):
                    fh.write("FRAME %s\n" % wire)
    print("wrote %s (%d scenarios)" % (path, len(DUMP_SCENARIOS)))
    print("check it with:  clang++ -std=gnu++14 -Icore scripts/emu-verify.cpp \\")
    print("                  core/*.cpp -o emu-verify && ./emu-verify %s" % path)


# ---------------------------------------------------------------------------
HELP = """
  show                      what the car currently looks like
  tyre <1-4|spare> <psi>    set one pressure        e.g.  tyre 2 24
  fuel <percent>            set the fuel level      e.g.  fuel 12
  lamps <hex> [hex]         0x213F bytes 2 and 3    e.g.  lamps 80 00
  fluids <hex>              0x2135 byte 2           e.g.  fluids 05

  dtc list [ecu]            the fault memory
  dtc add <ecu> <CODE> [stored|active|lamp]         e.g.  dtc add ccn P0301 active
  dtc del <ecu> <CODE>
  dtc clear [ecu]           empty one module, or all

  ecu on|off <name>         make a module answer, or go quiet
  ecu list

  clear accept|refuse|silent    how service 0x14 replies
  pending off|once|on           prepend 7F xx 78 to replies

  quit
"""


def control_file_loop(emu, car, path, stop):
    """Run console commands written to a file, then empty it.

    The prompt is the right interface for a person and the wrong one for
    anything else: a background run has no terminal to type into, and a
    scripted bench check wants to say "clear refuse", look at the screen, then
    say "clear accept". This gives both the same vocabulary, so there is one
    set of commands to learn and one to document.

    Truncated after each batch rather than tracked by offset, so
    `echo "tyre 2 20" > ctl.txt` works from any shell without anyone having to
    think about append versus overwrite.
    """
    while not stop.is_set():
        try:
            if os.path.exists(path) and os.path.getsize(path) > 0:
                with open(path, "r") as fh:
                    lines = fh.read().splitlines()
                open(path, "w").close()
                for raw in lines:
                    cmd = raw.strip()
                    if cmd and not cmd.startswith("#"):
                        sys.stdout.write("ctl> %s\n" % cmd)
                        sys.stdout.flush()
                        run_command(emu, car, cmd)
        except Exception as exc:
            sys.stdout.write("ctl error: %s\n" % exc)
            sys.stdout.flush()
        time.sleep(0.25)


def console(emu, car, stop):
    def out(s=""):
        sys.stdout.write(s + "\n")
        sys.stdout.flush()

    def find_ecu(token):
        t = token.upper()
        if t in ECUS:
            return t
        out("  no module called %r (try: ecu list)" % token)
        return None

    def show():
        with car.lock:
            out("  tyres  %s psi   spare %d psi"
                % ("  ".join("#%d %d" % (i + 1, p) for i, p in enumerate(car.tyres)),
                   car.spare))
            out("  fuel   %.1f%%   lamps %02X %02X   fluids %02X"
                % (car.fuel_pct, car.lamps, car.lamps2, car.fluids))
            on = [n for n in ECUS if car.present.get(n)]
            out("  awake  %s" % (" ".join(sorted(on)) or "(none)"))
            total = 0
            for name in sorted(car.faults):
                rows = car.faults[name]
                if not rows or not car.present.get(name):
                    continue
                total += len(rows)
                out("  %-6s %s" % (name, "  ".join(
                    "%s(%s)" % (raw_to_sae(c), status_text(s)) for c, s in rows)))
            out("  faults %d stored on awake modules" % total)
            out("  clear=%s  pending=%s" % (car.clear_mode, car.pending_mode))
        c = emu.counts
        out("  served %d requests: %d answered, %d refused, %d quiet"
            % (c["req"], c["ans"], c["neg"], c["quiet"]))

    out("type 'help' for commands, 'quit' to stop\n")
    while not stop.is_set():
        try:
            line = input("car> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        if run_command(emu, car, line):
            break
    stop.set()


def run_command(emu, car, line):
    """Execute one console command. Returns True if it asked to quit."""
    def out(s=""):
        sys.stdout.write(s + "\n")
        sys.stdout.flush()

    def find_ecu(token):
        tk = token.upper()
        if tk in ECUS:
            return tk
        out("  no module called %r (try: ecu list)" % token)
        return None

    def show():
        with car.lock:
            out("  tyres  %s psi   spare %d psi"
                % ("  ".join("#%d %d" % (i + 1, p)
                             for i, p in enumerate(car.tyres)), car.spare))
            out("  fuel   %.1f%%   lamps %02X %02X   fluids %02X"
                % (car.fuel_pct, car.lamps, car.lamps2, car.fluids))
            awake = [n for n in ECUS if car.present.get(n)]
            out("  awake  %s" % (" ".join(sorted(awake)) or "(none)"))
            total = 0
            for nm in sorted(car.faults):
                rows = car.faults[nm]
                if not rows or not car.present.get(nm):
                    continue
                total += len(rows)
                out("  %-6s %s" % (nm, "  ".join(
                    "%s(%s)" % (raw_to_sae(c), status_text(s)) for c, s in rows)))
            out("  faults %d stored on awake modules" % total)
            out("  clear=%s  pending=%s" % (car.clear_mode, car.pending_mode))
        c = emu.counts
        out("  served %d requests: %d answered, %d refused, %d quiet"
            % (c["req"], c["ans"], c["neg"], c["quiet"]))

    parts = line.split()
    cmd = parts[0].lower()
    try:
        if cmd in ("quit", "exit", "q"):
            return True
        elif cmd in ("help", "?", "h"):
            out(HELP)
        elif cmd in ("show", "s"):
            show()

        elif cmd == "tyre" and len(parts) == 3:
            psi = int(parts[2])
            with car.lock:
                if parts[1].lower().startswith("sp"):
                    car.spare = psi
                else:
                    idx = int(parts[1]) - 1
                    if not 0 <= idx < 4:
                        out("  sensor must be 1-4, or 'spare'")
                        return False
                    car.tyres[idx] = psi
            out("  ok -- next WCM poll carries it (the plan asks once a minute)")

        elif cmd == "fuel" and len(parts) == 2:
            with car.lock:
                car.fuel_pct = float(parts[1])
            out("  ok")

        elif cmd == "lamps" and len(parts) in (2, 3):
            with car.lock:
                car.lamps = int(parts[1], 16)
                if len(parts) == 3:
                    car.lamps2 = int(parts[2], 16)
            out("  ok")

        elif cmd == "fluids" and len(parts) == 2:
            with car.lock:
                car.fluids = int(parts[1], 16)
            out("  ok")

        elif cmd == "dtc" and len(parts) >= 2:
            sub = parts[1].lower()
            if sub == "list":
                with car.lock:
                    names = [find_ecu(parts[2])] if len(parts) > 2 else sorted(car.faults)
                    for name in names:
                        if not name:
                            continue
                        rows = car.faults.get(name, [])
                        out("  %-6s %s" % (name, "  ".join(
                            "%s(%s)" % (raw_to_sae(c), status_text(s))
                            for c, s in rows) or "(clean)"))
            elif sub == "add" and len(parts) >= 4:
                name = find_ecu(parts[2])
                if not name:
                    return False
                raw = sae_to_raw(parts[3])
                st = ST_STORED
                if len(parts) > 4:
                    w = parts[4].lower()
                    st = {"stored": ST_STORED,
                          "active": ST_ACTIVE | ST_LAMP,
                          "lamp": ST_STORED | ST_LAMP}.get(w, ST_STORED)
                with car.lock:
                    car.faults.setdefault(name, [])
                    if any(c == raw for c, _ in car.faults[name]):
                        out("  %s already has %s" % (name, raw_to_sae(raw)))
                        return False
                    car.faults[name].append((raw, st))
                out("  %s += %s (%s) -- RE-READ on the FAULTS page, or wait "
                    "5 min for the automatic scan" % (name, raw_to_sae(raw),
                                                      status_text(st)))
            elif sub == "del" and len(parts) >= 4:
                name = find_ecu(parts[2])
                if not name:
                    return False
                raw = sae_to_raw(parts[3])
                with car.lock:
                    before = len(car.faults.get(name, []))
                    car.faults[name] = [r for r in car.faults.get(name, [])
                                        if r[0] != raw]
                    gone = before - len(car.faults[name])
                out("  %s" % ("removed" if gone else "not there"))
            elif sub == "clear":
                with car.lock:
                    if len(parts) > 2:
                        name = find_ecu(parts[2])
                        if not name:
                            return False
                        car.faults[name] = []
                        out("  %s cleared" % name)
                    else:
                        car.faults = {}
                        out("  all modules cleared")
            else:
                out("  dtc list|add|del|clear -- see help")

        elif cmd == "ecu" and len(parts) >= 2:
            sub = parts[1].lower()
            if sub == "list":
                with car.lock:
                    for name in sorted(ECUS):
                        req, resp, _ = ECUS[name]
                        out("  %-6s %03X/%03X  %s"
                            % (name, req, resp,
                               "awake" if car.present.get(name) else "quiet"))
            elif sub in ("on", "off") and len(parts) == 3:
                name = find_ecu(parts[2])
                if not name:
                    return False
                with car.lock:
                    car.present[name] = (sub == "on")
                out("  %s is now %s" % (name, "awake" if sub == "on" else "quiet"))
            else:
                out("  ecu on|off <name>, or ecu list")

        elif cmd == "clear" and len(parts) == 2:
            mode = parts[1].lower()
            if mode not in ("accept", "refuse", "silent"):
                out("  clear accept|refuse|silent")
                return False
            with car.lock:
                car.clear_mode = mode
            out("  service 0x14 will %s" % {
                "accept": "erase the code and answer 54",
                "refuse": "answer 7F 14 22 -- the button should end up FAILED",
                "silent": "not answer at all -- the button should time out"}[mode])

        elif cmd == "pending" and len(parts) == 2:
            mode = parts[1].lower()
            if mode not in ("off", "once", "on"):
                out("  pending off|once|on")
                return False
            with car.lock:
                car.pending_mode = mode
            if mode != "off":
                out("  replies will be preceded by 7F xx 78. Expect the real"
                    " answer to be LOST -- that is the known hole in"
                    " IsoTpClient::transact(), not a fault in the emulator.")
            else:
                out("  ok")
        else:
            out("  ?  -- try 'help'")
    except ValueError as exc:
        out("  %s" % exc)
    except Exception as exc:                       # keep the bench alive
        out("  error: %s" % exc)
    return False


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", metavar="NAME|PATH",
                    help="LAWICEL capture to stream as broadcasts, "
                         "re-formatted into whatever AT@SML/AT@SM level the "
                         "carputer asked for (optional: without it only polled "
                         "data works). Either a path, or one of the named "
                         "captures: " + ", ".join(sorted(LOGS)))
    ap.add_argument("--rate", type=float, default=150.0,
                    help="frames/sec when the capture has no timestamps "
                         "(default 150, the measured rate of this bus)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="replay speed multiplier; >1 compresses time and will "
                         "make the fuel integral wrong, because dt is measured "
                         "on arrival rather than read from the log")
    ap.add_argument("--loop", action="store_true", help="restart the log at the end")
    ap.add_argument("--no-pad", action="store_true",
                    help="do not pad response frames to 8 bytes (real modules do)")
    ap.add_argument("--quiet", action="store_true",
                    help="do not print each request/response")
    ap.add_argument("--no-console", action="store_true",
                    help="skip the interactive prompt (for scripted runs)")
    ap.add_argument("--control", metavar="FILE",
                    help="also take console commands from FILE, emptying it "
                         "after each batch -- for background runs and scripted "
                         "bench checks. Same vocabulary as the prompt.")
    ap.add_argument("--dump-frames", metavar="FILE",
                    help="answer the requests the firmware asks, write the wire "
                         "lines to FILE and exit -- input for scripts/emu-verify.cpp, "
                         "which decodes them with the firmware's own core/ and so "
                         "catches this emulator disagreeing with the dashboard "
                         "about a layout. No serial port is opened.")
    args = ap.parse_args()

    if args.dump_frames:
        dump_frames(args.dump_frames)
        return

    frames, times = [], None
    if args.log:
        log = resolve_log(args.log)
        frames, skipped = parse_log(log)
        if not frames:
            sys.exit("no LAWICEL frames in %s" % log)
        times = unwrap(frames)
        span = (times[-1] - times[0]) / 1000.0 if times else len(frames) / args.rate
        print("%s: %d frames, %.1f s (%.0f f/s)%s"
              % (os.path.basename(log), len(frames), span,
                 len(frames) / span if span else 0,
                 "  [%d lines skipped]" % skipped if skipped else ""))
        print("pacing: %s" % ("capture timestamps" if times
                              else "%.0f f/s (no timestamps)" % args.rate))
    else:
        print("no --log: broadcasts are absent, so speed, rpm, flow and the trips")
        print("will stay dashed. Polled pages (TYRES, HEALTH, FAULTS) work fully.")

    car = Car()
    port = serial.Serial(args.port, args.baud, timeout=0.05)
    emu = Emulator(port, car, pad=not args.no_pad, verbose=not args.quiet)

    stop = emu.stop
    threading.Thread(target=emu.serve, daemon=True).start()
    if args.control:
        open(args.control, "w").close()
        threading.Thread(target=control_file_loop,
                         args=(emu, car, args.control, stop),
                         daemon=True).start()
        print("control file: %s  -- echo commands into it" % args.control)
    if frames:
        threading.Thread(target=broadcast_loop,
                         args=(emu, frames, times, args.rate, args.speed,
                               args.loop, stop),
                         daemon=True).start()

    print("emulating on %s at %d -- %d modules awake\n"
          % (args.port, args.baud, sum(1 for n in ECUS if car.present.get(n))))
    try:
        if args.no_console:
            while not stop.is_set():
                time.sleep(0.2)
        else:
            console(emu, car, stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        time.sleep(0.05)
        port.close()
        print("stopped")


if __name__ == "__main__":
    main()
