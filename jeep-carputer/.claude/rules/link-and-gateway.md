---
paths:
  - "src/At328Link.*"
  - "core/LinkLine.*"
  - "core/StreamClock.*"
  - "core/LinkHealth.*"
  - "core/BusCycle.*"
  - "core/IsoTp.*"
  - "scripts/emulate-car.py"
  - "scripts/emu-verify.cpp"
---

# The link to the 328, and the two stream formats

The gateway is a dumb frame pipe with an ELM327-style AT surface
(`../jeep-aux-and-can-gateway/docs/AT-COMMANDS.md`). ISO-TP, KWP and decoding
happen here.

## Bandwidth

Interior CAN-B at 83.3 kbps, measured at **151 frames/s** (152.2 over a 1449.6 s
capture of 220,669 frames) — **~30 % of the 115200 link**. That was 22 % before
the timestamps: a stamped LAWICEL line under `AT@SML2` is **22.68 bytes** mean
against the ELM line's 16.68, so **3.45 kB/s** where `AT@SM1` cost 2.5, and
`AT328_RX_BUF`'s 4096 covers **1.2 s** of stall instead of 1.6. Any doc still
saying ~22 % is describing the old stream.

**Peak-to-mean is 6.3× at a 10 ms scale.** The worst 10 ms window carries 218
bytes against a 115-byte budget, which the link cannot pass and the RX buffer
absorbs; at 50 ms the worst window is already at 90 % of line rate. **The next
byte added to a frame line needs a faster baud, not a bigger buffer.**

## Bring-up

- **The ladder is THREE rungs and each is a real adapter:** `AT@SML2` (stamped
  LAWICEL — both halves of the link live at once AND the reader can CHECK the
  stamp width), then `AT@SM1` (a gateway that knows `AT@SM` but not `AT@SML`:
  overlap kept, timestamps not), then ATMA windows on a 2.0 board.
  `setStreamFormat(LinkWire, uint8_t)` replaced `setStreamLevel(uint8_t)`
  because the two parsers disagree about where a frame's data ends, so the
  reader must know which one it is holding before it classifies a line.
- **Streaming is asked FIRST and `monitorBegin()` is not attempted alongside
  it.** ATMA would make a 2.1 adapter deaf to the requests the loop still wants
  to send, in exchange for frames that are already arriving. The failure it
  guards against is not a wrong number — it is the poll path going silent on the
  one adapter that supports everything.
- **`begin()` SILENCES THE STREAM BEFORE IT SENDS `ATZ`.** The 328 is flashed
  over ICSP and has no bootloader on these pins, so opening the port does not
  reset it: it may be mid-stream from a previous session in a format this
  session has not agreed to. Those lines would be classified against the wrong
  parser and filed as command replies into a queue **eight slots deep** — where
  `ATZ`'s own answer has to arrive — so the handshake starves inside
  `command()`'s 16-line budget and the link comes up DOWN for no visible reason.
  `streamStop()` sends both `AT@SML0` and `AT@SM0` because which one was running
  is exactly what we do not know yet.
- **NEVER `uart_flush_input()` once the reader task owns the port.**
  `uart_read_bytes()` HOLDS the driver's rx mutex for the whole time it is
  blocked waiting, `uart_flush_input()` wants the same mutex, and `at328rx` runs
  at priority 6 against `carTask`'s 5 **on the same core** — so it re-takes the
  mutex the instant it is released and the flush never wins. `carTask` stops
  inside `begin()` having sent three bytes. **The failure mode is why this cost
  hours**: it hangs BEFORE `ATZ`, so there is no `no response to ATZ`, no `link
  not ready, retrying`, no `adapter ready` — the console is silent and the panel
  dashes for ever, exactly like an unplugged gateway. **The rule lives inside
  `drain()`** — flush bytes while we own the port, reset `replyQ_`/`frameQ_` once
  the reader does — so there is no third site to miss. The five `ESP_LOGD`
  breadcrumbs through `begin()` are kept for the same reason: loud at both exits,
  silent in between.
- **`ready()` is not liveness.** It stays true after the cable is pulled; the
  watchdog uses silence plus an `AT@ST` probe and re-runs `begin()` to recover.
- **The link is single-requester by construction** (only `carTask` transacts).
  No mutex — a lock inside each method would not make a *transaction* atomic.

## Two parsers in `core/LinkLine.cpp`

`linkClassifyLine()` returns EMPTY / STREAM / FRAME / REPLY.
`linkParseStreamLine()` reads the LAWICEL stream; `linkParseFrameLine()` reads
the ELM-shaped request RESPONSE, which is NEVER stamped whatever the stream
format is, because LAWICEL has no request/response notion.

**The asymmetry between them is the entire reason the wire changed.** The ELM
shape could not express its own stamp width: length is `3 + 2n` unstamped and
`3 + 2n + 4` stamped, and those overlap — a 19-character line is *either* 8 data
bytes *or* 6 plus a stamp, both pure hex, both internally consistent. The width
had to be TOLD to the parser with no way to check it, and getting it wrong was
silent in both directions. A LAWICEL line carries a **DLC nibble**, so
`tsChars = len - 5 - 2*dlc` is an ARITHMETIC IDENTITY and the width is an
**output**. The reader compares the derived width against the level it asked for
and counts disagreements (`At328Link::stampWidthMismatches()`, zero being the
only healthy value) — where before, the same disagreement was unobservable and
surfaced as a dashboard full of plausible wrong numbers.

Classification order matters: a leading `t` is not a hex digit and nothing else
the firmware emits starts with one, so STREAM is tested first with one character
compare, and a `t` line failing the shape check is a REPLY, never retried as
ELM. `test_linkline` carries differential **ACCEPT IFF DECODE** sweeps — 12,657
cases across all three stamp widths for the ELM parser, 984 generated cases
asked three times for the stream parser.

## Two clocks

**Which clock a stamp is on is not guessable from its type.** Under `AT@SML2` a
**broadcast** frame reaches `onBroadcastFrame()` with the time the GATEWAY saw
it — the reader loop calls `sink_(f, streamClockMap(...))`. **Polled** values
stay on the host clock, as do `lastRxMs()` and everything `core/LinkHealth`
reads. The two consumers want opposite things: DIFFERENCES must be the
adapter's, because `Trip` integrates rate × interval and `Accel` times `stamp -
start`; ABSOLUTE values must be ours, because every staleness test is
`nowMs() - someMs < TTL` and a foreign epoch makes the panel read stale for ever.

**The offset is a MINIMUM, not an average, and that is correctness rather than
filtering preference.** `host = adapter + offset + latency`, latency is never
negative, so the smallest observed `host - adapter` is the best estimate. Take
the mean and half of all frames map LATER than the host clock reading — and
`nowMs() - stampMs` is unsigned everywhere, so a stamp one millisecond in the
future underflows to 4.29 billion and reads as maximally stale. A pedal that
blanks at random. `core/StreamClock` guarantees **`mapped <= hostMs`** and
**`mapped` never decreases**, by construction, and no caller checks either. The
offset creeps UP at ~977 ppm so genuine crystal drift is tracked (the adapter
measures +32 ppm against the vehicle's scheduler), and a gateway reboot needs no
special case — a minimum filter takes the corrected offset in one sample.

**`unwrap()` needs a rollover-vs-restart guard.** A backwards step in a rolling
stamp is a rollover only if it drops more than HALF the 60000 window; a smaller
backwards step is the gateway having RESTARTED. Real captures contain exactly
that event (the drive log has 24 genuine wraps and an engine-crank restart at
frame 613, `34742 -> 7709`), and reading it as a rollover advances the clock by
a minute — 33 s of frozen replay is what found it.

## Desk testing: `scripts/emulate-car.py`

It emulates the 328 AND the modules behind it, so the POLLED half of the
dashboard works at a desk.

```bash
python scripts/emulate-car.py --port COM5                      # requests only
python scripts/emulate-car.py --port COM5 --log playback --loop
python scripts/emulate-car.py --port COM5 --log drive          # the real drive
```

The log is OPTIONAL — without it speed, rpm, flow and the trips stay dashed.
At the prompt: `tyre 2 24`, `dtc add wcm C0077`, `dtc clear ccn`, `ecu off ccn`,
`show`, `help`.

**`--log` takes a NAME or a path.** `LOGS` maps `playback` to the 2015 capture
most measurements were taken on and `drive` to the owner's own 25-minute
timestamped drive of 2026-09-17 (224,709 frames, 1484.9 s, 48 ids, 151.3 f/s).
That file lives in `../../jeep-can-b-logs/` and is deliberately NOT copied in: a
quarter of a million lines is not a fixture, and a name resolving to a missing
file says so instead of reporting an empty capture. **15,095 of its lines are
skipped and that is correct**: nine are handshake and reconnect, the rest are
~14.9k `?` command replies the gateway emitted in the last three minutes,
interleaved through frame lines.

- **The two awkward modes are the point of it.** `clear refuse` makes service
  0x14 answer `7F 14 22`; `pending once` prepends `7F xx 78` — the two paths
  recorded as UNVERIFIED and as a KNOWN HOLE, neither reachable from a broadcast
  capture. The clear button should end up saying `FAILED`; the pending reply
  should demonstrably LOSE the real answer.
- **`ecu off <name>` is how "quiet" gets told from "clean".** An absent module is
  answered with SILENCE, not a negative reply — the only way to see HEALTH say
  `none (3 asked)` honestly.
- **All eight stream levels are implemented**, `AT@SM0..3` and `AT@SML0..3`,
  mutually exclusive in ONE level field exactly as the gateway keeps it, with
  queries truthful about their own format only, the `SM=`/`SML=` fields of
  `AT@ST`, and `ATZ` clearing the level. The `AT@SML`-before-`AT@SM` prefix order
  is the trap the gateway documents against itself — match the short one first
  and the level parser is handed `"L1"`.
- **Stamps are formatted at SEND time**, four hex digits of the emulator's own
  `ms % 60000`, so a level change mid-replay takes effect and two consecutive
  lines can never walk backwards. A replayed capture's own stamps PACE the
  replay and do nothing else; copying them onto a line emitted seconds later is
  the one dishonest thing available here and nothing downstream could catch it.

**`scripts/emu-verify.cpp` checks the emulator against the firmware.** An
emulator that disagrees about a response layout is worse than none — the page
stays dashed and the hunt happens in the firmware. `--dump-frames` writes its
answers out and the harness decodes them with the real `core/`
(`linkParseFrameLine` → `IsoTpReassembler` → `kwpParseResponse` →
`pollDecodeResponse`). It covers the response path only; `test_linkline` and
`test_streamclock` cover the broadcast path. Verified: 33/29/34/33 psi with
sensor 2 flagged low, spare 7, fuel 62.0 %, `C0077` resolving to "Low Tire
Pressure". **The `bitOffset` trap is what it guards**: offsets count from byte 0
of the FULL response, echo included, so the first pressure sits at bit 16 — get
it wrong and all four corners shift two bytes while looking plausible.
