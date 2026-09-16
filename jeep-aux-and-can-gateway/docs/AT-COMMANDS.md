# AT command reference

The board is an ATmega328 inside the radio of a 2005 Jeep Grand Cherokee WH, spliced
into the **interior CAN-B bus at 83.333 kbps**. Its day job (VES/AUX announce, BT power,
panel buttons) runs by itself. On top of that it exposes an ELM327-style AT command
surface on the hardware UART, so a laptop -- later an ESP32-S3 with a touch display --
can send raw CAN frames and read what comes back.

The board is a **dumb frame pipe**. It does not reassemble ISO-TP, does not parse
KWP2000, does not decode PIDs. It transmits the bytes you type and prints the frames it
receives. All interpretation happens on the host.

---

## 1. Connecting

| | |
|---|---|
| Port | hardware UART, pins 0 (RX) / 1 (TX) |
| Baud | 115200, 8N1, no flow control |
| Logic level | **5 V** |

Wiring to a USB-TTL adapter:

| Adapter | Board |
|---|---|
| TX | RX (pin 0) |
| RX | TX (pin 1) |
| GND | GND -- common ground is mandatory |
| VCC | leave disconnected when the board is powered from the car |

**Set the adapter to 5 V logic** (most CP2102/FT232/CH340 boards have a 3.3/5 V jumper)
or put a level shifter in between. A 3.3 V adapter driving the board's RX is marginal at
best, and the board's 5 V TX into a 3.3 V-only input can damage the adapter.

Two consequences of flashing over ICSP instead of a serial bootloader:

* Pins 0/1 belong to the AT link full time. There is no bootloader listening on them.
* **The board is not reset when you open the port.** Whatever a previous session left
  behind -- headers, filters, echo, bench mode -- is still in effect. Start every session
  with `ATZ`.
* `ATZ` resets the AT layer only and never re-initialises the CAN controller. Re-initialising
  takes the node off the bus for ~100 ms, during which VES announce cannot transmit; a host
  that resets in a loop -- exactly what a serial tool does when it cannot parse the greeting --
  would stall the announce, the head unit would drop its AUX source, and the music would stop.
  Controller health is the firmware's own business. If the bus really is wedged, the on-board
  supervisor re-initialises it after 3 s of silence without being asked.
* There is **no `Serial.print` debugging any more**. The old firmware narrated itself on
  this same UART; that is gone, because the UART is now a protocol port. Diagnostics come
  out of band as `AT@EV` event lines and on demand via `AT@ST`.

---

## 2. Command syntax

* One command per line, terminated by CR, LF or CRLF.
* Case-insensitive. Spaces inside a command are ignored (`ATSH 6A0` == `atsh6a0`).
* Maximum accepted line length is `AT_LINE_MAX` = 48 characters
  (`lib/Core/Config.h`). Longer lines are rejected.
* Lines starting with `AT` are commands. Anything else is treated as hex data to
  transmit.
* After every response the adapter prints the `>` prompt to say it is ready for the next
  line.

---

## 3. Command table

### Adapter and link

| Command | Meaning |
|---|---|
| `ATZ` | Reset the adapter: defaults restored, identity string printed. Does **not** re-initialise the CAN controller -- see the note below. |
| `ATI` | Print the identity string (`FW_IDENT`; see [section 7](#7-ati-and-adapter-detection)). |
| `ATE0` / `ATE1` | Command echo off / on. Off for machine parsing, on when typing by hand. |
| `ATL0` / `ATL1` | Linefeeds off / on. Use `ATL1` if your terminal puts everything on one line. |
| `ATS0` / `ATS1` | Spaces between output hex bytes off / on. `ATS1` to read by eye, `ATS0` to parse. |
| `ATH0` / `ATH1` | Response headers off / on. **`ATH1` for all probing** -- without it you cannot tell which id a frame came from. |

### Framing

| Command | Meaning |
|---|---|
| `ATCAF0` | CAN auto-formatting **off**: raw frames both ways, you supply the ISO-TP PCI byte. |
| `ATCAF1` | CAN auto-formatting on: the adapter adds the single-frame PCI byte for you. |

### Addressing and filtering

| Command | Meaning |
|---|---|
| `ATSH hhh` | Set the transmit header: the 11-bit id your data lines go out on. E.g. `ATSH 6A0` to talk to the cluster. |
| `ATCRA hhh` | Set the receive address: only frames with this id are printed. With no argument, clears it. |
| `ATCF hhh` | Set the receive filter pattern (used together with `ATCM`). |
| `ATCM hhh` | Set the receive filter mask; a 1 bit means "this bit must match `ATCF`". `ATCM 000` accepts everything. |

These filters are applied **in software, inside the adapter** -- they are never pushed
into the MCP2515's hardware acceptance masks, which stay wide open for the life of the
firmware. That is deliberate: the board's own VES/AUX and Bluetooth logic needs `0x09F`
and `0x394` at all times, and a hardware filter set for a diagnostic session would stop
those frames reaching the controller at all, freezing the Bluetooth power pin and killing
the seek buttons until the filter was cleared. Filtering in software costs nothing
measurable on an 83.3 kbps bus and keeps the car independent of whatever the host is
doing. Practical consequence: a filter changes only what you are *shown*, takes effect
immediately, and can never disturb the audio.

`ATCRA hhh` is shorthand for `ATCF hhh` + `ATCM 7FF`. Use `ATCRA` for single-module
probing, and `ATCF`/`ATCM` when you want a range -- e.g. `ATCF 500` / `ATCM 7F0` to
watch every `50x` response id at once.

### Flow control

| Command | Meaning |
|---|---|
| `ATFCSH hhh` | Id the flow-control frame is transmitted on. Normally the same as `ATSH`. |
| `ATFCSD hh hh hh` | Flow-control data bytes. Default `30 FF 00`: clear to send, no block limit, no separation time. |
| `ATFCSM n` | Flow-control mode: `0` auto (adapter builds the FC frame), `1` use `ATFCSD` data with the `ATFCSH` id, `2` use `ATFCSD` data with the auto id. |

### Timing

| Command | Meaning |
|---|---|
| `ATST hh` | Response timeout, ELM327 units of 4 ms (`ATST 64` = 400 ms). `ATST 00` restores the default `AT_RESP_TIMEOUT_MS` = 200 ms. Raise it for modules that answer `7F xx 78`. |
| `ATAT0` / `ATAT1` / `ATAT2` | Adaptive timing off / normal / aggressive. `ATAT0` plus an explicit `ATST` gives repeatable behaviour when logging. |

### Protocol (compatibility stubs)

| Command | Meaning |
|---|---|
| `ATSP6` | Select protocol 6, ISO 15765-4 CAN 11-bit -- which is what this bus speaks. Accepted, but **the bit rate is not selectable**: it is compiled in as `CAN_BITRATE_ID` = `CAN_83K3BPS` in `lib/Core/Config.h`. Any other `ATSPn` is rejected. |
| `ATPB xx yy` | Set user protocol B parameters. Parsed and accepted for tool compatibility; the timing it would set is ignored, for the same reason. |
| `ATDP` | Describe the current protocol as text. |
| `ATDPN` | Describe it as a number. |

Because the bit rate is fixed in the build, this board can only ever talk to the
83.3 kbps interior bus. Probing CAN-C (500 kbps) from here is a *reachability* question,
not a configuration one -- see [PROBE-LOG.md](PROBE-LOG.md).

### Monitoring

| Command | Meaning |
|---|---|
| `ATMA` | Monitor all traffic: print frames continuously until any character arrives on the link. Obeys `ATCF`/`ATCM`/`ATCRA` if they are set, so clear them first to see everything. |

Because matching is software-side, `ATMA` shows genuinely everything on the bus when no
filter is set -- including the frames this board transmits itself and the traffic its own
VES/AUX and button logic is reacting to.

`ATMA` is unchanged in 2.1 and stays the interactive way to watch the bus. It is a *mode*,
though: it obeys the shared filter, it ends as soon as any character arrives, and while it
runs you cannot send a request. When you want broadcasts and request/response traffic at
the same time, use [`AT@SM1`](#10-stream-mode-atsm) instead -- it prints received frames in
the background, unfiltered, without taking over the link.

### Transmitting

| Input | Meaning |
|---|---|
| `hh hh ...` (a bare hex line) | Transmit these bytes as one CAN frame on the `ATSH` id, then listen for replies until the `ATST` timeout expires. 1..8 bytes, spaces optional. |

### Custom (non-ELM327) commands

| Command | Meaning |
|---|---|
| `AT@ST` | Print adapter status: firmware version, CAN health, rx/tx/err counters, current header and filter, feature flags, bench state, the stream levels `SM=` and `SML=` (`0`..`3` each, and mutually exclusive) and the dropped-stream-frame counter `SD=` (see [section 10](#10-stream-mode-atsm)). |
| `AT@EV1` / `AT@EV0` | Enable / disable asynchronous event lines from the autonomous modules. This is the replacement for print debugging. |
| `AT@BENCH1` / `AT@BENCH0` | Turn bench mode on / off at runtime. Bench mode fakes ignition on `0x000` and, **if a VIN is set** ([`AT@VIN`](#9-provisioning-the-vin-atvin)), broadcasts it on `0x01B` so a desk radio powers up without demanding its anti-theft code; it listens 2 s first and stays silent if it hears real bus traffic. See [section 8](#8-the-at-surface-is-observe-only). |
| `AT@I` | Print the **true** identity of the board, whatever `ATI` has been set to report. |
| `AT@VIN` | Query the VIN bench mode broadcasts: prints `<vin> STORED`, `<vin> DEFAULT`, or `NONE` if no VIN is set. |
| `AT@VIN`_`<vin>`_ | Store a VIN (1..21 printable chars; a real VIN is 17) in EEPROM. Used by bench mode from then on, across reboots. |
| `AT@VIN-` | Remove the stored VIN. With none set, bench mode sends power-on only (no VIN frames). |
| `AT@SM0` / `AT@SM1` | Stream mode off / on (new in 2.1): print every received frame continuously in the background, unfiltered and always with its header, alongside normal command traffic. Default **off**; `ATZ` restores off. See [section 10](#10-stream-mode-atsm). |
| `AT@SM2` / `AT@SM3` | Stream on **with a per-frame timestamp** appended at the end of each frame line: 4 hex digits, `millis() % 60000`, bit-exact LAWICEL `Z1` / 8 hex digits, raw `millis()`. Also stamps `ATMA` lines; request/response lines are never stamped. See [Timestamps](#timestamps-stream-levels-2-and-3). |
| `AT@SM` | Print the current stream level, `0`, `1`, `2` or `3`. Any other digit is rejected with `?`. |
| `AT@SML0` .. `AT@SML3` | Stream in **true LAWICEL/SLCAN wire format** instead of the ELM327 shape. The levels mirror `AT@SM` exactly: off / no stamp / 4-digit stamp / 8-digit stamp. Lines are `t<id><dlc><data>[<stamp>]` terminated by a bare CR. Mutually exclusive with `AT@SM`. See [LAWICEL wire format](#lawicel-wire-format-atsml). |
| `AT@SML` | Print the current LAWICEL stream level, `0`, `1`, `2` or `3`. Any other digit is rejected with `?`. |

Event lines are printed asynchronously and are prefixed so a host parser can tell them
apart from command responses:

```
AT@EV CAN 0                # EV_CAN_STATE, value 0 = CAN_ST_OK
AT@EV RADIO_MODE 6         # EV_RADIO_MODE, raw mode nibble from 0x09F -- 6 is AUX
AT@EV AUX 1                # EV_AUX_STATE, the radio has AUX/VES selected
AT@EV BTPWR 1              # EV_BT_POWER, BT module powered up
AT@EV BUTTON 01            # EV_BUTTON, bitmask from 0x394 byte 3 -- seek right
AT@EV BTCMD 2              # EV_BT_COMMAND, OvcCommand actually sent to the OVC3860
AT@EV BENCH 1              # EV_BENCH_MODE, bench announce active
AT@EV SAFE 1               # EV_SAFE_STATE, entered safe state (bus silent too long)
```

The event codes are `EventCode` in `lib/Core/Events.h`. The exact tag spelling comes from
AtServer's single code-to-tag table, which is the authority if it disagrees with the
strings above.

---

## 4. Response conventions

| Response | Meaning |
|---|---|
| `OK` | Command accepted. |
| `?` | Command not understood, or a parameter out of range. |
| `NO DATA` | The frame went out but nothing came back before the `ATST` timeout. |
| `CAN ERROR` | The controller refused the frame, or the bus is in error / bus-off. Check `AT@ST`. |
| `BUFFER FULL` | Either a command line exceeded the input buffer (the line is discarded, never half-executed), or frames arrived faster than they could be printed and some were dropped. |
| `>` | Prompt: the adapter is idle and ready. |

`NO DATA` and `CAN ERROR` mean different things, and the difference matters when probing.
`NO DATA` is "nobody answered". `CAN ERROR` is "the frame never made it onto the wire" --
retrying it against a different module tells you nothing.

---

## 5. Why `ATCAF0` matters

With `ATCAF1` the adapter inserts the ISO-TP single-frame PCI byte for you: you type
`21 30`, and `02 21 30` goes out. Convenient for OBD-II-shaped requests, useless the
moment you want to see or build anything else -- multi-frame requests, non-standard
lengths, flow-control frames, or broadcast frames that are not ISO-TP at all.

With `ATCAF0` **you supply the PCI byte**, which is what this board is for.

| Intent | typed under `ATCAF1` | typed under `ATCAF0` |
|---|---|---|
| Read local id `2130` (2 bytes) | `21 30` | `02 21 30` |
| Read DTCs by status, all groups (4 bytes) | `18 00 FF 00` | `04 18 00 FF 00` |
| ECU ident (2 bytes) | `1A 87` | `02 1A 87` |

The leading byte is the single-frame PCI: high nibble `0`, low nibble = the number of
payload bytes that follow.

KWP2000 reply shapes:

* Positive response = request service byte **+ 0x40**. `21` -> `61`, `18` -> `58`,
  `1A` -> `5A`.
* Negative response = `7F <service> <code>`.
* `7F xx 78` means **"response pending, wait"**. It is not a failure -- keep reading, or
  raise `ATST`.

Everything in [PROBE-LOG.md](PROBE-LOG.md) is written for `ATCAF0`.

---

## 6. Flow control

A KWP response longer than 7 bytes does not arrive as one frame. The module sends a
**First Frame** and then stops, waiting for permission to continue. The PCI high nibble
of the first byte says which kind of frame you are looking at:

| PCI high nibble | Frame type | Rest of the leading byte(s) |
|---|---|---|
| `0` | Single Frame | low nibble = payload length, 1..7 |
| `1` | First Frame | low nibble + next byte = 12-bit total length |
| `2` | Consecutive Frame | low nibble = sequence number, wrapping 1..F, 0, 1, ... |
| `3` | Flow Control | `30` continue, `31` wait, `32` abort |

The adapter handles exactly one piece of this: when it sees a First Frame it
**auto-sends `30 FF 00`** to the `ATFCSH` id -- clear to send, unlimited block size, no
separation time -- so the module streams the rest. Everything else is the host's job. The
adapter prints Consecutive Frames as they arrive and **the HOST reassembles them**.

Worked example: read the cluster's DTC list, the first thing on this bus that needs
multi-frame at all.

```
>ATZ                          # known state; opening the port did not reset the board
latonita-jeep v2.0

>ATE0                         # echo off
OK
>ATH1                         # show the id of every received frame
OK
>ATS1                         # spaces between bytes, easier to read by eye
OK
>ATCAF0                       # raw frames; we supply the PCI byte
OK
>ATSH 6A0                     # transmit on the CCN request id
OK
>ATCRA 514                    # print only CCN responses
OK
>ATFCSH 6A0                   # flow control goes back out on the request id
OK
>04 18 00 FF 00               # readDiagnosticTroubleCodesByStatus, all groups
514 10 14 58 00 xx xx xx xx   # FF: PCI 1, total length 0x014 = 20 bytes; 6 carried here
                              # 58 = 0x18 + 0x40, so this is a positive response
                              # >>> adapter now auto-sends 30 FF 00 on 6A0 <<<
514 21 xx xx xx xx xx xx xx   # CF seq 1, 7 bytes  -> 13 of 20 delivered
514 22 xx xx xx xx xx xx xx   # CF seq 2, 7 bytes  -> 20 of 20, transfer complete

>
```

`xx` are placeholders -- the DTC payload layout is to be confirmed in the car. The PCI
bytes and the arithmetic above are protocol, not guesswork. The last Consecutive Frame
may be short or padded with filler, so count payload bytes against the declared total
length and do not trust the DLC of the final frame.

**Without flow control every DTC list stalls.** You get the First Frame, nothing more,
and it looks like the module went away. If you ever see a bare `1x ...` frame followed by
silence, flow control is what failed.

Two things to try when a module ignores the FC frame:

* The adapter sends FC **unpadded**, DLC 3. Some modules only accept DLC 8. **Padding is
  the first thing to try**: `ATFCSM 1` with `ATFCSD 30 FF 00 00 00 00 00 00`.
* If the module answers on an id you have filtered out, the adapter never sees the First
  Frame and so never sends FC. Widen `ATCRA` / `ATCM` and retry.

---

## 7. ATI and adapter detection

`ATI` prints `FW_IDENT` from `lib/Core/Version.h`, currently `latonita-jeep v2.0`.

Third-party diagnostic software often decides whether it is talking to a real adapter by
string-matching the `ATI` reply against something ELM327-shaped, e.g. `ELM327 v1.5`. If
you want such a tool to accept this board, change `FW_IDENT` to something it recognises.
That is a cosmetic lie for the benefit of a parser, and it is exactly why `AT@I` exists:
**`AT@I` always reports the true firmware name and version** (`jeep-aux-and-can-gateway
2.0`, from `FW_NAME`), so our own tooling never has to depend on whatever `ATI` has been
dressed up as.

Our own `jeepdiag` CLI keys `--adapter auto` on both: it recognises the `ATI` banner
(known rebrands, currently `JEEPB` and `LATONITA-JEEP`) and falls back to the stable
`AT@I` name, so renaming `FW_IDENT` never breaks auto-detection. If you rebrand the
banner, add the new substring to `GATEWAY_ATI_IDENTS` in `jeepdiag/transport.py`.

---

## 8. The AT surface is observe-only

This is deliberate, not an unfinished feature.

The AT commands **cannot command the car's features**. There is no command to enable or
disable the VES announce, none to change the radio source, none to power the Bluetooth
module or send it a transport command. Those behaviours belong to the autonomous modules
and are driven only by what they see on the bus, exactly as they have been for the last
ten years. Unplug the link and the car behaves identically.

The reason: VES/AUX and BT control are the functions that must never break. Giving a
laptop -- or a half-finished dashboard app, or a stuck byte in a serial buffer -- the
ability to poke them would turn a working car radio into something that depends on a
development tool being correct. Sending arbitrary frames is still possible with a bare
hex line if you really need it; it just is not dressed up as a supported feature.

`AT@BENCH0` / `AT@BENCH1` is the **single exception**. It exists only because bench mode
used to be a `#define BENCH_MODE_ON`, and changing it meant a recompile plus a reflash
over ICSP just to test a head unit on the desk. It fakes ignition on `0x000` and, when a
VIN has been provisioned ([`AT@VIN`](#9-provisioning-the-vin-atvin)), broadcasts it on
`0x01B` so a desk radio powers up and skips its anti-theft code. With no VIN set it sends
the power-on frame only.

These are transmit-only frames that a real module already owns on the car bus, so bench
mode has a hardware-independent guard: on activation it **listens for 2 s
(`BENCH_DETECT_WINDOW_MS`) before transmitting anything**, and if it hears a genuine
`0x000` power frame in that window -- which can only come from another node, since a CAN
controller never receives its own frames -- it concludes it is on a live vehicle bus and
stays silent for the rest of that power cycle. The board is powered only when the radio
soft-ons, and by then the car is already broadcasting, so this reliably distinguishes car
from bench. The EEPROM flag is left untouched, so the same board re-arms bench
automatically the next time it powers up on a silent bench bus. It is still bench work
only -- but a board accidentally left in bench mode and reinstalled in the car will no
longer spam the interior bus.

---

## 9. Provisioning the VIN (`AT@VIN`)

The VIN that bench mode broadcasts on `0x01B` is **not hard-coded** -- it is provisioned
at runtime and kept in EEPROM, so one firmware image serves any car. With **no VIN set
(the default)**, bench mode sends the power-on frame only and **no VIN frames** at all --
so a stock board never emits some other car's VIN. A board can still bake in a default
with `-DVEHICLE_VIN="..."`, but the shipped default is empty.

```
>AT@VIN                       # query, nothing provisioned yet
NONE                          # bench mode would send power-on only
>AT@VIN1J8HCE8M45Y588369      # store (spaces are stripped, so a space is optional)
OK
>AT@VIN                       # verify
1J8HCE8M45Y588369 STORED      # active VIN, and where it came from
>AT@VIN-                      # remove -> back to NONE (or a baked-in DEFAULT)
OK
```

* **Query** prints `<vin> STORED`, `<vin> DEFAULT` (a compiled-in default in use), or
  `NONE` when no VIN is set.
* **Store** accepts 1..21 printable-ASCII characters (a real VIN is 17; 21 is the three
  frames' capacity). Non-printable bytes are rejected with `?`. The write is
  payload-first, magic-last, so a power loss mid-write never leaves a truncated VIN marked
  valid. It survives reboots.
* **Remove** (`AT@VIN-`) invalidates the stored VIN; a lone `-` is unambiguous because a
  VIN never starts with one. Bench mode then sends power-on only, unless a `-DVEHICLE_VIN`
  default was compiled in.
* A change takes effect on the next VIN announce -- no reboot needed. Re-enabling bench
  mode (`AT@BENCH1`) re-arms the VIN burst with whatever VIN is active.

The 3-frame `0x01B` encoding (index byte + 7 chars, zero-padded) is unchanged; only the
source of the characters moved from a `#define` to EEPROM. Host-side, `scripts/send_frame.py`
and `scripts/bench_vin_test.py` can still replay these frames directly for bench testing.

---

## 10. Stream mode (`AT@SM`)

Until 2.1, `ATMA` was the only way to see broadcast traffic, and `ATMA` is a *mode*: it
holds the link until a character arrives, and while it runs no request can be sent. A host
that wants both -- live broadcasts *and* request/response probing -- has to alternate:
clear the filter, enter `ATMA`, collect a window, break out with a keystroke, reprogram
`ATSH`/`ATCRA`, send its requests, then start over. Broadcasts are lost for the whole
request burst, and the addressing is reprogrammed twice per cycle purely to work around a
filter that monitoring and requests were sharing.

Stream mode removes the alternation.

There are two commands, one per wire format, and **the level digit means the same thing in
both**:

| Level | `AT@SM` -- ELM327 shape | `AT@SML` -- [LAWICEL/SLCAN](#lawicel-wire-format-atsml) |
|---|---|---|
| `0` | stream off (**default**) | stream off (**default**) |
| `1` | frame lines exactly as 2.1 printed them | plain `t` lines, no stamp -- what stock slcan tooling expects |
| `2` | + **4** uppercase hex digits, `millis() % 60000`, bit-exact LAWICEL `Z1` | the same 4 digits |
| `3` | + **8** uppercase hex digits, raw `millis()`, the full `uint32_t` | the same 8 digits |
| query | `AT@SM` prints `0`..`3` | `AT@SML` prints `0`..`3` |

0 is off, 1 is the compatible baseline with no timestamp, 2 is four hex digits, 3 is eight.
Learn one command and you have learned the other; only the syntax of the frame line
differs. Any other digit answers `?` rather than being clamped, so `AT@SM4` reads as the
typo it is instead of quietly behaving like `AT@SM1`.

One link carries one format, so **the two commands are mutually exclusive**: setting either
to a non-zero level puts the other back to 0. The timestamp is a *level* rather than a
command of its own, which is a compatibility decision as much as a syntactic one --
[Timestamps](#timestamps-stream-levels-2-and-3) below says why.

Default is **off**, and `ATZ` restores level 0 -- it is part of `resetDefaults`, like echo
and headers. So a fresh session is always classic ELM327 behaviour, and an existing tool
that has never heard of `AT@SM` cannot be surprised by a frame it did not ask for. The
supervisor's safe-state path clears stream mode too.

**Levels 0 and 1 are unchanged, byte for byte.** That same argument extends one step: a
host that only ever sends `AT@SM0` or `AT@SM1` gets exactly what 2.1 gave it -- the same
frame lines, the same `AT@SM` query output (`0` or `1`), and the same `AT@ST` status line
with the same fields, in the same order, of the same shape. `SM=` simply now carries a
level that happens to be 0 or 1. A timestamp can only ever appear on a link whose *own*
host asked for one by sending `AT@SM2` or `AT@SM3`. This is also why the timestamp was
folded into `AT@SM` instead of being given a command and a status field of its own: a new
` TS=` field in `AT@ST` would have broken every host that parses that line positionally or
by field count, and it would have bought nothing, because the level number already says
everything such a field could say.

With stream mode on, **every** received CAN frame is printed as it arrives: while the
adapter sits idle at the prompt, and also while it is collecting the response to a request.
Unlike `ATMA` it does not stop when a character arrives -- only `AT@SM0`, a non-zero
`AT@SML` level, `ATZ` or safe state turn it off.

Three properties differ from `ATMA`, each of them deliberate.

**Streamed frames are not filtered.** `ATCF`/`ATCM`/`ATCRA` keep applying to
request/response matching and have no effect on the stream. One filter shared between
monitoring and requests is exactly what forced the old host into the clear-monitor-restore
dance above; with the two separated, a host can hold `ATCRA 514` for its probing and still
watch the entire bus at the same time.

**Streamed frames always carry the header**, whatever `ATH0`/`ATH1` says. A stream you
cannot attribute to an id is useless, and `ATH` exists for request responses, where the
host already knows what it asked. Keeping `ATH0` for compact responses therefore costs
nothing in the stream.

**The stream never blocks.** If the UART transmit buffer has no room for the whole line,
the frame is **dropped** and a counter incremented; the adapter does not wait for space.
Blocking would stall the board's main loop, the MCP2515's two receive buffers would
overflow, and the VES/AUX announce the car's audio depends on would be delayed -- so one
dropped diagnostic frame is strictly better than a stalled loop. The counter is reported by
`AT@ST` as `SD=`. If it climbs, the host is not draining the port fast enough; that is a
host problem, and the adapter has already chosen the car over the diagnostic.

### Bandwidth

The interior bus carries about **151 frames/s** with the engine running. With `ATS0` that
is roughly **2.5 kB/s**, about **22 %** of the 115200 baud link. The stream has ample
headroom; `SD=` staying at zero over a long session is the expected result, not a lucky
one.

Timestamps ([levels 2 and 3](#timestamps-stream-levels-2-and-3), below) are paid for out of
that headroom. `AT@SM2` adds 4 characters per frame -- about **604 B/s** at 151 frames/s,
taking the link to roughly **27 %**. `AT@SM3` adds 8, about **1.2 kB/s**, taking it to
roughly **32 %**. Neither is near the limit, so the choice between them is about what the
host wants to parse, not about what the link can carry.

[`AT@SML`](#lawicel-wire-format-atsml) costs two bytes per frame more than that. An ELM
frame line under `ATS0`/`ATL0` is `4 + 2n + w` bytes (3 id + `n` data bytes + CR + a stamp
`w` wide); a LAWICEL line is `6 + 2n + w`, the difference being the `t` prefix and the DLC
nibble. Read backwards, the 2.5 kB/s figure above implies an average of about **6.25 data
bytes** per frame on this bus (2500 / 151 = 16.6 = 4 + 2n), which gives:

| Level | Average line | Average rate | Share of the link |
|---|---|---|---|
| `AT@SML1` | ~18.5 B | ~2.8 kB/s | ~24 % |
| `AT@SML2` | ~22.5 B | ~3.4 kB/s | ~30 % |
| `AT@SML3` | ~26.5 B | ~4.0 kB/s | ~35 % |

Which puts plain LAWICEL (`AT@SML1`, ~24 %) *below* a stamped ELM stream (`AT@SM2`, ~27 %):
the `t` and the DLC cost two bytes a frame, a timestamp costs four or eight. Worst case on
the link is a full 8-byte frame with an 8-digit stamp, `1 + 3 + 1 + 16 + 8 + 1` = **30
bytes**. Still comfortable for the baud rate -- it is the TX ring, not the link, that feels
this (see [below](#timestamps-and-the-tx-ring)).

### Timestamps (stream levels 2 and 3)

A stream of frames with no arrival times is half a log: it says what the bus carried but
not when, so periods, latencies and the ordering of events across ids all have to be
inferred from when the *host* happened to read the port -- which is exactly the thing that
is not trustworthy under buffering. ELM327 has no answer here. LAWICEL/SLCAN does, and has
since the CAN232, where `Z1` turns on a 4-digit millisecond stamp per frame, appended at
the end of the message and looping at 60 000 ms (LAWICEL CAN232 v3 / CANUSB manuals).
Levels 2 and 3 are that feature, level 2 kept deliberately bit-exact to the original.

The counter-argument is worth acknowledging, because a respected implementation makes it:
the CANable 2.5 firmware dropped slcan timestamps on purpose, on the grounds that spending
8 ASCII characters per frame is waste when the host has a 64-bit counter of its own. That
holds for a USB CDC device where host arrival time is a good proxy for bus time. It does
not hold here. This link is a 115200 baud UART with a 128-byte ring at the adapter end and
unknown queueing at the host end, which is exactly the latency an on-adapter stamp removes
from the measurement -- so here the 4 or 8 characters buy something real.

| Level | Stamp | Range |
|---|---|---|
| `AT@SM2` | 4 uppercase hex digits, `millis() % 60000` | `0000`..`EA5F`, wrapping every 60 000 ms |
| `AT@SM3` | 8 uppercase hex digits, raw `millis()` | `00000000`..`FFFFFFFF`, wrapping after ~49.7 days |

The stamp is appended at the **end** of the line, after the last data byte and before the
terminator. Under `ATS1` a single space separates it from the data; under `ATS0` there is
no separator, because `ATS0` promises a line with no spaces in it and the host demux rule
in [the section below](#host-side-demultiplexing-the-line-stream) rests on that promise.

Stamping applies to unsolicited output only: stream lines and `ATMA` lines.
**Request/response lines are never stamped.** This is the `ATH` argument read backwards --
`ATH` exists for responses because the host already knows what it asked and when it asked
it, so a stamp on the answer adds nothing, while adding one would change the shape of every
response line and break ISO-TP reassembly in every host parser written against this
surface. A gain of nothing is not worth that. (`ATMA` lines are stamped, and `ATMA` still
blocks rather than drops, so a stamp never costs an `ATMA` frame -- it only makes the
monitor a little slower to drain.)

Safe state clears the level along with the stream, for the reason it clears the stream: a
board going to a safe state must go quiet on the link, and with the level back at 0 and the
state forced to idle nothing is left that could emit a stamped line.

Worked example -- `0x1A7`, the engine injection-quantity broadcast
([CAN-IHS-MAP.md](CAN-IHS-MAP.md)), 6 data bytes `02 41 00 00 00 00`, seen 1 725 991 ms
(28 min 45.991 s) after boot:

```
>ATS0                         # no spaces -- required by the host demux rule
OK
>AT@SM2                       # stream on, 4-digit stamp, LAWICEL Z1 shape
OK
1A7024100000000B3A7           # 1A7 | 02 41 00 00 00 00 | B3A7
                              #  id |    6 data bytes   | 0xB3A7 = 45991 ms into this minute
                              # 3 + 12 + 4 = 19 characters

>AT@SM3                       # stream on, 8-digit stamp, raw millis()
OK
1A7024100000000001A5627       # 1A7 | 02 41 00 00 00 00 | 001A5627
                              #  id |    6 data bytes   | 0x1A5627 = 1725991 ms since boot
                              # 3 + 12 + 8 = 23 characters
```

Both stamps describe the same instant: 1 725 991 ms is 28 whole minutes plus 45 991 ms, and
45 991 is `0xB3A7`. Under `ATS1` the same two lines read `1A7 02 41 00 00 00 00 B3A7` and
`1A7 02 41 00 00 00 00 001A5627`.

### The level is the width declaration

This is the sharp edge in the feature, and it is worth being blunt about: **the host cannot
detect the stamp width from the line. It must decode against the width it configured.**

A LAWICEL host does not have this problem, because a real LAWICEL frame carries an explicit
DLC nibble -- `t1002AB40` is id `100`, DLC `2`, data `AB 40` -- so the host can compute how
long the line *should* be and read whatever is left over as a stamp. Detection is free
there.

This adapter emits no `t` and no DLC. The data length is **implied** by the number of hex
characters, which leaves the line length carrying two unknowns and one equation. A
19-character line under `ATS0` is all three of the following, and nothing in the text can
separate them:

| A 19-character frame line may be | Arithmetic |
|---|---|
| 8 data bytes at `AT@SM1` | 3 + 16 |
| 6 data bytes at `AT@SM2` | 3 + 12 + 4 |
| 4 data bytes at `AT@SM3` | 3 + 8 + 8 |

All pure hex, all odd, all valid frame lines.

Which is precisely why the timestamp is a level of `AT@SM` and not a separate command: **the
level number *is* the width declaration**, and it tells the adapter and the host the same
thing in the same place. `AT@SM2` is simultaneously "start stamping" and "the stamp is four
digits wide". There is no second setting to get out of step with the first.

The failure mode this guards against is the nasty kind -- not an error, but plausible data.
A tool that selects level 2, forgets, and later reconnects assuming defaults will silently
read the last two data bytes of every frame as a timestamp and come up two bytes short on
payload, and both halves of that result look reasonable. Level 2 offers one weak check,
since a `Z1` stamp can never exceed `EA5F`: roughly one wrong value in twelve lands above
that and is provably not a timestamp. The other eleven do not announce themselves.

So a host that configured the session knows the width. A host that attached to a session
someone else configured -- a second terminal, a reconnect after a crash, a script
inheriting an open port -- has to ask, and `AT@ST`'s `SM=` field (or a bare `AT@SM`) is how
it finds out. That is the field doing double duty as the width report, which is the other
half of why no ` TS=` field was added.

**None of this applies to [`AT@SML`](#lawicel-wire-format-atsml)**, and the reason is the
single nibble the ELM shape does not have. A LAWICEL line carries an explicit DLC, so the
data byte count is stated rather than implied, `length - (5 + 2 * DLC)` is the stamp width,
and a host can detect the stamp exactly as real LAWICEL tooling does -- no configuration,
no ambiguity, no way to mis-slice a frame.

The symmetric levels make the contrast exact. `AT@SM1` and `AT@SM2` output can be
indistinguishable -- that is the table above. `AT@SML1` and `AT@SML2` output never is: the
DLC says six data bytes, so a 17-character line is unstamped and a 21-character line
carries four digits of stamp, and the host can see which it has. Same two levels, same two
widths, and one format can answer the question from the bytes while the other cannot. That
is what the DLC nibble is *for*. The ELM327 shape has to be told its width because it chose
brevity over that nibble, twenty years before anyone wanted to append a timestamp to it.

### LAWICEL wire format (`AT@SML`)

Everything above prints the ELM327-shaped frame line this adapter has always used, which
is a format nothing but this project understands. `AT@SML` streams the same frames in the
real LAWICEL/SLCAN syntax instead, so off-the-shelf slcan tooling can be pointed at the
port directly rather than taught a private format.

The levels are `AT@SML0`..`AT@SML3`, the same four the [table above](#10-stream-mode-atsm)
gives `AT@SM`: off, no stamp, 4 digits, 8 digits. The line is `t` + 3 hex id + **1 hex
DLC** + 2 hex per data byte + the stamp, if any + CR. Same frame throughout -- `0x1A7`,
6 data bytes `02 41 00 00 00 00`, 1 725 991 ms after boot:

```
>AT@SML1                      # plain LAWICEL, no stamp
OK
t1A76024100000000             # t | 1A7 | 6 | 02 41 00 00 00 00
                              #   |  id |DLC|    6 data bytes
                              # 1 + 3 + 1 + 12 = 17 characters

>AT@SML2                      # LAWICEL + 4-digit stamp (Z1 shape)
OK
t1A76024100000000B3A7         # t | 1A7 | 6 | 02 41 00 00 00 00 | B3A7
                              #   |  id |DLC|    6 data bytes   | 45991 ms into this minute
                              # 1 + 3 + 1 + 12 + 4 = 21 characters

>AT@SML3                      # LAWICEL + 8-digit stamp (raw millis)
OK
t1A76024100000000001A5627     # t | 1A7 | 6 | 02 41 00 00 00 00 | 001A5627
                              #   |  id |DLC|    6 data bytes   | 1725991 ms since boot
                              # 1 + 3 + 1 + 12 + 8 = 25 characters
```

Compare the ELM lines for the same frame -- `1A7024100000000B3A7` at `AT@SM2` and
`1A7024100000000001A5627` at `AT@SM3` -- and the two extra characters are the whole
difference: the `t` that makes demultiplexing trivial, and the DLC that makes the stamp
self-describing.

Four rules of this format differ from the ELM path, and each exists so that a parser
written against the LAWICEL manual keeps working:

* **Lowercase `t` only.** `T` is the 29-bit extended variant and `r`/`R` are the RTR
  variants. `CanFrame` carries an 11-bit id and has no extended or remote flag, so `t` is
  the only line this firmware can emit honestly. The limitation is real and worth stating
  plainly: **an RTR frame cannot be represented on this stream at all**, and this bus is
  11-bit throughout, so nothing is lost here that the board could have seen anyway.
* **No spaces, ever.** `AT@SML` ignores `ATS1` completely. The slcan grammar has no
  spaces in it, and a parser that has never seen one has no reason to tolerate one.
* **Bare CR, never CRLF.** `AT@SML` ignores `ATL1`. An slcan reader splits the byte stream
  on CR, so a trailing LF is not swallowed -- it becomes a *leading* LF on the next line,
  which then fails the `t` test or parses as a corrupt id. `ATL1` exists for humans reading
  a terminal; there are no humans in this format.
* **The DLC nibble is present**, which is why this format needs no
  [width declaration](#the-level-is-the-width-declaration): the DLC says how many data
  bytes to expect, so anything beyond them is the stamp, and its width follows from the
  line length. A host can detect timestamps here. In the ELM format it cannot. The nibble
  describes the bytes actually emitted -- `frame.len` clamped to 8, the same guard the ELM
  path applies -- so a host that sizes its read from the DLC can never be walked past the
  end of the line by a driver bug or a bus glitch.

**`AT@SML1` is the level to reach for with a tool you did not write.** python-can's
`slcan` bus and can-utils' `slcand` both default to LAWICEL `Z0` -- timestamps off -- so
plain `t` + id + DLC + data + CR is exactly the line they expect, byte for byte, with
nothing to configure. Levels 2 and 3 are the opt-in: useful, but only once the tool at the
other end has been told to expect a stamp.

Two smaller deviations from the CAN232FD manual, for anyone comparing: this firmware never
emits the bare `zz` line that signals a timer reset or overrun (its timer is `millis()`,
which neither resets nor overruns inside any plausible session), and it has no `f` status
command reporting the mode as `Z`/`z`/`-`. `AT@ST`'s `SML=` field, and a bare `AT@SML`
query, are where that lives instead.

**`AT@SM` and `AT@SML` are mutually exclusive.** One link, one format: setting either to a
non-zero level puts the other back to 0. Interleaving two frame syntaxes on one port would
leave the host demultiplexing a stream that changes grammar mid-session for no benefit
whatsoever, since no host wants both. The observable consequence is worth internalising
before it looks like a bug: under `AT@SML2` a bare `AT@SM` query answers **`0`**, while
`AT@SML` answers `2` and `AT@ST` reports `SM=0 SML=2`. All of those are correct --
ELM-format streaming genuinely is off, and each query is truthful about its own format.
Because at most one of the two can be non-zero, the pair of fields can never disagree about
what the link is doing.

**Scope.** The background stream and `ATMA` both use whichever format and width is active,
so under `AT@SML1` even `ATMA` emits LAWICEL lines. **Request/response lines are always
ELM-format and never stamped**, regardless of `AT@SML`: slcan has no notion of a request or
a reply, and those lines belong to the ELM327 surface -- `ATH`, `NO DATA`, `CAN ERROR`, the
ISO-TP flow in [section 6](#6-flow-control) -- that every host parser here depends on.
Nothing about that surface moves because the stream changed clothes.

**`AT@ST` grows one field, ` SML=`.** That is a different judgement from the one that kept
a ` TS=` field out, and the difference is what the field reports. A ` TS=` field would have
restated something `SM=` already said. `SML=` reports a genuinely separate piece of state:
which of the two wire formats the link is in, one digit, `0`..`3`, appended after `SM=`.

The alternative was to print the raw internal mode -- one field, `SM=0`..`6`, no growth at
all -- and it was rejected on purpose: `SM=` is read by every existing host as *the `AT@SM`
level*, and `SM=5` meaning "LAWICEL with a 4-digit stamp" would be a lie told to a field
that already has a meaning. **A wrong field is worse than a new one.** So `SM=` and `SML=`
each carry their own format's level, `0`..`3` -- the same digits that format's own query
returns, read through the same accessor, so the two queries and the status line cannot
disagree with each other.

The cost is stated plainly: **a host that parses `AT@ST` positionally or by counting fields
needs updating.** A host that looks fields up by name -- the only robust way to read that
line -- does not, and `AT@ST` has grown fields before: `REI=`, `SM=` and `SD=` all arrived
in 2.1. `SML=` sits between `SM=` and `SD=` so the stream group stays adjacent, and every
other field keeps its position.

The round trip this closes is the practical reason to care.
`..\jeep-carputer\scriptseplay-lawicel.py` reads exactly this format, and it detects the
stamp the self-describing way -- comparing the payload length against the DLC -- so an
**`AT@SML2`** capture taken in the car replays on the desk at its recorded pacing, with no
converter in between. An `AT@SML1` capture replays as well, at a fixed `--rate`, since
there is nothing to pace from. An `AT@SML3` capture does not: that script recognises the
4-digit stamp only and skips any line whose payload is 8 digits longer than its DLC. One
more reason `AT@SML2` is the interop level.

### Timestamps and the TX ring

The stream's non-blocking contract is paid for in TX buffer headroom, and both the stamp
and the LAWICEL framing eat into it. One full `CAN_RX_DRAIN_MAX` pass is 4 frames, and the
`SERIAL_TX_BUFFER_SIZE` note in `platformio.ini` is built on the typical one: 4 x 17 =
**68** bytes at `AT@SM1`. The ELM rows below count the CRLF terminator `ATL1` gives by
default; the LAWICEL rows count the bare CR that format forces, which is why an `AT@SML1`
line is 18 bytes where the same frame at `AT@SM1` is 17.

| Level | Typical line (6 data bytes) | Drain pass | Worst-case line (8 data bytes) | Worst-case pass |
|---|---|---|---|---|
| `AT@SM1` | 17 B | 68 B | 21 B | 84 B |
| `AT@SM2` | 21 B | 84 B | 25 B | 100 B |
| `AT@SM3` | 25 B | 100 B | 29 B | **116 B** |
| `AT@SML1` | 18 B | 72 B | 22 B | 88 B |
| `AT@SML2` | 22 B | 88 B | 26 B | 104 B |
| `AT@SML3` | 26 B | 104 B | 30 B | **120 B** |

Against the 128-byte ring the widest level of each format -- `AT@SM3` at 116 bytes and
`AT@SML3` at 120 -- leaves 12 and 8 bytes of margin on a worst-case pass, so
`SERIAL_TX_BUFFER_SIZE` **stays at 128**: every level fits, and the ring is draining at
~11.5 bytes/ms throughout the pass, so the real margin is wider than the arithmetic above
suggests. It is no longer *headroom*, though, and the `platformio.ini`
comment says what to do about it -- raise it to 256, at 128 bytes of the ~1256 free, if
`AT@ST` ever reports a non-zero `SD=`. That counter is the only symptom.

One combination genuinely does overrun: `ATS1` with a stamp. A full-DLC `AT@SM3` line
with spaces is 3 + 1 + 23 + 1 + 8 + 2 = **38** bytes, and a drain pass of those is 152 --
past the ring, so frames will drop on any busy drain. That is not worth fixing, because
`ATS1` is the read-it-by-eye mode and the host demux rule needs `ATS0` anyway; `AT@SML`
sidesteps it entirely by ignoring `ATS` altogether.

Behaviour at the limit is unchanged, and this is the part that matters: `streamFrame()`
counts the timestamp in the writable-bytes check it makes *before* it starts writing, so a
ring without room for the whole line still **drops** the frame and increments `SD=` rather
than blocking `loop()`. A tight margin therefore costs diagnostic frames and shows up as a
climbing `SD=`. It can never stall the loop, overflow the MCP2515 or delay the VES announce
-- the trade-off made for the stream in the first place still holds at every level and in
both formats.

### What the timestamp actually measures

Stated honestly: the stamp records **when the main loop reached the controller**, not when
the frame arrived on the wire. Frames are polled out of the MCP2515 by the bounded drain
loop in `src/main.cpp`; there is no receive interrupt and no hardware capture register, so
true bus arrival time is not something this board could record even in principle.

In practice the gap is small, and bounded by the same slack that makes the non-blocking
stream safe: the loop has to stay inside the roughly 3 ms of buffering the controller's two
receive buffers give at 83.3 kbps, and it does, comfortably. Realistic accuracy is
therefore **+/-1-2 ms** -- one or two units of the stamp's own resolution. Which is exactly
why there is no microsecond level. The later CAN232FD added a lowercase `z` command
carrying 8 hex digits of 32-bit *micro*seconds, also appended at the end of the line (the
CAN232FD reference manual); levels 3 and `AT@SML2` borrow that 8-digit width and keep the
unit at milliseconds, because microsecond precision on a polled frame would be a lie told
in more digits.

The value comes from the single `millis()` sample `loop()` takes per iteration and shares
with every module, so a stamp agrees with `AT@ST`'s `UP=` and with event-line ordering
instead of drifting against them. One consequence follows from that sharing: up to
`CAN_RX_DRAIN_MAX` frames drained in the same pass carry the **same** stamp. Inside one
millisecond, line order is the ordering information -- the stamp is not.

### Choosing a level

Two questions decide it: which format the consumer speaks, and whether a one-minute
counter is good enough.

| Use | Level |
|---|---|
| Logging through this project's own host | `AT@SM3` |
| Issuing ISO-TP requests while watching the bus | `AT@SM1`..`AT@SM3` |
| Host timestamps on arrival itself, narrowest ELM line on the wire | `AT@SM1` |
| ELM-shaped output that a LAWICEL-style stamp has to fit into | `AT@SM2` |
| Pointing stock slcan tooling at the board | `AT@SML1` |
| slcan tooling configured for LAWICEL-native timestamps | `AT@SML2` |
| A long slcan capture | `AT@SML3` |

**`AT@SM3` is the recommendation for logging through our own host**: ELM-shaped, the full
32-bit millisecond counter, no rollover to unwrap. Unwrapping a one-minute counter means
being sure you did not miss a whole minute, and after a `BUFFER FULL` or a burst of `SD=`
drops a host cannot be sure of that -- which is the case against level 2 for any long
session, in either format.

**`AT@SML1` is the one to reach for when the consumer is somebody else's slcan tool**,
because it is what such a tool expects by default. Move up to `AT@SML2` when that tool has
been told to expect LAWICEL timestamps, and to `AT@SML3` when the capture will run long
enough to make the wrap a liability -- bearing in mind that `AT@SML2` is the level our own
replay script can pace from.

**If the host issues requests, use `AT@SM`, not `AT@SML`.** This is the one combination to
avoid deliberately. Request/response lines stay ELM-shaped whatever the stream format is, so
a LAWICEL stream plus an ISO-TP conversation puts two grammars on one link: `t` lines from
the stream, and `OK` / `NO DATA` / `514 10 14 58 ...` from the request. Telling them apart
is easy -- see [the demux note](#host-side-demultiplexing-the-line-stream) -- but a strictly
slcan parser has no vocabulary for the ELM half and will choke on it, and making a stock
slcan parser work was the entire point of `AT@SML`. So `AT@SML` is for pointing slcan
tooling at the board to *watch* the bus; request/response work belongs on `AT@SM`.

### Host side: demultiplexing the line stream

With stream mode on, the port carries two interleaved things -- frames and command replies
-- and the host has to tell them apart. Set `ATS0` and `ATL0` first; then the rule is
purely lexical:

| A line is a **frame** when | A line is a **command reply** when |
|---|---|
| it is pure hex, contains no space, and has an **odd** number of characters | it contains a space, starts with `@`, is not pure hex, or has an **even** number of characters |

The odd length is not a heuristic: a frame line is 3 id nibbles plus 2 nibbles per data
byte, so its length is always odd (3 for a zero-length frame, then 5, 7 ... 19), while `OK`, `NO DATA`, `AT@EV ...` and
the `AT@ST` fields are none of those shapes. `?` and the `>` prompt are single characters
and not hex.

**A timestamp does not disturb this rule**, which is the entire reason it is appended
rather than prefixed. A stamped line is `3 + 2n + 4` characters at level 2 and `3 + 2n + 8`
at level 3; 4 and 8 are even, so the length stays odd and the line stays pure hex. Only the
upper bound moves -- the longest frame line is **19** characters at level 1, **23** at
level 2 and **27** at level 3. A host that reads whole lines and applies the table above
therefore needs no change at all to survive a stamped session; a host that decodes the
payload does, and the reason is in
[the width note](#the-level-is-the-width-declaration) above. A prefix, by contrast, would
have put non-hex or even-length material at the front of the line and broken every
ELM327-shaped parser ever pointed at this port.

**In LAWICEL mode none of this parity arithmetic is needed.** Under `AT@SML` the rule is
simply **a line starting with `t` is a frame**, and nothing else this firmware emits starts
with `t`: the banner starts with `l`, the replies are `OK`, `?`, `NO DATA`, `CAN ERROR` and
`BUFFER FULL`, event lines start with `AT@EV`, and the prompt is `>`. That is strictly more
robust than the odd-length rule, which is one more reason to prefer `AT@SML` when the host
is a general-purpose tool rather than something written against this document.

There is a corollary worth spelling out, because it bites in exactly one place. While a
request is being collected with a LAWICEL stream running, **the link carries both formats at
once**: stream lines are `t`-prefixed LAWICEL, while the request's own response lines are
still ELM-shaped and unstamped, because [request/response output never changes
format](#lawicel-wire-format-atsml). Demultiplexing that mixture is *easier* than the
all-ELM case -- the `t` prefix settles every line with no arithmetic at all -- but only if
the host understands both halves. A strictly slcan parser does not: the ELM-shaped response
lines are outside its vocabulary and it will choke on them. Hence the guidance in
[Choosing a level](#choosing-a-level): **a host that issues ISO-TP requests should stream
with `AT@SM`, not `AT@SML`.**

One trap worth knowing: **ignore streamed frames whose id equals your own `ATSH` transmit
header.** The adapter auto-sends the ISO-TP Flow Control frame ([section 6](#6-flow-control))
on the bus, the stream shows it like any other frame, and a host that does not skip it will
see its own flow control come back as if a module had sent it.

Stream mode does not change `ATMA`, which still works exactly as before for interactive use
and for scripts that already depend on it.
