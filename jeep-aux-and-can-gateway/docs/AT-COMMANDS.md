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

### Transmitting

| Input | Meaning |
|---|---|
| `hh hh ...` (a bare hex line) | Transmit these bytes as one CAN frame on the `ATSH` id, then listen for replies until the `ATST` timeout expires. 1..8 bytes, spaces optional. |

### Custom (non-ELM327) commands

| Command | Meaning |
|---|---|
| `AT@ST` | Print adapter status: firmware version, CAN health, rx/tx/err counters, current header and filter, feature flags, bench state. |
| `AT@EV1` / `AT@EV0` | Enable / disable asynchronous event lines from the autonomous modules. This is the replacement for print debugging. |
| `AT@BENCH1` / `AT@BENCH0` | Turn bench mode on / off at runtime. Bench mode fakes ignition on `0x000` and, **if a VIN is set** ([`AT@VIN`](#9-provisioning-the-vin-atvin)), broadcasts it on `0x01B` so a desk radio powers up without demanding its anti-theft code; it listens 2 s first and stays silent if it hears real bus traffic. See [section 8](#8-the-at-surface-is-observe-only). |
| `AT@I` | Print the **true** identity of the board, whatever `ATI` has been set to report. |
| `AT@VIN` | Query the VIN bench mode broadcasts: prints `<vin> STORED`, `<vin> DEFAULT`, or `NONE` if no VIN is set. |
| `AT@VIN`_`<vin>`_ | Store a VIN (1..21 printable chars; a real VIN is 17) in EEPROM. Used by bench mode from then on, across reboots. |
| `AT@VIN-` | Remove the stored VIN. With none set, bench mode sends power-on only (no VIN frames). |

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
