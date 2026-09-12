# Vendored `mcp_can` — provenance and the one local patch

`mcp_can.cpp`, `mcp_can.h` and `mcp_can_dfs.h` are the Seeed Studio MCP2515 library
(2012 Seeed Technology, LGPL 2.1; author Loovee, contributor Cory J. Fowler), as
modified in 2015-2017 for `jeep-aux-and-extra-features`.

**Do not replace these with an upstream release.** They carry a local change to the
bit-timing tables that adds 8 MHz-crystal support; upstream only handles 16 MHz
modules. This board runs a 16 MHz crystal, so `MCP_16MHz` + `CAN_83K3BPS` is
what the interior bus needs, but the 8 MHz table is kept because it exists only
in this vendored copy and other boards may need it.

---

## Patch applied

### 1. Clamp the received DLC to the data buffer (`mcp_can.cpp`, `mcp2515_read_canMsg`)

The only deliberate change to the vendored sources in this project.

`m_nDlc` is masked with `MCP_DLC_MASK` (`0x0F`), so it reaches the read as 0..15.
DLC values 9..15 are legal on a CAN bus and denote 8 data bytes. The original then
did:

```c
mcp2515_readRegisterS( mcp_addr+5, &(m_nDta[0]), m_nDlc );   // up to 15 bytes
```

into `INT8U m_nDta[8]`. The class members immediately after it are, in order,
`m_nRtr`, `m_nfilhit` and `SPICS` — so a single frame with DLC > 8 overwrites
**`SPICS`, the chip-select pin number**. Every subsequent SPI transaction then
addresses the wrong pin, and because `SPICS` is private and the driver object is
static, `reinit()` cannot recover it. The board would need a power cycle.

Low probability on a healthy bus, but unrecoverable, silent, and this firmware holds
the Bluetooth power pin and the VES announce in a moving vehicle. Fixed with a
two-line clamp, marked `LOCAL PATCH` in the source.

`readMsgBufID()` already clamps its own copy to the caller's buffer;
`readMsgBuf()` does not, which is a second reason `CanDriver` uses the former.

---

## Not patched — worked around in `CanDriver` instead

Left alone to keep the diff against the 2015 sources as small as possible. Each is
handled in the wrapper; if you ever bypass `CanDriver` and call `MCP_CAN` directly,
these bite.

| Library behaviour | Why it matters | Wrapper's answer |
|---|---|---|
| The 4-argument `sendMsgBuf()` overload never assigns `m_nRtr`, and that member is shared with the receive path | After receiving one remote-request frame, every later send goes out as an RTR frame | `send()` always uses the 5-argument overload with an explicit `rtr = 0` |
| Both `setMsg()` overloads copy a fixed 8 bytes from the caller's pointer regardless of `len` | Reads past a short caller buffer | `send()` stages into a zero-filled `uint8_t[8]`. Required, not defensive padding |
| `mcp2515_initCANBuffers()` has its mask/filter zeroing commented out, the MCP2515 does not reset `RXMn`/`RXFn`, and `mcp2515_init()` enables filtering (`MCP_RXB_RX_STDEXT`) | Acceptance state after `begin()` is **undefined** | `begin()` always programs acceptance explicitly, falling back to `acceptAll()`; `reinit()` restores whatever modules had requested |
| `mcp2515_init()` returns `MCP2515_OK` even when `mcp2515_configRate()` rejects the speed/clock pair | `begin() == CAN_OK` is not proof of a live controller — the chip can sit in CONFIG mode, deaf and mute | Harmless here (16 MHz/83.3k is in the patched table), but never treat `begin()` success as bus liveness. `Supervisor` watches for RX silence instead |
| `init_Mask()` / `init_Filt()` never usefully report a bad `num` — the `MCP2515_FAIL` is overwritten by the mode-restore result | Silent misconfiguration | Wrapper hardcodes valid ranges (masks 0..1, filters 0..5) rather than trusting the return |
| `init_Mask()` / `init_Filt()` each switch to `MODE_CONFIG` and back to `MODE_NORMAL` internally, with several `delay(10)`s | The node leaves the bus for ~200 ms to program 6 filters + 2 masks | Acceptance is a setup-time-only operation. The wrapper must **not** add its own mode switching |
| `RXnOVR` is latched and `mcp2515_modifyRegister` is private, so overflow cannot be cleared through the public API | Treating overflow as fatal would mark the node dead forever after one late drain | `healthy()` considers only `TXBO \| TXEP \| RXEP`. Only `reinit()` (a chip reset) clears overflow |
| TEC/REC are unreachable | "High error counters" cannot be read directly | Inferred from `TXEP`/`RXEP` (error-passive, > 127) |
| `begin()` blocks ~50 ms per attempt (`mcp2515_reset()` plus `delay(10)`s) and cannot be made non-blocking without editing the library | A retry loop in `begin()` would starve the watchdog and the AT link — the original firmware's `while (CAN_OK != CAN.begin(...)) delay(250)` is exactly the pattern to avoid | Bounded attempt count, then return `CAN_ST_NOT_READY` and let the caller own the retry cadence |

## Verifying the sources

To confirm nothing but the documented patch differs from the 2015 originals:

```sh
diff mcp_can.h     ../../../jeep-aux-and-extra-features/mcp_can.h        # expect no output
diff mcp_can_dfs.h ../../../jeep-aux-and-extra-features/mcp_can_dfs.h    # expect no output
diff mcp_can.cpp   ../../../jeep-aux-and-extra-features/mcp_can.cpp      # expect only the LOCAL PATCH hunk
```
