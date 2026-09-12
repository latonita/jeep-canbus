#pragma once
// Board wiring and tuning for the in-radio ATmega328 (Pro Mini 5V/16MHz).
#include <stdint.h>

// ---- Feature modules -----------------------------------------------------
// A disabled module is not instantiated and not linked.
#ifndef FEATURE_VES_AUX
#define FEATURE_VES_AUX     1
#endif
#ifndef FEATURE_BT_CONTROL
#define FEATURE_BT_CONTROL  1
#endif
#ifndef FEATURE_BENCH_MODE
#define FEATURE_BENCH_MODE  1
#endif
#ifndef FEATURE_AT_SERVER
#define FEATURE_AT_SERVER   1
#endif

// ---- Pins (Arduino numbering) -------------------------------------------
#define PIN_CAN_CS          10    // PB2 -- MCP2515 chip select
// BT power switch, HIGH = on. BOARD REWORK: the schematic routes this from PC1
// (physical pin 24), but on the real PCB that trace is cut and BT_ON is
// jumpered to physical pin 1 = PD3 = Arduino D3 -- pin 24 is presumed dead.
// Follow the board, not the schematic; it has already been wrong about the
// part number and about ICSP RESET.
#define PIN_BT_POWER        3     // PD3, physical pin 1 (was PC1 / pin 24)

// ACTIVE LOW. Verified in the car 2026-09-11: driving the pin HIGH left the BT
// module off and LOW turned it on -- exactly backwards from the pre-rework
// firmware. Q2/Q3 are IRLML6402, P-channel high-side switches, which conduct
// when the gate is pulled LOW; the rework that moved BT_ON from pin 24 to pin 1
// evidently bypassed the 2N7002 (Q1) that used to invert the MCU signal.
//
// Written as levels rather than flipped literals so the sense is stated once
// and can be put back if the board is ever reworked again.
#define BT_POWER_ON_LEVEL   LOW
#define BT_POWER_OFF_LEVEL  HIGH

// BT module UART. The board is 5V and the module 3.3V, so the Arduino TX line
// goes through a Schottky diode: it can pull the module's RX low but never
// drive it to 5V. RX is wired but never read -- the pin number exists only to
// satisfy the SoftwareSerial constructor.
#define PIN_BT_UART_RX      A4    // <- BT module TX (unused)
#define PIN_BT_UART_TX      A2    // -> BT module RX (via diode)

// ---- Serial --------------------------------------------------------------
#define LINK_BAUD           115200UL   // hardware UART: the AT link
#define BT_BAUD             115200UL   // SoftwareSerial to the OVC3860

// ---- Timing (ms) ---------------------------------------------------------
#define VES_ANNOUNCE_PERIOD_MS   500u
#define BENCH_ANNOUNCE_PERIOD_MS 500u
// Bench VIN: one frame per tick, round-robin over the 3 frames, so the full VIN
// lands every ~3x this. The car spaces its 0x01B frames evenly ~100 ms apart
// (measured in capture A: 0->1, 1->2 and 2->0 are all ~99 ms; full VIN ~300 ms
// -- NOT a burst-then-pause). 100 ms here reproduces that. Only in bench mode.
#define BENCH_VIN_PERIOD_MS      100u
// The radio latches the VIN once at power-up and stays unlocked -- confirmed on
// the bench 2026-09-12 -- so the VIN need only be sent a few times, not forever.
// After bench mode activates we send this many full 3-frame cycles, then stop
// the VIN (power-on keeps going). 0 = send indefinitely (the car's behaviour).
#define BENCH_VIN_BURST_CYCLES   10u
// Safety: this board is powered only when the radio soft-ons, and in the car the
// interior bus is already alive by then. So when bench mode is on we listen this
// long BEFORE transmitting anything; if a genuine 0x000 power frame is heard
// (another node sent it -- we never receive our own), we are in a vehicle and
// must NOT inject bench frames. Runtime-only: the EEPROM flag is untouched, so
// the same board self-re-enables bench once it is back on a silent bench bus.
#define BENCH_DETECT_WINDOW_MS   2000u
#define BUTTON_DEBOUNCE_MS       350u

// Supervisor thresholds.
#define CAN_SILENCE_REINIT_MS    3000u   // no frames for this long -> re-init
#define CAN_SILENCE_SAFE_MS     10000u   // still nothing -> safe state
#define WDT_PERIOD               WDTO_1S

// ---- CAN -----------------------------------------------------------------
#define CAN_BITRATE_ID      CAN_83K3BPS   // interior CAN-B

// MCP2515 crystal: 16 MHz. CONFIRMED ON THE CAR 2026-09-11 -- with this build
// the RX counter climbs at ~168 frames/s on the live interior bus and the
// radio accepts the VES announce, which is only possible if the bit timing
// matches. The question is closed; the 8 MHz path below is kept only because
// the vendored library carries the table. Override with
// -DCAN_CRYSTAL_8MHZ if a board ever turns up with the 8 MHz part.
//
// This is not interchangeable. At 83.3 kbps both tables use CFG2 = 0xBE and
// CFG3 = 0x07 (24 TQ per bit) and differ only in the CFG1 prescaler -- BRP 3
// for 16 MHz, BRP 1 for 8 MHz -- so since TQ = 2*(BRP+1)/Fosc, each gives
// TQ = 500 ns (24 x 500 ns = 12 us = 83.33 kbps) only with its own crystal.
// The wrong one puts the controller on the bus at the wrong bit rate and
// nothing works at all.
//
// Note for anyone comparing against the predecessor: jeep-aux-and-extra-features
// passed MCP_8MHz. The board is 16 MHz, so that value was wrong there too.
#ifdef CAN_CRYSTAL_8MHZ
#define CAN_CRYSTAL_ID      MCP_8MHz
#else
#define CAN_CRYSTAL_ID      MCP_16MHz
#endif

#define CAN_INIT_RETRY_MS   250u
#define CAN_RX_DRAIN_MAX    4u            // frames drained per loop iteration

// ---- AT server -----------------------------------------------------------
#define AT_LINE_MAX         48u   // longest accepted command line
#define AT_RESP_TIMEOUT_MS  200u  // default, overridden by ATST
