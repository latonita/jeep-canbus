#pragma once
// The single piece of ISO-TP (ISO 15765-2) that the adapter itself has to do.
//
// Under ATCAF0 the host builds the PCI bytes and reassembles multi-frame
// replies itself -- this board stays a dumb frame pipe. But there is one thing
// the host cannot do in time: when a module answers with a First Frame it will
// not send any Consecutive Frame until it receives a Flow Control frame. A
// round trip out to the laptop is far too slow, so the ADAPTER must answer the
// FF. Without this, every multi-frame response -- which means every DTC list --
// stalls after the first 6 bytes.
//
// PURE: only <stdint.h> and the dependency-free Core/CanFrame.h. No I/O here:
// isoTpBuildFc() returns the frame and the caller sends it, which keeps this
// module host-testable (test/test_isotp_fc).
#include <stdint.h>

#include "CanFrame.h"

// PCI high nibble of data[0].
#define ISOTP_PCI_SINGLE  0x0u
#define ISOTP_PCI_FIRST   0x1u
#define ISOTP_PCI_CONSEC  0x2u
#define ISOTP_PCI_FLOW    0x3u

// "No ATFCSH given" -- fall back to the current request header.
#define ISOTP_FC_HEADER_UNSET  0xFFFFu

// Clear to send, block size unlimited, STmin 0.
#define ISOTP_FC_DEFAULT_B0    0x30u
#define ISOTP_FC_DEFAULT_B1    0xFFu
#define ISOTP_FC_DEFAULT_B2    0x00u
#define ISOTP_FC_DEFAULT_LEN   3u

// ATFCSM, kept for ELM/doc compatibility (docs/AT-COMMANDS.md):
//   0 = auto, the adapter builds the FC frame
//   1 = use the ATFCSD data with the ATFCSH id
//   2 = use the ATFCSD data with the auto id
// The adapter ALWAYS answers a First Frame -- there is no mode that suppresses
// it, because a suppressed FC means every multi-frame reply stalls. The mode
// is stored and validated, but id and data resolution is uniform and needs no
// mode: ATFCSH if it was set, otherwise the request header; ATFCSD if it was
// set, otherwise 30 FF 00. So mode 1 behaves as documented (its purpose is
// the "pad the FC to DLC 8" workaround, which is really just an ATFCSD).
#define ISOTP_FC_MODE_AUTO     0u
#define ISOTP_FC_MODE_FCSH     1u
#define ISOTP_FC_MODE_FCSD     2u
#define ISOTP_FC_MODE_MAX      2u

enum IsoTpType : uint8_t {
  ISOTP_TYPE_UNKNOWN = 0,   // empty frame, or a PCI nibble ISO-TP does not use
  ISOTP_TYPE_SINGLE,
  ISOTP_TYPE_FIRST,
  ISOTP_TYPE_CONSEC,
  ISOTP_TYPE_FLOW,
};

// 12 bytes. Set by ATFCSH / ATFCSD / ATFCSM, reset by ATZ.
struct IsoTpFcConfig {
  uint16_t header;             // ISOTP_FC_HEADER_UNSET -> use request header
  uint8_t  data[CAN_MAX_DLC];
  uint8_t  len;                // 1..CAN_MAX_DLC
  uint8_t  mode;               // ISOTP_FC_MODE_*
};

enum IsoTpFcStatus : uint8_t {
  ISOTP_FC_OK = 0,
  ISOTP_FC_NOT_FIRST_FRAME,   // nothing to answer
  ISOTP_FC_NO_HEADER,         // no ATFCSH and no usable request header
  ISOTP_FC_BAD_CONFIG,        // ATFCSD length, mode or header out of range
};

void isoTpFcDefaults(IsoTpFcConfig& cfg);

IsoTpType isoTpClassify(const CanFrame& frame);

// Convenience for the hot path. Safe on len == 0: an empty frame carries no
// PCI byte at all, so it is not a First Frame.
bool isoTpIsFirstFrame(const CanFrame& frame);

// FF_DL, the total message length a First Frame announces (12 bits, so up to
// 4095). Returns 0 when the frame is not a First Frame or is too short to
// carry the length field.
uint16_t isoTpFirstFrameLen(const CanFrame& frame);

// SF_DL, the payload length a Single Frame announces (0..7). Returns 0 when
// the frame is not a Single Frame.
uint8_t isoTpSingleFrameLen(const CanFrame& frame);

// Payload bytes carried by a Consecutive Frame (len - 1). 0 when not a CF.
uint8_t isoTpConsecPayloadLen(const CanFrame& frame);

// Build the Flow Control frame that answers `first`.
//
// `requestHeader` is the id the pending request went out on; it is used when
// ATFCSH was never set. Exactly cfg.len bytes are sent, with NO padding to
// DLC 8 -- that is what ISO 15765 asks for and what every module tested on
// this bus accepts. If some module ever ignores an unpadded FC, padding the
// frame out to DLC 8 with 0x00 is the first thing to try (ELM327 pads when
// ATCFC/ATCAF are on), and the ATFCSD data bytes are the place to do it from
// the host side: "ATFCSD 30 FF 00 00 00 00 00 00".
IsoTpFcStatus isoTpBuildFc(const IsoTpFcConfig& cfg,
                           const CanFrame& first,
                           uint16_t requestHeader,
                           CanFrame& out);
