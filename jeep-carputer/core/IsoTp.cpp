// ISO 15765-2 reassembly and one-shot request/response, host side.
//
// PURE: <stdint.h> / <string.h> and the project headers only. Static storage,
// no allocation, bounded loops.
//
// Everything here was checked against a real capture of this vehicle
// (fixtures/CaptureIsoTp.h, from capture-20260910-120048.jsonl). Two facts from
// that capture drive the design:
//
//  1. Frames arrive fully padded to 8 bytes and the padding is NOT reliably
//     zero -- it is sometimes 0x00, sometimes 0xFF, and quite often stale bytes
//     left over from an earlier reply (e.g. WCM 600 `18 00 FF 00` answers with
//     `10 08 58 02 40 77 27 A1` / `21 02 27 50 20 00 07 35`, where `50 20 00
//     07 35` is junk from the previous exchange). The declared length is the
//     only truth; frame content past it is discarded.
//
//  2. A `7F xx 78` responsePending Single Frame is followed, inside the same
//     read window, by the First Frame of the real answer. So a Single Frame
//     completes a PDU immediately, and a First Frame must be able to start a
//     fresh sequence at any moment without inheriting anything.
#include "IsoTp.h"

#include <string.h>

namespace {

// PCI nibbles.
const uint8_t kPciSingle = 0x0u;
const uint8_t kPciFirst = 0x1u;
const uint8_t kPciConsecutive = 0x2u;
const uint8_t kPciFlow = 0x3u;

// A First Frame declares a length that would not have fit in a Single Frame,
// i.e. at least 8. The shortest First Frame in the capture is exactly `10 08`,
// so anything below 8 is malformed rather than merely unusual.
const uint16_t kMinFirstFrameLen = 8u;

// Largest 11-bit CAN identifier.
const uint16_t kMaxCanId = 0x7FFu;

// Worst case frame count for a full-size PDU: one First Frame carrying 6 bytes
// plus ceil((ISOTP_MAX_PDU - 6) / 7) Consecutive Frames.
const uint8_t kMaxPduFrames = 1u + (uint8_t)(((ISOTP_MAX_PDU - 6u) + 6u) / 7u);

// Frames transact() is willing to pull for one exchange. Generous over
// kMaxPduFrames so that Flow Control frames, or a responsePending Single Frame
// that the caller chose not to treat as the answer, cannot starve the loop --
// but still a hard bound, so a chattering adapter cannot spin us forever.
const uint8_t kMaxFramesPerTransact = (uint8_t)(2u * kMaxPduFrames + 8u);

// ICarLink offers no clock, so transact() cannot compute a true deadline. It
// spends the caller's whole budget waiting for the first frame -- that is where
// the latency actually is, up to ~140 ms on this vehicle when a module answers
// responsePending first -- and then allows only a short gap between frames of
// one burst. 150 ms is ISO 15765-2's N_Cr; in the capture the real inter-frame
// gap inside a burst is under a millisecond, so this is pure slack.
const uint32_t kConsecutiveFrameTimeoutMs = 150u;

IsoTpStatus mapLinkError(LinkStatus st) {
  switch (st) {
    case LINK_OK:
      return ISOTP_OK;
    case LINK_NO_DATA:
      return ISOTP_NO_RESPONSE;
    case LINK_BAD_PARAM:
      return ISOTP_BAD_PARAM;
    case LINK_TIMEOUT:
    case LINK_CAN_ERROR:
    case LINK_OVERFLOW:
    case LINK_PROTOCOL:
    case LINK_NOT_READY:
      return ISOTP_LINK_ERROR;
    default:
      return ISOTP_LINK_ERROR;
  }
}

}  // namespace

IsoTpType isoTpClassify(const CanFrame& frame) {
  // No PCI byte at all: nothing to classify.
  if (frame.len == 0u || frame.len > CAN_MAX_DLC) {
    return ISOTP_TYPE_UNKNOWN;
  }
  switch ((uint8_t)(frame.data[0] >> 4)) {
    case kPciSingle:
      return ISOTP_TYPE_SINGLE;
    case kPciFirst:
      return ISOTP_TYPE_FIRST;
    case kPciConsecutive:
      return ISOTP_TYPE_CONSECUTIVE;
    case kPciFlow:
      return ISOTP_TYPE_FLOW;
    default:
      return ISOTP_TYPE_UNKNOWN;
  }
}

IsoTpReassembler::IsoTpReassembler() { reset(); }

void IsoTpReassembler::reset() {
  buf_.len = 0u;
  expected_ = 0u;
  nextSeq_ = 0u;
  inProgress_ = false;
  // buf_.data is deliberately left alone: it is only ever read up to buf_.len.
}

IsoTpStatus IsoTpReassembler::feed(const CanFrame& frame, IsoTpPdu& out) {
  const IsoTpType type = isoTpClassify(frame);

  switch (type) {
    case ISOTP_TYPE_FLOW:
      // A module should never send Flow Control to us -- we are the one being
      // sent to, and the ATmega328 answers the module's FC on our behalf. If
      // one turns up anyway (bus noise, a second tester on the bus, an adapter
      // echoing its own auto-FC) it is not data: skip it and leave any
      // in-progress reassembly untouched. Reported as INCOMPLETE, meaning
      // "nothing wrong, keep reading".
      return ISOTP_INCOMPLETE;

    case ISOTP_TYPE_SINGLE: {
      const uint8_t declared = (uint8_t)(frame.data[0] & 0x0Fu);
      // A Single Frame that claims more than the frame physically carries, or
      // claims nothing at all, is unusable. Note frame.len >= 1 here.
      if (declared == 0u || declared > (uint8_t)(frame.len - 1u)) {
        reset();
        return ISOTP_PROTOCOL;
      }
      // A Single Frame is self-contained, so it abandons any partial sequence
      // rather than being merged into it.
      reset();
      memcpy(out.data, &frame.data[1], declared);
      out.len = declared;
      return ISOTP_OK;
    }

    case ISOTP_TYPE_FIRST: {
      if (frame.len < 2u) {
        reset();
        return ISOTP_PROTOCOL;
      }
      const uint16_t declared =
          (uint16_t)(((uint16_t)(frame.data[0] & 0x0Fu) << 8) | (uint16_t)frame.data[1]);
      if (declared > ISOTP_MAX_PDU) {
        reset();
        return ISOTP_OVERFLOW;
      }
      if (declared < kMinFirstFrameLen) {
        reset();
        return ISOTP_PROTOCOL;
      }
      // Restart, never merge: a First Frame mid-sequence means the previous
      // sequence is dead (module gave up, or we are looking at the answer that
      // followed a responsePending).
      reset();
      uint8_t take = (uint8_t)(frame.len - 2u);
      if ((uint16_t)take > declared) {
        take = (uint8_t)declared;
      }
      memcpy(buf_.data, &frame.data[2], take);
      buf_.len = take;
      expected_ = declared;
      nextSeq_ = 1u;
      inProgress_ = true;
      // declared >= 8 and take <= 6, so this can never already be complete.
      return ISOTP_INCOMPLETE;
    }

    case ISOTP_TYPE_CONSECUTIVE: {
      if (!inProgress_) {
        // A Consecutive Frame with no First Frame in front of it: a leftover
        // from an exchange we already finished, or a genuinely out-of-order
        // module. Either way there is no safe place to put the bytes.
        reset();
        return ISOTP_PROTOCOL;
      }
      if (frame.len < 2u) {
        // Carries no payload; accepting it would stall the sequence forever.
        reset();
        return ISOTP_PROTOCOL;
      }
      const uint8_t seq = (uint8_t)(frame.data[0] & 0x0Fu);
      if (seq != nextSeq_) {
        reset();
        return ISOTP_PROTOCOL;
      }
      const uint16_t remaining = (uint16_t)(expected_ - buf_.len);
      uint8_t take = (uint8_t)(frame.len - 1u);
      if ((uint16_t)take > remaining) {
        take = (uint8_t)remaining;  // drop the pad / stale tail
      }
      memcpy(&buf_.data[buf_.len], &frame.data[1], take);
      buf_.len = (uint16_t)(buf_.len + take);
      nextSeq_ = (uint8_t)((nextSeq_ + 1u) & 0x0Fu);  // wraps 15 -> 0
      if (buf_.len >= expected_) {
        memcpy(out.data, buf_.data, buf_.len);
        out.len = buf_.len;
        reset();
        return ISOTP_OK;
      }
      return ISOTP_INCOMPLETE;
    }

    case ISOTP_TYPE_UNKNOWN:
      reset();
      return ISOTP_PROTOCOL;

    default:
      reset();
      return ISOTP_PROTOCOL;
  }
}

IsoTpClient::IsoTpClient(ICarLink& link) : link_(link), reasm_() {}

IsoTpStatus IsoTpClient::transact(uint16_t requestId, uint16_t responseId,
                                  const uint8_t* request, uint8_t requestLen,
                                  IsoTpPdu& out, uint32_t timeoutMs) {
  out.len = 0u;

  if (request == 0 || requestLen == 0u) {
    return ISOTP_BAD_PARAM;
  }
  if (requestId > kMaxCanId || responseId > kMaxCanId) {
    return ISOTP_BAD_PARAM;
  }
  // One Single Frame holds a PCI byte plus seven payload bytes. Every request
  // this project sends is well inside that (see the note in IsoTp.h), so the
  // honest answer to a longer one is to refuse it.
  if (requestLen > (uint8_t)(CAN_MAX_DLC - 1u)) {
    return ISOTP_REQ_TOO_LONG;
  }

  LinkStatus ls = link_.setTarget(requestId, responseId);
  if (ls != LINK_OK) {
    return mapLinkError(ls);
  }

  CanFrame tx;
  tx.id = requestId;
  tx.len = (uint8_t)(requestLen + 1u);
  memset(tx.data, 0, sizeof(tx.data));
  tx.data[0] = requestLen;  // Single Frame PCI: high nibble 0, low nibble = length
  memcpy(&tx.data[1], request, requestLen);

  ls = link_.sendFrame(tx);
  if (ls != LINK_OK) {
    return mapLinkError(ls);
  }

  reasm_.reset();

  bool sawFrame = false;
  for (uint8_t i = 0u; i < kMaxFramesPerTransact; ++i) {
    const uint32_t wait = sawFrame ? (timeoutMs < kConsecutiveFrameTimeoutMs
                                          ? timeoutMs
                                          : kConsecutiveFrameTimeoutMs)
                                   : timeoutMs;
    CanFrame rx;
    rx.id = 0u;
    rx.len = 0u;
    memset(rx.data, 0, sizeof(rx.data));

    ls = link_.readFrame(rx, wait);
    if (ls == LINK_NO_DATA) {
      // Deadline passed. Partial data is not usable, so it is still "no
      // response" from the caller's point of view.
      reasm_.reset();
      return ISOTP_NO_RESPONSE;
    }
    if (ls != LINK_OK) {
      reasm_.reset();
      return mapLinkError(ls);
    }
    sawFrame = true;

    // The adapter filters by ATCRA, so software id filtering is belt and
    // braces. It is deliberately skipped when the link reports id 0: a link
    // running with headers off (as the recorded session did, ATH0) has no id
    // to report, and dropping every frame would be far worse than trusting the
    // adapter's own filter.
    if (rx.id != 0u && rx.id != responseId) {
      continue;
    }

    const IsoTpStatus st = reasm_.feed(rx, out);
    if (st == ISOTP_OK) {
      return ISOTP_OK;
    }
    if (st != ISOTP_INCOMPLETE) {
      // feed() has already reset itself; hand the reason up so the caller can
      // distinguish "module misbehaved" from "module said nothing".
      out.len = 0u;
      return st;
    }
  }

  // Ran out of frame budget with nothing complete.
  reasm_.reset();
  out.len = 0u;
  return ISOTP_NO_RESPONSE;
}
