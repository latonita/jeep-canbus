#pragma once
// ISO 15765-2 on the host side: build the request, reassemble the reply.
//
// Split deliberately in two. IsoTpReassembler is a pure state machine with no
// link and no clock, so it can be unit-tested against recorded frame
// sequences; IsoTpClient is the thin part that drives an ICarLink.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "CarLink.h"

// Longest response we will hold. The biggest thing we ask for is a DTC list;
// 128 bytes is far more than any module on this vehicle returns, and it is
// static, so being generous costs nothing but .bss.
#define ISOTP_MAX_PDU 128u

enum IsoTpStatus : uint8_t {
  ISOTP_OK = 0,
  ISOTP_INCOMPLETE,     // valid so far, more frames expected
  ISOTP_NO_RESPONSE,    // deadline passed with nothing usable
  ISOTP_OVERFLOW,       // declared length exceeds ISOTP_MAX_PDU
  ISOTP_PROTOCOL,       // sequence number gap, bad PCI, frame out of order
  ISOTP_REQ_TOO_LONG,   // multi-frame requests are not supported -- see below
  ISOTP_LINK_ERROR,
  ISOTP_BAD_PARAM,
};

// PCI frame types, high nibble of data[0].
enum IsoTpType : uint8_t {
  ISOTP_TYPE_SINGLE = 0,
  ISOTP_TYPE_FIRST,
  ISOTP_TYPE_CONSECUTIVE,
  ISOTP_TYPE_FLOW,
  ISOTP_TYPE_UNKNOWN,
};

struct IsoTpPdu {
  uint8_t  data[ISOTP_MAX_PDU];
  uint16_t len;
};

IsoTpType isoTpClassify(const CanFrame& frame);

// Pure reassembler. feed() one frame at a time; ISOTP_OK means `out` holds a
// complete PDU. Any error leaves the reassembler reset and ready for a retry.
class IsoTpReassembler {
 public:
  IsoTpReassembler();

  void reset();

  IsoTpStatus feed(const CanFrame& frame, IsoTpPdu& out);

  // True once a First Frame has been seen and consecutive frames are expected.
  bool inProgress() const { return inProgress_; }

  // Declared total length from the First Frame, 0 until one is seen.
  uint16_t expected() const { return expected_; }

 private:
  IsoTpPdu buf_;
  uint16_t expected_;
  uint8_t  nextSeq_;
  bool     inProgress_;
};

// Drives a link through one request/response exchange.
//
// Single-frame requests only. Every request this project sends fits in one
// frame -- readDataByLocalIdentifier is 3 bytes, readDiagnosticTroubleCodes is
// 4 -- and supporting multi-frame transmission would mean handling the ECU's
// Flow Control frame, which is real work for no current benefit. Longer
// requests return ISOTP_REQ_TOO_LONG rather than silently truncating.
class IsoTpClient {
 public:
  explicit IsoTpClient(ICarLink& link);

  IsoTpStatus transact(uint16_t requestId, uint16_t responseId,
                       const uint8_t* request, uint8_t requestLen,
                       IsoTpPdu& out, uint32_t timeoutMs);

 private:
  ICarLink& link_;
  IsoTpReassembler reasm_;
};
