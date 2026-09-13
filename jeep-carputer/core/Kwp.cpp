// KWP2000 request building, response classification and DTC decoding.
//
// PURE: <stdint.h> / <string.h> and the project headers only. Static storage,
// no allocation, bounded loops.
//
// Cross-checked byte for byte against the live capture of this vehicle
// (fixtures/CaptureIsoTp.h) and against the extracted decode tables
// (fixtures/DtcCodePairs.h, fixtures/DtcLiveScan.h).
#include "Kwp.h"

#include <string.h>

namespace {

// Letter for the top two bits of a raw DTC. Derived from -- not guessed at --
// all 1152 (raw, SAE) pairs in the extracted tables; see kwpDtcToSae below.
const char kDtcLetters[4] = {'P', 'C', 'B', 'U'};

const char kHexDigits[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                             '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

// Bytes of the DTC status byte we can actually justify from recorded data.
// live_dtc_scan.csv classifies 0x20, 0x27 and 0x30 as "stored" and 0x60 and
// 0xE0 as "active", which pins bit 6 as "failing now" and bit 5 as "present in
// memory" and nothing else. Bit 7 only ever appears together with bit 6
// (0xE0), so reading it as a lamp request is inference, flagged as such.
const uint8_t kDtcStatusActive = 0x40u;
const uint8_t kDtcStatusStored = 0x20u;
const uint8_t kDtcStatusLamp = 0x80u;

// A DTC reply is `58 <count>` followed by fixed-size records.
const uint8_t kDtcHeaderLen = 2u;
const uint8_t kDtcRecordLen = 3u;

}  // namespace

// ---- builders --------------------------------------------------------------

uint8_t kwpBuildReadLocalId(uint16_t localId, uint8_t* out, uint8_t cap) {
  if (out == 0) {
    return 0u;
  }
  // Every readDataByLocalIdentifier in the capture is a one-byte identifier
  // (`21 EA`, `21 36`, `21 E1`, ...), which is what the two-byte total in
  // IsoTp.h's comment refers to. A caller that hands us something above 0xFF
  // gets it emitted big-endian rather than silently truncated; it still fits
  // in one frame.
  // SignalDb and the poll plan both spell a PID service-qualified -- CCN
  // "2103" is service 0x21, identifier 0x03 -- because that is how the vendor
  // database names them. Accept that form and emit the two bytes the bus
  // actually carries. Every recorded request confirms the shape: the captures
  // hold "022103" / "022136" / "02213E", i.e. PCI 02 then 21 xx.
  //
  // Without this, 0x2103 was emitted as `21 21 03` and every module answered
  // nothing. It stayed hidden because the tests only covered a bare one-byte
  // identifier and a genuine two-byte one (0xF1A0), and nothing had ever run
  // against a vehicle.
  if ((localId >> 8) == KWP_SVC_READ_LOCAL_ID) {
    localId &= 0xFFu;
  }
  if (localId <= 0xFFu) {
    if (cap < 2u) {
      return 0u;
    }
    out[0] = KWP_SVC_READ_LOCAL_ID;
    out[1] = (uint8_t)localId;
    return 2u;
  }
  if (cap < 3u) {
    return 0u;
  }
  out[0] = KWP_SVC_READ_LOCAL_ID;
  out[1] = (uint8_t)(localId >> 8);
  out[2] = (uint8_t)(localId & 0xFFu);
  return 3u;
}

uint8_t kwpBuildReadEcuIdent(uint8_t option, uint8_t* out, uint8_t cap) {
  if (out == 0 || cap < 2u) {
    return 0u;
  }
  out[0] = KWP_SVC_READ_ECU_IDENT;
  out[1] = option;
  return 2u;
}

uint8_t kwpBuildReadDtc(uint8_t* out, uint8_t cap) {
  if (out == 0 || cap < 4u) {
    return 0u;
  }
  // `18 00 FF 00` = readDiagnosticTroubleCodesByStatus, status mask 0x00
  // (all), group 0xFF00 (all groups). Byte-for-byte what the app sent to all
  // 27 modules in the capture, as `04 18 00 FF 00` on the wire.
  out[0] = KWP_SVC_READ_DTC;
  out[1] = 0x00u;
  out[2] = 0xFFu;
  out[3] = 0x00u;
  return 4u;
}

// ---- response classification ----------------------------------------------

KwpStatus kwpParseResponse(const uint8_t* request, uint8_t requestLen,
                           const IsoTpPdu& pdu, KwpResponse& out) {
  out.data = 0;
  out.len = 0u;
  out.service = 0u;
  out.nrc = 0u;

  if (request == 0 || requestLen == 0u) {
    return KWP_BAD_PARAM;
  }
  if (pdu.len > ISOTP_MAX_PDU) {
    return KWP_BAD_PARAM;
  }
  if (pdu.len == 0u) {
    return KWP_SHORT;
  }

  const uint8_t requested = request[0];

  if (pdu.data[0] == KWP_NEGATIVE_MARKER) {
    // `7F <echoed service> <nrc>`. Anything shorter tells us nothing.
    if (pdu.len < 3u) {
      return KWP_SHORT;
    }
    out.service = pdu.data[1];
    out.nrc = pdu.data[2];
    // The echo is checked on negatives too: a rejection that names a service
    // we did not ask about belongs to somebody else's request. Every negative
    // in the capture echoes correctly (`21 05` -> `7F 21 31`, `22 F1 A0` ->
    // `7F 22 11`, `18 00 FF 00` -> `7F 18 78`), so this is safe to enforce.
    if (out.service != requested) {
      return KWP_WRONG_SERVICE;
    }
    // data/len point into the caller's pdu -- never copied, never a pointer to
    // anything of ours.
    out.data = pdu.data;
    out.len = pdu.len;
    return (out.nrc == KWP_NRC_PENDING) ? KWP_PENDING : KWP_NEGATIVE;
  }

  out.service = pdu.data[0];
  out.data = pdu.data;
  out.len = pdu.len;

  if (out.service != (uint8_t)(requested + KWP_POSITIVE_OFFSET)) {
    return KWP_WRONG_SERVICE;
  }

  // For the two services that echo the identifier back in byte 1 -- and only
  // when we sent a one-byte identifier -- check that echo as well. Every
  // recorded `21 xx` / `1A xx` reply does echo it (`21 EA` -> `61 EA ...`,
  // `1A 87` -> `5A 87 ...`), and a reply carrying the wrong identifier would
  // otherwise be decoded as if it were the right one, which in this project
  // means a wrong number on a gauge. Deliberately NOT applied to 0x18, whose
  // byte 1 is a record count and not an echo.
  if ((requested == KWP_SVC_READ_LOCAL_ID || requested == KWP_SVC_READ_ECU_IDENT) &&
      requestLen == 2u) {
    if (pdu.len < 2u) {
      return KWP_SHORT;
    }
    if (pdu.data[1] != request[1]) {
      return KWP_WRONG_SERVICE;
    }
  }

  return KWP_OK;
}

// ---- DTCs ------------------------------------------------------------------

uint8_t kwpParseDtcList(const KwpResponse& response, DtcRecord* out, uint8_t cap,
                        bool& truncated) {
  truncated = false;

  if (response.data == 0 || response.len < kDtcHeaderLen) {
    return 0u;
  }
  if (response.data[0] != (uint8_t)(KWP_SVC_READ_DTC + KWP_POSITIVE_OFFSET)) {
    return 0u;  // not a DTC reply at all
  }

  uint16_t len = response.len;
  if (len > ISOTP_MAX_PDU) {
    len = ISOTP_MAX_PDU;  // defensive: never index past a real IsoTpPdu
  }

  const uint8_t claimed = response.data[1];
  // How many whole records the payload can actually supply. A count byte that
  // disagrees with the length -- in either direction -- is bounded here, so
  // the copy loop below can never read past the payload.
  const uint16_t availableWide = (uint16_t)((len - kDtcHeaderLen) / kDtcRecordLen);
  const uint8_t available = (availableWide > 0xFFu) ? 0xFFu : (uint8_t)availableWide;

  uint8_t n = claimed;
  if (n > available) {
    n = available;
    truncated = true;  // module claimed more records than it sent
  }
  if (out == 0 || cap == 0u) {
    if (n > 0u) {
      truncated = true;
    }
    return 0u;
  }
  if (n > cap) {
    n = cap;
    truncated = true;  // more records than the caller can hold
  }

  for (uint8_t i = 0u; i < n; ++i) {
    const uint16_t base = (uint16_t)(kDtcHeaderLen + (uint16_t)i * kDtcRecordLen);
    out[i].code = (uint16_t)(((uint16_t)response.data[base] << 8) |
                             (uint16_t)response.data[base + 1u]);
    out[i].status = response.data[base + 2u];
  }
  return n;
}

bool kwpDtcToSae(uint16_t code, char* buf, uint8_t cap) {
  if (buf == 0 || cap < 6u) {
    return false;
  }
  // Derived from all 1152 (raw_code_hex_2byte, sae_code) pairs in
  // extracted/dtc_codes.csv and re-verified in test/test_kwp against every one
  // of them:
  //   bits 15..14 -> letter,      index into "PCBU"
  //   bits 13..12 -> first digit, 0..3 (so never needs hex)
  //   bits 11..0  -> the last three characters, as upper-case hex digits
  // e.g. 0x0119 -> P0119, 0x5008 -> C1008, 0x9851 -> B1851, 0xD504 -> U1504,
  // and 0x213A -> P213A, which is why the tail must be hex and not decimal.
  buf[0] = kDtcLetters[(code >> 14) & 0x3u];
  buf[1] = kHexDigits[(code >> 12) & 0x3u];
  buf[2] = kHexDigits[(code >> 8) & 0xFu];
  buf[3] = kHexDigits[(code >> 4) & 0xFu];
  buf[4] = kHexDigits[code & 0xFu];
  buf[5] = '\0';
  // Every 16-bit value has a rendering under this rule, 0x0000 ("P0000")
  // included; whether a code is meaningful is the caller's business, not the
  // formatter's. So the only way to fail is a buffer we cannot write.
  return true;
}

const char* kwpDtcStatusText(uint8_t status) {
  if (status == 0u) {
    return "no fault";
  }
  const bool active = (status & kDtcStatusActive) != 0u;
  const bool stored = (status & kDtcStatusStored) != 0u;
  const bool lamp = (status & kDtcStatusLamp) != 0u;
  if (active) {
    return lamp ? "active, lamp on" : "active";
  }
  if (stored) {
    return lamp ? "stored, lamp on" : "stored";
  }
  if (lamp) {
    return "lamp on";
  }
  // The remaining bits (0x1F) do occur on this vehicle -- 0x27 and 0x30 were
  // both recorded -- but nothing in the captured or extracted data says what
  // they mean, so they are not invented into a label here.
  return "unknown";
}
