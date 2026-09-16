#pragma once
// KWP2000 request building and response parsing.
//
// This vehicle needs no session setup and no security access on any read path:
// a request goes straight out and the module answers. There is also no gateway
// hop -- every module is addressed directly by its own request/response ids.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "IsoTp.h"

// Services we use.
#define KWP_SVC_CLEAR_DTC       0x14u  // clearDiagnosticInformation
#define KWP_SVC_READ_DTC        0x18u  // readDiagnosticTroubleCodesByStatus
#define KWP_SVC_READ_ECU_IDENT  0x1Au  // readEcuIdentification
#define KWP_SVC_READ_LOCAL_ID   0x21u  // readDataByLocalIdentifier

// A positive response echoes the service byte + 0x40.
#define KWP_POSITIVE_OFFSET     0x40u
#define KWP_NEGATIVE_MARKER     0x7Fu
#define KWP_NRC_PENDING         0x78u  // "response pending, keep waiting"

enum KwpStatus : uint8_t {
  KWP_OK = 0,
  KWP_PENDING,        // 7F xx 78 -- caller should wait and read again
  KWP_NEGATIVE,       // 7F xx nn -- see KwpResponse::nrc
  KWP_WRONG_SERVICE,  // reply does not correspond to the request
  KWP_SHORT,          // truncated
  KWP_BAD_PARAM,
};

struct KwpResponse {
  const uint8_t* data;   // points into the caller's IsoTpPdu, service byte included
  uint16_t       len;
  uint8_t        service;
  uint8_t        nrc;    // meaningful only when status is KWP_NEGATIVE / KWP_PENDING
};

// Builders. Return the number of bytes written, or 0 if `cap` is too small.
// These emit the bare service PDU: the ISO-TP PCI byte is added by IsoTp.
uint8_t kwpBuildReadLocalId(uint16_t localId, uint8_t* out, uint8_t cap);
uint8_t kwpBuildReadEcuIdent(uint8_t option, uint8_t* out, uint8_t cap);
uint8_t kwpBuildReadDtc(uint8_t* out, uint8_t cap);   // 18 00 FF 00, all groups

// Clear ONE stored code: `14 <code_hi> <code_lo>` -- clearDiagnosticInformation
// with that raw code standing in the two-byte groupOfDTC field. `14 FF 00` is
// the all-groups form that wipes a module's whole fault memory; naming a single
// group is how KWP2000 says "only this one", and it is the only form this car
// can use, because it carries permanent known faults that clearing everything
// would destroy along with the one code actually being dismissed.
//
// UNVERIFIED on this vehicle: service 0x14 appears nowhere in the capture, so
// these modules may honour only the all-groups form, may ignore the group field
// and clear the lot anyway, or may reject 0x14 outside a started diagnostic
// session. Every read path here needs no session, which says nothing about a
// write. Expect a KWP_NEGATIVE and read it as the answer, not as a bug.
uint8_t kwpBuildClearDtc(uint16_t code, uint8_t* out, uint8_t cap);

// Classify a reassembled reply against the request that produced it.
KwpStatus kwpParseResponse(const uint8_t* request, uint8_t requestLen,
                           const IsoTpPdu& pdu, KwpResponse& out);

// ---- DTCs ----------------------------------------------------------------
//
// A positive DTC reply is 58 <count> followed by <count> records of
// {code_hi, code_lo, status}.

struct DtcRecord {
  uint16_t code;     // raw two-byte code as sent
  uint8_t  status;   // raw status byte
};

// Returns the number of records written to `out`, capped at `cap`. `truncated`
// is set when the module reported more records than would fit.
uint8_t kwpParseDtcList(const KwpResponse& response, DtcRecord* out, uint8_t cap,
                        bool& truncated);

// Render a raw code as its SAE string, e.g. 0x0119 -> "P0119". `buf` needs 6
// bytes. Returns false if the code cannot be rendered.
//
// IMPORTANT for whoever implements this: do not guess the letter derivation.
// D:\Repos\car\extracted\dtc_codes.csv contains 1152 rows with BOTH
// raw_code_hex_2byte and sae_code, so the mapping can be derived from real data
// and then verified against every one of those pairs in a unit test.
bool kwpDtcToSae(uint16_t code, char* buf, uint8_t cap);

// Human-readable status decode (test failed now / stored / pending etc.).
// Returns a static string, never NULL.
const char* kwpDtcStatusText(uint8_t status);
