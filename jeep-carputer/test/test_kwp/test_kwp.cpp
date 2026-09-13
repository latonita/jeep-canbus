// KWP2000 host tests.
//
// Driven from three sets of real data copied into fixtures/:
//   * CaptureIsoTp.h  -- reassembled replies from a live session on this car
//   * DtcCodePairs.h  -- all 1152 (raw code, SAE string) pairs from the
//                        extracted decode tables, used to check kwpDtcToSae
//                        against every known pair rather than a hand-picked few
//   * DtcLiveScan.h   -- the real all-module DTC scan, 16 fault records
#include "doctest_min.h"

#include <stdint.h>
#include <string.h>

#include "IsoTp.h"
#include "Kwp.h"

#include "../../fixtures/CaptureIsoTp.h"
#include "../../fixtures/DtcCodePairs.h"
#include "../../fixtures/DtcLiveScan.h"

namespace {

void setPdu(IsoTpPdu& pdu, const uint8_t* bytes, uint16_t len) {
  memset(&pdu, 0, sizeof(pdu));
  if (len > ISOTP_MAX_PDU) {
    len = ISOTP_MAX_PDU;
  }
  memcpy(pdu.data, bytes, len);
  pdu.len = len;
}

const CapMultiSeq* findMulti(const char* name) {
  for (uint8_t i = 0; i < CAP_MULTI_COUNT; ++i) {
    if (strcmp(kCapMulti[i].name, name) == 0) {
      return &kCapMulti[i];
    }
  }
  return 0;
}

const CapSingleSeq* findSingle(const char* name) {
  for (uint8_t i = 0; i < CAP_SINGLE_COUNT; ++i) {
    if (strcmp(kCapSingle[i].name, name) == 0) {
      return &kCapSingle[i];
    }
  }
  return 0;
}

// Turn a recorded request frame ("041800FF00") into the bare service PDU.
uint8_t requestFromCap(const char* hex, uint8_t* out, uint8_t cap) {
  const uint8_t n = (uint8_t)(strlen(hex) / 2u);
  if (n == 0u || n > cap + 1u) {
    return 0u;
  }
  uint8_t bytes[16];
  for (uint8_t i = 0; i < n && i < 16u; ++i) {
    uint8_t v = 0u;
    for (uint8_t k = 0; k < 2u; ++k) {
      const char c = hex[i * 2u + k];
      const uint8_t d = (c >= '0' && c <= '9')
                            ? (uint8_t)(c - '0')
                            : (uint8_t)((c >= 'A' && c <= 'F') ? (c - 'A' + 10) : 0);
      v = (uint8_t)((v << 4) | d);
    }
    bytes[i] = v;
  }
  const uint8_t payload = (uint8_t)(bytes[0] & 0x0Fu);  // the SF PCI length
  if (payload == 0u || payload > cap || (uint8_t)(payload + 1u) > n) {
    return 0u;
  }
  memcpy(out, &bytes[1], payload);
  return payload;
}

}  // namespace

TEST_MAIN_BEGIN()

// -------------------------------------------------------------- builders
{
  uint8_t buf[8];

  memset(buf, 0xCCu, sizeof(buf));
  CHECK_EQ(kwpBuildReadLocalId(0xEAu, buf, sizeof(buf)), 2);
  CHECK_EQ(buf[0], KWP_SVC_READ_LOCAL_ID);
  CHECK_EQ(buf[1], 0xEA);
  CHECK_EQ(buf[2], 0xCC);  // nothing beyond the declared length was touched

  CHECK_EQ(kwpBuildReadLocalId(0x36u, buf, 2u), 2);
  CHECK_EQ(kwpBuildReadLocalId(0x36u, buf, 1u), 0);
  CHECK_EQ(kwpBuildReadLocalId(0x36u, 0, 8u), 0);

  // A two-byte identifier is emitted big-endian rather than truncated.
  CHECK_EQ(kwpBuildReadLocalId(0xF1A0u, buf, sizeof(buf)), 3);
  CHECK_EQ(buf[0], KWP_SVC_READ_LOCAL_ID);
  CHECK_EQ(buf[1], 0xF1);
  CHECK_EQ(buf[2], 0xA0);
  CHECK_EQ(kwpBuildReadLocalId(0xF1A0u, buf, 2u), 0);

  // Service-qualified ids, the form SignalDb and the poll plan use. These must
  // collapse to the two bytes the captures show, NOT emit the service byte
  // twice. Byte-matched against fixtures/CaptureIsoTp.h ("022103", "022136").
  CHECK_EQ(kwpBuildReadLocalId(0x2103u, buf, sizeof(buf)), 2);
  CHECK_EQ(buf[0], 0x21u);
  CHECK_EQ(buf[1], 0x03u);
  CHECK_EQ(kwpBuildReadLocalId(0x2136u, buf, sizeof(buf)), 2);
  CHECK_EQ(buf[0], 0x21u);
  CHECK_EQ(buf[1], 0x36u);
  CHECK_EQ(kwpBuildReadLocalId(0x213Bu, buf, sizeof(buf)), 2);
  CHECK_EQ(buf[0], 0x21u);
  CHECK_EQ(buf[1], 0x3Bu);

  memset(buf, 0xCCu, sizeof(buf));
  CHECK_EQ(kwpBuildReadEcuIdent(0x87u, buf, sizeof(buf)), 2);
  CHECK_EQ(buf[0], KWP_SVC_READ_ECU_IDENT);
  CHECK_EQ(buf[1], 0x87);
  CHECK_EQ(kwpBuildReadEcuIdent(0x87u, buf, 1u), 0);
  CHECK_EQ(kwpBuildReadEcuIdent(0x87u, 0, 8u), 0);

  // `18 00 FF 00` -- byte for byte what the app sent to all 27 modules.
  memset(buf, 0xCCu, sizeof(buf));
  CHECK_EQ(kwpBuildReadDtc(buf, sizeof(buf)), 4);
  CHECK_EQ(buf[0], 0x18);
  CHECK_EQ(buf[1], 0x00);
  CHECK_EQ(buf[2], 0xFF);
  CHECK_EQ(buf[3], 0x00);
  CHECK_EQ(buf[4], 0xCC);
  CHECK_EQ(kwpBuildReadDtc(buf, 3u), 0);
  CHECK_EQ(kwpBuildReadDtc(0, 8u), 0);

  // What the builders produce is exactly what the recorded requests carry.
  const CapSingleSeq* dtcSingle = findSingle("6B0 041800FF00");
  CHECK(dtcSingle != 0);
  if (dtcSingle != 0) {
    uint8_t recorded[8];
    const uint8_t n = requestFromCap(dtcSingle->request, recorded, sizeof(recorded));
    CHECK_EQ(n, 4);
    CHECK_EQ(kwpBuildReadDtc(buf, sizeof(buf)), n);
    CHECK(memcmp(buf, recorded, n) == 0);
  }
  const CapMultiSeq* ident = findMulti("600 021A87");
  CHECK(ident != 0);
  if (ident != 0) {
    uint8_t recorded[8];
    const uint8_t n = requestFromCap(ident->request, recorded, sizeof(recorded));
    CHECK_EQ(n, 2);
    CHECK_EQ(kwpBuildReadEcuIdent(0x87u, buf, sizeof(buf)), 2);
    CHECK(memcmp(buf, recorded, 2) == 0);
  }
}

// ------------------------------------------- every recorded reply, classified
{
  // The recorded reply for a `21 xx` / `1A xx` / `18 ..` request must classify
  // as OK (or PENDING / NEGATIVE where the module said so), with the service
  // byte exactly request + 0x40.
  for (uint8_t i = 0; i < CAP_MULTI_COUNT; ++i) {
    const CapMultiSeq& s = kCapMulti[i];
    uint8_t req[8];
    const uint8_t reqLen = requestFromCap(s.request, req, sizeof(req));
    CHECK_MSG(reqLen != 0u, "%s: could not parse request %s", s.name, s.request);
    if (reqLen == 0u) {
      continue;
    }
    IsoTpPdu pdu;
    setPdu(pdu, s.pdu, s.pduLen);
    KwpResponse resp;
    const KwpStatus st = kwpParseResponse(req, reqLen, pdu, resp);
    CHECK_MSG(st == KWP_OK, "%s: status %d", s.name, (int)st);
    CHECK_MSG(resp.service == (uint8_t)(req[0] + KWP_POSITIVE_OFFSET),
              "%s: service 0x%02X vs 0x%02X", s.name, (unsigned)resp.service,
              (unsigned)(req[0] + KWP_POSITIVE_OFFSET));
    CHECK_MSG(resp.len == s.pduLen, "%s: len %u vs %u", s.name, (unsigned)resp.len,
              (unsigned)s.pduLen);
    // data must alias the caller's PDU, never a copy.
    CHECK_MSG(resp.data == pdu.data, "%s: data does not point into the caller's PDU", s.name);
  }

  for (uint8_t i = 0; i < CAP_SINGLE_COUNT; ++i) {
    const CapSingleSeq& s = kCapSingle[i];
    uint8_t req[8];
    const uint8_t reqLen = requestFromCap(s.request, req, sizeof(req));
    if (reqLen == 0u) {
      CHECK_MSG(false, "%s: could not parse request %s", s.name, s.request);
      continue;
    }
    IsoTpPdu pdu;
    setPdu(pdu, s.pdu, s.pduLen);
    KwpResponse resp;
    const KwpStatus st = kwpParseResponse(req, reqLen, pdu, resp);
    if (s.pdu[0] == KWP_NEGATIVE_MARKER) {
      const KwpStatus want = (s.pdu[2] == KWP_NRC_PENDING) ? KWP_PENDING : KWP_NEGATIVE;
      CHECK_MSG(st == want, "%s: status %d, wanted %d", s.name, (int)st, (int)want);
      CHECK_MSG(resp.service == req[0], "%s: echoed service 0x%02X vs 0x%02X", s.name,
                (unsigned)resp.service, (unsigned)req[0]);
      CHECK_MSG(resp.nrc == s.pdu[2], "%s: nrc 0x%02X", s.name, (unsigned)resp.nrc);
    } else {
      CHECK_MSG(st == KWP_OK, "%s: status %d", s.name, (int)st);
      CHECK_MSG(resp.service == (uint8_t)(req[0] + KWP_POSITIVE_OFFSET), "%s: service 0x%02X",
                s.name, (unsigned)resp.service);
    }
    CHECK_MSG(resp.data == pdu.data, "%s: data does not alias the caller's PDU", s.name);
  }
}

// The three distinct negative replies the car actually produced.
{
  // `7F 1A 78` responsePending -- must be PENDING, not lumped in with NEGATIVE.
  const CapSingleSeq* pending = findSingle("7E0 021A90");
  CHECK(pending != 0);
  if (pending != 0) {
    IsoTpPdu pdu;
    setPdu(pdu, pending->pdu, pending->pduLen);
    const uint8_t req[2] = {0x1Au, 0x90u};
    KwpResponse resp;
    CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_PENDING);
    CHECK_EQ(resp.service, 0x1A);
    CHECK_EQ(resp.nrc, KWP_NRC_PENDING);
    CHECK_EQ(resp.len, 3);
  }

  // `7F 21 31` requestOutOfRange -- a plain negative.
  const CapSingleSeq* oor = findSingle("620 022105");
  CHECK(oor != 0);
  if (oor != 0) {
    IsoTpPdu pdu;
    setPdu(pdu, oor->pdu, oor->pduLen);
    const uint8_t req[2] = {0x21u, 0x05u};
    KwpResponse resp;
    CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_NEGATIVE);
    CHECK_EQ(resp.nrc, 0x31);
  }

  // `7F 22 11` serviceNotSupported for a service this vehicle does not answer.
  const CapSingleSeq* nsvc = findSingle("7E0 0322F1A0");
  CHECK(nsvc != 0);
  if (nsvc != 0) {
    IsoTpPdu pdu;
    setPdu(pdu, nsvc->pdu, nsvc->pduLen);
    const uint8_t req[3] = {0x22u, 0xF1u, 0xA0u};
    KwpResponse resp;
    CHECK_EQ(kwpParseResponse(req, 3u, pdu, resp), KWP_NEGATIVE);
    CHECK_EQ(resp.service, 0x22);
    CHECK_EQ(resp.nrc, 0x11);
  }
}

// ---------------------------------------------------- kwpParseResponse errors
{
  IsoTpPdu pdu;
  KwpResponse resp;
  const uint8_t req[2] = {0x21u, 0xEAu};

  // Bad parameters.
  const uint8_t good[3] = {0x61u, 0xEAu, 0x2Bu};
  setPdu(pdu, good, 3u);
  CHECK_EQ(kwpParseResponse(0, 2u, pdu, resp), KWP_BAD_PARAM);
  CHECK_EQ(kwpParseResponse(req, 0u, pdu, resp), KWP_BAD_PARAM);
  CHECK_EQ(resp.data, (const uint8_t*)0);  // out is cleared before anything else
  CHECK_EQ(resp.len, 0);

  // A PDU whose length lies about itself.
  setPdu(pdu, good, 3u);
  pdu.len = (uint16_t)(ISOTP_MAX_PDU + 1u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_BAD_PARAM);

  // Empty reply.
  setPdu(pdu, good, 0u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_SHORT);

  // Truncated negative replies: `7F` and `7F 21` say nothing usable.
  const uint8_t neg1[1] = {0x7Fu};
  setPdu(pdu, neg1, 1u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_SHORT);
  const uint8_t neg2[2] = {0x7Fu, 0x21u};
  setPdu(pdu, neg2, 2u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_SHORT);

  // Right shape, wrong service: 0x18's answer arriving for a 0x21 request.
  const uint8_t wrong[2] = {0x58u, 0x00u};
  setPdu(pdu, wrong, 2u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_WRONG_SERVICE);
  CHECK_EQ(resp.service, 0x58);

  // The classic off-by-one: an echo of the request instead of request + 0x40.
  const uint8_t echoed[2] = {0x21u, 0xEAu};
  setPdu(pdu, echoed, 2u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_WRONG_SERVICE);

  // A negative response naming a service we never asked about is somebody
  // else's rejection, not ours.
  const uint8_t negOther[3] = {0x7Fu, 0x22u, 0x11u};
  setPdu(pdu, negOther, 3u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_WRONG_SERVICE);
  CHECK_EQ(resp.service, 0x22);
  CHECK_EQ(resp.nrc, 0x11);

  // The +0x40 rule holds across the services this project uses.
  const uint8_t reqDtc[4] = {0x18u, 0x00u, 0xFFu, 0x00u};
  const uint8_t rDtc[2] = {0x58u, 0x00u};
  setPdu(pdu, rDtc, 2u);
  CHECK_EQ(kwpParseResponse(reqDtc, 4u, pdu, resp), KWP_OK);

  const uint8_t reqIdent[2] = {0x1Au, 0x87u};
  const uint8_t rIdent[3] = {0x5Au, 0x87u, 0x02u};
  setPdu(pdu, rIdent, 3u);
  CHECK_EQ(kwpParseResponse(reqIdent, 2u, pdu, resp), KWP_OK);

  // Identifier echo. Every recorded `21 xx` / `1A xx` reply echoes byte 1, so
  // a reply carrying a different identifier is a reply to a different request
  // -- decoding it would put the wrong number on a gauge.
  const uint8_t rIdentOther[3] = {0x5Au, 0x88u, 0x02u};
  setPdu(pdu, rIdentOther, 3u);
  CHECK_EQ(kwpParseResponse(reqIdent, 2u, pdu, resp), KWP_WRONG_SERVICE);

  const uint8_t rLidOther[3] = {0x61u, 0x36u, 0x00u};
  setPdu(pdu, rLidOther, 3u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_WRONG_SERVICE);

  // A positive reply too short to hold the echo.
  const uint8_t rBare[1] = {0x61u};
  setPdu(pdu, rBare, 1u);
  CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_SHORT);

  // ... but 0x18's byte 1 is a record count, not an echo, so it is not checked.
  const uint8_t rDtcCount[2] = {0x58u, 0x07u};
  setPdu(pdu, rDtcCount, 2u);
  CHECK_EQ(kwpParseResponse(reqDtc, 4u, pdu, resp), KWP_OK);
}

// resp.data really is a window into the caller's buffer, with the service byte
// included, and nothing was copied.
{
  const CapMultiSeq* s = findMulti("7E0 0221E1");
  CHECK(s != 0);
  if (s != 0) {
    IsoTpPdu pdu;
    setPdu(pdu, s->pdu, s->pduLen);
    const uint8_t req[2] = {0x21u, 0xE1u};
    KwpResponse resp;
    CHECK_EQ(kwpParseResponse(req, 2u, pdu, resp), KWP_OK);
    CHECK_EQ(resp.data, pdu.data);
    CHECK_EQ(resp.data[0], 0x61);  // service byte included
    CHECK_EQ(resp.data[1], 0xE1);
    CHECK_EQ(resp.len, 16);
    // Serial number "T00ED3345A0196" as ASCII, straight off the wire.
    CHECK_EQ(resp.data[2], 'T');
    CHECK_EQ(resp.data[3], '0');
    CHECK_EQ(resp.data[4], '0');
    // Mutating the caller's PDU is visible through resp.data -- proof it aliases.
    pdu.data[2] = 'X';
    CHECK_EQ(resp.data[2], 'X');
  }
}

// ---------------------------------------------------------- kwpParseDtcList
// Every DTC reply in the capture, against the module's own record count.
{
  struct DtcCase {
    const char* name;
    bool multi;
  };
  static const DtcCase cases[] = {
      {"600 041800FF00", true},  {"784 041800FF00", true},  {"7E0 041800FF00", true},
      {"6E0 041800FF00", true},  {"7E1 041800FF00", true},  {"6A0 041800FF00", false},
      {"6F0 041800FF00", false}, {"6B0 041800FF00", false}, {"788 041800FF00", false},
      {"620 041800FF00", false}, {"7B6 041800FF00", false},
  };
  const uint8_t nCases = (uint8_t)(sizeof(cases) / sizeof(cases[0]));
  uint8_t totalRecords = 0u;

  for (uint8_t c = 0; c < nCases; ++c) {
    const uint8_t* raw = 0;
    uint16_t rawLen = 0u;
    if (cases[c].multi) {
      const CapMultiSeq* m = findMulti(cases[c].name);
      CHECK_MSG(m != 0, "missing fixture %s", cases[c].name);
      if (m == 0) {
        continue;
      }
      raw = m->pdu;
      rawLen = m->pduLen;
    } else {
      const CapSingleSeq* s = findSingle(cases[c].name);
      CHECK_MSG(s != 0, "missing fixture %s", cases[c].name);
      if (s == 0) {
        continue;
      }
      raw = s->pdu;
      rawLen = s->pduLen;
    }

    IsoTpPdu pdu;
    setPdu(pdu, raw, rawLen);
    const uint8_t req[4] = {0x18u, 0x00u, 0xFFu, 0x00u};
    KwpResponse resp;
    CHECK_MSG(kwpParseResponse(req, 4u, pdu, resp) == KWP_OK, "%s: not a positive reply",
              cases[c].name);

    DtcRecord recs[16];
    bool truncated = true;
    const uint8_t n = kwpParseDtcList(resp, recs, 16u, truncated);
    const uint8_t claimed = raw[1];
    CHECK_MSG(n == claimed, "%s: %u records, module claimed %u", cases[c].name, (unsigned)n,
              (unsigned)claimed);
    CHECK_MSG(!truncated, "%s: unexpectedly truncated", cases[c].name);
    // The payload length and the count byte agree on every recorded reply.
    CHECK_MSG((uint16_t)(2u + (uint16_t)claimed * 3u) == rawLen,
              "%s: payload %u bytes for %u records", cases[c].name, (unsigned)rawLen,
              (unsigned)claimed);
    totalRecords = (uint8_t)(totalRecords + n);

    // Codes and statuses must be the raw bytes, in order.
    for (uint8_t i = 0; i < n; ++i) {
      const uint16_t base = (uint16_t)(2u + (uint16_t)i * 3u);
      CHECK_EQ(recs[i].code, (uint16_t)(((uint16_t)raw[base] << 8) | raw[base + 1u]));
      CHECK_EQ(recs[i].status, raw[base + 2u]);
    }
  }
  CHECK_EQ(nCases, 11);
  CHECK_EQ(totalRecords, 20);
}

// The recorded scan, module by module, against the extracted scan table. This
// ties the wire bytes to the SAE codes and the status classification.
{
  struct ScanCase {
    const char* module;
    const char* fixture;
    bool multi;
  };
  static const ScanCase cases[] = {
      {"ABS", "784 041800FF00", true},   {"PCM", "7E0 041800FF00", true},
      {"WCM", "600 041800FF00", true},   {"CCN", "6A0 041800FF00", false},
      {"ESM", "788 041800FF00", false},  {"FCMCGW", "620 041800FF00", false},
      {"FDCM", "7B6 041800FF00", false},
  };
  const uint8_t nCases = (uint8_t)(sizeof(cases) / sizeof(cases[0]));

  for (uint8_t c = 0; c < nCases; ++c) {
    const uint8_t* raw = 0;
    uint16_t rawLen = 0u;
    if (cases[c].multi) {
      const CapMultiSeq* m = findMulti(cases[c].fixture);
      if (m == 0) {
        CHECK_MSG(false, "missing fixture %s", cases[c].fixture);
        continue;
      }
      raw = m->pdu;
      rawLen = m->pduLen;
    } else {
      const CapSingleSeq* s = findSingle(cases[c].fixture);
      if (s == 0) {
        CHECK_MSG(false, "missing fixture %s", cases[c].fixture);
        continue;
      }
      raw = s->pdu;
      rawLen = s->pduLen;
    }

    IsoTpPdu pdu;
    setPdu(pdu, raw, rawLen);
    const uint8_t req[4] = {0x18u, 0x00u, 0xFFu, 0x00u};
    KwpResponse resp;
    CHECK_EQ(kwpParseResponse(req, 4u, pdu, resp), KWP_OK);

    DtcRecord recs[16];
    bool truncated = true;
    const uint8_t n = kwpParseDtcList(resp, recs, 16u, truncated);
    CHECK(!truncated);

    // Walk the scan table rows for this module and match them up in order.
    uint8_t matched = 0u;
    for (uint16_t k = 0; k < DTC_SCAN_RECORD_COUNT; ++k) {
      const DtcScanRecord& want = kDtcScanRecords[k];
      if (strcmp(want.module, cases[c].module) != 0) {
        continue;
      }
      CHECK_MSG(want.countByte == n, "%s: count byte %u vs parsed %u", want.module,
                (unsigned)want.countByte, (unsigned)n);
      if (want.recordNo == 0u || want.recordNo > n) {
        CHECK_MSG(false, "%s: record %u out of range", want.module, (unsigned)want.recordNo);
        continue;
      }
      const DtcRecord& got = recs[want.recordNo - 1u];
      CHECK_MSG(got.code == want.code, "%s rec %u: code 0x%04X vs 0x%04X", want.module,
                (unsigned)want.recordNo, (unsigned)got.code, (unsigned)want.code);
      char sae[8];
      CHECK(kwpDtcToSae(got.code, sae, sizeof(sae)));
      CHECK_MSG(strcmp(sae, want.sae) == 0, "%s rec %u: \"%s\" vs \"%s\"", want.module,
                (unsigned)want.recordNo, sae, want.sae);
      // The only status byte disagreement with the extracted scan is ESM, so
      // that module's status is checked separately below.
      if (strcmp(want.module, "ESM") != 0) {
        CHECK_MSG(got.status == want.status, "%s rec %u: status 0x%02X vs 0x%02X", want.module,
                  (unsigned)want.recordNo, (unsigned)got.status, (unsigned)want.status);
        const bool wantActive = (strcmp(want.statusText, "active") == 0);
        const bool gotActive = (strncmp(kwpDtcStatusText(got.status), "active", 6) == 0);
        CHECK_MSG(gotActive == wantActive, "%s rec %u: \"%s\" vs \"%s\"", want.module,
                  (unsigned)want.recordNo, kwpDtcStatusText(got.status), want.statusText);
      }
      ++matched;
    }
    CHECK_MSG(matched == n, "%s: matched %u of %u records", cases[c].module, (unsigned)matched,
              (unsigned)n);
  }

  // ESM: the extracted scan says status 0x30, the capture says 0x20. Both are
  // "stored", so the classification survives; the raw byte simply differs
  // between the two sessions. Recorded here so the discrepancy is not silently
  // "fixed" one way or the other.
  const CapSingleSeq* esm = findSingle("788 041800FF00");
  CHECK(esm != 0);
  if (esm != 0) {
    CHECK_EQ(esm->pdu[4], 0x20);  // wire
    for (uint16_t k = 0; k < DTC_SCAN_RECORD_COUNT; ++k) {
      if (strcmp(kDtcScanRecords[k].module, "ESM") == 0) {
        CHECK_EQ(kDtcScanRecords[k].status, 0x30);  // extracted table
        CHECK_STR(kwpDtcStatusText(0x20u), "stored");
        CHECK_STR(kwpDtcStatusText(0x30u), "stored");
      }
    }
  }
}

// Zero-fault replies: `58 00` yields no records and is not "truncated".
{
  const CapSingleSeq* none = findSingle("6F0 041800FF00");
  CHECK(none != 0);
  if (none != 0) {
    IsoTpPdu pdu;
    setPdu(pdu, none->pdu, none->pduLen);
    const uint8_t req[4] = {0x18u, 0x00u, 0xFFu, 0x00u};
    KwpResponse resp;
    CHECK_EQ(kwpParseResponse(req, 4u, pdu, resp), KWP_OK);
    DtcRecord recs[4];
    bool truncated = true;
    CHECK_EQ(kwpParseDtcList(resp, recs, 4u, truncated), 0);
    CHECK(!truncated);
  }
}

// kwpParseDtcList error and boundary paths.
{
  KwpResponse resp;
  DtcRecord recs[8];
  bool truncated = false;

  // No data at all.
  resp.data = 0;
  resp.len = 0u;
  resp.service = 0x58u;
  resp.nrc = 0u;
  truncated = true;
  CHECK_EQ(kwpParseDtcList(resp, recs, 8u, truncated), 0);
  CHECK(!truncated);

  // Not a DTC reply: a `61 EA ..` block must not be read as records.
  static const uint8_t notDtc[4] = {0x61u, 0xEAu, 0x2Bu, 0x00u};
  resp.data = notDtc;
  resp.len = 4u;
  truncated = true;
  CHECK_EQ(kwpParseDtcList(resp, recs, 8u, truncated), 0);
  CHECK(!truncated);

  // `58` with no count byte.
  static const uint8_t bare[1] = {0x58u};
  resp.data = bare;
  resp.len = 1u;
  truncated = true;
  CHECK_EQ(kwpParseDtcList(resp, recs, 8u, truncated), 0);
  CHECK(!truncated);

  // Count byte claims 4 records, payload holds one and a bit. This is the case
  // that must not read past the payload: only the whole record is returned.
  static const uint8_t liar[7] = {0x58u, 0x04u, 0x04u, 0x60u, 0x20u, 0xC1u, 0x14u};
  resp.data = liar;
  resp.len = 7u;
  truncated = false;
  CHECK_EQ(kwpParseDtcList(resp, recs, 8u, truncated), 1);
  CHECK(truncated);
  CHECK_EQ(recs[0].code, 0x0460);
  CHECK_EQ(recs[0].status, 0x20);

  // Count byte claims records where there is no payload at all.
  static const uint8_t emptyClaim[2] = {0x58u, 0x03u};
  resp.data = emptyClaim;
  resp.len = 2u;
  truncated = false;
  CHECK_EQ(kwpParseDtcList(resp, recs, 8u, truncated), 0);
  CHECK(truncated);

  // Count byte claims 0xFF records.
  static const uint8_t maxClaim[5] = {0x58u, 0xFFu, 0x21u, 0x3Au, 0xE0u};
  resp.data = maxClaim;
  resp.len = 5u;
  truncated = false;
  CHECK_EQ(kwpParseDtcList(resp, recs, 8u, truncated), 1);
  CHECK(truncated);
  CHECK_EQ(recs[0].code, 0x213A);

  // Count byte claims fewer records than the payload carries: believe the
  // module, and do NOT flag truncation -- there is nothing the caller lost.
  static const uint8_t extra[11] = {0x58u, 0x01u, 0x04u, 0x60u, 0x20u,
                                    0xC1u, 0x14u, 0x20u, 0xC1u, 0x03u, 0x20u};
  resp.data = extra;
  resp.len = 11u;
  truncated = true;
  CHECK_EQ(kwpParseDtcList(resp, recs, 8u, truncated), 1);
  CHECK(!truncated);
  CHECK_EQ(recs[0].code, 0x0460);

  // Caller's buffer is smaller than the record count.
  static const uint8_t seven[23] = {0x58u, 0x07u, 0x04u, 0x60u, 0x20u, 0xC1u, 0x14u, 0x20u,
                                    0xC1u, 0x03u, 0x20u, 0xC1u, 0x14u, 0x20u, 0xC1u, 0x03u,
                                    0x20u, 0x06u, 0x76u, 0x60u, 0x21u, 0x3Au, 0xE0u};
  resp.data = seven;
  resp.len = 23u;
  truncated = false;
  CHECK_EQ(kwpParseDtcList(resp, recs, 3u, truncated), 3);
  CHECK(truncated);
  CHECK_EQ(recs[0].code, 0x0460);
  CHECK_EQ(recs[2].code, 0xC103);

  // Exactly enough room is not truncation.
  truncated = true;
  CHECK_EQ(kwpParseDtcList(resp, recs, 7u, truncated), 7);
  CHECK(!truncated);
  CHECK_EQ(recs[6].code, 0x213A);
  CHECK_EQ(recs[6].status, 0xE0);

  // No output buffer, or a zero-sized one, with records available.
  truncated = false;
  CHECK_EQ(kwpParseDtcList(resp, 0, 8u, truncated), 0);
  CHECK(truncated);
  truncated = false;
  CHECK_EQ(kwpParseDtcList(resp, recs, 0u, truncated), 0);
  CHECK(truncated);

  // A response claiming to be longer than any real PDU is clamped, not trusted.
  static uint8_t big[ISOTP_MAX_PDU];
  memset(big, 0, sizeof(big));
  big[0] = 0x58u;
  big[1] = 0x2Au;  // 42 records, which is more than 128 bytes can hold
  resp.data = big;
  resp.len = (uint16_t)(ISOTP_MAX_PDU + 64u);
  truncated = false;
  const uint8_t n = kwpParseDtcList(resp, recs, 8u, truncated);
  CHECK_EQ(n, 8);
  CHECK(truncated);
}

// ------------------------------------------------------------- kwpDtcToSae
// The whole point: check the derived rule against every known pair.
{
  uint16_t matched = 0u;
  uint16_t reported = 0u;
  for (uint16_t i = 0; i < DTC_CODE_PAIR_COUNT; ++i) {
    char buf[8];
    memset(buf, 0x7Fu, sizeof(buf));
    if (!kwpDtcToSae(kDtcCodePairs[i].code, buf, sizeof(buf))) {
      if (reported < 12u) {
        CHECK_MSG(false, "0x%04X: kwpDtcToSae refused to render",
                  (unsigned)kDtcCodePairs[i].code);
        ++reported;
      }
      continue;
    }
    if (strcmp(buf, kDtcCodePairs[i].sae) == 0) {
      ++matched;
    } else if (reported < 12u) {
      CHECK_MSG(false, "0x%04X -> \"%s\", expected \"%s\"", (unsigned)kDtcCodePairs[i].code, buf,
                kDtcCodePairs[i].sae);
      ++reported;
    }
  }
  std::printf("kwpDtcToSae: %u / %u known (raw, SAE) pairs reproduced\n", (unsigned)matched,
              (unsigned)DTC_CODE_PAIR_COUNT);
  CHECK_EQ(DTC_CODE_PAIR_COUNT, 1152);
  CHECK_EQ(matched, DTC_CODE_PAIR_COUNT);
}

// The same rule against the live scan's own (code, SAE) pairs.
{
  for (uint16_t i = 0; i < DTC_SCAN_RECORD_COUNT; ++i) {
    char buf[8];
    CHECK(kwpDtcToSae(kDtcScanRecords[i].code, buf, sizeof(buf)));
    CHECK_MSG(strcmp(buf, kDtcScanRecords[i].sae) == 0, "%s 0x%04X -> \"%s\", expected \"%s\"",
              kDtcScanRecords[i].module, (unsigned)kDtcScanRecords[i].code, buf,
              kDtcScanRecords[i].sae);
  }
  CHECK_EQ(DTC_SCAN_RECORD_COUNT, 16);
}

// Each letter and each first digit, plus the corners of the encoding.
{
  char buf[8];
  CHECK(kwpDtcToSae(0x0119u, buf, sizeof(buf)));
  CHECK_STR(buf, "P0119");   // the example in Kwp.h
  CHECK(kwpDtcToSae(0x0000u, buf, sizeof(buf)));
  CHECK_STR(buf, "P0000");
  CHECK(kwpDtcToSae(0x3FFFu, buf, sizeof(buf)));
  CHECK_STR(buf, "P3FFF");
  CHECK(kwpDtcToSae(0x4000u, buf, sizeof(buf)));
  CHECK_STR(buf, "C0000");
  CHECK(kwpDtcToSae(0x7FFFu, buf, sizeof(buf)));
  CHECK_STR(buf, "C3FFF");
  CHECK(kwpDtcToSae(0x8000u, buf, sizeof(buf)));
  CHECK_STR(buf, "B0000");
  CHECK(kwpDtcToSae(0xC000u, buf, sizeof(buf)));
  CHECK_STR(buf, "U0000");
  CHECK(kwpDtcToSae(0xFFFFu, buf, sizeof(buf)));
  CHECK_STR(buf, "U3FFF");
  // Hex digits in the tail really do occur: PCM reported 0x213A = P213A.
  CHECK(kwpDtcToSae(0x213Au, buf, sizeof(buf)));
  CHECK_STR(buf, "P213A");
  CHECK(kwpDtcToSae(0x500Au, buf, sizeof(buf)));
  CHECK_STR(buf, "C100A");

  // Buffer handling: six bytes is the documented minimum.
  CHECK(!kwpDtcToSae(0x0119u, 0, 6u));
  CHECK(!kwpDtcToSae(0x0119u, buf, 5u));
  CHECK(!kwpDtcToSae(0x0119u, buf, 0u));
  memset(buf, 0x7Fu, sizeof(buf));
  CHECK(kwpDtcToSae(0x0119u, buf, 6u));
  CHECK_EQ(buf[5], 0);
  CHECK_EQ(buf[6], 0x7F);  // nothing written past the six documented bytes
}

// ---------------------------------------------------------- status decoding
{
  // Never NULL, for any of the 256 values.
  for (uint16_t v = 0; v < 256u; ++v) {
    const char* s = kwpDtcStatusText((uint8_t)v);
    CHECK_MSG(s != 0, "status 0x%02X gave NULL", (unsigned)v);
    if (s != 0) {
      CHECK_MSG(s[0] != '\0', "status 0x%02X gave an empty string", (unsigned)v);
    }
  }

  // The five status bytes this vehicle actually produced, and the split the
  // extracted scan draws between them.
  CHECK_STR(kwpDtcStatusText(0x20u), "stored");
  CHECK_STR(kwpDtcStatusText(0x27u), "stored");
  CHECK_STR(kwpDtcStatusText(0x30u), "stored");
  CHECK_STR(kwpDtcStatusText(0x60u), "active");
  CHECK_STR(kwpDtcStatusText(0xE0u), "active, lamp on");
  CHECK_STR(kwpDtcStatusText(0x00u), "no fault");
  CHECK_STR(kwpDtcStatusText(0xA0u), "stored, lamp on");
  CHECK_STR(kwpDtcStatusText(0x80u), "lamp on");
  // Bits nothing in the data explains are not given an invented meaning.
  CHECK_STR(kwpDtcStatusText(0x01u), "unknown");
  CHECK_STR(kwpDtcStatusText(0x1Fu), "unknown");
}

// ---------------------------------------------- end to end, one real module
// The recorded PCM DTC reply, from reassembled PDU to seven SAE strings.
{
  const CapMultiSeq* pcm = findMulti("7E0 041800FF00");
  CHECK(pcm != 0);
  if (pcm != 0) {
    IsoTpPdu pdu;
    setPdu(pdu, pcm->pdu, pcm->pduLen);
    uint8_t req[8];
    const uint8_t reqLen = kwpBuildReadDtc(req, sizeof(req));
    CHECK_EQ(reqLen, 4);

    KwpResponse resp;
    CHECK_EQ(kwpParseResponse(req, reqLen, pdu, resp), KWP_OK);

    DtcRecord recs[8];
    bool truncated = true;
    CHECK_EQ(kwpParseDtcList(resp, recs, 8u, truncated), 7);
    CHECK(!truncated);

    static const char* kWant[7] = {"P0460", "U0114", "U0103", "U0114", "U0103", "P0676", "P213A"};
    static const uint8_t kWantStatus[7] = {0x20u, 0x20u, 0x20u, 0x20u, 0x20u, 0x60u, 0xE0u};
    for (uint8_t i = 0; i < 7u; ++i) {
      char sae[8];
      CHECK(kwpDtcToSae(recs[i].code, sae, sizeof(sae)));
      CHECK_STR(sae, kWant[i]);
      CHECK_EQ(recs[i].status, kWantStatus[i]);
    }
    // The PCM really does report U0114 and U0103 twice each -- duplicated
    // records are legal and must not be collapsed by the parser.
    CHECK_EQ(recs[1].code, recs[3].code);
    CHECK_EQ(recs[2].code, recs[4].code);
    CHECK_STR(kwpDtcStatusText(recs[5].status), "active");
    CHECK_STR(kwpDtcStatusText(recs[6].status), "active, lamp on");
  }
}

TEST_MAIN_END()
