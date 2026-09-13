// ISO-TP host tests.
//
// The happy paths are driven entirely by frames recorded off this vehicle
// (fixtures/CaptureIsoTp.h). The error paths are synthetic, because a healthy
// module never produces them -- which is exactly why they need a test: a wrong
// guess here only shows up as a garbled reading in the car.
#include "doctest_min.h"

#include <stdint.h>
#include <string.h>

#include "CarLink.h"
#include "IsoTp.h"

#include "../../fixtures/CaptureIsoTp.h"

namespace {

CanFrame mk(uint16_t id, uint8_t len, const uint8_t* bytes) {
  CanFrame f;
  f.id = id;
  f.len = len;
  memset(f.data, 0, sizeof(f.data));
  for (uint8_t i = 0; i < len && i < CAN_MAX_DLC; ++i) {
    f.data[i] = bytes[i];
  }
  return f;
}

CanFrame fromCap(uint16_t id, const CapFrame& cf) { return mk(id, cf.len, cf.data); }

// Scripted ICarLink. Static storage only, like everything else in core/.
const uint8_t kScriptCap = 72u;

class FakeLink : public ICarLink {
 public:
  FakeLink() { clear(); }

  void clear() {
    scriptCount_ = 0u;
    scriptPos_ = 0u;
    sentCount_ = 0u;
    setTargetCalls_ = 0u;
    readCalls_ = 0u;
    lastRequestId_ = 0xFFFFu;
    lastResponseId_ = 0xFFFFu;
    lastTimeoutMs_ = 0u;
    firstTimeoutMs_ = 0u;
    beginStatus_ = LINK_OK;
    setTargetStatus_ = LINK_OK;
    sendStatus_ = LINK_OK;
    tailStatus_ = LINK_NO_DATA;
    ready_ = true;
    memset(script_, 0, sizeof(script_));
    memset(scriptStatus_, 0, sizeof(scriptStatus_));
    memset(sent_, 0, sizeof(sent_));
  }

  void push(const CanFrame& f) {
    if (scriptCount_ < kScriptCap) {
      script_[scriptCount_] = f;
      scriptStatus_[scriptCount_] = LINK_OK;
      ++scriptCount_;
    }
  }
  void pushStatus(LinkStatus st) {
    if (scriptCount_ < kScriptCap) {
      memset(&script_[scriptCount_], 0, sizeof(CanFrame));
      scriptStatus_[scriptCount_] = st;
      ++scriptCount_;
    }
  }

  LinkStatus begin() { return beginStatus_; }

  LinkStatus setTarget(uint16_t requestId, uint16_t responseId) {
    ++setTargetCalls_;
    lastRequestId_ = requestId;
    lastResponseId_ = responseId;
    return setTargetStatus_;
  }

  LinkStatus sendFrame(const CanFrame& frame) {
    if (sentCount_ < 8u) {
      sent_[sentCount_] = frame;
    }
    ++sentCount_;
    return sendStatus_;
  }

  LinkStatus readFrame(CanFrame& out, uint32_t timeoutMs) {
    if (readCalls_ == 0u) {
      firstTimeoutMs_ = timeoutMs;
    }
    ++readCalls_;
    lastTimeoutMs_ = timeoutMs;
    if (scriptPos_ >= scriptCount_) {
      memset(&out, 0, sizeof(out));
      return tailStatus_;
    }
    const LinkStatus st = scriptStatus_[scriptPos_];
    out = script_[scriptPos_];
    ++scriptPos_;
    return st;
  }

  bool ready() { return ready_; }

  // knobs / observations
  LinkStatus beginStatus_;
  LinkStatus setTargetStatus_;
  LinkStatus sendStatus_;
  LinkStatus tailStatus_;
  bool ready_;
  uint8_t sentCount_;
  uint8_t setTargetCalls_;
  uint8_t readCalls_;
  uint16_t lastRequestId_;
  uint16_t lastResponseId_;
  uint32_t lastTimeoutMs_;
  uint32_t firstTimeoutMs_;
  CanFrame sent_[8];

 private:
  CanFrame script_[kScriptCap];
  LinkStatus scriptStatus_[kScriptCap];
  uint8_t scriptCount_;
  uint8_t scriptPos_;
};

// A helper that feeds a whole recorded sequence and returns the final status.
IsoTpStatus feedAll(IsoTpReassembler& r, uint16_t id, const CapFrame* frames, uint8_t n,
                    IsoTpPdu& out) {
  IsoTpStatus st = ISOTP_INCOMPLETE;
  for (uint8_t i = 0; i < n; ++i) {
    const CanFrame f = fromCap(id, frames[i]);
    st = r.feed(f, out);
    if (st != ISOTP_INCOMPLETE) {
      // Report the frame index on an unexpected early stop.
      if (i + 1u != n) {
        CHECK_MSG(false, "sequence stopped early at frame %u/%u with status %d", (unsigned)i,
                  (unsigned)n, (int)st);
      }
      return st;
    }
  }
  return st;
}

bool sameBytes(const uint8_t* a, const uint8_t* b, uint16_t n) {
  return memcmp(a, b, n) == 0;
}

}  // namespace

TEST_MAIN_BEGIN()

// ---------------------------------------------------------------- classify
{
  uint8_t z[8] = {0};
  CanFrame f = mk(0x7E8u, 0u, z);
  CHECK_EQ(isoTpClassify(f), ISOTP_TYPE_UNKNOWN);  // no PCI byte at all

  const uint8_t sf[2] = {0x02u, 0x50u};
  f = mk(0x7E8u, 2u, sf);
  CHECK_EQ(isoTpClassify(f), ISOTP_TYPE_SINGLE);

  const uint8_t ff[3] = {0x10u, 0x17u, 0x58u};
  f = mk(0x7E8u, 3u, ff);
  CHECK_EQ(isoTpClassify(f), ISOTP_TYPE_FIRST);

  const uint8_t cf[2] = {0x21u, 0x14u};
  f = mk(0x7E8u, 2u, cf);
  CHECK_EQ(isoTpClassify(f), ISOTP_TYPE_CONSECUTIVE);

  const uint8_t fc[3] = {0x30u, 0x00u, 0x00u};
  f = mk(0x7E8u, 3u, fc);
  CHECK_EQ(isoTpClassify(f), ISOTP_TYPE_FLOW);

  for (uint8_t hi = 4u; hi <= 0x0Fu; ++hi) {
    const uint8_t bad[2] = {(uint8_t)(hi << 4), 0x00u};
    f = mk(0x7E8u, 2u, bad);
    CHECK_EQ(isoTpClassify(f), ISOTP_TYPE_UNKNOWN);
  }

  // A frame claiming more than a classical CAN frame can hold is not classified.
  f = mk(0x7E8u, 2u, sf);
  f.len = 9u;
  CHECK_EQ(isoTpClassify(f), ISOTP_TYPE_UNKNOWN);
}

// ------------------------------------------- real single frames from the car
{
  for (uint8_t i = 0; i < CAP_SINGLE_COUNT; ++i) {
    const CapSingleSeq& s = kCapSingle[i];
    IsoTpReassembler r;
    IsoTpPdu pdu;
    memset(&pdu, 0, sizeof(pdu));
    const CanFrame f = fromCap(s.respId, s.frame);
    const IsoTpStatus st = r.feed(f, pdu);
    CHECK_MSG(st == ISOTP_OK, "%s: expected ISOTP_OK, got %d", s.name, (int)st);
    CHECK_MSG(pdu.len == s.pduLen, "%s: len %u vs %u", s.name, (unsigned)pdu.len,
              (unsigned)s.pduLen);
    if (pdu.len == s.pduLen) {
      CHECK_MSG(sameBytes(pdu.data, s.pdu, s.pduLen), "%s: payload mismatch", s.name);
    }
    // A completed Single Frame leaves nothing behind.
    CHECK(!r.inProgress());
    CHECK_EQ(r.expected(), 0);
  }
  // Sanity: the recorded set really does contain the padding variety that
  // makes the declared length load-bearing.
  CHECK_EQ(CAP_SINGLE_COUNT, 17);
}

// ------------------------------------ real multi-frame sequences from the car
{
  for (uint8_t i = 0; i < CAP_MULTI_COUNT; ++i) {
    const CapMultiSeq& s = kCapMulti[i];
    IsoTpReassembler r;
    IsoTpPdu pdu;
    memset(&pdu, 0, sizeof(pdu));

    // First Frame alone must not complete, and must publish the declared length.
    const CanFrame f0 = fromCap(s.respId, s.frames[0]);
    const IsoTpStatus st0 = r.feed(f0, pdu);
    CHECK_MSG(st0 == ISOTP_INCOMPLETE, "%s: FF gave %d", s.name, (int)st0);
    CHECK_MSG(r.inProgress(), "%s: FF did not start a sequence", s.name);
    CHECK_MSG(r.expected() == s.pduLen, "%s: expected() %u vs %u", s.name,
              (unsigned)r.expected(), (unsigned)s.pduLen);

    IsoTpStatus st = ISOTP_INCOMPLETE;
    for (uint8_t k = 1u; k < s.frameCount; ++k) {
      const CanFrame f = fromCap(s.respId, s.frames[k]);
      st = r.feed(f, pdu);
      if (k + 1u < s.frameCount) {
        CHECK_MSG(st == ISOTP_INCOMPLETE, "%s: CF %u gave %d", s.name, (unsigned)k, (int)st);
      }
    }
    CHECK_MSG(st == ISOTP_OK, "%s: sequence did not complete (%d)", s.name, (int)st);
    CHECK_MSG(pdu.len == s.pduLen, "%s: len %u vs %u", s.name, (unsigned)pdu.len,
              (unsigned)s.pduLen);
    if (pdu.len == s.pduLen) {
      CHECK_MSG(sameBytes(pdu.data, s.pdu, s.pduLen), "%s: payload mismatch", s.name);
    }
    CHECK_MSG(!r.inProgress(), "%s: not reset after completion", s.name);
  }
  CHECK_EQ(CAP_MULTI_COUNT, 15);
}

// The reason the declared length is the only truth: the WCM's own reply carries
// five stale bytes in the tail of its last Consecutive Frame.
{
  const CapMultiSeq* wcm = 0;
  for (uint8_t i = 0; i < CAP_MULTI_COUNT; ++i) {
    if (strcmp(kCapMulti[i].name, "600 041800FF00") == 0) {
      wcm = &kCapMulti[i];
    }
  }
  CHECK(wcm != 0);
  if (wcm != 0) {
    CHECK_EQ(wcm->frameCount, 2);
    CHECK_EQ(wcm->pduLen, 8);
    // frame 1 = 10 08 58 02 40 77 27 A1, frame 2 = 21 02 27 50 20 00 07 35.
    // Only `02 27` of that second frame is real; `50 20 00 07 35` is junk.
    CHECK_EQ(wcm->frames[1].data[0], 0x21);
    CHECK_EQ(wcm->frames[1].data[3], 0x50);
    IsoTpReassembler r;
    IsoTpPdu pdu;
    memset(&pdu, 0, sizeof(pdu));
    const IsoTpStatus st = feedAll(r, wcm->respId, wcm->frames, wcm->frameCount, pdu);
    CHECK_EQ(st, ISOTP_OK);
    CHECK_EQ(pdu.len, 8);
    CHECK_EQ(pdu.data[6], 0x02);
    CHECK_EQ(pdu.data[7], 0x27);
  }
}

// ---------------------------------------------------------- reassembler errors
{
  IsoTpReassembler r;
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));

  // len == 0: no PCI, nothing to do with it.
  uint8_t z[8] = {0};
  CanFrame f = mk(0x7E8u, 0u, z);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);
  CHECK(!r.inProgress());

  // Single Frame claiming zero bytes.
  const uint8_t sf0[8] = {0x00u, 0x58u, 0x00u, 0u, 0u, 0u, 0u, 0u};
  f = mk(0x7E8u, 8u, sf0);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);

  // Single Frame claiming more than the frame carries (len 4 => max 3 payload).
  const uint8_t sfBig[4] = {0x07u, 0x58u, 0x01u, 0x02u};
  f = mk(0x7E8u, 4u, sfBig);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);

  // ... but exactly filling the frame is fine.
  const uint8_t sfFit[4] = {0x03u, 0x7Fu, 0x1Au, 0x78u};
  f = mk(0x7E8u, 4u, sfFit);
  CHECK_EQ(r.feed(f, pdu), ISOTP_OK);
  CHECK_EQ(pdu.len, 3);
  CHECK_EQ(pdu.data[0], 0x7F);
  CHECK_EQ(pdu.data[2], 0x78);

  // First Frame declaring more than we will hold.
  const uint8_t ffOver[8] = {0x10u, 0x81u, 0u, 0u, 0u, 0u, 0u, 0u};  // 129
  f = mk(0x7E8u, 8u, ffOver);
  CHECK_EQ(r.feed(f, pdu), ISOTP_OVERFLOW);
  CHECK(!r.inProgress());
  CHECK_EQ(r.expected(), 0);

  const uint8_t ffWay[8] = {0x1Fu, 0xFFu, 0u, 0u, 0u, 0u, 0u, 0u};  // 4095
  f = mk(0x7E8u, 8u, ffWay);
  CHECK_EQ(r.feed(f, pdu), ISOTP_OVERFLOW);

  // Exactly ISOTP_MAX_PDU is accepted -- the boundary belongs on the good side.
  const uint8_t ffMax[8] = {0x10u, 0x80u, 1u, 2u, 3u, 4u, 5u, 6u};  // 128
  f = mk(0x7E8u, 8u, ffMax);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  CHECK_EQ(r.expected(), ISOTP_MAX_PDU);
  r.reset();

  // First Frame declaring a length that should have been a Single Frame.
  const uint8_t ffTiny[8] = {0x10u, 0x07u, 1u, 2u, 3u, 4u, 5u, 6u};
  f = mk(0x7E8u, 8u, ffTiny);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);

  // First Frame truncated to one byte: the length field is not even present.
  const uint8_t ffShort[1] = {0x10u};
  f = mk(0x7E8u, 1u, ffShort);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);

  // Consecutive Frame with no First Frame in front of it.
  const uint8_t cfLone[8] = {0x21u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
  f = mk(0x7E8u, 8u, cfLone);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);
  CHECK(!r.inProgress());

  // Unknown PCI.
  const uint8_t junk[8] = {0x4Au, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
  f = mk(0x7E8u, 8u, junk);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);
}

// Sequence-number gap, then a clean retry.
{
  IsoTpReassembler r;
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));

  const uint8_t ff[8] = {0x10u, 0x10u, 0x61u, 0x36u, 0x00u, 0x01u, 0x02u, 0x03u};
  CanFrame f = mk(0x7E8u, 8u, ff);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);

  const uint8_t cf2[8] = {0x22u, 9u, 9u, 9u, 9u, 9u, 9u, 9u};  // expected 0x21
  f = mk(0x7E8u, 8u, cf2);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);
  CHECK(!r.inProgress());
  CHECK_EQ(r.expected(), 0);

  // The error left the object clean, so the very next retry works.
  f = mk(0x7E8u, 8u, ff);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  const uint8_t cf1[8] = {0x21u, 0x04u, 0x05u, 0x06u, 0x07u, 0x08u, 0x09u, 0x0Au};
  f = mk(0x7E8u, 8u, cf1);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  const uint8_t cfB[8] = {0x22u, 0x0Bu, 0x0Cu, 0x0Du, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
  f = mk(0x7E8u, 8u, cfB);
  CHECK_EQ(r.feed(f, pdu), ISOTP_OK);
  CHECK_EQ(pdu.len, 0x10);
  CHECK_EQ(pdu.data[0], 0x61);
  CHECK_EQ(pdu.data[15], 0x0D);  // last real byte; the 0xFF pad was dropped
}

// A Consecutive Frame carrying no payload would stall the sequence.
{
  IsoTpReassembler r;
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t ff[8] = {0x10u, 0x0Au, 0x61u, 0x04u, 0xE7u, 0x00u, 0x40u, 0x01u};
  CanFrame f = mk(0x7E8u, 8u, ff);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  const uint8_t cfEmpty[1] = {0x21u};
  f = mk(0x7E8u, 1u, cfEmpty);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);
  CHECK(!r.inProgress());
}

// A Flow Control frame from a module is skipped, not treated as data.
{
  IsoTpReassembler r;
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));

  // Idle: an FC changes nothing.
  const uint8_t fc[8] = {0x30u, 0x00u, 0x00u, 0u, 0u, 0u, 0u, 0u};
  CanFrame f = mk(0x7E8u, 8u, fc);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  CHECK(!r.inProgress());
  CHECK_EQ(r.expected(), 0);

  // Mid-sequence: an FC wedged between the FF and the CF must not consume the
  // sequence number, or the CF that follows would look like a gap.
  const uint8_t ff[8] = {0x10u, 0x08u, 0x58u, 0x02u, 0x40u, 0x77u, 0x27u, 0xA1u};
  f = mk(0x500u, 8u, ff);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  f = mk(0x500u, 8u, fc);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  CHECK(r.inProgress());
  CHECK_EQ(r.expected(), 8);
  const uint8_t cf[8] = {0x21u, 0x02u, 0x27u, 0x50u, 0x20u, 0x00u, 0x07u, 0x35u};
  f = mk(0x500u, 8u, cf);
  CHECK_EQ(r.feed(f, pdu), ISOTP_OK);
  CHECK_EQ(pdu.len, 8);
  CHECK_EQ(pdu.data[7], 0x27);

  // Flow Control with a "wait" or "abort" flow status is equally ignored.
  const uint8_t fcWait[3] = {0x31u, 0x00u, 0x00u};
  f = mk(0x500u, 3u, fcWait);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  const uint8_t fcAbort[1] = {0x32u};
  f = mk(0x500u, 1u, fcAbort);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
}

// A First Frame arriving mid-sequence restarts; it never merges.
{
  IsoTpReassembler r;
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));

  const uint8_t ffA[8] = {0x10u, 0x28u, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu};  // 40
  CanFrame f = mk(0x7E8u, 8u, ffA);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  const uint8_t cfA[8] = {0x21u, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu};
  f = mk(0x7E8u, 8u, cfA);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);

  // New First Frame: the 13 bytes of the abandoned sequence must vanish.
  const uint8_t ffB[8] = {0x10u, 0x08u, 0x58u, 0x02u, 0x40u, 0x77u, 0x27u, 0xA1u};
  f = mk(0x7E8u, 8u, ffB);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  CHECK_EQ(r.expected(), 8);
  const uint8_t cfB[8] = {0x21u, 0x02u, 0x27u, 0x50u, 0x20u, 0x00u, 0x07u, 0x35u};
  f = mk(0x7E8u, 8u, cfB);
  CHECK_EQ(r.feed(f, pdu), ISOTP_OK);
  CHECK_EQ(pdu.len, 8);
  CHECK_EQ(pdu.data[0], 0x58);
  CHECK_EQ(pdu.data[1], 0x02);
  CHECK_EQ(pdu.data[2], 0x40);
  for (uint16_t i = 0; i < pdu.len; ++i) {
    CHECK(pdu.data[i] != 0xAAu);  // nothing from the abandoned sequence survived
  }
}

// A Single Frame arriving mid-sequence also abandons the partial sequence.
// This is the shape the car actually produces: `7F 18 78` responsePending in
// the middle of a read window.
{
  IsoTpReassembler r;
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t ff[8] = {0x10u, 0x28u, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu};
  CanFrame f = mk(0x7E9u, 8u, ff);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  const uint8_t sf[8] = {0x03u, 0x7Fu, 0x18u, 0x78u, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
  f = mk(0x7E9u, 8u, sf);
  CHECK_EQ(r.feed(f, pdu), ISOTP_OK);
  CHECK_EQ(pdu.len, 3);
  CHECK_EQ(pdu.data[0], 0x7F);
  CHECK(!r.inProgress());
}

// Sequence numbers wrap 15 -> 0. Nothing in the capture is long enough to
// reach that (the longest recorded reply is 102 bytes over 15 frames, so SN
// tops out at 14), but a full ISOTP_MAX_PDU reply needs 18 Consecutive Frames,
// so the wrap is reachable in the car and is tested here synthetically.
{
  IsoTpReassembler r;
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));

  uint8_t ff[8];
  ff[0] = 0x10u;
  ff[1] = (uint8_t)ISOTP_MAX_PDU;  // 128
  for (uint8_t i = 0; i < 6u; ++i) {
    ff[2 + i] = i;
  }
  CanFrame f = mk(0x7E8u, 8u, ff);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  CHECK_EQ(r.expected(), 128);

  uint8_t next = 6u;
  IsoTpStatus st = ISOTP_INCOMPLETE;
  uint8_t frames = 0u;
  for (uint8_t seq = 1u; frames < 20u; ++seq) {
    uint8_t cf[8];
    cf[0] = (uint8_t)(0x20u | (seq & 0x0Fu));
    for (uint8_t i = 0; i < 7u; ++i) {
      cf[1 + i] = next++;
    }
    f = mk(0x7E8u, 8u, cf);
    st = r.feed(f, pdu);
    ++frames;
    if (st != ISOTP_INCOMPLETE) {
      break;
    }
  }
  CHECK_EQ(st, ISOTP_OK);
  CHECK_EQ(frames, 18);  // 6 + 18*7 = 132 >= 128
  CHECK_EQ(pdu.len, ISOTP_MAX_PDU);
  for (uint16_t i = 0; i < ISOTP_MAX_PDU; ++i) {
    CHECK_MSG(pdu.data[i] == (uint8_t)i, "byte %u == %u, got %u", (unsigned)i, (unsigned)i,
              (unsigned)pdu.data[i]);
  }
}

// reset() mid-sequence really does discard state.
{
  IsoTpReassembler r;
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t ff[8] = {0x10u, 0x0Au, 0x61u, 0x04u, 0xE7u, 0x00u, 0x40u, 0x01u};
  CanFrame f = mk(0x7E8u, 8u, ff);
  CHECK_EQ(r.feed(f, pdu), ISOTP_INCOMPLETE);
  CHECK(r.inProgress());
  r.reset();
  CHECK(!r.inProgress());
  CHECK_EQ(r.expected(), 0);
  const uint8_t cf[8] = {0x21u, 0xA4u, 0x08u, 0xFFu, 0xFFu, 0u, 0u, 0u};
  f = mk(0x7E8u, 8u, cf);
  CHECK_EQ(r.feed(f, pdu), ISOTP_PROTOCOL);
}

// ------------------------------------------------------------------- transact
// Happy path, single-frame reply. Uses the recorded WCM `21 1A` exchange.
{
  FakeLink link;
  const uint8_t sfBytes[8] = {0x03u, 0x61u, 0x1Au, 0x9Fu, 0x00u, 0x03u, 0x03u, 0x00u};
  link.push(mk(0x500u, 8u, sfBytes));

  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[2] = {0x21u, 0x1Au};
  CHECK_EQ(client.transact(0x600u, 0x500u, req, 2u, pdu, 200u), ISOTP_OK);

  CHECK_EQ(link.setTargetCalls_, 1);
  CHECK_EQ(link.lastRequestId_, 0x600);
  CHECK_EQ(link.lastResponseId_, 0x500);

  // Exactly one frame out, and no Flow Control: the ATmega328 sends that.
  CHECK_EQ(link.sentCount_, 1);
  CHECK_EQ(link.sent_[0].id, 0x600);
  CHECK_EQ(link.sent_[0].len, 3);
  CHECK_EQ(link.sent_[0].data[0], 0x02);  // SF PCI = payload length
  CHECK_EQ(link.sent_[0].data[1], 0x21);
  CHECK_EQ(link.sent_[0].data[2], 0x1A);

  // The caller's whole budget goes on the first read, where the latency is.
  CHECK_EQ(link.firstTimeoutMs_, 200);

  CHECK_EQ(pdu.len, 3);
  CHECK_EQ(pdu.data[0], 0x61);
  CHECK_EQ(pdu.data[1], 0x1A);
  CHECK_EQ(pdu.data[2], 0x9F);
}

// Happy path, multi-frame reply, straight off the recorded PCM DTC read.
{
  const CapMultiSeq* pcm = 0;
  for (uint8_t i = 0; i < CAP_MULTI_COUNT; ++i) {
    if (strcmp(kCapMulti[i].name, "7E0 041800FF00") == 0) {
      pcm = &kCapMulti[i];
    }
  }
  CHECK(pcm != 0);
  if (pcm != 0) {
    FakeLink link;
    for (uint8_t i = 0; i < pcm->frameCount; ++i) {
      link.push(fromCap(pcm->respId, pcm->frames[i]));
    }
    IsoTpClient client(link);
    IsoTpPdu pdu;
    memset(&pdu, 0, sizeof(pdu));
    const uint8_t req[4] = {0x18u, 0x00u, 0xFFu, 0x00u};
    CHECK_EQ(client.transact(pcm->reqId, pcm->respId, req, 4u, pdu, 500u), ISOTP_OK);
    CHECK_EQ(link.sentCount_, 1);
    CHECK_EQ(link.sent_[0].len, 5);
    CHECK_EQ(link.sent_[0].data[0], 0x04);
    CHECK_EQ(pdu.len, pcm->pduLen);
    CHECK(sameBytes(pdu.data, pcm->pdu, pcm->pduLen));
    // Follow-on frames get a bounded gap, not the caller's whole budget again.
    CHECK(link.lastTimeoutMs_ <= 500u);
    CHECK(link.lastTimeoutMs_ < 500u);
  }
}

// A responsePending Single Frame completes a PDU, so transact returns it and
// lets the KWP layer decide -- this is exactly what module 6E0 does on the
// wire, and the real answer is still queued for the next read.
{
  FakeLink link;
  const uint8_t pending[8] = {0x03u, 0x7Fu, 0x18u, 0x78u, 0x00u, 0x00u, 0x00u, 0x00u};
  link.push(mk(0x51Cu, 8u, pending));
  const uint8_t ff[8] = {0x10u, 0x08u, 0x58u, 0x02u, 0x9Bu, 0xB8u, 0x20u, 0xA1u};
  link.push(mk(0x51Cu, 8u, ff));
  const uint8_t cf[8] = {0x21u, 0x2Du, 0x20u, 0x00u, 0x05u, 0xE3u, 0x18u, 0x30u};
  link.push(mk(0x51Cu, 8u, cf));

  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[4] = {0x18u, 0x00u, 0xFFu, 0x00u};
  CHECK_EQ(client.transact(0x6E0u, 0x51Cu, req, 4u, pdu, 500u), ISOTP_OK);
  CHECK_EQ(pdu.len, 3);
  CHECK_EQ(pdu.data[0], 0x7F);
  CHECK_EQ(pdu.data[2], 0x78);
  CHECK_EQ(link.readCalls_, 1);  // stopped at the first complete PDU
}

// Requests that do not fit one frame are refused before anything is sent.
{
  FakeLink link;
  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[9] = {0x2Eu, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};

  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 8u, pdu, 100u), ISOTP_REQ_TOO_LONG);
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 9u, pdu, 100u), ISOTP_REQ_TOO_LONG);
  CHECK_EQ(link.sentCount_, 0);
  CHECK_EQ(link.setTargetCalls_, 0);
  CHECK_EQ(pdu.len, 0);

  // Seven bytes is the largest that does fit.
  link.clear();
  const uint8_t sf[3] = {0x02u, 0x6Eu, 0x00u};
  link.push(mk(0x7E8u, 3u, sf));
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 7u, pdu, 100u), ISOTP_OK);
  CHECK_EQ(link.sent_[0].len, 8);
  CHECK_EQ(link.sent_[0].data[0], 0x07);
}

// Parameter validation.
{
  FakeLink link;
  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[2] = {0x21u, 0xEAu};

  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, 0, 2u, pdu, 100u), ISOTP_BAD_PARAM);
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 0u, pdu, 100u), ISOTP_BAD_PARAM);
  CHECK_EQ(client.transact(0x800u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_BAD_PARAM);
  CHECK_EQ(client.transact(0x7E0u, 0x800u, req, 2u, pdu, 100u), ISOTP_BAD_PARAM);
  CHECK_EQ(link.sentCount_, 0);
  CHECK_EQ(link.setTargetCalls_, 0);
}

// Link failures map through, and nothing is sent once addressing failed.
{
  FakeLink link;
  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[2] = {0x21u, 0xEAu};

  link.setTargetStatus_ = LINK_NOT_READY;
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_LINK_ERROR);
  CHECK_EQ(link.sentCount_, 0);

  link.clear();
  link.setTargetStatus_ = LINK_BAD_PARAM;
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_BAD_PARAM);

  link.clear();
  link.sendStatus_ = LINK_CAN_ERROR;
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_LINK_ERROR);
  CHECK_EQ(link.sentCount_, 1);

  // Nothing came back at all.
  link.clear();
  link.tailStatus_ = LINK_NO_DATA;
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_NO_RESPONSE);
  CHECK_EQ(pdu.len, 0);

  // Adapter fell over mid-read.
  link.clear();
  link.pushStatus(LINK_OVERFLOW);
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_LINK_ERROR);
}

// A First Frame that never gets its Consecutive Frames is "no response", not a
// half-filled PDU.
{
  FakeLink link;
  const uint8_t ff[8] = {0x10u, 0x17u, 0x58u, 0x07u, 0x04u, 0x60u, 0x20u, 0xC1u};
  link.push(mk(0x7E8u, 8u, ff));
  link.tailStatus_ = LINK_NO_DATA;

  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[4] = {0x18u, 0x00u, 0xFFu, 0x00u};
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 4u, pdu, 300u), ISOTP_NO_RESPONSE);
  CHECK_EQ(pdu.len, 0);
}

// A protocol error is reported, not swallowed, and the next transact is clean.
{
  FakeLink link;
  const uint8_t stray[8] = {0x25u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};  // CF with no FF
  link.push(mk(0x7E8u, 8u, stray));

  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[2] = {0x21u, 0xEAu};
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_PROTOCOL);
  CHECK_EQ(pdu.len, 0);

  link.clear();
  const uint8_t sf[8] = {0x04u, 0x61u, 0xEAu, 0x2Bu, 0x00u, 0x00u, 0x07u, 0x35u};
  link.push(mk(0x7E8u, 8u, sf));
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_OK);
  CHECK_EQ(pdu.len, 4);
}

// An overflowing First Frame surfaces as ISOTP_OVERFLOW through transact too.
{
  FakeLink link;
  const uint8_t ffOver[8] = {0x14u, 0x00u, 1u, 2u, 3u, 4u, 5u, 6u};  // 1024 bytes
  link.push(mk(0x7E8u, 8u, ffOver));
  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[2] = {0x21u, 0x93u};
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 100u), ISOTP_OVERFLOW);
  CHECK_EQ(pdu.len, 0);
}

// Frames addressed to somebody else are skipped; frames with no reported id
// (a link running headers-off, as the recorded session did) are accepted.
{
  FakeLink link;
  const uint8_t other[8] = {0x03u, 0x7Fu, 0x21u, 0x31u, 0u, 0u, 0u, 0u};
  link.push(mk(0x7E9u, 8u, other));  // wrong responder
  const uint8_t mine[8] = {0x04u, 0x61u, 0xEAu, 0x2Bu, 0x00u, 0u, 0u, 0u};
  link.push(mk(0x0u, 8u, mine));  // id not reported by the link

  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[2] = {0x21u, 0xEAu};
  CHECK_EQ(client.transact(0x600u, 0x500u, req, 2u, pdu, 100u), ISOTP_OK);
  CHECK_EQ(pdu.len, 4);
  CHECK_EQ(pdu.data[1], 0xEA);
  CHECK_EQ(link.readCalls_, 2);
}

// An adapter that only ever hands back Flow Control frames must not spin us
// forever: the frame budget ends the exchange.
{
  FakeLink link;
  const uint8_t fc[8] = {0x30u, 0x00u, 0x00u, 0u, 0u, 0u, 0u, 0u};
  for (uint8_t i = 0; i < 70u; ++i) {
    link.push(mk(0x7E8u, 8u, fc));
  }
  IsoTpClient client(link);
  IsoTpPdu pdu;
  memset(&pdu, 0, sizeof(pdu));
  const uint8_t req[2] = {0x21u, 0xEAu};
  CHECK_EQ(client.transact(0x7E0u, 0x7E8u, req, 2u, pdu, 50u), ISOTP_NO_RESPONSE);
  CHECK(link.readCalls_ <= 70u);
  CHECK(link.readCalls_ >= 20u);
  CHECK_EQ(pdu.len, 0);
}

// Every recorded exchange, replayed end to end through transact.
{
  for (uint8_t i = 0; i < CAP_MULTI_COUNT; ++i) {
    const CapMultiSeq& s = kCapMulti[i];
    FakeLink link;
    for (uint8_t k = 0; k < s.frameCount; ++k) {
      link.push(fromCap(s.respId, s.frames[k]));
    }
    IsoTpClient client(link);
    IsoTpPdu pdu;
    memset(&pdu, 0, sizeof(pdu));
    const uint8_t req[2] = {0x21u, 0x00u};
    const IsoTpStatus st = client.transact(s.reqId, s.respId, req, 2u, pdu, 400u);
    CHECK_MSG(st == ISOTP_OK, "%s: transact gave %d", s.name, (int)st);
    CHECK_MSG(pdu.len == s.pduLen, "%s: len %u vs %u", s.name, (unsigned)pdu.len,
              (unsigned)s.pduLen);
    if (pdu.len == s.pduLen) {
      CHECK_MSG(sameBytes(pdu.data, s.pdu, s.pduLen), "%s: payload mismatch", s.name);
    }
    CHECK_MSG(link.sentCount_ == 1u, "%s: sent %u frames, expected exactly 1 (no Flow Control)",
              s.name, (unsigned)link.sentCount_);
  }
}

TEST_MAIN_END()
