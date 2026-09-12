// Host tests for the pure ISO-TP Flow Control helper.
//   pio test -e native -f test_isotp_fc
// These were written and then actually run on the host with clang++ 20 and a
// vendored Unity, outside PlatformIO (the AVR toolchain is not installed here,
// so `pio test` itself has not been exercised -- expect at most include-path
// fixups, not logic changes).
#include <string.h>
#include <unity.h>

#include "IsoTpFc.h"

static IsoTpFcConfig cfg;

void setUp(void) {
  isoTpFcDefaults(cfg);
}

void tearDown(void) {}

static CanFrame frame(uint16_t id, uint8_t len, uint8_t b0, uint8_t b1) {
  CanFrame f;
  f.id = id;
  f.len = len;
  for (uint8_t i = 0; i < CAN_MAX_DLC; ++i) {
    f.data[i] = 0x00;
  }
  f.data[0] = b0;
  f.data[1] = b1;
  return f;
}

// ---- classification -----------------------------------------------------

static void test_classify(void) {
  TEST_ASSERT_EQUAL(ISOTP_TYPE_SINGLE, isoTpClassify(frame(0x7E8, 8, 0x03, 0x41)));
  TEST_ASSERT_EQUAL(ISOTP_TYPE_FIRST, isoTpClassify(frame(0x7E8, 8, 0x10, 0x14)));
  TEST_ASSERT_EQUAL(ISOTP_TYPE_CONSEC, isoTpClassify(frame(0x7E8, 8, 0x21, 0x00)));
  TEST_ASSERT_EQUAL(ISOTP_TYPE_FLOW, isoTpClassify(frame(0x7E0, 3, 0x30, 0xFF)));
  // 0x4x..0xFx are not ISO-TP PCI values.
  TEST_ASSERT_EQUAL(ISOTP_TYPE_UNKNOWN, isoTpClassify(frame(0x09F, 8, 0x46, 0x00)));
  TEST_ASSERT_EQUAL(ISOTP_TYPE_UNKNOWN, isoTpClassify(frame(0x09F, 8, 0xF0, 0x00)));
}

// A zero-length frame is legal CAN. data[0] must not be looked at.
static void test_zero_length_frame_is_safe(void) {
  CanFrame f = frame(0x7E8, 0, 0x10, 0x14);   // FF-looking bytes, but len 0
  TEST_ASSERT_EQUAL(ISOTP_TYPE_UNKNOWN, isoTpClassify(f));
  TEST_ASSERT_FALSE(isoTpIsFirstFrame(f));
  TEST_ASSERT_EQUAL_UINT16(0, isoTpFirstFrameLen(f));
  TEST_ASSERT_EQUAL_UINT8(0, isoTpSingleFrameLen(f));
  TEST_ASSERT_EQUAL_UINT8(0, isoTpConsecPayloadLen(f));
}

static void test_first_frame_detection(void) {
  TEST_ASSERT_TRUE(isoTpIsFirstFrame(frame(0x7E8, 8, 0x10, 0x14)));
  TEST_ASSERT_TRUE(isoTpIsFirstFrame(frame(0x7E8, 8, 0x1F, 0xFF)));   // FF_DL 4095
  TEST_ASSERT_FALSE(isoTpIsFirstFrame(frame(0x7E8, 8, 0x03, 0x41)));
  TEST_ASSERT_FALSE(isoTpIsFirstFrame(frame(0x7E8, 8, 0x21, 0x00)));
  TEST_ASSERT_FALSE(isoTpIsFirstFrame(frame(0x7E8, 8, 0x30, 0x00)));
}

static void test_first_frame_length(void) {
  TEST_ASSERT_EQUAL_UINT16(0x014, isoTpFirstFrameLen(frame(0x7E8, 8, 0x10, 0x14)));
  TEST_ASSERT_EQUAL_UINT16(0xFFF, isoTpFirstFrameLen(frame(0x7E8, 8, 0x1F, 0xFF)));
  // Too short to carry the low byte of FF_DL.
  TEST_ASSERT_EQUAL_UINT16(0, isoTpFirstFrameLen(frame(0x7E8, 1, 0x10, 0x14)));
}

static void test_payload_lengths(void) {
  TEST_ASSERT_EQUAL_UINT8(3, isoTpSingleFrameLen(frame(0x7E8, 8, 0x03, 0x41)));
  TEST_ASSERT_EQUAL_UINT8(0, isoTpSingleFrameLen(frame(0x7E8, 8, 0x10, 0x14)));
  TEST_ASSERT_EQUAL_UINT8(7, isoTpConsecPayloadLen(frame(0x7E8, 8, 0x21, 0x00)));
  TEST_ASSERT_EQUAL_UINT8(2, isoTpConsecPayloadLen(frame(0x7E8, 3, 0x22, 0x00)));
  TEST_ASSERT_EQUAL_UINT8(0, isoTpConsecPayloadLen(frame(0x7E8, 8, 0x03, 0x41)));
}

// ---- defaults -----------------------------------------------------------

static void test_defaults(void) {
  TEST_ASSERT_EQUAL_HEX16(ISOTP_FC_HEADER_UNSET, cfg.header);
  TEST_ASSERT_EQUAL_UINT8(3, cfg.len);
  TEST_ASSERT_EQUAL_HEX8(0x30, cfg.data[0]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, cfg.data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, cfg.data[2]);
  TEST_ASSERT_EQUAL_UINT8(ISOTP_FC_MODE_AUTO, cfg.mode);
}

// ---- FC construction ----------------------------------------------------

static void test_fc_default_uses_request_header(void) {
  const CanFrame ff = frame(0x7E8, 8, 0x10, 0x14);
  CanFrame out;
  TEST_ASSERT_EQUAL(ISOTP_FC_OK, isoTpBuildFc(cfg, ff, 0x7E0, out));

  // No ATFCSH given -> the FC goes back out on the request header.
  TEST_ASSERT_EQUAL_HEX16(0x7E0, out.id);
  // Exactly 3 bytes: clear to send, block size unlimited, STmin 0. No padding
  // to DLC 8 -- that is deliberate.
  TEST_ASSERT_EQUAL_UINT8(3, out.len);
  TEST_ASSERT_EQUAL_HEX8(0x30, out.data[0]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, out.data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, out.data[2]);
}

static void test_fc_header_override(void) {
  cfg.header = 0x123;                     // ATFCSH 123
  const CanFrame ff = frame(0x7E8, 8, 0x10, 0x14);
  CanFrame out;
  TEST_ASSERT_EQUAL(ISOTP_FC_OK, isoTpBuildFc(cfg, ff, 0x7E0, out));
  TEST_ASSERT_EQUAL_HEX16(0x123, out.id);
}

static void test_fc_data_override(void) {
  // ATFCSD 30 00 0A -- clear to send, unlimited, 10 ms between frames.
  cfg.data[0] = 0x30;
  cfg.data[1] = 0x00;
  cfg.data[2] = 0x0A;
  cfg.len = 3;
  const CanFrame ff = frame(0x7E8, 8, 0x10, 0x14);
  CanFrame out;
  TEST_ASSERT_EQUAL(ISOTP_FC_OK, isoTpBuildFc(cfg, ff, 0x7E0, out));
  TEST_ASSERT_EQUAL_UINT8(3, out.len);
  TEST_ASSERT_EQUAL_HEX8(0x0A, out.data[2]);
}

static void test_fc_data_override_padded_to_eight(void) {
  // The workaround for a module that ignores an unpadded FC:
  //   ATFCSD 30 FF 00 00 00 00 00 00
  for (uint8_t i = 0; i < CAN_MAX_DLC; ++i) {
    cfg.data[i] = 0x00;
  }
  cfg.data[0] = 0x30;
  cfg.data[1] = 0xFF;
  cfg.len = CAN_MAX_DLC;
  const CanFrame ff = frame(0x7E8, 8, 0x10, 0x14);
  CanFrame out;
  TEST_ASSERT_EQUAL(ISOTP_FC_OK, isoTpBuildFc(cfg, ff, 0x7E0, out));
  TEST_ASSERT_EQUAL_UINT8(8, out.len);
  TEST_ASSERT_EQUAL_HEX8(0xFF, out.data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, out.data[7]);
}

static void test_fc_single_byte_data(void) {
  cfg.data[0] = 0x30;
  cfg.len = 1;
  CanFrame out;
  TEST_ASSERT_EQUAL(ISOTP_FC_OK, isoTpBuildFc(cfg, frame(0x7E8, 8, 0x10, 0x14), 0x7E0, out));
  TEST_ASSERT_EQUAL_UINT8(1, out.len);
  // Bytes beyond len are zeroed rather than left as whatever was on the stack.
  TEST_ASSERT_EQUAL_HEX8(0x00, out.data[1]);
}

// ---- refusals -----------------------------------------------------------

static void test_fc_not_first_frame(void) {
  CanFrame out;
  TEST_ASSERT_EQUAL(ISOTP_FC_NOT_FIRST_FRAME,
                    isoTpBuildFc(cfg, frame(0x7E8, 8, 0x03, 0x41), 0x7E0, out));
  TEST_ASSERT_EQUAL(ISOTP_FC_NOT_FIRST_FRAME,
                    isoTpBuildFc(cfg, frame(0x7E8, 0, 0x10, 0x14), 0x7E0, out));
}

// Every ATFCSM mode still answers the First Frame -- a mode that suppressed
// the FC would stall every multi-frame reply, so none does. Mode 1 is the
// documented "pad the FC to DLC 8" workaround, which is really an ATFCSD.
static void test_fc_every_mode_still_answers(void) {
  const CanFrame ff = frame(0x7E8, 8, 0x10, 0x14);
  CanFrame out;
  for (uint8_t m = 0; m <= ISOTP_FC_MODE_MAX; ++m) {
    cfg.mode = m;
    TEST_ASSERT_EQUAL(ISOTP_FC_OK, isoTpBuildFc(cfg, ff, 0x7E0, out));
    TEST_ASSERT_EQUAL_HEX16(0x7E0, out.id);
    TEST_ASSERT_EQUAL_HEX8(0x30, out.data[0]);
  }
  cfg.mode = ISOTP_FC_MODE_MAX + 1u;
  TEST_ASSERT_EQUAL(ISOTP_FC_BAD_CONFIG, isoTpBuildFc(cfg, ff, 0x7E0, out));
}

static void test_fc_no_header(void) {
  CanFrame out;
  // No ATFCSH and no usable request header either.
  TEST_ASSERT_EQUAL(ISOTP_FC_NO_HEADER,
                    isoTpBuildFc(cfg, frame(0x7E8, 8, 0x10, 0x14),
                                 ISOTP_FC_HEADER_UNSET, out));
}

static void test_fc_bad_config(void) {
  CanFrame out;
  const CanFrame ff = frame(0x7E8, 8, 0x10, 0x14);

  cfg.len = 0;
  TEST_ASSERT_EQUAL(ISOTP_FC_BAD_CONFIG, isoTpBuildFc(cfg, ff, 0x7E0, out));

  isoTpFcDefaults(cfg);
  cfg.len = CAN_MAX_DLC + 1u;
  TEST_ASSERT_EQUAL(ISOTP_FC_BAD_CONFIG, isoTpBuildFc(cfg, ff, 0x7E0, out));

  isoTpFcDefaults(cfg);
  cfg.header = 0x800;                     // not an 11-bit id
  TEST_ASSERT_EQUAL(ISOTP_FC_BAD_CONFIG, isoTpBuildFc(cfg, ff, 0x7E0, out));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_classify);
  RUN_TEST(test_zero_length_frame_is_safe);
  RUN_TEST(test_first_frame_detection);
  RUN_TEST(test_first_frame_length);
  RUN_TEST(test_payload_lengths);
  RUN_TEST(test_defaults);
  RUN_TEST(test_fc_default_uses_request_header);
  RUN_TEST(test_fc_header_override);
  RUN_TEST(test_fc_data_override);
  RUN_TEST(test_fc_data_override_padded_to_eight);
  RUN_TEST(test_fc_single_byte_data);
  RUN_TEST(test_fc_not_first_frame);
  RUN_TEST(test_fc_every_mode_still_answers);
  RUN_TEST(test_fc_no_header);
  RUN_TEST(test_fc_bad_config);
  return UNITY_END();
}
