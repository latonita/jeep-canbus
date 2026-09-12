// Host tests for the pure line accumulator and hex helpers.
//   pio test -e native -f test_at_line
// These were written and then actually run on the host with clang++ 20 and a
// vendored Unity, outside PlatformIO (the AVR toolchain is not installed here,
// so `pio test` itself has not been exercised -- expect at most include-path
// fixups, not logic changes).
#include <string.h>
#include <unity.h>

#include "AtLine.h"

static AtLine line;

void setUp(void) {
  line.reset();
}

void tearDown(void) {}

// Feeds a string and returns the status of the last character.
static AtLineStatus feed(const char* s) {
  AtLineStatus st = AT_LINE_PENDING;
  for (const char* p = s; *p != '\0'; ++p) {
    st = line.push(*p);
  }
  return st;
}

// ---- line accumulation ---------------------------------------------------

static void test_cr_terminates(void) {
  TEST_ASSERT_EQUAL(AT_LINE_PENDING, line.push('A'));
  TEST_ASSERT_EQUAL(AT_LINE_PENDING, line.push('T'));
  TEST_ASSERT_EQUAL(AT_LINE_PENDING, line.push('Z'));
  TEST_ASSERT_EQUAL(AT_LINE_READY, line.push('\r'));
  TEST_ASSERT_EQUAL_STRING("ATZ", line.text());
  TEST_ASSERT_EQUAL_UINT8(3, line.length());
}

static void test_lf_terminates(void) {
  TEST_ASSERT_EQUAL(AT_LINE_READY, feed("ATI\n"));
  TEST_ASSERT_EQUAL_STRING("ATI", line.text());
}

static void test_crlf_gives_one_line_then_empty(void) {
  TEST_ASSERT_EQUAL(AT_LINE_READY, feed("ATI\r"));
  TEST_ASSERT_EQUAL_STRING("ATI", line.text());
  // The LF of the pair must not look like a second command.
  TEST_ASSERT_EQUAL(AT_LINE_EMPTY, line.push('\n'));
}

static void test_bare_enter_is_empty(void) {
  TEST_ASSERT_EQUAL(AT_LINE_EMPTY, line.push('\r'));
  TEST_ASSERT_EQUAL_UINT8(0, line.length());
}

static void test_buffer_recycles_after_ready(void) {
  TEST_ASSERT_EQUAL(AT_LINE_READY, feed("ATE0\r"));
  TEST_ASSERT_EQUAL(AT_LINE_READY, feed("ATE1\r"));
  TEST_ASSERT_EQUAL_STRING("ATE1", line.text());
}

static void test_partial_tracks_half_typed_line(void) {
  TEST_ASSERT_FALSE(line.partial());
  line.push('A');
  TEST_ASSERT_TRUE(line.partial());
  line.push('T');
  line.push('I');
  line.push('\r');
  // Reported lines are not "partial" even though text() still holds them.
  TEST_ASSERT_FALSE(line.partial());
}

// ---- overflow ------------------------------------------------------------

static void test_overflow_is_reported_and_discarded(void) {
  for (uint8_t i = 0; i < AT_LINE_MAX; ++i) {
    TEST_ASSERT_EQUAL(AT_LINE_PENDING, line.push('A'));
  }
  TEST_ASSERT_EQUAL_UINT8(AT_LINE_MAX, line.length());
  // One char too many: swallowed, not stored.
  TEST_ASSERT_EQUAL(AT_LINE_PENDING, line.push('B'));
  TEST_ASSERT_EQUAL_UINT8(AT_LINE_MAX, line.length());
  TEST_ASSERT_TRUE(line.partial());

  TEST_ASSERT_EQUAL(AT_LINE_OVERFLOW, line.push('\r'));
  // Discarded, NOT truncated-and-executed.
  TEST_ASSERT_EQUAL_UINT8(0, line.length());
  TEST_ASSERT_EQUAL_STRING("", line.text());
}

static void test_overflow_recovers(void) {
  for (uint8_t i = 0; i < AT_LINE_MAX + 5u; ++i) {
    line.push('A');
  }
  TEST_ASSERT_EQUAL(AT_LINE_OVERFLOW, line.push('\r'));
  TEST_ASSERT_EQUAL(AT_LINE_READY, feed("ATI\r"));
  TEST_ASSERT_EQUAL_STRING("ATI", line.text());
}

static void test_exactly_max_is_not_overflow(void) {
  for (uint8_t i = 0; i < AT_LINE_MAX; ++i) {
    line.push('7');
  }
  TEST_ASSERT_EQUAL(AT_LINE_READY, line.push('\r'));
  TEST_ASSERT_EQUAL_UINT8(AT_LINE_MAX, line.length());
}

// ---- normalisation -------------------------------------------------------

static void test_spaces_stripped_and_uppercased(void) {
  TEST_ASSERT_EQUAL(AT_LINE_READY, feed("at sh 7e0\r"));
  TEST_ASSERT_EQUAL_STRING("ATSH7E0", line.text());
}

static void test_tabs_and_control_bytes_ignored(void) {
  TEST_ASSERT_EQUAL(AT_LINE_READY, feed("a\tt\x01i\r"));
  TEST_ASSERT_EQUAL_STRING("ATI", line.text());
}

static void test_spaces_only_line_is_empty(void) {
  TEST_ASSERT_EQUAL(AT_LINE_EMPTY, feed("   \r"));
}

// ---- hex byte parsing ----------------------------------------------------

static void test_hex_round_trip(void) {
  uint8_t bytes[8];
  uint8_t n = 0;
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseHexBytes("022130", bytes, sizeof(bytes), n));
  TEST_ASSERT_EQUAL_UINT8(3, n);
  TEST_ASSERT_EQUAL_HEX8(0x02, bytes[0]);
  TEST_ASSERT_EQUAL_HEX8(0x21, bytes[1]);
  TEST_ASSERT_EQUAL_HEX8(0x30, bytes[2]);

  char out[32];
  TEST_ASSERT_EQUAL_UINT8(6, atFormatHexBytes(bytes, n, out, sizeof(out), false));
  TEST_ASSERT_EQUAL_STRING("022130", out);
  TEST_ASSERT_EQUAL_UINT8(8, atFormatHexBytes(bytes, n, out, sizeof(out), true));
  TEST_ASSERT_EQUAL_STRING("02 21 30", out);

  uint8_t again[8];
  uint8_t m = 0;
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseHexBytes("022130", again, sizeof(again), m));
  TEST_ASSERT_EQUAL_UINT8(n, m);
  TEST_ASSERT_EQUAL_MEMORY(bytes, again, n);
}

static void test_hex_full_dlc(void) {
  uint8_t bytes[8];
  uint8_t n = 0;
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseHexBytes("00112233445566FF", bytes, sizeof(bytes), n));
  TEST_ASSERT_EQUAL_UINT8(8, n);
  TEST_ASSERT_EQUAL_HEX8(0xFF, bytes[7]);
}

static void test_hex_lowercase_accepted(void) {
  uint8_t bytes[4];
  uint8_t n = 0;
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseHexBytes("abcd", bytes, sizeof(bytes), n));
  TEST_ASSERT_EQUAL_UINT8(2, n);
  TEST_ASSERT_EQUAL_HEX8(0xAB, bytes[0]);
  TEST_ASSERT_EQUAL_HEX8(0xCD, bytes[1]);
}

static void test_hex_odd_digits_rejected(void) {
  uint8_t bytes[8];
  uint8_t n = 99;
  TEST_ASSERT_EQUAL(AT_HEX_ODD_DIGITS, atParseHexBytes("02213", bytes, sizeof(bytes), n));
  TEST_ASSERT_EQUAL_UINT8(0, n);
}

static void test_hex_bad_char_rejected(void) {
  uint8_t bytes[8];
  uint8_t n = 99;
  TEST_ASSERT_EQUAL(AT_HEX_BAD_CHAR, atParseHexBytes("02G1", bytes, sizeof(bytes), n));
  TEST_ASSERT_EQUAL_UINT8(0, n);
  TEST_ASSERT_EQUAL(AT_HEX_BAD_CHAR, atParseHexBytes("02-1", bytes, sizeof(bytes), n));
}

static void test_hex_empty_and_too_long(void) {
  uint8_t bytes[2];
  uint8_t n = 99;
  TEST_ASSERT_EQUAL(AT_HEX_EMPTY, atParseHexBytes("", bytes, sizeof(bytes), n));
  TEST_ASSERT_EQUAL_UINT8(0, n);
  TEST_ASSERT_EQUAL(AT_HEX_TOO_LONG, atParseHexBytes("001122", bytes, sizeof(bytes), n));
  TEST_ASSERT_EQUAL_UINT8(0, n);
}

static void test_is_hex_string(void) {
  TEST_ASSERT_TRUE(atIsHexString("0AF"));
  TEST_ASSERT_FALSE(atIsHexString(""));
  TEST_ASSERT_FALSE(atIsHexString("ATZ"));
  TEST_ASSERT_FALSE(atIsHexString("0A "));
}

// ---- 11-bit id ----------------------------------------------------------

static void test_id11_bounds(void) {
  uint16_t id = 0xFFFF;
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseId11("7E0", id));
  TEST_ASSERT_EQUAL_HEX16(0x7E0, id);
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseId11("7FF", id));
  TEST_ASSERT_EQUAL_HEX16(0x7FF, id);
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseId11("000", id));
  TEST_ASSERT_EQUAL_HEX16(0x000, id);

  // Fewer than 3 nibbles is accepted, like ELM.
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseId11("9F", id));
  TEST_ASSERT_EQUAL_HEX16(0x09F, id);
  TEST_ASSERT_EQUAL(AT_HEX_OK, atParseId11("6", id));
  TEST_ASSERT_EQUAL_HEX16(0x006, id);

  // 3 nibbles can express 0xFFF, so anything over 0x7FF must be rejected
  // rather than silently masked into a different id.
  TEST_ASSERT_EQUAL(AT_HEX_RANGE, atParseId11("800", id));
  TEST_ASSERT_EQUAL(AT_HEX_RANGE, atParseId11("FFF", id));
  TEST_ASSERT_EQUAL(AT_HEX_RANGE, atParseId11("07E0", id));
  TEST_ASSERT_EQUAL(AT_HEX_BAD_CHAR, atParseId11("7G0", id));
  TEST_ASSERT_EQUAL(AT_HEX_EMPTY, atParseId11("", id));
}

static void test_format_id11(void) {
  char out[4];
  TEST_ASSERT_EQUAL_UINT8(3, atFormatId11(0x09F, out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("09F", out);
  TEST_ASSERT_EQUAL_UINT8(3, atFormatId11(0x7FF, out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("7FF", out);
  TEST_ASSERT_EQUAL_UINT8(3, atFormatId11(0x000, out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("000", out);

  char tiny[3];
  TEST_ASSERT_EQUAL_UINT8(0, atFormatId11(0x123, tiny, sizeof(tiny)));
  TEST_ASSERT_EQUAL_STRING("", tiny);
}

// ---- formatting guards --------------------------------------------------

static void test_format_hex_capacity_guard(void) {
  const uint8_t bytes[3] = {0x01, 0x02, 0x03};
  char small[6];
  // Needs 7 with separators, 7 without (6 digits + NUL) -- both must refuse.
  TEST_ASSERT_EQUAL_UINT8(0, atFormatHexBytes(bytes, 3, small, sizeof(small), true));
  TEST_ASSERT_EQUAL_STRING("", small);
  char exact[7];
  TEST_ASSERT_EQUAL_UINT8(6, atFormatHexBytes(bytes, 3, exact, sizeof(exact), false));
  TEST_ASSERT_EQUAL_STRING("010203", exact);
  TEST_ASSERT_EQUAL_UINT8(0, atFormatHexBytes(bytes, 0, exact, sizeof(exact), false));
}

static void test_format_dec(void) {
  char out[8];
  TEST_ASSERT_EQUAL_UINT8(1, atFormatDec(0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("0", out);
  TEST_ASSERT_EQUAL_UINT8(5, atFormatDec(65535u, out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("65535", out);
  char small[3];
  TEST_ASSERT_EQUAL_UINT8(0, atFormatDec(1234u, small, sizeof(small)));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_cr_terminates);
  RUN_TEST(test_lf_terminates);
  RUN_TEST(test_crlf_gives_one_line_then_empty);
  RUN_TEST(test_bare_enter_is_empty);
  RUN_TEST(test_buffer_recycles_after_ready);
  RUN_TEST(test_partial_tracks_half_typed_line);
  RUN_TEST(test_overflow_is_reported_and_discarded);
  RUN_TEST(test_overflow_recovers);
  RUN_TEST(test_exactly_max_is_not_overflow);
  RUN_TEST(test_spaces_stripped_and_uppercased);
  RUN_TEST(test_tabs_and_control_bytes_ignored);
  RUN_TEST(test_spaces_only_line_is_empty);
  RUN_TEST(test_hex_round_trip);
  RUN_TEST(test_hex_full_dlc);
  RUN_TEST(test_hex_lowercase_accepted);
  RUN_TEST(test_hex_odd_digits_rejected);
  RUN_TEST(test_hex_bad_char_rejected);
  RUN_TEST(test_hex_empty_and_too_long);
  RUN_TEST(test_is_hex_string);
  RUN_TEST(test_id11_bounds);
  RUN_TEST(test_format_id11);
  RUN_TEST(test_format_hex_capacity_guard);
  RUN_TEST(test_format_dec);
  return UNITY_END();
}
