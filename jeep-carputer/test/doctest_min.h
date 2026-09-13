#pragma once
// Minimal assertion helpers, so the host suites need no external framework and
// no package fetch. Each suite is a plain main() that calls CHECK/CHECK_EQ and
// returns non-zero on failure.
#include <cstdio>
#include <cstring>
#include <cstdlib>

extern int g_checks;
extern int g_failures;

#define CHECK_MSG(cond, ...)                                              \
  do {                                                                    \
    ++g_checks;                                                           \
    if (!(cond)) {                                                        \
      ++g_failures;                                                       \
      std::printf("FAIL %s:%d  ", __FILE__, __LINE__);                    \
      std::printf(__VA_ARGS__);                                           \
      std::printf("\n");                                                  \
    }                                                                     \
  } while (0)

#define CHECK(cond) CHECK_MSG((cond), "%s", #cond)

#define CHECK_EQ(a, b)                                                    \
  do {                                                                    \
    const long long _a = (long long)(a);                                  \
    const long long _b = (long long)(b);                                  \
    CHECK_MSG(_a == _b, "%s == %s  (%lld vs %lld)", #a, #b, _a, _b);      \
  } while (0)

#define CHECK_STR(a, b)                                                   \
  do {                                                                    \
    const char* _a = (a);                                                 \
    const char* _b = (b);                                                 \
    CHECK_MSG(_a && _b && std::strcmp(_a, _b) == 0, "%s == %s  (\"%s\" vs \"%s\")", \
              #a, #b, _a ? _a : "(null)", _b ? _b : "(null)");            \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                             \
  do {                                                                    \
    const double _a = (double)(a);                                        \
    const double _b = (double)(b);                                        \
    const double _d = _a > _b ? _a - _b : _b - _a;                        \
    CHECK_MSG(_d <= (eps), "%s ~= %s  (%f vs %f)", #a, #b, _a, _b);       \
  } while (0)

#define TEST_MAIN_BEGIN() int g_checks = 0; int g_failures = 0; int main() {
#define TEST_MAIN_END()                                                   \
  std::printf("%d checks, %d failures\n", g_checks, g_failures);          \
  return g_failures == 0 ? 0 : 1;                                         \
  }
