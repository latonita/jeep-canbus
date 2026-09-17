#!/usr/bin/env sh
# Build and run every host test suite in test/.
#
# The whole of core/ is pure C++ with no ESP-IDF dependency, precisely so it can
# be tested on a laptop instead of by flashing a board and driving to a car.
# Each test/test_*/ directory is one suite, linked against all of core/.
#
# Usage:  sh scripts/run-host-tests.sh [suite-name-substring]
#         CXX=clang++ sh scripts/run-host-tests.sh
#         HOSTTEST_SAN=0 sh scripts/run-host-tests.sh      # no sanitizers
set -e
cd "$(dirname "$0")/.."

# CXX is a BARE COMPILER PATH, never a compiler plus flags: the check below is
# `command -v "$CXX"`, which looks the whole string up as one executable name
# and fails on `clang++ -m32`. Everything that is not the binary goes in FLAGS.
CXX=${CXX:-$(command -v clang++ >/dev/null 2>&1 && echo clang++ || echo g++)}
command -v "$CXX" >/dev/null 2>&1 || { echo "no C++ compiler ($CXX)"; exit 1; }

OUT=build/hosttest
mkdir -p "$OUT"

# SANITIZERS, ON BY DEFAULT.
#
# All seventeen suites were verified clean under address+undefined before this was
# switched on, so it costs nothing today and names the next mistake instead of
# leaving it as a bare exit code -- a segfault in test_accel during recent work
# was minutes of bisecting that ASan would have answered in one line. core/ is
# where every buffer here lives (ISO-TP reassembly, the 16-slot fault store, the
# line parsers) and it is compiled from source into every suite, so the whole of
# the code under test is instrumented rather than just the harness.
#
# -fno-sanitize-recover=undefined is what makes UBSan COUNT. Its default is to
# print a diagnostic and carry on, which means a signed overflow or a
# misaligned load would scroll past above a "0 failures" line and the script
# would exit 0 -- a finding nobody acts on is worse than no finding, because it
# teaches the reader that the noise is normal. ASan aborts of its own accord.
#
# -g and -fno-omit-frame-pointer are for the report, not the checking: without
# them the stack in an ASan trace is file-less and often short, and the point of
# turning this on is being TOLD where, not being told that.
#
# HOSTTEST_SAN=0 for a plain build. About 2x on these suites, which is nothing
# at this size -- the opt-out exists for bisecting a build issue where the
# instrumentation is one variable too many, not as a routine shortcut.
SAN_FLAGS=""
if [ "${HOSTTEST_SAN:-1}" != "0" ]; then
  SAN_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=undefined \
-fno-omit-frame-pointer -g"
fi

FLAGS="-std=gnu++14 -Wall -Wextra -Wshadow -DHOST_TEST -Icore -Itest $SAN_FLAGS"

CORE=$(find core -name '*.cpp' | sort | tr '\n' ' ')
[ -n "$CORE" ] || { echo "no core sources yet"; exit 1; }

echo "CXX=$CXX"
if [ -n "$SAN_FLAGS" ]; then
  echo "san: address,undefined"
else
  echo "san: OFF (HOSTTEST_SAN=0)"
fi
echo "core: $CORE"

# Run one built suite, and let NOTHING but the suite decide pass or fail.
#
# ASan on this Windows/clang target (18.1.8, x86_64-pc-windows-msvc) prints one
#   ==NNNN==interception_win: unhandled instruction at 0x...: 0f 29 74 ...
# line to stderr at startup on every run. It is harmless -- an interceptor
# declining one instruction encoding in a system DLL -- and it is not a
# finding, but it is scary enough to be read as one, so it is filtered out of
# the display. Only that line: a genuine ASan or UBSan report also arrives on
# stderr and has to be seen in full.
#
# The filter must not become the thing that decides the outcome. POSIX sh has
# no `pipefail`, so a pipeline's status is the LAST command's -- pipe a suite
# through grep and grep's exit code is what `|| fail=1` tests, which is 1
# whenever the filter removed every line and 0 for a suite that failed with
# output. So the suite's own status is stashed in a file and read back
# afterwards, and the `|| true` keeps `set -e` out of it.
#
# ONLY STDERR goes through the pipe -- the fd 3 dance is what keeps the suite's
# own stdout on whatever it was already pointed at. `cmd 2>&1 | grep` would
# have been shorter and is wrong twice over: it interleaves a sanitizer report
# into the check output, and it puts stdout into a pipe, where the CRT buffers
# it in full. test/doctest_min.h flushes each FAIL line for the same reason --
# ASan ABORTS, and an unflushed buffer dies with the process, so the report
# would be a sanitizer trace with no sign of which checks had already failed.
# Both halves are needed and both are verified: a deliberately failing check
# followed by a heap overflow shows the FAIL line, then the ASan report, and
# exits 1.
run_suite() {
  # $1 = executable, $2 = name
  rc_file="$OUT/$2.rc"
  rm -f "$rc_file"
  { { "$1"; echo $? >"$rc_file"; } 2>&1 1>&3 |
      grep -v 'interception_win' >&2 || true; } 3>&1
  # A missing status file means the shell never got to write one -- count that
  # as a failure rather than as a pass by absence.
  [ -f "$rc_file" ] && [ "$(cat "$rc_file")" = "0" ]
}

filter=${1:-}
fail=0
found=0
for dir in test/test_*; do
  [ -d "$dir" ] || continue
  name=$(basename "$dir")
  case "$name" in *"$filter"*) ;; *) continue ;; esac
  srcs=$(find "$dir" -name '*.cpp' | sort | tr '\n' ' ')
  [ -n "$srcs" ] || continue
  found=1
  printf '\n--- %s ---\n' "$name"
  # shellcheck disable=SC2086
  "$CXX" $FLAGS $srcs $CORE -o "$OUT/$name" || { fail=1; continue; }
  run_suite "$OUT/$name" "$name" || fail=1
done

[ "$found" = 1 ] || { echo "no suites matched '${filter}'"; exit 1; }
exit $fail
