#!/usr/bin/env sh
# Build and run the host unit tests for the dependency-free logic
# (AtLine, IsoTpFc) with whatever C/C++ compiler is available.
#
# Why this exists instead of `pio test -e native`: PlatformIO's native platform
# hard-codes gcc/g++, and overriding that from an extra_script does not take
# effect for `pio test`. If you have gcc/g++ on PATH, `pio test -e native`
# works and this script is unnecessary. With only clang installed, use this.
#
# Usage:  sh scripts/run-host-tests.sh
#         CXX=clang++ CC=clang sh scripts/run-host-tests.sh
set -e
cd "$(dirname "$0")/.."

CC=${CC:-$(command -v clang >/dev/null 2>&1 && echo clang || echo gcc)}
CXX=${CXX:-$(command -v clang++ >/dev/null 2>&1 && echo clang++ || echo g++)}
command -v "$CC"  >/dev/null 2>&1 || { echo "no C compiler ($CC)";   exit 1; }
command -v "$CXX" >/dev/null 2>&1 || { echo "no C++ compiler ($CXX)"; exit 1; }

UNITY_SRC=$(find .pio/libdeps -name unity.c -path '*src*' 2>/dev/null | head -1)
if [ -z "$UNITY_SRC" ]; then
  echo "Unity not found. Fetch it once with:  pio pkg install -e native"
  exit 1
fi
UNITY_DIR=$(dirname "$UNITY_SRC")

OUT=.pio/hosttest
mkdir -p "$OUT"
FLAGS="-std=gnu++14 -Wall -Wextra -DUNIT_TEST -I$UNITY_DIR -Ilib/Core -Ilib/AtServer -Ilib/IsoTpFc"

echo "CC=$CC CXX=$CXX"
"$CC" -c "$UNITY_SRC" -I"$UNITY_DIR" -o "$OUT/unity.o"

fail=0
run_suite() {  # name, test source, sources under test...
  name=$1; shift
  echo "--- $name ---"
  # shellcheck disable=SC2086
  "$CXX" $FLAGS "$@" "$OUT/unity.o" -o "$OUT/$name"
  "$OUT/$name" || fail=1
}

run_suite t_at_line  test/test_at_line/test_at_line.cpp   lib/AtServer/AtLine.cpp
run_suite t_isotp_fc test/test_isotp_fc/test_isotp_fc.cpp lib/IsoTpFc/IsoTpFc.cpp

exit $fail
