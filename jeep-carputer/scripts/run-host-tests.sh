#!/usr/bin/env sh
# Build and run every host test suite in test/.
#
# The whole of core/ is pure C++ with no ESP-IDF dependency, precisely so it can
# be tested on a laptop instead of by flashing a board and driving to a car.
# Each test/test_*/ directory is one suite, linked against all of core/.
#
# Usage:  sh scripts/run-host-tests.sh [suite-name-substring]
#         CXX=clang++ sh scripts/run-host-tests.sh
set -e
cd "$(dirname "$0")/.."

CXX=${CXX:-$(command -v clang++ >/dev/null 2>&1 && echo clang++ || echo g++)}
command -v "$CXX" >/dev/null 2>&1 || { echo "no C++ compiler ($CXX)"; exit 1; }

OUT=build/hosttest
mkdir -p "$OUT"
FLAGS="-std=gnu++14 -Wall -Wextra -Wshadow -DHOST_TEST -Icore -Itest"

CORE=$(find core -name '*.cpp' | sort | tr '\n' ' ')
[ -n "$CORE" ] || { echo "no core sources yet"; exit 1; }

echo "CXX=$CXX"
echo "core: $CORE"

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
  "$OUT/$name" || fail=1
done

[ "$found" = 1 ] || { echo "no suites matched '${filter}'"; exit 1; }
exit $fail
