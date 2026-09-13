# PlatformIO + ESP-IDF entry point. `src/` is the main component; the pure
# logic in `core/` is compiled into it (see src/CMakeLists.txt) rather than
# being a separate component, so the same sources build unchanged in the host
# test runner.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(jeep-carputer)
