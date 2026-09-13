# core/ is deliberately plain C++ with no ESP-IDF dependency: it is unit-tested
# on a host machine by scripts/run-host-tests.sh. Globbing is normally poor
# CMake practice, but here it keeps the IDF build and the host runner from
# drifting apart as files are added.
file(GLOB CORE_SRCS ${CMAKE_SOURCE_DIR}/core/*.cpp)

# Glob src/ too, so a new IDF-side file (link implementations, UI, tasks) is
# picked up without editing this list.
file(GLOB APP_SRCS ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

# Generated LVGL fonts are C, not C++, and live in their own directory so a
# regeneration is an obvious diff rather than noise in src/.
file(GLOB FONT_SRCS ${CMAKE_CURRENT_LIST_DIR}/fonts/*.c)

idf_component_register(
    SRCS ${APP_SRCS} ${FONT_SRCS} ${CORE_SRCS}
    INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/core"
    REQUIRES driver esp_timer esp_lcd
)
