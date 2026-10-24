cmake_minimum_required(VERSION 3.20.0)

# One app, two zones. Pick the zone at build time:
#   west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=2   (N2 front / steering)
#   west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=3   (N3 rear / drivetrain)
set(NODE_ID 2 CACHE STRING "Zone node number (2 = steering, 3 = drivetrain)")

if(NOT NODE_ID STREQUAL "2" AND NOT NODE_ID STREQUAL "3")
  message(FATAL_ERROR "NODE_ID must be 2 or 3 (got ${NODE_ID})")
endif()

# boards/nucleo_f401re.overlay (shared pins) is picked up automatically;
# the zone-specific pins come from node2.overlay or node3.overlay.
# This must be set before find_package(Zephyr).
set(EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/node${NODE_ID}.overlay)

# Part 3 measurement build: -DWCET=1 adds per-task execution-time
# measurement (src/wcet.c) and the thread runtime statistics it needs.
set(WCET 0 CACHE STRING "1 = build with execution-time measurement")
if(WCET)
  list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/wcet.conf)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(car_can_app)

target_compile_definitions(app PRIVATE NODE_ID=${NODE_ID} WCET=${WCET})
if(WCET)
  target_sources(app PRIVATE src/wcet.c)
endif()

target_sources(app PRIVATE
  src/main.c
  src/canbus.c
  src/state.c
  src/monitor.c
  src/button.c
  src/blink.c
  src/relay.c
  src/node${NODE_ID}.c
)
