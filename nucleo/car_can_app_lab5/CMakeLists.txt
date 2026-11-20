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

# Lab 5 Raft config: raft_config.json is the one config shared with the Pi.
# Overrides for the checkoff, e.g.  -DLEADER=random -DSEED=3
set(LEADER "" CACHE STRING "Override default_leader: cockpit | steering | drivetrain | random")
set(SEED "" CACHE STRING "Override election_seed")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(car_can_app)

set(RAFT_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/raft_gen)
file(MAKE_DIRECTORY ${RAFT_GEN_DIR})
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_raft_config.py
          ${CMAKE_CURRENT_SOURCE_DIR}/raft_config.json ${RAFT_GEN_DIR}/raft_config.h
          "${LEADER}" "${SEED}"
  RESULT_VARIABLE RAFT_GEN_RC)
if(NOT RAFT_GEN_RC EQUAL 0)
  message(FATAL_ERROR "raft_config.json: generation failed")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/raft_config.json)
target_include_directories(app PRIVATE ${RAFT_GEN_DIR} src)

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
  src/raft_node.c
  src/pps.c
  src/raft/raft_core.c
  src/raft/raft_proto.c
  src/raft/route.c
  src/raft/tsync.c
)
