# pi_node - Lab 5 Pi runtime. Build on the Raspberry Pi:  make
#
# shared/ holds Aniket's pure Raft modules (catalog v0.8), copied word for word from
# car_can_app/src/raft/. When he changes them, copy them again: do not edit them here.

CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Wextra
CFLAGS  += -std=gnu11 -Iinclude -Ishared -Isrc -pthread

SHARED_SRC = shared/raft_core.c shared/raft_proto.c shared/route.c shared/tsync.c
LIB_SRC    = src/pi_node.c src/config.c src/gpio.c $(SHARED_SRC)
HEADERS    = $(wildcard include/*.h shared/*.h src/*.h)

all: libpi_node.so pi_node_run ref_stamp_test

libpi_node.so: $(LIB_SRC) $(HEADERS)
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $(LIB_SRC)

pi_node_run: tools/pi_node_run.c libpi_node.so
	$(CC) $(CFLAGS) -o $@ tools/pi_node_run.c -L. -lpi_node -Wl,-rpath,'$$ORIGIN'

ref_stamp_test: tools/ref_stamp_test.c src/gpio.c src/gpio.h
	$(CC) $(CFLAGS) -o $@ tools/ref_stamp_test.c src/gpio.c

# Pi-side checks (catalog + config); runs on any PC
host-test: tests/host_test.c src/config.c shared/raft_proto.c $(HEADERS)
	$(CC) $(CFLAGS) -o host_test tests/host_test.c src/config.c shared/raft_proto.c
	./host_test raft_config.json

# Aniket's three-node simulator over the shared modules (all ten checkoff scenarios)
sim: tests/sim.c $(SHARED_SRC)
	$(CC) -O2 -Wall -Wextra -Ishared -o sim tests/sim.c $(SHARED_SRC) -lm
	./sim

clean:
	rm -f libpi_node.so pi_node_run ref_stamp_test host_test sim

.PHONY: all clean host-test sim
