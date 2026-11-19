# pi_node - Lab 5 Pi runtime (Team 2)

The Pi's half of the Lab 5 Raft message catalog **v0.8**. `pi_node` is a C library that owns `canA`
and `canB` and runs Aniket's shared Raft core and time sync; `cockpit.py` will call it through
`pi_node.h` (ctypes).

Status: builds on the Pi with no warnings; `make host-test` and `make sim` pass there (sim includes
scenario 4b). On the real bus the Pi leads in term 1 and sends REF (20 ms) and APPEND on both buses.
**Not yet run against Nucleos with the Lab 5 firmware** (the ones on the bench ran Lab 3/4 firmware:
heartbeats only, no ACKs, so nothing commits).

## Layout
| Path | What |
|---|---|
| `raft_config.json` | The one shared config (same keys as `car_can_app/raft_config.json`, plus a Pi-only `pi` section, which the Nucleo generator ignores). |
| `shared/` | Aniket's pure modules from `car_can_app/src/raft/`, **word for word**: `raft_proto`, `raft_core`, `route`, `tsync`. Re-copy them when he changes them; do not edit here. |
| `include/catalog.h` | Lab 3 IDs and `cat_source()` for every catalog frame. |
| `include/pi_node.h` | The API `cockpit.py` calls. |
| `src/pi_node.c` | The runtime: sockets, threads, dedup, detection, `HB_N1`, relay, glue to the core. |
| `src/config.c` | JSON reader for `raft_config.json`. |
| `src/gpio.c` | Direct GPIO register writes (Pi 4, `/dev/gpiomem`) for scope test points and PPS. |
| `tools/pi_node_run.c` | Runs `pi_node` alone (no wheel) for bench tests. |
| `tools/ref_stamp_test.c` | Scope test for the `RECV_OWN_MSGS` stamp. |
| `tests/host_test.c` | Pi-side checks: catalog + config (`make host-test`, any PC). |
| `tests/sim.c` | Aniket's three-node simulator (`make sim`). |

## Build and run (on the Pi)
```
sudo sh ../../lab_3/cockpit_pi/cockpit_pi/setup_can.sh    # every boot: canA / canB at 500 kbit/s
make
make host-test sim
sudo ./pi_node_run raft_config.json --ok                   # status every second, RAFT lines as entries commit
```
Root is needed for SCHED_FIFO and for the `ip link` TX-queue flush when a bus goes degraded. Without
root it still runs, with warnings, at normal priority.

`pi_node_run` keys (then Enter): `o` toggle cockpit OK/ERROR, `t` toggle the test-button fault
(silent in Raft, `HB_N1` stops), `e 3` report APP_EVENT code 3, `q` quit.

Election mode for checkoff 8: `sudo LEADER=random ELECTION_SEED=3 ./pi_node_run` (seed 3 makes the
Pi win in Aniket's simulator).

## How the Pi drives the core (same as `raft_node.c` on the Nucleos)
| Thread | Priority | Does |
|---|---|---|
| rx canA, rx canB | `pi.rt_priority` (60) | kernel receive stamp; own-REF TX confirm -> `tsync_on_tx_done`; dedup; per-bus and per-zone liveness; relay via `route_relay_mask()`; Raft frames and test-button FAULTs to the tick thread, Lab 3 frames to `pi_node_poll_app()` |
| tick | 59 | every 1 ms: `raft_on_frame` / `raft_on_tick`, `tsync_on_ref` for REFs from the leader, `tsync_on_tick` (REF). Every 20 ms: DEG (35 ms) and zone (66 ms) detection, `HB_N1`, then `raft_set_own_links`, `raft_set_silent`, `raft_observe_peers`, `raft_observe_cockpit` |
| log | normal | prints core debug lines, `RAFT idx ...` lines, and our own messages |

The CAN layer fills the seq nibble of every Raft frame (one counter per base, as `canbus_send_raft`).
One deliberate difference from the Nucleos: **every** copy of a REF (both buses) goes to tsync, and
only the first to the core (as `tsync.h` asks).

## PPS and global_gettime() (Part 6)
A C thread (same priority as the RX threads) drives `pi.pps_gpio` (BCM 16, header pin 36) high at every
whole second of global time for 100 ms, through `/dev/gpiomem` (the user must be in the `gpio` group).
It sleeps until 300 us before the edge, re-reads the clock model and spins to the exact microsecond;
if a REF moved the edge meanwhile (a step either way) it recomputes instead of spinning. Each edge
prints the report line:
```
PPS global=3467.000000 s local=3467.022580 s late=0 us drift=-36379 ppb corr=-6 us (follower)
```
`pi_node_global_us()` is `global_gettime()` (Python: `PiNode.global_us()`, cockpit key `g`).

## Pi-only time-sync settings (`pi` section of raft_config.json)
| Key | Value | Why |
|---|---|---|
| `rx_skew_b_us` | 185 | The Pi's two MCP2515s report arrivals with different delays. The stamp bus alternates, so without this the Pi's clock was pulled +-180 us every REF. Swept on the bench (drivetrain leading): skew 0 -> corrections median 177 us, 84 -> 98, 170 -> 7, **185 -> 6**, 200 -> 20. |
| `rx_latency_us` | 0 (measure) | The Pi's absolute receive delay (overrides the shared value, which is the Nucleos'). Scope the Pi's PPS against a Nucleo's and put the difference here. |
| `tsync_trace` | false | `true` (or `PI_TSYNC_TRACE=1`) prints every correction: data for the drift section of the report. |
| `ref_stamp_adjust_us` | 0 (measure) | Correction of the Pi's own REF TX stamp when it leads (`ref_stamp_test`). |

## Measure the REF stamp first (catalog open question)
Stop `pi_node` and `cockpit.py`, keep one Nucleo powered on bus A to ACK, then:
```
sudo ./ref_stamp_test -i canA -g 26 -n 500 > stamp.csv
```
Scope CH1 = GPIO 26 (header pin 37), CH2 = CAN_H of bus A; measure CH1 rising -> end of frame with
statistics on, and compare with the program's "kernel stamp" line.
- Means agree, small spread: leave `pi.ref_stamp_adjust_us` = 0.
- Constant offset: put (stamp mean - scope mean) in `pi.ref_stamp_adjust_us`.
- Wide spread (tens of us or more): the stamp is not usable; fall back to queue time + a constant.

## Not done yet
- A run with the Nucleos (they need Aniket's firmware and power).
