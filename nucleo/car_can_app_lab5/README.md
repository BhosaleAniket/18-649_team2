# car_can_app - Lab 5 zone firmware (N2 steering, N3 drivetrain)

Lab 4 firmware plus Raft, time sync and PPS, per the Raft message catalog v0.8.

```
west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=2                          # N2, LEADER=cockpit
west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=3                          # N3
west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=2 -DLEADER=random -DSEED=3 # checkoff 8
```
Built against Zephyr 4.4.99 (commit 15723416) with SDK 1.0.1: no warnings.
RAM 96.2 KB (N2) / 96.8 KB (N3) of 96 KB with log_size = 30000: about 2 KB free.
If you add code and run out of RAM, lower `log_size` in raft_config.json (2 bytes per entry).

## Configuration
`raft_config.json` is the one config shared with the Pi; the build turns it into
`raft_config.h`. `-DLEADER=` and `-DSEED=` override `default_leader` and `election_seed`.
Measure and set `rx_latency_us` and `relay_delay_us` on the bench.

## New pins (boards/nucleo_f401re.overlay, morpho CN7, both zones)
| Signal | Pin | Meaning |
|---|---|---|
| PPS | PC10 (CN7-1) | rising edge at every whole second of global time |
| LEADER | PC11 (CN7-2) | this node is the Raft leader |
| SYNC | PD2 (CN7-4) | time reference current: leader, or a REF under 100 ms old |
| REF_RX | PC2 (CN7-35) | toggles on every REF from the leader (direct or relayed) |

## Files
- `src/raft/` - shared with the Pi word for word: `raft_proto` (catalog), `raft_core`
  (elections with pre-vote, replication, catch-up, the network graph), `route`
  (the relay rule), `tsync` (global time, drift). No Zephyr calls.
- `src/raft_node.c` - RAFT thread: feeds frames and a 1 ms tick to the core, sends REF,
  drives LEADER / SYNC / REF_RX, and a RAFT_LOG thread that prints applied entries.
- `src/canbus.c` - RX timestamps, Raft filters and dedup, route-based relay,
  REF TX-complete stamps.
- `src/blink.c` - lamp edges on global time: 2 Hz, 4 Hz with the cockpit lost.
- `src/pps.c` - PPS pin and the per-second console line.
- `tests/sim.c` - three nodes on two simulated buses running the shared code through the
  checkoff scenarios: `gcc -O2 -Isrc/raft -o sim tests/sim.c src/raft/*.c -lm && ./sim`

## Console
Every applied entry: `RAFT idx 7 term 2 MEMBER LINK drivetrain A DOWN (replayed)`.
Every PPS edge: `PPS global=... local=... drift=... ppb corr=... us`.
Every second: the Lab 4 report plus `raft ...`, `getNetwork ...` and `time ...` lines.
