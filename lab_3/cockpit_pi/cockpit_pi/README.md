# N1 cockpit (Raspberry Pi) - Lab 3, Team 2

## Run
```
sudo sh setup_can.sh                 # every boot: names the interfaces canA / canB and brings them up
sudo python3 cockpit.py --rt         # SCHED_FIFO 50, pinned to CPU 3
```
Start the proxy on the laptop with `--interval-ms 20`.

Bus A is `canA` (MCP2515 on CE0 = spi0.0) and bus B is `canB` (CE1 = spi0.1). The kernel's
`can0` / `can1` numbers follow probe order and can swap between boots, so `setup_can.sh`
finds each interface by its SPI device and renames it. `cockpit.py` stops with a clear
message if `canA` or `canB` does not exist (usually a loose wire on that module: check
`dmesg | grep mcp251`).

Console keys (type, then Enter):

| Key | Action |
|---|---|
| `b` | next CTRL carries steer = -32768 (out of range, valid CRC) |
| `c` | next CTRL carries a wrong CRC |
| `t` | simulate a press of the wheel test button |
| `s` | print state, counters and UDP_RX to CAN-write latency (p99, max) |
| `q` | quit (also writes `latency.csv`) |

Options: `--rt`, `--raw`, `--no-gpio`, `--duration N`, `--latency-csv FILE`,
`--tp-mode pulse|toggle` (default `pulse`), `--can-interface virtual` (desk test, no hardware).

## Check before the demo
1. Run `sudo python3 cockpit.py --raw` and move each pedal. Copy the released and
   pressed raw values into `THR_RAW_*` / `BRK_RAW_*` at the top of `cockpit.py`.
   The defaults (released +32767, pressed -32767) are a guess based on
   "pedals read inverted".
2. Button indices at the top of the file: left blinker = left gear shift (5),
   right blinker = right gear shift (4), cockpit test = Y (3). Change them if
   your values table says otherwise.

## Test points (BCM GPIO, header pin)

| Signal | GPIO | Pin |
|---|---|---|
| UDP_RX | 17 | 11 |
| CMD_TX | 27 | 13 |
| CAN_TX_A | 22 | 15 |
| CAN_TX_B | 23 | 16 |
| DEG_A LED | 5 | 29 |
| DEG_B LED | 6 | 31 |

These avoid SPI0 (GPIO 7-11) and the MCP2515 interrupt pins (GPIO 24, 25).

`UDP_RX` goes high right after the socket read returns and `CMD_TX` goes high right after
the first CAN write returns; both go low when the handler finishes (`--tp-mode pulse`).
Every packet therefore gives a rising edge on both pins, which is what a scope Delay
measurement (ch1 rising to ch2 rising, statistics on) needs. `--tp-mode toggle` keeps the
original behaviour (flip on each event), but then the edges alternate rising and falling
and a rising-to-rising Delay statistic breaks. `CAN_TX_A/B` still toggle before each write.

## Measuring the Pi stage (Part 3.3)
Scope UDP_RX against CMD_TX for at least a minute, with and without
`stress-ng --cpu 4 --io 2 --vm 1 --timeout 120s` running. Report the maximum and
the 99th percentile. `latency.csv` (same interval, software timestamps) is a
cross-check, not a replacement for the scope numbers. Note that the software figure runs
from the read to the end of BOTH CAN writes, while the scope stops at the first one, so the
software maximum is expected to be at or above the scope maximum.

State in the report that the toggles are Python (`lgpio`) and whether `--rt` was used.

## Changelog (review of the first version)
- **Interface names.** The first version used `can0` = bus A and `can1` = bus B. The kernel
  numbers them in probe order, and one boot gave `can0` to the CE1 module, which would swap
  A and B (wrong test points, DEG LEDs and checkoff jumpers). Now canA / canB by SPI device.
- **Lock held during socket writes.** The state lock was held while writing frames to the
  buses, while the 20 ms timer tick (heartbeat + blink + both bus writes) held the same
  lock, so a UDP packet arriving during a tick waited for it: extra Pi-stage latency in the
  tail. Now the lock covers state only; frames are queued and written after it is released.
- **Send ordering.** With the lock no longer covering writes, a small `tx_lock` assigns the
  sequence number and does both socket writes together, so the CTRL from the UDP thread and
  the keep-alive CTRL from the timer thread cannot reach the wire out of order (the STM32s
  would drop a frame that arrives a few counts "behind" as stale).
- **Timer bursts.** After a stall the 20 ms timer ran back-to-back ticks to catch up,
  producing burst heartbeats (outside the +/-10% period). It now skips and resynchronises
  (`timer_skips` in the stats).
- **Printing in the real-time paths.** `print()` ran under the state lock and could block on
  a slow SSH terminal. All messages now go through a logger thread.
- **Receive threads.** An unexpected exception in a receive thread silently ended it, so
  the bus then looked dead. Handlers are wrapped; the kernel filters only pass HB_N2, HB_N3,
  FAULT and FORCE (the Pi's own relayed frames no longer keep a dead bus looking alive, and
  Python wakes up for far fewer frames).
- **Bad packet.** A malformed UDP packet is logged and ignored instead of stopping the loop.
- **GC pauses.** The garbage collector is frozen/raised before the run so it cannot pause
  the 20 ms loop at random.
- **Test-point polarity.** Pulses by default (see above).
- `setup_can.sh` exits non-zero when a module is missing instead of stopping half-way.

## Not changed (check against your values table)
Wheel calibration constants, button indices, and steering full scale (the thresholds use the
STM32 value 8192 against a raw +/-32767 wheel range).

## Stream-lost timeout
`PROXY_TIMEOUT_S` is now 200 ms (was 100 ms). Measured with `--udp-diag` (kernel arrival
times, 300 s): the wheel stream arrives every 62.6 ms on average (the proxy asks for 50 ms,
but the Windows timer rounds to 4 x 15.6 ms), with occasional gaps of 100-170 ms that start
after about two minutes. At 100 ms each of those gaps made the cockpit flip between
`ERROR reasons=0x01` and `NORMAL`. The cockpit itself handles a packet within about 1 ms of
its arrival, so the gaps come from the sending side or the network.
Tradeoff: when the wheel stream really stops, CTRL keeps flowing 100 ms longer before the
zones' own 65 ms CTRL timeout starts, so the car brakes about 100 ms later than with the old
value. Check this against your values table / the checkoff limit.

To re-measure the stream: `python3 cockpit.py --udp-diag --duration 300`.
