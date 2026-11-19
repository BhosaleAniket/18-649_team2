# N1 cockpit (Raspberry Pi) - Lab 5, Team 2

The Lab 3 cockpit on top of `pi_node` (`../pi_node`). Wheel, CTRL, blinker and error-state logic
are unchanged from `lab_3/cockpit_pi/cockpit_pi/cockpit.py`; everything on the CAN side moved into C.

| Lab 3 `cockpit.py` did | Lab 5: now done by |
|---|---|
| open canA / canB (python-can), kernel filters | `pi_node` (C) |
| HB_N1 every 20 ms | `pi_node`, from the state `cockpit.py` pushes every 20 ms |
| per-bus DEG detection, TX-queue flush | `pi_node` (DEG LEDs still driven here, from `pi_node` status) |
| de-dup of received frames | `pi_node` (Lab 3 and Raft rules) |
| -- | Raft, REF, time sync, relaying, LINK/NODE/COCKPIT entries: `pi_node` |

`cockpit.py` still sends CTRL, BLINK and FAULT (with their seq/CRC) through `pi_node_send_app()`,
and reads HB_N2 / HB_N3 / FAULT / FORCE from `pi_node_poll_app()`.
`pi_node.py` is the ctypes wrapper.

## Run
```
sudo sh ../../lab_3/cockpit_pi/cockpit_pi/setup_can.sh   # every boot
make -C ../pi_node                                       # once: builds libpi_node.so
sudo python3 cockpit.py --rt
```
Start the wheel proxy on the laptop as in Lab 3. Election mode (checkoff 8):
`sudo LEADER=random ELECTION_SEED=3 python3 cockpit.py --rt`.

Keys (then Enter): `s` stats, `n` getNetwork, `e N` report APP_EVENT code N, `t` test button,
`b` bad steer, `c` bad CRC, `q` quit. Options as in Lab 3, plus `--lib` and `--config`.

Do **not** run the Lab 3 `cockpit.py` at the same time: both would own the buses and send HB_N1.

## Tested on the bench (Nov 16, all three nodes, no root)
- No wheel: error state CTRL_LOST only (0x01) once both zones are heard; getNetwork matches the
  Nucleos. With drivetrain already leading (term 2), the cockpit rejoined as a follower, applied the
  held entries marked replayed, and its clock synced (drift estimate -23 to -67 ppm).
- Fake wheel stream (brake full, throttle 0, steer 0): NORMAL, COCKPIT_OK committed by the leader,
  303 CTRL frames, UDP_RX -> CAN write p99 421 us; stream stopped -> CTRL_LOST, COCKPIT_ERROR committed.
- Not yet tested: the real wheel and proxy, `--rt` as root, the test button and failover checkoffs.
