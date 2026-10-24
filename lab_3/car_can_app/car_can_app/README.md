# car_can_app - Lab 3 zone firmware (N2 steering, N3 drivetrain)

```
west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=2          # demo, N2
west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=3          # demo, N3
west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=3 -DWCET=1 # optional software timing
```
Built and checked against Zephyr v4.4.0 (no warnings, all four variants).

Matches the Oct 22 clarification:
- RELAY runs on both STM32s (src/relay.c). N2 relays in checkoff cases 1-2 and N3 in cases 3-4. RELAY_TX is PC0 on N2 and PB13 (morpho CN10-30) on N3, which is optional.
- Test button (blue B1): one FAULT, then this node's heartbeats stop. A double press within 500 ms recovers it.
- Probe points: CAN_RX_x toggles when dispatch gets the frame; RELAY_TX toggles after the bus-B send returns.

Pins are in boards/nucleo_f401re.overlay (shared), node2.overlay and node3.overlay.
Priorities, periods and thresholds are in src/config.h; catalog payloads in src/catalog.h.
