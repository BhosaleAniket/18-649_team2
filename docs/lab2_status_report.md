# Lab 2 status: current sensors and dev environment (Team 2)

## Summary
- The repo already has an RTOS integration: `car_demo_app` runs on Zephyr with priority-based threads. Part 4 is mostly finishing and verifying it, not starting over.
- The one big missing piece is the **current sensors**. All three ACS712-05B modules are wired and reading. Left and right respond to stalls. Right lost a connection, and the servo sensor isn't verified.
- The dev environment is now set up on Avi's PC, and flashing works with `west flash`.

## 1. What's in the repo (`zephyr_app` branch)
- Bring-up apps: `motor_controller_app`, `servo_app`, `led_blinker_app`, `blinker_steering_app`, `command_parser_app`, `current_sensor_app`.
- `car_demo_app` merges them for the Nucleo-F401RE. The task table from `config.h`:

| Task | Kind | Period / trigger | Priority | Deadline |
|---|---|---|---|---|
| UART RX parser | ISR | each byte | IRQ | one byte time |
| control | thread | new command, or 20 ms | 1 | 2 ms brake/throttle, 50 ms steering |
| status TX | thread | 20 ms (timer-synced) | 3 | 20 ms +/-10% |
| blinker logic | thread | 20 ms | 5 | 100 ms |
| blink output | timer ISR | 250 ms | IRQ | pattern edge |
| monitor (printk) | thread | 500 ms | 7 | best effort |

- Shared state uses a spinlock (command struct) and `atomic_t` (published values).

**Gaps against the spec:**
1. The current sensors are **not** in `car_demo_app`. `link.c` sends `i_left = i_right = i_servo = 0; /* TODO */` in the status frame.
2. Only two test points exist (CMD_RX on PB10 and PWM_SET on PA12). The others need routing to the test board.
3. `FAIL_SAFE_MS` is 60 ms. The handout says 3 missed commands (150 ms), with a 100 ms checkoff.
4. The self-test double-press window is 800 ms, and the spec says 500 ms (R3.8).
5. The 2 ms brake budget has no separate high-priority path.
6. The task table doesn't yet distinguish thread, work item, timer callback and ISR the way the Part 4 questions ask.

## 2. Dev environment
Avi's machine now has: Zephyr v4.4.2, Zephyr SDK 1.0.1 (ARM toolchain), a Python 3.12 virtual environment with `west`, and the xPack OpenOCD. Everything is in `C:\Users\vivek\zephyrproject`, outside the repo.
- **Python:** Zephyr v4.4.2 needs Python 3.12 or newer. Python 3.10 fails at configure time.
- **SDK host tools:** they aren't available for Windows yet, so OpenOCD came from winget instead.
- **ST-LINK driver:** the debug interface needed a WinUSB driver. We installed it with Zadig, on the "ST-Link Debug" interface only.
- **Alternative flash method:** copying `zephyr.bin` to the Nucleo's `NOD_F401RE` drive also works with no driver.

Build and flash (PowerShell):
```powershell
cd C:\Users\vivek\zephyrproject
$env:Path += ";C:\Program Files\7-Zip"; $env:ZEPHYR_BASE="C:\Users\vivek\zephyrproject\zephyr"
.\.venv\Scripts\west.exe build -b nucleo_f401re <path-to-app> -d build_current
.\.venv\Scripts\west.exe flash -d build_current --runner openocd
```
Serial console (COM3, 115200): `python -m serial.tools.miniterm COM3 115200`.

**Suggestion:** a shared `west.yml` in the repo would pin the Zephyr version so everyone builds against the same release.

## 3. Current sensor hardware (ACS712ELC-05B, 185 mV/A)
| Sensor | STM32 pin (Arduino) | Placed in series with |
|---|---|---|
| Left motor | PA0 (A0) | one left-motor wire from L298N OUT1/OUT2 |
| Right motor | PA4 (A2) | one right-motor wire from OUT3/OUT4 |
| Servo | PB0 (A3) | the servo's +5 V supply lead |

- Each OUT goes through a **47k / 47k divider** (ratio 1:2) with a **100 nF** cap from the divider node to GND. That's because the ACS712 outputs 0 to 5 V and the STM32 pins are 3.3 V.
- One ADC step is about 8.7 mA. The code uses `divider = 2.0f`.
- VCC comes from the buck converter at 5 V, and the servo runs from the same buck rail. The Nucleo stays on USB power, and all grounds are common.
- Servo signal is on **PB8** (Arduino D15, TIM10 at 50 Hz).
- Encoders are powered from the Nucleo's 3.3 V.

## 4. Measurements
- **Idle baseline:**
  - With the 12 V supply off, the raw noise is sd ~ 60-70 mA per sample, and average readings sit near 0 mA.
  - With 12 V on and the motors idle, the noise is sd ~ 250-900 mA per sample (spikes to +/-10 A peak-to-peak), but a 250 ms window average is still within about +/-50 mA of zero.
- **Stall response** (50% duty, both wheels held), first cycle:
  - Left changed by about -312 mA and right by about -220 mA compared with REST.
  - Both went negative, so their IP+ and IP- wires are effectively reversed. That's harmless because thresholds use magnitude, but swapping the wires makes it positive.
  - FREE (unloaded) changed by less than 100 mA, which is within noise.
- **Servo sensor:** no measurable response in FREE (servo sweeping) or STALL (servo held).

## 5. Problems found and lessons
- **Zero is captured once at boot.** If the 5 V rail isn't up when the Nucleo boots, the zero is wrong and every reading is offset by ~13-14 A (this happened three times). **Power the buck and 12 V first, then reset the Nucleo, or press `z` at REST.** The boot line `zero (sensor mV)` should read about 2500 for all three.
- **Right sensor dropped out** partway through a run: it jumped to -13,000 mA and later stayed at -14,000 mA, so its output collapsed to nearly 0 V. It flickered before settling, which looks like a loose connection. Not yet re-seated or verified.
- **Servo sensor (S)** showed a 170 mV zero once, and now shows no response to servo current. Still unresolved. The topology is buck -> IP+ -> IP- -> 5 V rail -> servo and all module VCCs, so the sensor sees the rail's total current, and servo motion can also add noise on everyone's supply.
- **Nucleo failed to enumerate** with everything connected (red LD1 blinking). It worked after stripping all wires and reconnecting in groups. The cause wasn't identified.
- **Noise:** 12 V being on adds much more noise than the sensor itself. A windowed average (about 100-200 ms) is needed for the 50 mA open-circuit threshold (DD-17). The 2 A overload threshold (DD-18) is unaffected.

## 6. Code changes
- `current_sensor_app/src/main.c`: `cal[]` now uses `divider = 2.0f` (was placeholder 1.0). This change is in the working tree and **not committed**.
- Nothing else in the repo was modified.

## 7. Open items and next steps
1. Re-seat the right sensor's wires and check its OUT reads about 2.5 V.
2. Find out why the servo sensor doesn't respond (test that servo current really flows through sensor 3, and re-zero it).
3. Rerun FREE and STALL with both wheels and the steering held. Optionally raise the test duty from 50% to 80% (within DD-15) for cleaner numbers.
4. Record three numbers per sensor (rest, free, stalled) for the fault thresholds.
5. Add extra decoupling (10 uF plus 100 nF at each module's VCC) and consider taking sensor VCC straight from the buck output to cut noise.
6. Merge ADC sampling into `car_demo_app` and fill in the current fields in the status frame.
7. Fix `FAIL_SAFE_MS` and the double-press window, add the missing test points, and update the task table for the writeup.
8. Deliverables still to do: block diagram, schematic (including power rails and ground), updated values table, task table, and the one-page CAN integration writeup.
