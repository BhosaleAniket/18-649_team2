# Lab 2 status: current sensors and dev environment (Team 2)

## Summary
- The repo already has an RTOS integration: `car_demo_app` runs on Zephyr with priority-based threads. Part 4 is mostly finishing and verifying it, not starting over.
- The **current sensors are now wired and validated** on the bench with `current_sensor_app`: all three ACS712-05B modules read about 0 at rest, respond to motor and servo load, and are quiet enough to use (window scatter about +/-15-30 mA).
- Still to do: merge ADC sampling into `car_demo_app` and fill in the current fields in the status frame.
- The dev environment is set up on Avi's PC, and flashing works with `west flash`.

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
Avi's machine has: Zephyr v4.4.2, Zephyr SDK 1.0.1 (ARM toolchain), a Python 3.12 virtual environment with `west`, and the xPack OpenOCD. Everything is in `C:\Users\vivek\zephyrproject`, outside the repo.
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
| Servo | PB0 (A3) | the servo's +5 V supply lead (buck -> sensor -> servo) |

- Each OUT goes through a **47k / 47k divider** (ratio 1:2) with a **100 nF** cap from the divider node to GND. That's because the ACS712 outputs 0 to 5 V and the STM32 pins are 3.3 V.
- One ADC step is about 8.7 mA. The code uses `divider = 2.0f`.
- **Supply (final arrangement):** the three modules' VCC comes from the **Nucleo's 5V pin** (a quiet supply; they draw about 30 mA total), with decoupling capacitors at each module. The **servo** runs from the buck converter through sensor 3's current path. All grounds are common. This separation cut the noise a lot (see section 4). Do not tie the Nucleo 5V pin to the buck rail.
- IP+ / IP- on all three modules were swapped so readings are positive with load.
- Servo signal is on **PB8** (Arduino D15, TIM10 at 50 Hz).
- Encoders are powered from the Nucleo's 3.3 V.

## 4. Measurements
All from `current_sensor_app`, at 80% test duty (DD-15 limit), windows of 250 ms.

| Channel | Rest | Free-run (wheels off ground; servo sweeping) | Stalled |
|---|---|---|---|
| Left motor | about 0 mA | about +130 mA | about 1.0 A |
| Right motor | about 0 mA | about +50 mA | about 0.45-0.5 A |
| Servo | about +12 mA | about +90 mA | no extra beyond moving (about +90 mA) |

- **Noise:**
  - Before changes (12 V on, sensors on the shared rail): sd about 300-700 mA per sample, window averages scattering +/-200-700 mA.
  - After powering the modules from the Nucleo 5V and adding decoupling: sd about 60-250 mA per sample, window scatter about +/-15-30 mA.
- **Servo sensor in series:** confirmed by the pull-wire test (pulling a wire from its IP terminals stops the servo).
- **Servo sensor baseline:** its output dropped by about 190 mV (about 1 A equivalent) between two runs and stayed there. That is an offset shift, not a stall. The boot zero hides it. If a channel reads a steady large value at rest, press `z` at REST, or check VCC and OUT with a meter.

## 5. Implications for the thresholds
- **DD-17 (open circuit, below 50 mA at 20% PWM or more):** right-motor free-run current at 80% is only about 50 mA, which sits right at the threshold, so it would false-trip. Lower it (about 20 mA), or back it with the encoders (commanded motion but no counts). Left and servo are well above it. Low duty (20%) still needs testing.
- **DD-18 (overload, above 2.0 A for 20 ms):** real stalls at 80% reached only about 1.0 A (left) and 0.5 A (right), so 2.0 A never trips. Revisit using the measured data.
- Use window averages (about 100-200 ms) for any threshold decision.

## 6. Problems found and lessons
- **Zero is captured once at boot.** If the 5 V rail isn't up when the Nucleo boots, the zero is wrong and every reading is offset by about 13-14 A (this happened several times). Power the rails first, then reset the Nucleo, or press `z` at REST. The boot line `zero (sensor mV)` should read about 2500 for all three.
- **Intermittent contact on the right module** caused `OVERLOAD` glitches (output dropping to near 0 V) until its wires were re-seated.
- **Hand-held stalls are unreliable.** Wedge or clamp the wheels and steering for repeatable stall numbers.
- **ADC-pin swap test is ambiguous:** swapping the divider wires at the Nucleo moves the sensor between pins, but each sensor stays with its own motor, so it cannot separate a sensor fault from a motor effect. A meter on OUT, or the pull-wire test, is more decisive.
- **Nucleo failed to enumerate** with everything connected (red LD1 blinking). It worked after stripping all wires and reconnecting in groups. The cause wasn't identified.

## 7. Code changes
- `current_sensor_app/src/main.c`: `cal[]` uses `divider = 2.0f` (47k/47k divider), and `FREE_DUTY` and `STALL_DUTY` were raised from 0.5 to 0.8.
- No other repo code has been modified yet.

## 8. Open items and next steps
1. Merge ADC sampling into `car_demo_app` and fill in the current fields in the status frame. Average over a window for each status frame.
2. Decide the thresholds (DD-17, DD-18) from the measured data, and test open-circuit detection at 20% duty.
3. Fix `FAIL_SAFE_MS` and the double-press window, add the missing test points, and update the task table for the writeup.
4. Deliverables still to do: block diagram, schematic (including power rails and ground), updated values table, task table, and the one-page CAN integration writeup.
