# Lab 2 requirements check (firmware vs. the Lab 1 requirements that apply)

Checked against `car_demo_app` on branch `zephyr_app` (Zephyr v4.4.2, Nucleo-F401RE) and the Lab 1 document
(`Lab 1_Aniket_Updated_Changes_4-9.docx`). The firmware is the reference: where a Lab 1 value differed from the
code, the Lab 1 document was updated to match (see `Lab1_Team2_updated_with_Lab2_changes.docx`, tracked changes,
Section 13). "Not yet measured" means the code supports the requirement but no scope capture has been taken yet.

| Lab 1 requirement | Applies in Lab 2 | Firmware status | Evidence / notes |
|---|---|---|---|
| R1 priority-based preemptive RTOS | Yes | **Met** | Zephyr. Threads at priorities 1, 3, 5, 6, 7 (all non-negative, so preemptible); time slicing off. Table in `config.h`. |
| R2.1 throttle within 2 ms (CMD_RX to PWM_SET) | Yes | **Implemented, not yet measured** | UART RX ISR parses the frame, toggles CMD_RX, gives a semaphore; the priority-1 control thread wakes, runs the fault check and PI step, and writes the PWM (toggling PWM_SET right after the timer write). |
| R2.2 brake within 2 ms (CMD_RX to DIR_A) | Yes | **Implemented, not yet measured** | Brake is checked before throttle and wins. DIR_A (IN1) goes low first. Braking is IN1=IN2=0 with EN held at 100% (no modulation), not literally PWM turned off. |
| R2.3 steering within 50 ms (no force feedback) | Yes | **Implemented, not yet measured** | Servo written on every command, after the drive outputs. The servo PWM period is 20 ms, so the worst case is about one period plus processing. |
| R2.4 blinkers: 100 ms start, 1 Hz +/-10%, 50% duty | Yes | **Implemented, not yet measured** | One 250 ms `k_timer` ISR: turn = 500 ms on/off (1 Hz), hazard = 250 ms (2 Hz). Start is applied by the 20 ms blinker thread (about 40 ms worst case). All four outputs are set in one function, so front/rear skew is microseconds. |
| R2.5 test button: local fail within 10 ms, debounced | Yes | **Met** (scope timing not yet measured) | Button level arrives in each command frame; 50 ms debounce; local fail happens on the same CMD_RX wake; 800 ms double-press window restores. |
| R3 error state (self-test, Pi link loss, out-of-range command; power-up in error) | Partial | **Met** (with notes) | Power-up starts in error (POWERUP fault). Link loss at 60 ms (3 x 20 ms). Wrong type, bad CRC or reserved flag bits are rejected and hold error for 500 ms. Error = motor braked, hazards 2 Hz. In any fault the servo is centered (R3.6 steering tracking applies once the steering zone is separate). |
| R4.1-R4.4, R4.6, R4.7 CAN | No | n/a | The Pi-to-STM32 link is the stand-in. Its format is in `link_proto.h` and summarized in the Lab 1 document (Section 13.2). |
| R4.5 heartbeat | Substituted | **Met** | 13-byte status frame every 20 ms from a periodic `k_timer` (no drift); three missed commands = fail safe at 60 ms. The status frame carries the three current readings. |
| R4.8 fault-injection jumpers | No | n/a | Unplugging the serial cable is the fault injection. |
| S3 force feedback | No | n/a | Not implemented. |
| S6 CAN fault tolerance | No | n/a | Nothing to be tolerant of yet. |

## Lab 1 values changed to match the firmware

| Item | Lab 1 document (before) | Firmware / now in the document |
|---|---|---|
| DD-06 / DD-07 / DD-08 turn threshold | -45 / +45 deg, return at -40 / +40, 5 deg hysteresis | arms at raw +/-2048 (25% of full lock), returns at raw +/-1639; about 28 / 22 deg and 5.6 deg hysteresis for a 900 deg lock-to-lock wheel |
| DD-19 brake threshold | active at 128 or above | above 25 |
| DD-12 / DD-13 throttle mapping | `1.50 * throttle / 255` m/s, max 1.50 m/s | `200 * (throttle - 5) / 250` wheel rpm, max 200 rpm, 5-count deadband |
| DD-14 control period | 1 ms | 20 ms PI period plus an immediate update on every new command |
| DD-15 max PWM | 80% normal, 90% stuck | 95% (`MAX_DUTY`); stuck-wheel attempt not implemented |
| R3.8 double-press window | 500 ms | 800 ms (`DOUBLE_PRESS_MS`) |
| DD-04 steering mapping | `clamp(30*raw/32767, -30, +30)` | full lock at raw +/-8192, mapped to servo 45-135 deg (1000-2000 us) |
| DD-16 stuck wheel, DD-17 / DD-18 current faults | required | not implemented; bench data and proposed values added in Section 13.4 |
| Link-loss time | three missed CAN heartbeats | 60 ms (3 x 20 ms) |

The three sequence diagrams for S3 to S5 in the Lab 1 document are pictures and still show the old numbers (45/40 deg turn
thresholds, 500 ms double press); they need to be redrawn by whoever owns those diagrams.
