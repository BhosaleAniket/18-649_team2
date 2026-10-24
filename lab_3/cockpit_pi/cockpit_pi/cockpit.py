#!/usr/bin/env python3
"""
cockpit.py - Lab 3 N1 cockpit zone (Raspberry Pi 4), Team 2

Tasks (template task set, N1):
  UDP_RX     main thread: blocks on the wheel stream (port 8000), raises UDP_RX
             right after the socket read returns,
             decodes the DIJOYSTATE2 packet, runs the blinker logic, then
  CTRL_TX    builds CTRL (0x010) and writes it to canA and canB (CAN_TX_A/B toggle
             before each write, CMD_TX right after the first write returns).
             Every UDP update -> one CTRL (R4.4). If 20 ms pass with no update,
             the last values are re-sent (as in Lab 2), so UDP jitter on the
             network never looks like a lost cockpit; after 200 ms of silence
             from the proxy (PROXY_TIMEOUT_S) CTRL stops and the zones fail safe.
  BLINK_TX   BLINK (0x200) on every blinker state change (from UDP_RX) and on
             every 500 ms toggle edge (timer thread). The cockpit is the master.
  HB_TX      HB_N1 (0x100) every 20 ms on both buses.
  HB_MON     every 20 ms: per-bus degraded detection (DEG LEDs), N2/N3
             heartbeat timeout (three missed on both buses = zone failed),
             error state.
  FORCE_RX   one receive thread per interface: HB_N2/HB_N3, FAULT, FORCE,
             de-duplicated by (ID, sender, seq).

Error state (R3): power-up in error; out when HB_N2 and HB_N3 are heard (either
bus) and the wheel stream is live; in on wheel test button (single press,
double press within 500 ms restores; heartbeats stop meanwhile), N2/N3 lost, or another zone reporting TEST / BADCMD.
In error: BLINK = hazard and CTRL carries full brake.

Bad input (Part 5): type 'b' + Enter -> next CTRL has steer = -32768 (out of
range, valid CRC); 'c' + Enter -> next CTRL has a wrong CRC.

Interface names: bus A is canA (MCP2515 on CE0 = spi0.0), bus B is canB (CE1 =
spi0.1). The kernel's can0/can1 numbering follows probe order and can swap
between boots, so setup_can.sh renames the interfaces by their SPI device.

Run (root needed for SCHED_FIFO, GPIO and the TX-queue flush):
  sudo python3 cockpit.py --rt
Other keys: 's' stats, 't' simulate cockpit test button press, 'q' quit.

Changes from the first version (see README "Changelog"): the big state lock is no
longer held while writing to the CAN sockets, sends are ordered by a small
dedicated lock, the 20 ms timer no longer bursts after a stall, console output
goes through a logger thread, the receive threads use kernel CAN filters and
survive exceptions, and the scope test points are clean pulses by default.
"""

import argparse
import csv
import gc
import os
import queue
import select
import socket
import struct
import subprocess
import sys
import threading
import time

import can  # python-can

# ----------------------------------------------------------------------------
# Catalog (mirrors car_can_app/src/catalog.h)
# ----------------------------------------------------------------------------
ID_CTRL, ID_FAULT = 0x010, 0x020
ID_HB_N1, ID_HB_N2, ID_HB_N3 = 0x100, 0x101, 0x102
ID_BLINK, ID_FORCE = 0x200, 0x300

ZONE_N1, ZONE_N2, ZONE_N3 = 1, 2, 3
FAULT_TEST_BUTTON, FAULT_BAD_CTRL, FAULT_ZONE_LOST = 1, 2, 3

ERR_CTRL_LOST = 1 << 0     # on the Pi: wheel stream (proxy) lost
ERR_ZONE_LOST = 1 << 1
ERR_TEST = 1 << 2
ERR_BADCMD = 1 << 3
ERR_REMOTE = 1 << 4
ERR_PROPAGATE = ERR_TEST | ERR_BADCMD

BLINK_OFF, BLINK_LEFT, BLINK_RIGHT, BLINK_HAZARD = 0, 1, 2, 3

# ----------------------------------------------------------------------------
# Team-defined values (CHECK against your Lab 2 values table)
# ----------------------------------------------------------------------------
UDP_PORT = 8000
PROXY_TIMEOUT_S = 0.200        # no UDP for this long -> stream lost. Measured stream (kernel arrival
                               # times, 300 s): mean interval 62.6 ms (Windows rounds the proxy's 50 ms
                               # timer to 4 x 15.6 ms), occasional gaps up to 170 ms. 100 ms tripped
                               # on every gap; 200 ms covers the largest gap seen.

# Raw axis values from Lab 2 Part 1 (Windows proxy). Pedals read inverted.
STEER_RAW_LIMIT = 32767
THR_RAW_RELEASED, THR_RAW_PRESSED = 32767, -32767    # CHECK: run with --raw
BRK_RAW_RELEASED, BRK_RAW_PRESSED = 32767, -32767    # CHECK: run with --raw

# Raw button indices (Lab 2 Part 1): A 0, B 1, X 2, Y 3, right gear 4,
# left gear 5, RSB 8, LSB 9.
BTN_LEFT_BLINKER = 5           # left gear shift
BTN_RIGHT_BLINKER = 4          # right gear shift
BTN_TEST = 3                   # Y

BUTTON_DEBOUNCE_S = 0.050
DOUBLE_PRESS_S = 0.500        # clarification: double press within 500 ms restores
STEER_FULL_SCALE = 8192        # same as the STM32 servo mapping
TURN_THRESHOLD = STEER_FULL_SCALE // 4
TURN_HYSTERESIS = STEER_FULL_SCALE // 20

HB_PERIOD_S = 0.020
DEG_TIMEOUT_S = 0.035
ZONE_TIMEOUT_S = 0.066
BLINK_HALF_S = 0.500           # 1 Hz, 50 %
DEDUP_WINDOW_S = 0.150         # same as the STM32s (see config.h)
STALE_WINDOW_S = 0.250
REMOTE_HOLD_S = 0.100

# Test points (BCM numbers; avoid SPI0 7-11 and the CAN INT pins 24/25)
GPIO_UDP_RX = 17     # pin 11
GPIO_CMD_TX = 27     # pin 13
GPIO_CAN_TX_A = 22   # pin 15
GPIO_CAN_TX_B = 23   # pin 16
GPIO_DEG_A = 5       # pin 29
GPIO_DEG_B = 6       # pin 31

IFACES = ("canA", "canB")      # canA = bus A (CE0, spi0.0), canB = bus B (CE1, spi0.1)

# Linux kernel receive timestamps for --udp-diag (SO_TIMESTAMPNS = SCM_TIMESTAMPNS = 35)
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
SCM_TIMESTAMPNS = SO_TIMESTAMPNS

# Frames the Pi needs from the buses; everything else is filtered in the kernel
RX_IDS = (ID_HB_N2, ID_HB_N3, ID_FAULT, ID_FORCE)


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------
def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def axis_to_u8(raw, released, pressed):
    if pressed == released:
        return 0
    x = (raw - released) / (pressed - released)
    return int(round(clamp(x, 0.0, 1.0) * 255))


def now():
    return time.monotonic()


class TestPoints:
    """GPIO test points via lgpio (preinstalled on Raspberry Pi OS Bookworm).
    Falls back to no-ops so the program still runs without GPIO access."""

    def __init__(self, enabled=True):
        self.h = None
        self.level = {}
        if not enabled:
            return
        try:
            import lgpio
            self.lg = lgpio
            self.h = lgpio.gpiochip_open(0)
            for pin in (GPIO_UDP_RX, GPIO_CMD_TX, GPIO_CAN_TX_A, GPIO_CAN_TX_B,
                        GPIO_DEG_A, GPIO_DEG_B):
                lgpio.gpio_claim_output(self.h, pin, 0)
                self.level[pin] = 0
        except Exception as e:  # noqa: BLE001
            print(f"[tp] GPIO unavailable ({e}); test points disabled")
            self.h = None

    def toggle(self, pin):
        if self.h is not None:
            self.level[pin] ^= 1
            self.lg.gpio_write(self.h, pin, self.level[pin])

    def set(self, pin, value):
        if self.h is not None and self.level.get(pin) != int(value):
            self.level[pin] = int(value)
            self.lg.gpio_write(self.h, pin, int(value))


# ----------------------------------------------------------------------------
# Cockpit
# ----------------------------------------------------------------------------
class Cockpit:
    def __init__(self, args):
        self.args = args
        self.tp = TestPoints(enabled=not args.no_gpio)
        self.tp_pulse = (args.tp_mode == "pulse")

        # Locks: self.lock guards the cockpit STATE only and is never held while a
        # frame is written to a socket or a line is printed. self.tx_lock orders
        # sequence numbers and the two socket writes (a few tens of us).
        self.lock = threading.RLock()
        self.tx_lock = threading.Lock()
        self.outbox = []                       # frames queued under self.lock, sent after it is released
        self.log_q = queue.SimpleQueue()

        if args.can_interface == "socketcan":
            missing = [i for i in IFACES if not os.path.exists(f"/sys/class/net/{i}")]
            if missing:
                sys.exit(f"CAN interface(s) {missing} not found. Run 'sudo sh setup_can.sh' "
                         f"(it names the interfaces by SPI device) and check the module wiring.")
        self.buses = [can.interface.Bus(channel=i, interface=args.can_interface,
                                        receive_own_messages=False) for i in IFACES]
        flt = [{"can_id": i, "can_mask": 0x7FF} for i in RX_IDS]
        for bus in self.buses:
            try:
                bus.set_filters(flt)
            except Exception as e:  # noqa: BLE001
                print(f"[can] could not set kernel filters ({e}); receiving everything")
        self.tx_pins = (GPIO_CAN_TX_A, GPIO_CAN_TX_B)
        self.deg_pins = (GPIO_DEG_A, GPIO_DEG_B)

        self.seq = {ID_CTRL: 0, ID_BLINK: 0, ID_HB_N1: 0, ID_FAULT: 0}
        self.tx_refused = [0, 0]
        self.rx_count = [0, 0]
        self.dup = 0
        self.stale = 0
        self.timer_skips = 0

        # health
        t0 = now()
        self.last_rx = [t0, t0]
        self.degraded = [False, False]
        self.peer_hb = {ZONE_N2: 0.0, ZONE_N3: 0.0}
        self.peer_reasons = {ZONE_N2: 0, ZONE_N3: 0}
        self.peer_alive = {ZONE_N2: False, ZONE_N3: False}
        self.last_remote_fault = 0.0
        self.seen = {}

        # error state
        self.reasons = ERR_CTRL_LOST | ERR_ZONE_LOST
        self.last_udp = 0.0

        # wheel / blinkers
        self.btn_level = {}
        self.btn_change = {}
        self.turn = BLINK_OFF
        self.armed = False
        self.blink_state_sent = None
        self.blink_phase = 1
        self.blink_next_edge = 0.0
        self.last_test_press = None
        self.force = 0

        # injection
        self.inject = None          # 'steer' or 'crc'

        # last wheel values, re-sent every 20 ms when no new UDP update arrives
        # (Lab 2 rule: network jitter must not look like a lost cockpit)
        self.last_wheel = (0, 0, 0, 0)    # steer, thr, brk, btn_bits (before error override)
        self.last_ctrl_tx = 0.0

        # latency log (UDP_RX -> both CAN writes done)
        self.lat_us = []
        self.running = True

        # --udp-diag: kernel arrival timestamps
        self.diag_last_ts = None
        self.diag_t0 = None
        self.diag_lag_max = 0.0
        self.diag_win = {}
        self.diag_gaps = []

    # ---------------- logging (never blocks the real-time paths) ----------------
    def log(self, msg):
        self.log_q.put(msg)

    def log_loop(self):
        while True:
            m = self.log_q.get()
            if m is None:
                return
            print(m, flush=True)

    # ---------------- test points ----------------
    def udp_rx_edge(self):
        if self.tp_pulse:
            self.tp.set(GPIO_UDP_RX, 1)
        else:
            self.tp.toggle(GPIO_UDP_RX)

    def cmd_tx_edge(self):
        if self.tp_pulse:
            self.tp.set(GPIO_CMD_TX, 1)
        else:
            self.tp.toggle(GPIO_CMD_TX)

    def tp_clear(self):
        if self.tp_pulse:
            self.tp.set(GPIO_UDP_RX, 0)
            self.tp.set(GPIO_CMD_TX, 0)

    # ---------------- transmit ----------------
    def send_both(self, can_id, data, heartbeat=False, mark_cmd=False):
        """Send one frame on both buses. Call WITHOUT self.lock held."""
        data = bytearray(data)
        first = True
        with self.tx_lock:                       # keeps seq order == wire order across threads
            s = self.seq[can_id] = (self.seq[can_id] + 1) & 0x0F
            data[0] = (data[0] & 0xF0) | s
            if can_id == ID_CTRL:
                data[6] = crc8(data[0:6])
                if self.inject == "crc":
                    data[6] ^= 0xFF
                    self.inject = None
                    self.log("[inject] sent CTRL with a bad CRC")
            msg = can.Message(arbitration_id=can_id, data=bytes(data), is_extended_id=False)
            for b in range(2):
                if self.degraded[b] and not heartbeat:
                    continue
                self.tp.toggle(self.tx_pins[b])
                try:
                    self.buses[b].send(msg, timeout=0)
                except can.CanError:
                    self.tx_refused[b] += 1      # queue full: nobody ACKing on that bus
                if mark_cmd and first:
                    self.cmd_tx_edge()           # CMD_TX: right after the FIRST CAN write returns
                    first = False

    def queue_frame(self, can_id, data, heartbeat=False):
        """Queue a frame for sending. Call with self.lock held; flush_outbox() sends it."""
        self.outbox.append((can_id, data, heartbeat))

    def flush_outbox(self):
        """Send everything queued. Call WITHOUT self.lock held."""
        with self.lock:
            items, self.outbox = self.outbox, []
        for can_id, data, hb in items:
            self.send_both(can_id, data, heartbeat=hb)

    def flush_bus(self, b):
        """Drop frames stuck in the kernel/MCP2515 TX queue (down/up keeps bitrate)."""
        iface = IFACES[b]
        try:
            subprocess.run(["ip", "link", "set", iface, "down"], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["ip", "link", "set", iface, "up"], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:  # noqa: BLE001
            self.log(f"[deg] could not flush {iface} ({e}); run as root")

    # ---------------- error state (self.lock held by the caller) ----------------
    def set_reason(self, bits):
        was = self.reasons
        self.reasons |= bits
        if was == 0 and self.reasons:
            self.log(f"[state] ERROR reasons=0x{self.reasons:02x}")

    def clear_reason(self, bits):
        was = self.reasons
        self.reasons &= ~bits
        if was and self.reasons == 0:
            self.log("[state] NORMAL")

    def send_fault(self, failing, reason):
        d = bytearray(8)
        d[1], d[2], d[3] = failing, reason, ZONE_N1
        self.queue_frame(ID_FAULT, d)

    # ---------------- UDP_RX + CTRL_TX ----------------
    def button_pressed(self, idx, level, t):
        old = self.btn_level.get(idx, False)
        if level != old and (t - self.btn_change.get(idx, -1e9)) >= BUTTON_DEBOUNCE_S:
            self.btn_level[idx] = level
            self.btn_change[idx] = t
            return level
        return False

    def update_blinkers(self, steer, buttons, t):
        if self.button_pressed(BTN_LEFT_BLINKER, buttons[BTN_LEFT_BLINKER], t):
            self.turn = BLINK_OFF if self.turn == BLINK_LEFT else BLINK_LEFT
            self.armed = False
        if self.button_pressed(BTN_RIGHT_BLINKER, buttons[BTN_RIGHT_BLINKER], t):
            self.turn = BLINK_OFF if self.turn == BLINK_RIGHT else BLINK_RIGHT
            self.armed = False
        # self-cancel: armed once the wheel goes past the threshold toward the
        # active side, cancels when it comes back inside threshold - hysteresis
        if self.turn == BLINK_LEFT:
            if not self.armed and steer <= -TURN_THRESHOLD:
                self.armed = True
            elif self.armed and steer > -(TURN_THRESHOLD - TURN_HYSTERESIS):
                self.turn, self.armed = BLINK_OFF, False
        elif self.turn == BLINK_RIGHT:
            if not self.armed and steer >= TURN_THRESHOLD:
                self.armed = True
            elif self.armed and steer < (TURN_THRESHOLD - TURN_HYSTERESIS):
                self.turn, self.armed = BLINK_OFF, False

    def test_press(self, t):
        if not (self.reasons & ERR_TEST):
            self.set_reason(ERR_TEST)
            self.send_fault(ZONE_N1, FAULT_TEST_BUTTON)
            self.last_test_press = None      # the failing press never counts toward a double press
        elif self.last_test_press is not None and (t - self.last_test_press) <= DOUBLE_PRESS_S:
            self.clear_reason(ERR_TEST)
            self.last_test_press = None
        else:
            self.last_test_press = t

    def blink_display(self):
        return BLINK_HAZARD if self.reasons else self.turn

    def queue_blink(self, t, state=None):
        st = self.blink_display() if state is None else state
        if st != self.blink_state_sent:
            # new state: start "on" and restart the 500 ms cadence from here
            self.blink_phase = 1 if st != BLINK_OFF else 0
            self.blink_next_edge = t + BLINK_HALF_S
        d = bytearray(8)
        d[1], d[2] = st, self.blink_phase
        self.queue_frame(ID_BLINK, d)
        self.blink_state_sent = st

    def build_ctrl(self, t):
        """Build CTRL from the last wheel values. Call with self.lock held.
        Called on every UDP update, and by the timer when 20 ms pass with no
        update while the stream is alive."""
        steer, thr, brk, btn_bits = self.last_wheel
        if self.reasons:                     # system error: brake, no throttle
            thr, brk = 0, 255
        if self.inject == "steer":
            steer = -32768                   # out of range by the values table
            self.inject = None
            self.log("[inject] sent CTRL with steer = -32768")
        d = bytearray(8)
        struct.pack_into("<h", d, 1, steer)
        d[3], d[4], d[5] = thr, brk, btn_bits
        self.last_ctrl_tx = t
        return d

    def handle_udp(self, pkt, t_rx_ns):
        if len(pkt) < 4 + 48 + 128:
            return
        # 4-byte counter, then DIJOYSTATE2: lX, lY, lZ, lRx, lRy, lRz, sliders[2],
        # POV[4], buttons[128]
        lx, ly, _lz, _lrx, _lry, lrz = struct.unpack_from("<6i", pkt, 4)
        buttons = [b & 0x80 != 0 for b in pkt[4 + 48:4 + 48 + 128]]
        if self.args.raw:
            pressed = [i for i, v in enumerate(buttons) if v]
            self.log(f"raw lX={lx} lY={ly} lRz={lrz} buttons={pressed}")
        t = now()
        self.last_udp = t

        steer = int(clamp(lx, -STEER_RAW_LIMIT, STEER_RAW_LIMIT))
        thr = axis_to_u8(ly, THR_RAW_RELEASED, THR_RAW_PRESSED)
        brk = axis_to_u8(lrz, BRK_RAW_RELEASED, BRK_RAW_PRESSED)

        with self.lock:                      # state only: no socket write, no print in here
            if self.button_pressed(BTN_TEST, buttons[BTN_TEST], t):
                self.test_press(t)
            self.update_blinkers(steer, buttons, t)

            btn_bits = (1 if buttons[BTN_LEFT_BLINKER] else 0) | \
                       (2 if buttons[BTN_RIGHT_BLINKER] else 0) | \
                       (4 if buttons[BTN_TEST] else 0)
            self.last_wheel = (steer, thr, brk, btn_bits)
            ctrl = self.build_ctrl(t)
            if self.blink_display() != self.blink_state_sent:
                self.queue_blink(t)          # state change: BLINK right after CTRL

        self.send_both(ID_CTRL, ctrl, mark_cmd=True)
        t_done = time.perf_counter_ns()
        self.flush_outbox()                  # BLINK / FAULT queued above
        self.lat_us.append((t_done - t_rx_ns) / 1000.0)

    def udp_diag_sample(self, anc):
        """--udp-diag: use the kernel's arrival timestamp (taken before Python runs) to tell a
        network gap (packets arrive late) from a cockpit stall (packets arrive on time but are
        handled late)."""
        ts = None
        for level, ctype, data in anc:
            if level == socket.SOL_SOCKET and ctype == SCM_TIMESTAMPNS and len(data) >= 16:
                sec, nsec = struct.unpack("qq", data[:16])
                ts = sec * 1_000_000_000 + nsec
        if ts is None:
            return
        lag_ms = (time.time_ns() - ts) / 1e6          # kernel arrival -> Python handling
        gap_ms = None if self.diag_last_ts is None else (ts - self.diag_last_ts) / 1e6
        self.diag_last_ts = ts
        t = now()
        if self.diag_t0 is None:
            self.diag_t0 = t
        self.diag_lag_max = max(self.diag_lag_max, lag_ms)
        win = int((t - self.diag_t0) // 10)           # 10 s windows: does it get worse over time?
        w = self.diag_win.setdefault(win, [0, 0.0, 0.0])
        w[0] += 1
        w[2] = max(w[2], lag_ms)
        if gap_ms is not None:
            self.diag_gaps.append(gap_ms)
            w[1] = max(w[1], gap_ms)
            if gap_ms > 100:
                self.log(f"[udp] kernel arrival gap {gap_ms:.0f} ms; this packet handled "
                         f"{lag_ms:.1f} ms after it arrived")

    def udp_diag_report(self):
        g = sorted(self.diag_gaps)
        if not g:
            print("[udp-diag] no samples (kernel timestamps unavailable?)")
            return
        n = len(g)
        print(f"[udp-diag] {n + 1} packets, mean interval {sum(g) / n:.1f} ms, "
              f"p99 {g[int(0.99 * (n - 1))]:.0f} ms, max gap {g[-1]:.0f} ms (kernel arrival times)")
        print("[udp-diag] gaps > 60 / 100 / 150 / 200 ms: "
              + " / ".join(str(sum(1 for x in g if x > lim)) for lim in (60, 100, 150, 200)))
        print(f"[udp-diag] worst delay from kernel arrival to Python handling: {self.diag_lag_max:.1f} ms")
        print("[udp-diag] per 10 s window (packets, max gap ms, max handling lag ms):")
        for k in sorted(self.diag_win):
            c, mg, ml = self.diag_win[k]
            print(f"   t={k * 10:4d}s  n={c:4d}  max_gap={mg:6.0f}  max_lag={ml:6.1f}")

    def udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", UDP_PORT))
        sock.settimeout(0.5)
        diag = self.args.udp_diag
        if diag:
            try:
                sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            except OSError as e:
                print(f"[udp-diag] kernel timestamps unavailable ({e}); diagnostics off")
                diag = False
        self.log(f"[udp] listening on {UDP_PORT}")
        while self.running:
            try:
                if diag:
                    pkt, anc, _flags, _addr = sock.recvmsg(1024, socket.CMSG_SPACE(16))
                else:
                    pkt, _addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            t_rx = time.perf_counter_ns()
            if diag:
                self.udp_diag_sample(anc)
            self.udp_rx_edge()
            try:
                self.handle_udp(pkt, t_rx)
            except Exception as e:  # noqa: BLE001  (a bad packet must not kill the cockpit)
                self.log(f"[udp] packet error: {e!r}")
            finally:
                self.tp_clear()

    # ---------------- receive (FORCE_RX / heartbeats / FAULT) ----------------
    def check_seq(self, can_id, src, seq, t):
        key = (can_id, src)
        e = self.seen.get(key)
        if e is None:
            self.seen[key] = [seq, t]
            return "accept"
        ahead = (seq - e[0]) & 0x0F
        age = t - e[1]
        if ahead == 0 and age < DEDUP_WINDOW_S:
            return "dup"
        if ahead >= 9 and age < STALE_WINDOW_S:
            return "stale"
        e[0], e[1] = seq, t
        return "accept"

    def handle_rx(self, b, msg):
        t = now()
        self.last_rx[b] = t
        self.rx_count[b] += 1
        cid, d = msg.arbitration_id, msg.data
        src = {ID_HB_N2: ZONE_N2, ID_FORCE: ZONE_N2, ID_HB_N3: ZONE_N3}.get(cid)
        if cid == ID_FAULT:
            src = d[3]
        if src is None or src == ZONE_N1:
            return
        with self.lock:
            v = self.check_seq(cid, src, d[0] & 0x0F, t)
            if v == "dup":
                self.dup += 1
                return
            if v == "stale":
                self.stale += 1
                return
            if cid in (ID_HB_N2, ID_HB_N3):
                self.peer_hb[src] = t
                self.peer_reasons[src] = d[3]
                if d[3] & ERR_PROPAGATE:
                    self.last_remote_fault = t
                    self.set_reason(ERR_REMOTE)
            elif cid == ID_FAULT:
                if d[2] in (FAULT_TEST_BUTTON, FAULT_BAD_CTRL):
                    self.last_remote_fault = t
                    self.set_reason(ERR_REMOTE)
                self.log(f"[fault] zone {d[1]} reason {d[2]} (from N{d[3]})")
            elif cid == ID_FORCE:
                self.force = struct.unpack_from("<b", d, 1)[0]

    def rx_loop(self, b):
        bus = self.buses[b]
        while self.running:
            try:
                msg = bus.recv(timeout=0.5)
            except can.CanError:
                time.sleep(0.01)             # interface being flushed
                continue
            except Exception as e:  # noqa: BLE001
                self.log(f"[rx] {IFACES[b]} receive error: {e!r}")
                time.sleep(0.05)
                continue
            if msg is None or msg.is_error_frame or msg.dlc != 8:
                continue
            try:
                self.handle_rx(b, msg)
            except Exception as e:  # noqa: BLE001  (one bad frame must not stop the receive thread)
                self.log(f"[rx] {IFACES[b]} handler error: {e!r}")

    # ---------------- HB_TX / HB_MON / BLINK timer ----------------
    def timer_loop(self):
        next_t = now()
        while self.running:
            next_t += HB_PERIOD_S
            delay = next_t - now()
            if delay > 0:
                time.sleep(delay)
            elif delay < -HB_PERIOD_S:
                self.timer_skips += 1        # stalled: resynchronise, never burst to catch up
                next_t = now()
            t = now()
            with self.lock:
                # HB_MON: buses
                for b in range(2):
                    deg = (t - self.last_rx[b]) >= DEG_TIMEOUT_S
                    if deg != self.degraded[b]:
                        self.degraded[b] = deg
                        self.tp.set(self.deg_pins[b], deg)
                        self.log(f"[deg] {IFACES[b]} {'DEGRADED' if deg else 'ok'}")
                        if deg:
                            threading.Thread(target=self.flush_bus, args=(b,), daemon=True).start()
                # HB_MON: zones
                all_alive, remote = True, False
                for z in (ZONE_N2, ZONE_N3):
                    alive = self.peer_hb[z] > 0 and (t - self.peer_hb[z]) < ZONE_TIMEOUT_S
                    if self.peer_alive[z] and not alive:
                        self.log(f"[zone] N{z} LOST")
                        self.send_fault(z, FAULT_ZONE_LOST)
                    self.peer_alive[z] = alive
                    all_alive &= alive
                    remote |= alive and bool(self.peer_reasons[z] & ERR_PROPAGATE)
                (self.clear_reason if all_alive else self.set_reason)(ERR_ZONE_LOST)
                stream_ok = self.last_udp > 0 and (t - self.last_udp) < PROXY_TIMEOUT_S
                (self.clear_reason if stream_ok else self.set_reason)(ERR_CTRL_LOST)

                # CTRL keep-alive: no UDP update for 20 ms but the stream is
                # alive -> re-send the last values. If the proxy is quiet for
                # 100 ms we stop, so the zones see CTRL_LOST (fail safe).
                # 30 ms threshold: only when an update is really late, so the
                # keep-alive never doubles the normal 20 ms CTRL stream.
                if stream_ok and (t - self.last_ctrl_tx) >= HB_PERIOD_S * 1.5:
                    self.queue_frame(ID_CTRL, self.build_ctrl(t))
                if (self.reasons & ERR_REMOTE) and not remote and \
                        (t - self.last_remote_fault) >= REMOTE_HOLD_S:
                    self.clear_reason(ERR_REMOTE)

                # HB_TX (none while in test-button error: FAULT once, heartbeats stop)
                if not (self.reasons & ERR_TEST):
                    d = bytearray(8)
                    d[1] = 0 if self.reasons else 1
                    d[2] = (0 if self.degraded[0] else 1) | (0 if self.degraded[1] else 2)
                    d[3] = self.reasons & 0xFF
                    d[4] = (1 << ZONE_N1) | sum(1 << z for z in (ZONE_N2, ZONE_N3) if self.peer_alive[z])
                    self.queue_frame(ID_HB_N1, d, heartbeat=True)

                # BLINK_TX: state follows the error state even with no UDP;
                # toggle edges every 500 ms while lit
                st = self.blink_display()
                if st != self.blink_state_sent:
                    self.queue_blink(t, st)
                elif st != BLINK_OFF and t >= self.blink_next_edge:
                    self.blink_phase ^= 1
                    self.blink_next_edge += BLINK_HALF_S
                    self.queue_blink(t, st)
            self.flush_outbox()              # all socket writes happen outside the state lock

    # ---------------- console ----------------
    def stats(self):
        lat = sorted(self.lat_us)
        p99 = lat[int(0.99 * (len(lat) - 1))] if lat else 0
        mx = lat[-1] if lat else 0
        print(f"[stats] reasons=0x{self.reasons:02x} deg A/B={self.degraded} "
              f"rx A/B={self.rx_count} refused A/B={self.tx_refused} dup={self.dup} "
              f"stale={self.stale} force={self.force} timer_skips={self.timer_skips}")
        print(f"[stats] UDP_RX->CAN write: n={len(lat)} p99={p99:.0f} us max={mx:.0f} us")

    def console_loop(self):
        while self.running:
            r, _, _ = select.select([sys.stdin], [], [], 0.5)
            if not r:
                continue
            line = sys.stdin.readline()
            if line == "":
                return                       # stdin closed (unattended run): stop polling it
            cmd = line.strip().lower()
            if cmd == "b":
                self.inject = "steer"
            elif cmd == "c":
                self.inject = "crc"
            elif cmd == "t":
                with self.lock:
                    self.test_press(now())
                self.flush_outbox()
            elif cmd == "s":
                self.stats()
            elif cmd == "q":
                self.running = False

    def save_latency(self, path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["udp_rx_to_can_write_us"])
            for v in self.lat_us:
                w.writerow([f"{v:.1f}"])
        print(f"[stats] wrote {len(self.lat_us)} samples to {path}")

    def run(self):
        logger = threading.Thread(target=self.log_loop, daemon=True)
        logger.start()
        threads = [threading.Thread(target=self.rx_loop, args=(b,), daemon=True) for b in range(2)]
        threads.append(threading.Thread(target=self.timer_loop, daemon=True))
        threads.append(threading.Thread(target=self.console_loop, daemon=True))
        for th in threads:
            th.start()
        if self.args.duration > 0:
            def stop_later():
                time.sleep(self.args.duration)
                self.running = False
            threading.Thread(target=stop_later, daemon=True).start()
        try:
            self.udp_loop()
        except KeyboardInterrupt:
            pass
        self.running = False
        time.sleep(0.1)
        self.log_q.put(None)
        logger.join(timeout=1.0)             # let queued messages print before the stats
        self.stats()
        if self.args.udp_diag:
            self.udp_diag_report()
        self.save_latency(self.args.latency_csv)
        for b in self.buses:
            b.shutdown()


def main():
    ap = argparse.ArgumentParser(description="Lab 3 cockpit zone (N1)")
    ap.add_argument("--rt", action="store_true", help="SCHED_FIFO 50 and pin to CPU 3 (root)")
    ap.add_argument("--raw", action="store_true", help="print raw axes/buttons (calibration)")
    ap.add_argument("--no-gpio", action="store_true", help="run without test points")
    ap.add_argument("--tp-mode", choices=("pulse", "toggle"), default="pulse",
                    help="UDP_RX / CMD_TX test points: 'pulse' = high from the event to the end of "
                         "the handler (every frame gives a rising edge, use for scope Delay "
                         "statistics); 'toggle' = flip on each event (original behaviour)")
    ap.add_argument("--udp-diag", action="store_true",
                    help="record kernel arrival times of the wheel stream: separates network gaps "
                         "from cockpit stalls (prints a report at the end)")
    ap.add_argument("--latency-csv", default="latency.csv")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many seconds (measurement runs)")
    ap.add_argument("--can-interface", default="socketcan",
                    help="python-can interface (use 'virtual' for a desk test without hardware)")
    args = ap.parse_args()

    if args.rt:
        try:
            os.sched_setaffinity(0, {3})
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
            print("[rt] SCHED_FIFO 50 on CPU 3")
        except PermissionError:
            print("[rt] need root for --rt; continuing without it")

    cockpit = Cockpit(args)
    # Python's garbage collector pauses every thread for hundreds of us at random
    # times; keep it out of the 20 ms loop (a run is minutes, the data is tiny).
    gc.collect()
    gc.freeze()
    gc.set_threshold(200000, 50, 50)
    cockpit.run()


if __name__ == "__main__":
    main()
