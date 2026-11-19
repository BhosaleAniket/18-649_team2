#!/usr/bin/env python3
"""
cockpit.py - Lab 5 N1 cockpit (Raspberry Pi 4), Team 2

The Lab 3 cockpit (lab_3/cockpit_pi/cockpit_pi/cockpit.py) on top of pi_node. The wheel, CTRL,
blinker and error-state logic is unchanged; the CAN side moved into C:

  pi_node (C, libpi_node.so)                 cockpit.py (this file)
  -------------------------------------      --------------------------------------------------
  owns canA / canB, de-dups, relays           UDP_RX: wheel stream -> CTRL on every update
  HB_N1 every 20 ms (from our state)          BLINK on changes and 500 ms edges, FAULT
  per-bus DEG detection (+ TX flush)          error state (R3): CTRL lost, zones lost, test button,
  Raft (leader / follower), REF, time sync      remote TEST/BADCMD -> pushed to pi_node every 20 ms
  writes LINK / NODE / COCKPIT entries        zone liveness from HB_N2 / HB_N3 (via poll_app)
  goes silent in Raft on ERR_TEST             console: stats, getNetwork, events, test button

Run (root for SCHED_FIFO in pi_node and for the DEG TX-queue flush):
  cd ../pi_node && make            # once: builds libpi_node.so
  sudo python3 cockpit.py --rt
Keys (then Enter): s stats, n getNetwork, g global_gettime(), e N report APP_EVENT code N, t test button,
b bad steer, c bad CRC, q quit.
"""

import argparse
import csv
import gc
import os
import queue
import select
import socket
import struct
import sys
import threading
import time

from pi_node import DEFAULT_CONFIG, DEFAULT_LIB, PiNode, format_network

# ----------------------------------------------------------------------------
# Catalog (Lab 3 frames, unchanged in Lab 5; mirrors pi_node/include/catalog.h)
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
# Team values (same as Lab 3)
# ----------------------------------------------------------------------------
UDP_PORT = 8000
PROXY_TIMEOUT_S = 0.200

STEER_RAW_LIMIT = 32767
THR_RAW_RELEASED, THR_RAW_PRESSED = 32767, -32767
BRK_RAW_RELEASED, BRK_RAW_PRESSED = 32767, -32767

BTN_LEFT_BLINKER = 5
BTN_RIGHT_BLINKER = 4
BTN_TEST = 3

BUTTON_DEBOUNCE_S = 0.050
DOUBLE_PRESS_S = 0.500
STEER_FULL_SCALE = 8192
TURN_THRESHOLD = STEER_FULL_SCALE // 4
TURN_HYSTERESIS = STEER_FULL_SCALE // 20

TICK_S = 0.020
ZONE_TIMEOUT_S = 0.066
BLINK_HALF_S = 0.500
REMOTE_HOLD_S = 0.100

# Test points (BCM). CAN_TX_A/B now toggle around the pi_node send of both copies.
GPIO_UDP_RX = 17     # pin 11
GPIO_CMD_TX = 27     # pin 13
GPIO_CAN_TX_A = 22   # pin 15
GPIO_CAN_TX_B = 23   # pin 16
GPIO_DEG_A = 5       # pin 29
GPIO_DEG_B = 6       # pin 31

SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
SCM_TIMESTAMPNS = SO_TIMESTAMPNS


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
    """GPIO test points via lgpio; no-ops if GPIO is unavailable."""

    def __init__(self, enabled=True):
        self.h = None
        self.level = {}
        if not enabled:
            return
        try:
            import lgpio
            self.lg = lgpio
            self.h = lgpio.gpiochip_open(0)
            for pin in (GPIO_UDP_RX, GPIO_CMD_TX, GPIO_CAN_TX_A, GPIO_CAN_TX_B, GPIO_DEG_A, GPIO_DEG_B):
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

        self.lock = threading.RLock()          # cockpit state only; never held across a pi_node call
        self.tx_lock = threading.Lock()        # sequence numbers in wire order
        self.outbox = []
        self.log_q = queue.SimpleQueue()

        self.node = PiNode(args.lib)
        if self.node.start(args.config) != 0:
            sys.exit("pi_node_start failed: see the message above (canA/canB up? run setup_can.sh)")

        self.seq = {ID_CTRL: 0, ID_BLINK: 0, ID_FAULT: 0}
        self.rx_count = [0, 0]
        self.send_errors = 0
        self.timer_skips = 0
        self.log_entries = 0

        t0 = now()
        self.degraded = [False, False]
        self.peer_hb = {ZONE_N2: 0.0, ZONE_N3: 0.0}
        self.peer_reasons = {ZONE_N2: 0, ZONE_N3: 0}
        self.peer_alive = {ZONE_N2: False, ZONE_N3: False}
        self.last_remote_fault = 0.0
        self.boot = t0

        self.reasons = ERR_CTRL_LOST | ERR_ZONE_LOST   # power up in error (R3)
        self.last_udp = 0.0

        self.btn_level = {}
        self.btn_change = {}
        self.turn = BLINK_OFF
        self.armed = False
        self.blink_state_sent = None
        self.blink_phase = 1
        self.blink_next_edge = 0.0
        self.last_test_press = None
        self.force = 0
        self.inject = None

        self.last_wheel = (0, 0, 0, 0)
        self.last_ctrl_tx = 0.0
        self.lat_us = []
        self.running = True

        self.diag_last_ts = None
        self.diag_t0 = None
        self.diag_lag_max = 0.0
        self.diag_gaps = []

        self.push_state()

        # Extra credit: uplink to the GUI server (off unless --cloud-url is given)
        self.cloud = None
        if args.cloud_url:
            from cloud_pub import CloudPublisher, PiNodeSource
            self.cloud_src = PiNodeSource(self.node)
            self.cloud = CloudPublisher(args.cloud_url, args.cloud_token, args.cloud_ca,
                                        self.cloud_src.status, log=self.log)

    # ---------------- logging ----------------
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
        self.tp.set(GPIO_UDP_RX, 1) if self.tp_pulse else self.tp.toggle(GPIO_UDP_RX)

    def cmd_tx_edge(self):
        self.tp.set(GPIO_CMD_TX, 1) if self.tp_pulse else self.tp.toggle(GPIO_CMD_TX)

    def tp_clear(self):
        if self.tp_pulse:
            self.tp.set(GPIO_UDP_RX, 0)
            self.tp.set(GPIO_CMD_TX, 0)

    # ---------------- transmit (through pi_node) ----------------
    def send_both(self, can_id, data, mark_cmd=False):
        """Send one Lab 3 frame; pi_node puts it on every healthy bus. Call WITHOUT self.lock."""
        data = bytearray(data)
        with self.tx_lock:
            s = self.seq[can_id] = (self.seq[can_id] + 1) & 0x0F
            data[0] = (data[0] & 0xF0) | s
            if can_id == ID_CTRL:
                data[6] = crc8(data[0:6])
                if self.inject == "crc":
                    data[6] ^= 0xFF
                    self.inject = None
                    self.log("[inject] sent CTRL with a bad CRC")
            self.tp.toggle(GPIO_CAN_TX_A)
            self.tp.toggle(GPIO_CAN_TX_B)
            if self.node.send_app(can_id, data) != 0:
                self.send_errors += 1
            if mark_cmd:
                self.cmd_tx_edge()

    def queue_frame(self, can_id, data):
        self.outbox.append((can_id, data))

    def flush_outbox(self):
        with self.lock:
            items, self.outbox = self.outbox, []
        for can_id, data in items:
            self.send_both(can_id, data)

    def push_state(self):
        """Our error state -> pi_node: HB_N1 contents, Raft silence (ERR_TEST), COCKPIT entries."""
        with self.lock:
            r = self.reasons
        self.node.set_cockpit_state(r == 0, r & 0xFF)

    # ---------------- error state (self.lock held) ----------------
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
            self.send_fault(ZONE_N1, FAULT_TEST_BUTTON)     # the leader logs it as APP_EVENT
            self.last_test_press = None
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
            self.blink_phase = 1 if st != BLINK_OFF else 0
            self.blink_next_edge = t + BLINK_HALF_S
        d = bytearray(8)
        d[1], d[2] = st, self.blink_phase
        self.queue_frame(ID_BLINK, d)
        self.blink_state_sent = st

    def build_ctrl(self, t):
        steer, thr, brk, btn_bits = self.last_wheel
        if self.reasons:
            thr, brk = 0, 255
        if self.inject == "steer":
            steer = -32768
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

        with self.lock:
            test_before = self.reasons & ERR_TEST
            if self.button_pressed(BTN_TEST, buttons[BTN_TEST], t):
                self.test_press(t)
            self.update_blinkers(steer, buttons, t)
            btn_bits = (1 if buttons[BTN_LEFT_BLINKER] else 0) | \
                       (2 if buttons[BTN_RIGHT_BLINKER] else 0) | \
                       (4 if buttons[BTN_TEST] else 0)
            self.last_wheel = (steer, thr, brk, btn_bits)
            ctrl = self.build_ctrl(t)
            if self.blink_display() != self.blink_state_sent:
                self.queue_blink(t)
            test_changed = (self.reasons & ERR_TEST) != test_before

        self.send_both(ID_CTRL, ctrl, mark_cmd=True)
        t_done = time.perf_counter_ns()
        self.flush_outbox()
        if test_changed:
            self.push_state()                    # go silent in Raft / stop HB_N1 at once
        self.lat_us.append((t_done - t_rx_ns) / 1000.0)

    def udp_diag_sample(self, anc):
        ts = None
        for level, ctype, data in anc:
            if level == socket.SOL_SOCKET and ctype == SCM_TIMESTAMPNS and len(data) >= 16:
                sec, nsec = struct.unpack("qq", data[:16])
                ts = sec * 1_000_000_000 + nsec
        if ts is None:
            return
        lag_ms = (time.time_ns() - ts) / 1e6
        if self.diag_last_ts is not None:
            gap_ms = (ts - self.diag_last_ts) / 1e6
            self.diag_gaps.append(gap_ms)
            if gap_ms > 100:
                self.log(f"[udp] kernel arrival gap {gap_ms:.0f} ms; handled {lag_ms:.1f} ms after arrival")
        self.diag_last_ts = ts
        self.diag_lag_max = max(self.diag_lag_max, lag_ms)

    def udp_diag_report(self):
        g = sorted(self.diag_gaps)
        if not g:
            print("[udp-diag] no samples")
            return
        n = len(g)
        print(f"[udp-diag] {n + 1} packets, mean interval {sum(g) / n:.1f} ms, p99 {g[int(0.99 * (n - 1))]:.0f} ms, "
              f"max gap {g[-1]:.0f} ms, worst handling lag {self.diag_lag_max:.1f} ms")

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
                print(f"[udp-diag] kernel timestamps unavailable ({e})")
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
            except Exception as e:  # noqa: BLE001
                self.log(f"[udp] packet error: {e!r}")
            finally:
                self.tp_clear()

    # ---------------- receive: other zones' Lab 3 frames, de-duplicated by pi_node ----------------
    def handle_rx(self, cid, d, bus):
        t = now()
        self.rx_count[bus - 1] += 1
        src = {ID_HB_N2: ZONE_N2, ID_FORCE: ZONE_N2, ID_HB_N3: ZONE_N3}.get(cid)
        if cid == ID_FAULT:
            src = d[3]
        if src is None or src == ZONE_N1:
            return
        with self.lock:
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

    def rx_loop(self):
        while self.running:
            r = self.node.poll_app(200)
            if r is None:
                continue
            try:
                self.handle_rx(r[0], r[1], r[2])
            except Exception as e:  # noqa: BLE001
                self.log(f"[rx] handler error: {e!r}")

    def raft_log_loop(self):
        """Drain applied entries (pi_node prints them itself when debug is on)."""
        while self.running:
            e = self.node.poll_log(200)
            if e is not None:
                self.log_entries += 1
                if self.cloud:
                    self.cloud.push_entry(e.index, e.term, e.entry, e.replayed, self.node.global_us())

    # ---------------- 20 ms timer: DEG LEDs, zones, CTRL keep-alive, state -> pi_node, BLINK ----------------
    def timer_loop(self):
        next_t = now()
        while self.running:
            next_t += TICK_S
            delay = next_t - now()
            if delay > 0:
                time.sleep(delay)
            elif delay < -TICK_S:
                self.timer_skips += 1
                next_t = now()
            t = now()
            st = self.node.status()
            with self.lock:
                for b in range(2):
                    deg = not (st.bus_ok >> b) & 1
                    if deg != self.degraded[b]:
                        self.degraded[b] = deg
                        self.tp.set((GPIO_DEG_A, GPIO_DEG_B)[b], deg)
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

                if stream_ok and (t - self.last_ctrl_tx) >= TICK_S * 1.5:
                    self.queue_frame(ID_CTRL, self.build_ctrl(t))
                if (self.reasons & ERR_REMOTE) and not remote and \
                        (t - self.last_remote_fault) >= REMOTE_HOLD_S:
                    self.clear_reason(ERR_REMOTE)

                stb = self.blink_display()
                if stb != self.blink_state_sent:
                    self.queue_blink(t, stb)
                elif stb != BLINK_OFF and t >= self.blink_next_edge:
                    self.blink_phase ^= 1
                    self.blink_next_edge += BLINK_HALF_S
                    self.queue_blink(t, stb)
            self.flush_outbox()
            self.push_state()                    # HB_N1 itself is sent by pi_node every 20 ms

    # ---------------- console ----------------
    def stats(self):
        lat = sorted(self.lat_us)
        p99 = lat[int(0.99 * (len(lat) - 1))] if lat else 0
        mx = lat[-1] if lat else 0
        print(f"[stats] reasons=0x{self.reasons:02x} deg A/B={self.degraded} rx A/B={self.rx_count} "
              f"send_errors={self.send_errors} force={self.force} timer_skips={self.timer_skips} "
              f"log_entries={self.log_entries}")
        print(f"[stats] UDP_RX->CAN write: n={len(lat)} p99={p99:.0f} us max={mx:.0f} us")
        print("[stats] " + format_network(self.node.status()))

    def console_loop(self):
        while self.running:
            r, _, _ = select.select([sys.stdin], [], [], 0.5)
            if not r:
                continue
            line = sys.stdin.readline()
            if line == "":
                return                                   # stdin closed: stop polling it
            cmd = line.strip().lower()
            if cmd == "b":
                self.inject = "steer"
            elif cmd == "c":
                self.inject = "crc"
            elif cmd == "t":
                with self.lock:
                    self.test_press(now())
                self.flush_outbox()
                self.push_state()
            elif cmd == "s":
                self.stats()
            elif cmd == "n":
                print(format_network(self.node.status()))
            elif cmd == "g":
                g = self.node.global_us()
                st = self.node.status()
                print(f"global_gettime() = {g // 1000000}.{g % 1000000:06d} s, drift {st.drift_ppb / 1000:.3f} ppm")
            elif cmd.startswith("e"):
                code = int(cmd[1:].strip() or "0") & 7
                print(f"[event] APP_EVENT code {code}: {'queued' if self.node.report_event(code) else 'refused'}")
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
        threads = [threading.Thread(target=f, daemon=True)
                   for f in (self.rx_loop, self.raft_log_loop, self.timer_loop, self.console_loop)]
        for th in threads:
            th.start()
        if self.cloud:
            self.cloud.start()
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
        time.sleep(0.3)
        self.log_q.put(None)
        logger.join(timeout=1.0)
        self.stats()
        if self.args.udp_diag:
            self.udp_diag_report()
        self.save_latency(self.args.latency_csv)
        self.node.stop()


def main():
    ap = argparse.ArgumentParser(description="Lab 5 cockpit zone (N1) on pi_node")
    ap.add_argument("--rt", action="store_true", help="SCHED_FIFO 50 and pin to CPU 3 (root)")
    ap.add_argument("--raw", action="store_true", help="print raw axes/buttons (calibration)")
    ap.add_argument("--no-gpio", action="store_true", help="run without test points")
    ap.add_argument("--tp-mode", choices=("pulse", "toggle"), default="pulse")
    ap.add_argument("--udp-diag", action="store_true")
    ap.add_argument("--latency-csv", default="latency.csv")
    ap.add_argument("--duration", type=float, default=0.0)
    ap.add_argument("--lib", default=DEFAULT_LIB, help="path to libpi_node.so")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="path to raft_config.json")
    ap.add_argument("--cloud-url", help="extra credit: GUI server, e.g. https://LAPTOP_IP:8443")
    ap.add_argument("--cloud-token", help="INGEST_TOKEN printed by setup.py")
    ap.add_argument("--cloud-ca", default="server.crt", help="the server's certificate")
    args = ap.parse_args()

    if args.rt:
        try:
            os.sched_setaffinity(0, {3})
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
            print("[rt] SCHED_FIFO 50 on CPU 3")
        except PermissionError:
            print("[rt] need root for --rt; continuing without it")

    cockpit = Cockpit(args)
    gc.collect()
    gc.freeze()
    gc.set_threshold(200000, 50, 50)
    cockpit.run()


if __name__ == "__main__":
    main()
