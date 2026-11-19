#!/usr/bin/env python3
"""
cloud_pub.py - Lab 5 extra credit: the Pi's uplink to the GUI server.

Publishes every committed Raft log entry (-> vehicle:log) and a 1 s heartbeat
(-> vehicle:heartbeat) over HTTPS. Python stdlib only.

Never on the CAN path: push_entry() only appends to an in-memory queue, and
one background thread (SCHED_OTHER, kept off the RT core) drains it. REF,
APPEND and CTRL timing is untouched; show that with a scope capture.
Uplink down: the queue keeps the last 1,000 entries; on reconnect the server
says which index it already has and sending resumes after it.

On the car: cockpit.py --cloud-url ... starts it. Test without the car:
  python3 cloud_pub.py --url https://LAPTOP_IP:8443 --token TOKEN --cafile server.crt --demo
"""
import argparse
import collections
import json
import os
import ssl
import threading
import time
import urllib.error
import urllib.request

NODE = {1: "cockpit", 2: "steering", 3: "drivetrain"}
QUEUE_LEN = 1000
HB_PERIOD_S = 1.0
BATCH = 50


def decode_entry(e):
    """Entry byte (catalog v0.7): bits 7-5 type, 4-3 node, 2 bus (0 A, 1 B), 0 up.
    Returns (class, type, payload) as the console prints it."""
    t, node, bus, up = e >> 5, (e >> 3) & 3, "AB"[(e >> 2) & 1], e & 1
    n = NODE.get(node, str(node))
    ud = "UP" if up else "DOWN"
    if t == 1:
        return "MEMBER", "NOP", ""
    if t == 2:
        return "MEMBER", "COCKPIT", "OK" if up else "ERROR"
    if t == 3:
        return "MEMBER", "NODE", f"{n} {ud}"
    if t == 4:
        return "MEMBER", "LINK", f"{n} {bus} {ud}"
    if t == 5:
        return "EVENT", "APP_EVENT", f"{n} {e & 7}"
    return "?", f"RESERVED{t}", f"0x{e:02x}"


def _leave_rt():
    """Called on the uplink threads: never run at the cockpit's SCHED_FIFO
    priority or on its pinned core. On Linux pid 0 = the calling thread."""
    try:
        os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    except (AttributeError, OSError):
        pass
    try:
        cpus = os.sched_getaffinity(0) | set(range(os.cpu_count() or 1))
        if len(cpus) >= 4:
            os.sched_setaffinity(0, cpus - {3})      # cockpit.py --rt pins to CPU 3
    except (AttributeError, OSError):
        pass


class CloudPublisher:
    """status_fn() -> dict for the heartbeat (see PiNodeSource.status)."""

    def __init__(self, url, token, cafile, status_fn, log=print):
        self.url = url.rstrip("/")
        self.token = token
        self.ctx = ssl.create_default_context(cafile=cafile)   # trusts only our server's own cert
        # The cloud VM gets a new public IP each time it starts. The cert itself is
        # still pinned (only our self-signed cert is trusted), so skip the IP check.
        self.ctx.check_hostname = False
        self.status_fn = status_fn
        self.log = log
        self.q = collections.deque(maxlen=QUEUE_LEN)
        self.cv = threading.Condition()
        self.dropped = 0
        self.first = True
        self.up = None
        self.last_id = None
        self.paused = False          # demo only: pretend the Pi is off
        self._stop = threading.Event()

    # ---- called by cockpit.py's log thread; never blocks
    def push_entry(self, index, term, entry, replayed, global_us=None):
        cls, typ, payload = decode_entry(entry)
        item = {"index": index, "term": term, "entry": entry, "cls": cls, "type": typ,
                "payload": payload, "replayed": bool(replayed), "global_us": global_us,
                # A live (not replayed) index 1 is a cold start of the whole car:
                # the server archives the old run. A Pi reboot comes back replayed.
                "fresh": self.first and index == 1 and not replayed}
        self.first = False
        with self.cv:
            if len(self.q) == QUEUE_LEN:
                self.dropped += 1
            self.q.append(item)
            self.cv.notify()

    def start(self):
        threading.Thread(target=self._send_loop, name="cloud_tx", daemon=True).start()
        threading.Thread(target=self._hb_loop, name="cloud_hb", daemon=True).start()

    def stop(self):
        self._stop.set()
        with self.cv:
            self.cv.notify()

    # ---- internals
    def _post(self, path, body):
        req = urllib.request.Request(self.url + path, data=json.dumps(body).encode(), method="POST",
                                     headers={"Authorization": "Bearer " + self.token,
                                              "Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=2, context=self.ctx) as resp:
            return json.loads(resp.read())

    def _set_up(self, ok, why=""):
        if ok != self.up:
            self.log(f"[cloud] uplink {'up' if ok else 'DOWN: ' + why}")
        self.up = ok

    def _send_loop(self):
        _leave_rt()
        while not self._stop.is_set():
            with self.cv:
                while (not self.q or self.paused) and not self._stop.is_set():
                    self.cv.wait(0.5)
                batch = [self.q[i] for i in range(min(BATCH, len(self.q)))]
            if not batch:
                continue
            try:
                resp = self._post("/api/ingest/log", {"entries": batch})
            except (urllib.error.URLError, OSError, ValueError) as e:
                self._set_up(False, str(e))
                self._stop.wait(1.0)
                continue
            self._set_up(True)
            self.last_id = resp.get("last_id")
            sent = {id(x) for x in batch}
            with self.cv:
                while self.q and id(self.q[0]) in sent:
                    self.q.popleft()

    def _hb_loop(self):
        _leave_rt()
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += HB_PERIOD_S
            if not self.paused:
                try:
                    hb = self.status_fn()
                    hb.update(pi_wall_ms=int(time.time() * 1000), queue_len=len(self.q),
                              dropped=self.dropped)
                    self._post("/api/ingest/heartbeat", hb)
                    self._set_up(True)
                except (urllib.error.URLError, OSError, ValueError) as e:
                    self._set_up(False, str(e))
            self._stop.wait(max(0.0, nxt - time.monotonic()))


# ---------------------------------------------------------------- pi_node (via cockpit_pi/pi_node.py)
class PiNodeSource:
    """Heartbeat contents from the PiNode wrapper cockpit.py already owns.
    cockpit.py keeps the only pi_node_poll_log() reader and calls pub.push_entry()."""

    def __init__(self, node):
        self.node = node
        self.fl_rl_offset_us = None        # set this if you add an FL/RL measurement

    def global_us(self):
        return int(self.node.global_us())

    def status(self):
        st = self.node.status()
        return {"alive": st.alive, "link": list(st.links), "leader": st.leader, "term": st.term,
                "failure": bool(st.failure), "bus_ok_mask": st.bus_ok, "drift_ppb": st.drift_ppb,
                "global_us": self.global_us(), "fl_rl_offset_us": self.fl_rl_offset_us}


# ---------------------------------------------------------------- demo (no car needed)
def run_demo(pub):
    """Walks the extra-credit demonstration: link fault, Pi down (STM32 leads,
    events held), Pi back (held events replayed). Loops forever."""
    st = {"alive": 7, "link": [3, 3, 3], "leader": 1, "term": 1, "failure": False,
          "bus_ok_mask": 3, "drift_ppb": 12000, "fl_rl_offset_us": 140}
    t0 = time.monotonic()
    st_fn = lambda: dict(st, global_us=int((time.monotonic() - t0) * 1e6))
    pub.status_fn = st_fn
    idx = 0

    def add(entry, replayed=False):
        nonlocal idx
        idx += 1
        pub.push_entry(idx, st["term"], entry, replayed, st_fn()["global_us"])

    pub.start()
    add(0x20)                                         # NOP
    for step in range(10 ** 9):
        time.sleep(3)
        print(f"[demo] step {step % 6}")
        s = step % 6
        if s == 0:
            st["link"][1] = 2; add(0x90)              # steering loses bus A
        elif s == 1:
            st["link"][1] = 3; add(0x91)              # back
        elif s == 2:                                  # Pi off: drivetrain leads, events held
            pub.paused = True
            st.update(alive=6, leader=3, term=st["term"] + 1, failure=True)
            held = [0x20, 0x40, 0x68, 0xB3, 0xB3]     # NOP, COCKPIT ERROR, cockpit DOWN, 2 events
        elif s == 3:                                  # Pi back: catch-up, marked replayed
            st.update(alive=7)
            for e in held:
                add(e, replayed=True)
            pub.paused = False
            add(0x69); add(0x41)                      # cockpit UP, COCKPIT OK
            st.update(failure=False)


def main():
    ap = argparse.ArgumentParser(description="Demo only. On the car, cockpit.py --cloud-url ... runs the uplink.")
    ap.add_argument("--url", required=True, help="https://LAPTOP_IP:8443")
    ap.add_argument("--token", required=True)
    ap.add_argument("--cafile", required=True, help="the laptop's server.crt")
    ap.add_argument("--demo", action="store_true", required=True, help="fake vehicle, no car needed")
    a = ap.parse_args()
    run_demo(CloudPublisher(a.url, a.token, a.cafile, status_fn=dict))


if __name__ == "__main__":
    main()
