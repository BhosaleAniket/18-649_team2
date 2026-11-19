"""
pi_node.py - ctypes wrapper for libpi_node.so (lab_5/pi_node, catalog v0.8).

pi_node owns canA/canB and runs Raft + time sync in C (SCHED_FIFO threads). Python only:
  - sends CTRL / BLINK / FAULT with send_app() (C sends them on the healthy buses and relays),
  - reads the other zones' Lab 3 frames with poll_app() (already de-duplicated by C),
  - pushes the cockpit's state with set_cockpit_state() (C builds HB_N1 from it every 20 ms,
    goes silent in Raft on ERR_TEST, and writes COCKPIT entries when this node leads),
  - reads Raft status and applied log entries.
Every call releases the GIL while it runs in C, so a blocking poll only blocks its own thread.
"""
import ctypes
import os
from collections import namedtuple

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LIB = os.path.join(HERE, "..", "pi_node", "libpi_node.so")
DEFAULT_CONFIG = os.path.join(HERE, "..", "pi_node", "raft_config.json")

NODE_NAMES = {0: "none", 1: "cockpit", 2: "steering", 3: "drivetrain"}


class RaftNet(ctypes.Structure):
    _fields_ = [("alive", ctypes.c_uint8), ("link", ctypes.c_uint8 * 3)]


Status = namedtuple("Status", "alive links leader term failure bus_ok drift_ppb")
LogEntry = namedtuple("LogEntry", "index term entry replayed text")

_u8, _u16, _i32, _bool = ctypes.c_uint8, ctypes.c_uint16, ctypes.c_int32, ctypes.c_bool
_Data8 = ctypes.c_uint8 * 8


class PiNode:
    def __init__(self, lib_path=DEFAULT_LIB):
        self.lib = L = ctypes.CDLL(os.path.abspath(lib_path))
        L.pi_node_start.argtypes = [ctypes.c_char_p]
        L.pi_node_start.restype = ctypes.c_int
        L.pi_node_stop.argtypes = []
        L.pi_node_stop.restype = None
        L.pi_node_send_app.argtypes = [_u16, _Data8]
        L.pi_node_send_app.restype = ctypes.c_int
        L.pi_node_poll_app.argtypes = [ctypes.POINTER(_u16), _Data8, ctypes.POINTER(_u8), ctypes.c_int]
        L.pi_node_poll_app.restype = ctypes.c_int
        L.pi_node_set_cockpit_state.argtypes = [_bool, _u8]
        L.pi_node_set_cockpit_state.restype = None
        L.pi_node_report_event.argtypes = [_u8]
        L.pi_node_report_event.restype = ctypes.c_int
        L.pi_node_status.argtypes = [ctypes.POINTER(RaftNet), ctypes.POINTER(_u8), ctypes.POINTER(_u8),
                                     ctypes.POINTER(_bool), ctypes.POINTER(_u8), ctypes.POINTER(_i32)]
        L.pi_node_status.restype = None
        L.pi_node_poll_log.argtypes = [ctypes.POINTER(_u16), ctypes.POINTER(_u8), ctypes.POINTER(_u8),
                                       ctypes.POINTER(_bool), ctypes.c_int]
        L.pi_node_poll_log.restype = ctypes.c_int
        L.pi_node_global_us.argtypes = []
        L.pi_node_global_us.restype = ctypes.c_uint64
        L.raft_entry_str.argtypes = [_u8, ctypes.c_char_p, ctypes.c_int]
        L.raft_entry_str.restype = ctypes.c_int
        self._started = False

    def start(self, config_path=DEFAULT_CONFIG):
        rc = self.lib.pi_node_start(os.path.abspath(config_path).encode())
        self._started = rc == 0
        return rc

    def stop(self):
        if self._started:
            self.lib.pi_node_stop()
            self._started = False

    def send_app(self, can_id, data):
        return self.lib.pi_node_send_app(can_id, _Data8(*bytes(data)[:8]))

    def poll_app(self, timeout_ms):
        """(can_id, bytes, bus 1/2) or None on timeout"""
        cid, bus, d = _u16(), _u8(), _Data8()
        if self.lib.pi_node_poll_app(ctypes.byref(cid), d, ctypes.byref(bus), timeout_ms) != 0:
            return None
        return cid.value, bytes(d), bus.value

    def set_cockpit_state(self, ok, err_bits):
        self.lib.pi_node_set_cockpit_state(bool(ok), err_bits & 0xFF)

    def report_event(self, code):
        return self.lib.pi_node_report_event(code & 7) == 0

    def status(self):
        net, leader, term, failure, bus_ok, drift = RaftNet(), _u8(), _u8(), _bool(), _u8(), _i32()
        self.lib.pi_node_status(ctypes.byref(net), ctypes.byref(leader), ctypes.byref(term),
                                ctypes.byref(failure), ctypes.byref(bus_ok), ctypes.byref(drift))
        return Status(net.alive, tuple(net.link), leader.value, term.value, failure.value,
                      bus_ok.value, drift.value)

    def global_us(self):
        """global_gettime(): global time in microseconds (the leader's clock, drift-corrected)"""
        return self.lib.pi_node_global_us()

    def entry_str(self, entry):
        buf = ctypes.create_string_buffer(64)
        self.lib.raft_entry_str(entry, buf, len(buf))
        return buf.value.decode()

    def poll_log(self, timeout_ms):
        """LogEntry for the next applied (committed) entry, or None on timeout"""
        idx, term, entry, rep = _u16(), _u8(), _u8(), _bool()
        if self.lib.pi_node_poll_log(ctypes.byref(idx), ctypes.byref(term), ctypes.byref(entry),
                                     ctypes.byref(rep), timeout_ms) != 0:
            return None
        return LogEntry(idx.value, term.value, entry.value, rep.value, self.entry_str(entry.value))


def format_network(st):
    """getNetwork() as one line, same shape as the Nucleos' console"""
    def links(m):
        return ("A" if m & 1 else "-") + ("B" if m & 2 else "-")
    alive = "".join(str(n) if st.alive & (1 << (n - 1)) else "-" for n in (1, 2, 3))
    return (f"getNetwork alive={alive} cockpit={links(st.links[0])} steering={links(st.links[1])} "
            f"drivetrain={links(st.links[2])} | term {st.term} leader {NODE_NAMES.get(st.leader, '?')} "
            f"failure {int(st.failure)} | buses {links(st.bus_ok)} | drift {st.drift_ppb / 1000:.3f} ppm")
