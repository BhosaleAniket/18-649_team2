#!/usr/bin/env python3
"""Capture several Nucleo consoles at once into zephyrproject\\logs, for review.

    capture.bat                      (auto-detects every ST-LINK port)
    capture.bat COM5=N2 COM7=N3      (name the ports yourself)

Writes, per run (timestamps are this PC's clock, so boards line up):
    logs\\<run>_N2.log, logs\\<run>_N3.log     one file per board
    logs\\<run>_all.log                        every line from every board, interleaved
    logs\\latest.txt                           name of the current run

Type a note and press Enter at any time (e.g. "pulled B on N3"): it goes into
every log as a === MARK === line. Ctrl+C stops. Close PuTTY first (a COM port
can only be open in one program). If a board resets or is reflashed, its port
is reopened automatically.
"""
import datetime
import os
import sys
import threading
import time

import serial
from serial.tools import list_ports

BAUD = 115200
ST_VID = 0x0483
LOG_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs"))

lock = threading.Lock()
stop = threading.Event()


def now():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


class Sink:
    def __init__(self, run):
        self.run = run
        self.all = open(os.path.join(LOG_DIR, f"{run}_all.log"), "a", encoding="utf-8", buffering=1)
        self.files = {}

    def write(self, tag, line):
        stamp = now()
        with lock:
            f = self.files.get(tag)
            if f is None:
                f = open(os.path.join(LOG_DIR, f"{self.run}_{tag}.log"), "a", encoding="utf-8", buffering=1)
                self.files[tag] = f
            f.write(f"[{stamp}] {line}\n")
            self.all.write(f"[{stamp}] {tag:>4} | {line}\n")

    def mark(self, text):
        stamp = now()
        with lock:
            for f in list(self.files.values()) + [self.all]:
                f.write(f"[{stamp}] === MARK: {text} ===\n")


def reader(port, tag, sink):
    buf = b""
    ser = None
    announced = False
    while not stop.is_set():
        if ser is None:
            try:
                ser = serial.Serial(port, BAUD, timeout=0.1)
                sink.write(tag, f"--- capture: {port} open ---")
                if not announced:
                    print(f"  {port} -> {tag}")
                    announced = True
            except serial.SerialException:
                time.sleep(1.0)                 # board resetting / being flashed
                continue
        try:
            # return as soon as anything has arrived (at most 0.1 s wait), so a
            # line's timestamp is within a few ms of when the board sent it
            data = ser.read(max(1, ser.in_waiting))
        except serial.SerialException:
            sink.write(tag, f"--- capture: {port} lost (reset/flash?) ---")
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            continue
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            sink.write(tag, raw.rstrip(b"\r").decode("utf-8", errors="replace"))
    if ser is not None:
        ser.close()


def stlink_ports():
    return [p.device for p in list_ports.comports() if p.vid == ST_VID]


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    pairs = []
    for a in sys.argv[1:]:
        port, _, tag = a.partition("=")
        pairs.append((port, tag or port))
    if not pairs:
        pairs = [(p, p) for p in stlink_ports()]
    if not pairs:
        print("No ST-LINK serial ports found. Pass them explicitly: capture.bat COM5=N2 COM7=N3")
        return 1

    run = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    sink = Sink(run)
    with open(os.path.join(LOG_DIR, "latest.txt"), "w") as f:
        f.write(run + "\n")
    print(f"Logging to {LOG_DIR}\\{run}_*.log")
    for port, tag in pairs:
        threading.Thread(target=reader, args=(port, tag, sink), daemon=True).start()
    print("Type a note + Enter to mark the logs; Ctrl+C to stop.")
    try:
        while True:
            text = input()
            if text.strip():
                sink.mark(text.strip())
                print(f"  marked: {text.strip()}")
    except (KeyboardInterrupt, EOFError):
        pass
    stop.set()
    time.sleep(0.3)
    print("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
