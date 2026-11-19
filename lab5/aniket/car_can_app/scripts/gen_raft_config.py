#!/usr/bin/env python3
"""Generate raft_config.h from raft_config.json (the one config shared with the Pi).

usage: gen_raft_config.py <raft_config.json> <out.h> [LEADER] [SEED]
  LEADER overrides default_leader: cockpit | steering | drivetrain | random | none
  SEED   overrides election_seed
"""
import json
import sys

NAMES = {"none": 0, "random": 0, "cockpit": 1, "steering": 2, "drivetrain": 3}


def main():
    src, out = sys.argv[1], sys.argv[2]
    leader = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] else None
    seed = sys.argv[4] if len(sys.argv) > 4 and sys.argv[4] else None
    with open(src) as f:
        c = json.load(f)

    dl = (leader or str(c["default_leader"])).strip().lower()
    if dl not in NAMES:
        sys.exit(f"default_leader / LEADER must be one of {sorted(NAMES)}, got {dl!r}")
    if seed is not None:
        c["election_seed"] = int(seed, 0)

    if c["N"] != 3:
        sys.exit("only N = 3 is supported")
    if not 1 <= c["log_size"] <= 65535:
        sys.exit("log_size must be 1..65535 (16-bit index)")

    vals = {
        "N_NODES": c["N"],
        "DEFAULT_LEADER": NAMES[dl],
        "HEARTBEAT_MS": c["heartbeat_ms"],
        "MISSED_HEARTBEATS": c["missed_heartbeats"],
        "ELECTION_JITTER_MS": c["election_timeout_jitter_ms"],
        "LOG_SIZE": c["log_size"],
        "DEBUG": 1 if c["debug"] in (True, "yes", "Yes", 1) else 0,
        "APPEND_PERIOD_MS": c["append_period_ms"],
        "ELECTION_SEED": f'{int(c["election_seed"]) & 0xFFFFFFFF}u',
        "CATCHUP_PACE_MS": c["catchup_pace_ms"],
        "REPORT_RETRY_MS": c["report_retry_ms"],
        "BOOT_LISTEN_MS": c["boot_listen_ms"],
        "RX_LATENCY_US": c["rx_latency_us"],
        "RELAY_DELAY_US": c["relay_delay_us"],
    }
    lines = [
        "/* Generated from raft_config.json by scripts/gen_raft_config.py - do not edit */",
        "#ifndef RAFT_CONFIG_H",
        "#define RAFT_CONFIG_H",
        f'#define RAFT_CFG_CATALOG_VERSION "{c.get("catalog_version", "?")}"',
        f'#define RAFT_CFG_DEFAULT_LEADER_NAME "{dl}"',
    ]
    lines += [f"#define RAFT_CFG_{k} {v}" for k, v in vals.items()]
    lines += ["#endif", ""]
    text = "\n".join(lines)
    try:
        with open(out) as f:
            if f.read() == text:
                return
    except FileNotFoundError:
        pass
    with open(out, "w") as f:
        f.write(text)


if __name__ == "__main__":
    main()
