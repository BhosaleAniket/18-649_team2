# Lab 5 extra credit: Redis + GUI (Team 2)

The Pi pushes the committed Raft log and a 1 s heartbeat over HTTPS to a small server (FastAPI + Redis) running on the laptop. The browser logs in and watches it live.

```
Pi (cockpit.py + cloud_pub.py) --HTTPS + token--> laptop :8443 (app) --> Redis (not exposed)
                                                         ^
                                     browser --HTTPS + login, websocket
```

## Files

| File | What it is |
|---|---|
| `docker-compose.yml` | Redis + app. Redis has a password and no published port |
| `server/app.py` | Login, ingest API for the Pi, websocket for the GUI |
| `server/static/index.html` | The GUI: nodes, CAN links, leader/time master, held events, live log, stale banner |
| `server/setup.py` | One-time setup: HTTPS cert + `.env` secrets |
| `pi/cloud_pub.py` | Pi side, stdlib only. Queue + uplink thread, `pi_node` adapter, `--demo` mode |

## Redis keys

- `vehicle:log`: stream (XADD). One row per committed entry: term, index, type, payload, global time, replayed.
- `vehicle:heartbeat`: hash with a 3 s TTL, written every 1 s. Holds the Pi wall clock, global time, FL/RL offset, and the node, link and leader status.
- `vehicle:meta` and `vehicle:heartbeat:last`: our additions. They hold the last stored index (used for resume and dedup), the held-event count, and the last-known heartbeat so the stale view still shows something.

## Laptop setup (once)

Needs Docker Desktop. Run this in the `lab5_extra_credit` folder. `LAPTOP_IP` is the address the Pi already uses to reach the laptop (the proxy link).

```powershell
docker build -t lab5-gui server
docker run --rm -it --user root -v "${PWD}:/host" lab5-gui python setup.py --ip LAPTOP_IP --out /host
docker compose up -d
```

1. `setup.py` asks for a GUI password and prints the Pi token.
2. Allow TCP 8443 inbound in Windows Firewall.
3. Open `https://localhost:8443` and log in as `team2`. The cert is self-signed, so the browser warns once.

## Pi setup

Copy `pi/cloud_pub.py`, `pi/cockpit.py` and `certs/server.crt` into `~/lab5/cockpit_pi/`. This `cockpit.py` is your Nov 17 version plus 3 small additions:
- the `--cloud-url`, `--cloud-token` and `--cloud-ca` options,
- creating the publisher in `__init__`,
- `push_entry()` in `raft_log_loop`.

Without `--cloud-url` it behaves exactly as before. Run it with:

```
sudo python3 cockpit.py --rt --cloud-url https://LAPTOP_IP:8443 --cloud-token TOKEN --cloud-ca server.crt
```

The global time on each row is when the Pi applied the entry. For replayed entries, that's when the Pi caught up, not when the leader wrote them.

## How it meets the handout

- **Asynchronous, best effort.** `push_entry()` only appends to a queue. One thread, dropped to `SCHED_OTHER` and kept off CPU 3, does the networking. Nothing runs on the CAN receive path. The scope capture of REF/APPEND/CTRL with the uplink running is still yours to record.
- **Uplink down.** The queue keeps the last 1,000 entries. Each send returns the server's last stored index and stream id, and the queue drops only what was acknowledged.
- **Replay without duplicates.** When the Pi reboots it catches up from index 1 with REPLAYED set. The server skips indexes it already has and counts the new replayed ones as "held events". A live index 1 means the whole car cold-started, so the old log is archived as `vehicle:log:runN`.
- **Secure.** Redis has a password and no port. The GUI uses HTTPS and a login (PBKDF2 hash, signed session cookie). The Pi ingest uses HTTPS, a bearer token, and pins the laptop's cert.

Tested here with real Redis: login (bad and good), the demo run, dedup after a replay, the queue draining after the server restarts, the stale banner, and a 401 without the token. Not tested: the Docker build itself (no Docker daemon here) and a run on the real Pi (tested with your `pi_node.py` types and a fake node).

## Paragraph for the demo (draft)

The GUI goes stale during a cockpit failure because the Pi is the vehicle's only network interface: every byte the server sees leaves through it. To keep the view alive, a second node on both CAN buses would need its own uplink, for example a second Pi, or an ESP32 or a CAN-to-Ethernet gateway listening to the APPEND and REF traffic. It would publish the committed entries it sees, and the server would merge both feeds by log index, which the dedup already does. The Nucleos can't do this: the F401RE has no Ethernet or Wi-Fi, so they can't reach the server at all, and they already spend their RAM on the 60 KB Raft log and their CPU on the 20 ms control loop. Adding a network stack and a radio module is a hardware change, not a code change.

## Laptop vs cloud

The handout says "a small cloud VM". This setup runs unchanged on one: copy the folder, run the same three commands with the VM's IP, and point the Pi at it. Ask the TA whether the laptop is accepted before the demo.
