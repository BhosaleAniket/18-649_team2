"""
app.py - Lab 5 extra credit: GUI server for the vehicle's Raft log.

The Pi pushes over HTTPS (bearer token) -> this app writes Redis -> browsers
watch over a websocket after logging in. Redis is never reachable from
outside: it has no published port and needs a password.

Redis keys (handout table):
  vehicle:log        stream, one entry per committed log entry
  vehicle:heartbeat  hash, rewritten every 1 s by the Pi, TTL 3 s
  vehicle:meta       hash, ours: last_index, last_id, run, held, held_at_ms
  vehicle:heartbeat:last  ours: copy of the heartbeat without TTL
"""
import asyncio
import hashlib
import hmac
import json
import os
import time
from pathlib import Path

import redis.asyncio as aioredis
from fastapi import FastAPI, Form, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, RedirectResponse
from starlette.middleware.sessions import SessionMiddleware

LOG_KEY, HB_KEY, META_KEY = "vehicle:log", "vehicle:heartbeat", "vehicle:meta"
HB_LAST_KEY = "vehicle:heartbeat:last"   # ours: no TTL, so the GUI can show the last-known state while stale
HB_TTL_S = 3            # heartbeat key expires if the Pi stops for 3 s
STALE_MS = 2000         # GUI shows the stale banner after 2 s without a heartbeat
LOG_MAXLEN = 30000      # same as the Raft log_size
SNAPSHOT_ROWS = 300     # log rows sent to a browser when it connects

GUI_USER = os.environ["GUI_USER"]
GUI_PASS_HASH = os.environ["GUI_PASS_HASH"]          # pbkdf2_sha256:iters:salt_hex:hash_hex
INGEST_TOKEN = os.environ["INGEST_TOKEN"]
STATIC = Path(__file__).parent / "static"

r = aioredis.from_url(os.environ["REDIS_URL"], decode_responses=True)
app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
app.add_middleware(SessionMiddleware, secret_key=os.environ["SESSION_SECRET"],
                   https_only=True, same_site="strict", max_age=8 * 3600)


# ---------------------------------------------------------------- auth
def check_password(pw: str) -> bool:
    try:
        _, iters, salt, want = GUI_PASS_HASH.split(":")
        got = hashlib.pbkdf2_hmac("sha256", pw.encode(), bytes.fromhex(salt), int(iters)).hex()
        return hmac.compare_digest(got, want)
    except ValueError:
        return False


def check_token(req: Request):
    auth = req.headers.get("authorization", "")
    if not hmac.compare_digest(auth, "Bearer " + INGEST_TOKEN):
        raise HTTPException(401)


@app.get("/login")
async def login_page():
    return FileResponse(STATIC / "login.html")


@app.post("/login")
async def login(request: Request, username: str = Form(...), password: str = Form(...)):
    if hmac.compare_digest(username, GUI_USER) and check_password(password):
        request.session["user"] = username
        return RedirectResponse("/", status_code=303)
    await asyncio.sleep(1.0)                         # slow down guessing
    return RedirectResponse("/login?bad=1", status_code=303)


@app.post("/logout")
async def logout(request: Request):
    request.session.clear()
    return RedirectResponse("/login", status_code=303)


@app.get("/")
async def index(request: Request):
    if "user" not in request.session:
        return RedirectResponse("/login")
    return FileResponse(STATIC / "index.html")


# ---------------------------------------------------------------- ingest (Pi -> Redis)
@app.get("/api/ingest/last")
async def ingest_last(request: Request):
    check_token(request)
    m = await r.hgetall(META_KEY)
    return {"last_index": int(m.get("last_index", 0)), "last_id": m.get("last_id", "0-0")}


@app.post("/api/ingest/log")
async def ingest_log(request: Request):
    """Body: {"entries": [{index, term, entry, cls, type, payload, replayed, global_us, fresh}]}.
    Dedup by log index, so a resend after an uplink drop or the Pi's catch-up
    replay after a reboot never writes an entry twice."""
    check_token(request)
    body = await request.json()
    m = await r.hgetall(META_KEY)
    last_index = int(m.get("last_index", 0))
    last_id = m.get("last_id", "0-0")
    for e in body.get("entries", []):
        if e.get("fresh"):
            # Whole car cold-started: the log restarts at index 1. Keep the old run.
            if await r.exists(LOG_KEY):
                await r.rename(LOG_KEY, f"vehicle:log:run{m.get('run', 0)}")
            await r.hset(META_KEY, mapping={"last_index": 0, "held": 0, "held_at_ms": 0})
            await r.hincrby(META_KEY, "run", 1)
            m = await r.hgetall(META_KEY)
            last_index = 0
        idx = int(e["index"])
        if idx <= last_index:
            continue                                  # already stored
        fields = {
            "index": idx, "term": int(e["term"]), "entry": int(e["entry"]),
            "cls": e["cls"], "type": e["type"], "payload": e["payload"],
            "replayed": int(bool(e["replayed"])),
            "global_us": "" if e.get("global_us") is None else int(e["global_us"]),
        }
        last_id = await r.xadd(LOG_KEY, fields, maxlen=LOG_MAXLEN, approximate=True)
        last_index = idx
        if e["replayed"]:
            # Entries the leader held while the Pi was down and replayed on its return.
            if time.time() * 1000 - float(m.get("held_at_ms", 0)) > 5000:
                await r.hset(META_KEY, "held", 0)    # new outage: restart the count
            await r.hincrby(META_KEY, "held", 1)
            await r.hset(META_KEY, "held_at_ms", int(time.time() * 1000))
            m["held_at_ms"] = time.time() * 1000
        await r.hset(META_KEY, mapping={"last_index": last_index, "last_id": last_id})
    return {"last_index": last_index, "last_id": last_id}


@app.post("/api/ingest/heartbeat")
async def ingest_heartbeat(request: Request):
    check_token(request)
    hb = await request.json()
    hb["rx_ms"] = int(time.time() * 1000)
    async with r.pipeline(transaction=True) as p:
        p.delete(HB_KEY)
        p.hset(HB_KEY, mapping={k: json.dumps(v) for k, v in hb.items()})
        p.expire(HB_KEY, HB_TTL_S)
        p.delete(HB_LAST_KEY)
        p.hset(HB_LAST_KEY, mapping={k: json.dumps(v) for k, v in hb.items()})
        await p.execute()
    return {"ok": True}


# ---------------------------------------------------------------- browser websocket
def row(sid, f):
    f = dict(f)
    f["id"] = sid
    for k in ("index", "term", "entry", "replayed"):
        f[k] = int(f[k])
    f["global_us"] = int(f["global_us"]) if f.get("global_us") else None
    return f


async def state():
    hb = await r.hgetall(HB_KEY) or await r.hgetall(HB_LAST_KEY)
    hb = {k: json.loads(v) for k, v in hb.items()}
    m = await r.hgetall(META_KEY)
    now = int(time.time() * 1000)
    age = now - hb["rx_ms"] if hb else None
    return {
        "kind": "state",
        "hb": hb or None,
        "stale": (not hb) or age > STALE_MS,
        "age_ms": age,
        "held": int(m.get("held", 0)),
        "held_at_ms": int(float(m.get("held_at_ms", 0))),
        "run": int(m.get("run", 0)),
        "last_index": int(m.get("last_index", 0)),
        "server_ms": now,
    }


@app.websocket("/ws")
async def ws(sock: WebSocket):
    if "user" not in sock.session:
        await sock.close(code=4401)
        return
    await sock.accept()
    try:
        while True:                                   # restarts on a new run
            run = int(await r.hget(META_KEY, "run") or 0)
            rows = await r.xrevrange(LOG_KEY, count=SNAPSHOT_ROWS)
            await sock.send_json({"kind": "snapshot", "rows": [row(i, f) for i, f in reversed(rows)]})
            last = rows[0][0] if rows else "0-0"
            next_state = 0.0
            while True:
                got = await r.xread({LOG_KEY: last}, block=250, count=200)
                if got:
                    entries = got[0][1]
                    last = entries[-1][0]
                    await sock.send_json({"kind": "rows", "rows": [row(i, f) for i, f in entries]})
                if time.monotonic() >= next_state:
                    st = await state()
                    await sock.send_json(st)
                    next_state = time.monotonic() + 0.5
                    if st["run"] != run:
                        break
    except (WebSocketDisconnect, RuntimeError):
        pass
