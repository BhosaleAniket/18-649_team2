/*
 * config.c - minimal JSON reader for raft_config.json (objects, strings, numbers, booleans;
 * arrays are skipped). Nested keys are flattened to "a.b".
 */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

#define MAX_KV 96

typedef struct { char key[64]; char val[64]; } kv_t;
static kv_t kv[MAX_KV];
static int nkv;

static void store(const char *path, const char *val)
{
    if (!path || nkv >= MAX_KV) return;
    snprintf(kv[nkv].key, sizeof kv[nkv].key, "%s", path);
    snprintf(kv[nkv].val, sizeof kv[nkv].val, "%s", val);
    nkv++;
}

static void skip_ws(const char **p)
{
    while (**p && isspace((unsigned char)**p)) (*p)++;
}

static int parse_string(const char **p, char *out, size_t n)
{
    if (**p != '"') return -1;
    (*p)++;
    size_t i = 0;
    while (**p && **p != '"') {
        char c = **p;
        if (c == '\\' && (*p)[1]) { (*p)++; c = **p; }
        if (i + 1 < n) out[i++] = c;
        (*p)++;
    }
    if (**p != '"') return -1;
    (*p)++;
    out[i] = 0;
    return 0;
}

static int parse_value(const char **p, const char *path);

static int parse_object(const char **p, const char *path)
{
    (*p)++;
    skip_ws(p);
    if (**p == '}') { (*p)++; return 0; }
    for (;;) {
        char k[64], sub[128];
        skip_ws(p);
        if (parse_string(p, k, sizeof k)) return -1;
        skip_ws(p);
        if (**p != ':') return -1;
        (*p)++;
        skip_ws(p);
        if (path && path[0]) snprintf(sub, sizeof sub, "%s.%s", path, k);
        else snprintf(sub, sizeof sub, "%s", k);
        if (parse_value(p, path ? sub : NULL)) return -1;
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (**p == '}') { (*p)++; return 0; }
        return -1;
    }
}

static int parse_array(const char **p)
{
    (*p)++;
    skip_ws(p);
    if (**p == ']') { (*p)++; return 0; }
    for (;;) {
        if (parse_value(p, NULL)) return -1;
        skip_ws(p);
        if (**p == ',') { (*p)++; skip_ws(p); continue; }
        if (**p == ']') { (*p)++; return 0; }
        return -1;
    }
}

static int parse_value(const char **p, const char *path)
{
    skip_ws(p);
    if (**p == '{') return parse_object(p, path);
    if (**p == '[') return parse_array(p);
    char buf[64];
    if (**p == '"') {
        if (parse_string(p, buf, sizeof buf)) return -1;
    } else {
        size_t i = 0;
        while (**p && !isspace((unsigned char)**p) && **p != ',' && **p != '}' && **p != ']') {
            if (i + 1 < sizeof buf) buf[i++] = **p;
            (*p)++;
        }
        buf[i] = 0;
        if (i == 0) return -1;
    }
    store(path, buf);
    return 0;
}

static const char *get(const char *key)
{
    for (int i = 0; i < nkv; i++)
        if (strcmp(kv[i].key, key) == 0) return kv[i].val;
    return NULL;
}

static long get_int(const char *key, long def)
{
    const char *v = get(key);
    return v ? strtol(v, NULL, 0) : def;
}

static int is_true(const char *v)
{
    return v && (!strcmp(v, "true") || !strcmp(v, "yes") || !strcmp(v, "1"));
}

/* node name ("cockpit") or number ("1") -> id; "none"/"random" -> 0; -1 if unknown.
 * Names come from "nodes": { "1": "cockpit", ... } (catalog v0.8 layout). */
static int node_id(const char *name)
{
    if (!name) return -1;
    if (!strcmp(name, "none") || !strcmp(name, "random")) return 0;
    if (isdigit((unsigned char)name[0])) return atoi(name);
    for (int i = 0; i < nkv; i++)
        if (!strncmp(kv[i].key, "nodes.", 6) && !strcmp(kv[i].val, name))
            return atoi(kv[i].key + 6);
    return -1;
}

int config_load(const char *path, raft_config_t *rc, pi_extra_t *px, char *err, size_t errlen)
{
    FILE *f = fopen(path, "rb");
    if (!f) { snprintf(err, errlen, "cannot open %s", path); return -1; }
    static char text[16384];
    size_t n = fread(text, 1, sizeof text - 1, f);
    fclose(f);
    text[n] = 0;

    nkv = 0;
    const char *p = text;
    skip_ws(&p);
    if (*p != '{' || parse_value(&p, "")) {
        snprintf(err, errlen, "%s: JSON parse error near byte %ld", path, (long)(p - text));
        return -1;
    }

    const char *ver = get("catalog_version");
    if (ver && strcmp(ver, RAFT_CATALOG_VERSION) != 0)
        fprintf(stderr, "[config] warning: %s is catalog v%s, this build is v%s\n",
                path, ver, RAFT_CATALOG_VERSION);

    memset(rc, 0, sizeof *rc);
    rc->n_nodes            = (uint8_t)get_int("N", 3);
    rc->heartbeat_ms       = (uint16_t)get_int("heartbeat_ms", 20);
    rc->missed_heartbeats  = (uint8_t)get_int("missed_heartbeats", 3);
    rc->election_jitter_ms = (uint16_t)get_int("election_timeout_jitter_ms", 39);
    rc->log_size           = (uint16_t)get_int("log_size", 30000);
    rc->debug              = is_true(get("debug"));
    rc->append_period_ms   = (uint16_t)get_int("append_period_ms", 100);
    rc->election_seed      = (uint32_t)get_int("election_seed", 0);
    rc->catchup_pace_ms    = (uint16_t)get_int("catchup_pace_ms", 2);
    rc->report_retry_ms    = (uint16_t)get_int("report_retry_ms", 50);
    rc->boot_listen_ms     = (uint16_t)get_int("boot_listen_ms", 100);

    const char *leader = getenv("LEADER");
    int dl = node_id(leader ? leader : get("default_leader"));
    if (dl < 0 || dl > 3) { snprintf(err, errlen, "default_leader / LEADER not a known node"); return -1; }
    rc->default_leader = (uint8_t)dl;

    const char *seed = getenv("ELECTION_SEED");
    if (seed) rc->election_seed = (uint32_t)strtoul(seed, NULL, 0);

    int self = node_id(get("pi.self") ? get("pi.self") : "cockpit");
    if (self < 1 || self > 3) { snprintf(err, errlen, "pi.self is not a known node"); return -1; }
    rc->self_id = (uint8_t)self;

    memset(px, 0, sizeof *px);
    px->ref_stamp_adjust_us = (int32_t)get_int("pi.ref_stamp_adjust_us", 0);
    px->rx_skew_b_us        = (int32_t)get_int("pi.rx_skew_b_us", 0);
    px->tsync_trace         = is_true(get("pi.tsync_trace"));
    const char *skew = getenv("PI_RX_SKEW_B_US");
    if (skew) px->rx_skew_b_us = (int32_t)strtol(skew, NULL, 0);
    if (getenv("PI_TSYNC_TRACE")) px->tsync_trace = true;
    px->pps_gpio            = (int)get_int("pi.pps_gpio", 16);
    px->rt_priority         = (int)get_int("pi.rt_priority", 60);
    px->cpu                 = (int)get_int("pi.cpu", -1);
    px->rx_latency_us       = (uint32_t)get_int("pi.rx_latency_us", get_int("rx_latency_us", 0));
    px->relay_delay_us      = (uint32_t)get_int("relay_delay_us", 0);
    return 0;
}
