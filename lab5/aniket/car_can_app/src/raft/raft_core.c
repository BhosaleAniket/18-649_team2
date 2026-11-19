/*
 * raft_core.c - pure Raft core for the Lab 5 cluster (catalog v0.8)
 *
 * Roles: follower, candidate, leader. Terms are u8 serial numbers.
 * Elections use pre-vote; a node whose own links are both down, or that is
 * silent (test button), never campaigns. One entry per APPEND; the leader
 * broadcasts new entries and paces catch-up per follower (REPLAYED flag).
 *
 * The state machine is the network graph (alive nodes, links per bus)
 * plus the committed cockpit state. Every node starts from the same full
 * graph (all nodes alive, all links up) and applies entries in index order.
 */
#include "raft_core.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define US_PER_MS 1000ull
#define TEST_PENDING_US (2000ull * US_PER_MS)
#define REPORT_GIVE_UP_US (3000ull * US_PER_MS)
#define EVENT_DUP_WINDOW 8

/* ------------------------------------------------------------------ */
/* Small helpers                                                      */
/* ------------------------------------------------------------------ */

static void dbg(raft_t *r, const char *fmt, ...)
{
    if (!r->cfg.debug || r->debug_cb == NULL) {
        return;
    }
    char line[112];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    r->debug_cb(r->ctx, line);
}

static uint32_t rnd(raft_t *r)
{
    uint32_t x = r->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    r->rng = x;
    return x;
}

static int popcount3(uint8_t m)
{
    return (m & 1) + ((m >> 1) & 1) + ((m >> 2) & 1);
}

static bool majority(const raft_t *r, int count)
{
    return count * 2 > r->cfg.n_nodes;
}

static uint8_t term_at(const raft_t *r, uint16_t i)
{
    return (i == 0 || i > r->last) ? 0 : r->log_term[i - 1];
}

static bool replayed_at(const raft_t *r, uint16_t i)
{
    uint16_t k = (uint16_t)(i % RAFT_REPLAY_RING);
    return (r->log_replayed[k >> 3] >> (k & 7)) & 1;
}

static void set_replayed(raft_t *r, uint16_t i, bool v)
{
    uint16_t k = (uint16_t)(i % RAFT_REPLAY_RING);
    uint8_t bit = (uint8_t)(1u << (k & 7));
    if (v) {
        r->log_replayed[k >> 3] |= bit;
    } else {
        r->log_replayed[k >> 3] &= (uint8_t)~bit;
    }
}

static uint64_t election_timeout_us(raft_t *r)
{
    uint32_t base = (uint32_t)r->cfg.missed_heartbeats * r->cfg.heartbeat_ms;
    uint32_t jit = r->cfg.election_jitter_ms ? rnd(r) % (r->cfg.election_jitter_ms + 1u) : 0;
    return (uint64_t)(base + jit) * US_PER_MS;
}

static void reset_election(raft_t *r)
{
    r->election_deadline = r->now + election_timeout_us(r);
}

static void emit(raft_t *r, raft_out_t *out, uint16_t base, const uint8_t d[8])
{
    if (r->silent || out == NULL || out->n >= RAFT_OUT_MAX) {
        return;
    }
    raft_frame_t *f = &out->f[out->n++];
    f->id = (uint16_t)(base + r->cfg.self_id);
    memcpy(f->data, d, 8);
    f->bus_mask = 0x03;
}

/* ------------------------------------------------------------------ */
/* State machine                                                      */
/* ------------------------------------------------------------------ */

static void apply_entry(raft_net_t *net, bool *failure, uint8_t e)
{
    uint8_t n = RAFT_ENT_NODE(e);
    switch (RAFT_ENT_TYPE(e)) {
    case RAFT_ET_COCKPIT:
        *failure = !RAFT_ENT_UP(e);
        break;
    case RAFT_ET_NODE:
        if (n >= 1 && n <= 3) {
            if (RAFT_ENT_UP(e)) {
                net->alive |= RAFT_NODEBIT(n);      /* addNode: alive, no links yet */
            } else {
                net->alive &= (uint8_t)~RAFT_NODEBIT(n);
            }
            net->link[n - 1] = 0;
        }
        break;
    case RAFT_ET_LINK:
        if (n >= 1 && n <= 3) {
            uint8_t b = RAFT_BUSBIT(RAFT_ENT_BUS(e));
            if (RAFT_ENT_UP(e)) {
                net->alive |= RAFT_NODEBIT(n);
                net->link[n - 1] |= b;
            } else {
                net->link[n - 1] &= (uint8_t)~b;
            }
        }
        break;
    default:                                    /* NOP, APP_EVENT: logged only */
        break;
    }
}

/* Graph after every entry in the log, committed or not (leader's view) */
static void projected(const raft_t *r, raft_net_t *net, bool *failure)
{
    *net = r->net;
    *failure = r->failure;
    for (uint16_t i = (uint16_t)(r->applied + 1); i <= r->last; i++) {
        apply_entry(net, failure, r->log_ent[i - 1]);
    }
}

static void apply_committed(raft_t *r)
{
    uint16_t upto = r->commit < r->last ? r->commit : r->last;
    while (r->applied < upto) {
        uint16_t i = ++r->applied;
        uint8_t e = r->log_ent[i - 1];
        apply_entry(&r->net, &r->failure, e);
        if (RAFT_ENT_TYPE(e) == RAFT_ET_EVENT && RAFT_ENT_CODE(e) == RAFT_EVT_TEST_BUTTON) {
            r->test_pending_until[RAFT_ENT_NODE(e) & 3] = 0;
        }
        for (int k = 0; k < RAFT_MAX_REPORTS; k++) {
            if (r->rep[k].used && r->rep[k].entry == e && r->rep[k].min_index < i) {
                r->rep[k].used = false;         /* our report is in the log */
            }
        }
        if (r->apply_cb) {
            r->apply_cb(r->ctx, i, r->log_term[i - 1], e, replayed_at(r, i));
        }
    }
}

/* ------------------------------------------------------------------ */
/* Role changes                                                       */
/* ------------------------------------------------------------------ */

static void become_follower(raft_t *r, uint8_t term, uint8_t leader)
{
    if (term != r->term) {
        r->term = term;
        r->voted_for = 0;
    }
    if (r->role != RAFT_FOLLOWER) {
        dbg(r, "raft: follower, term %u, leader %u", term, leader);
    }
    r->role = RAFT_FOLLOWER;
    r->prevote = false;
    r->leader = leader;
    reset_election(r);
}

/* A REF or APPEND from `sender` with term `t` that is not stale */
static void leader_contact(raft_t *r, uint8_t sender, uint8_t t)
{
    if (raft_term_newer(t, r->term) || r->role != RAFT_FOLLOWER || r->leader != sender) {
        if (r->role == RAFT_LEADER && t == r->term) {
            return;                             /* cannot happen: one leader per term */
        }
        become_follower(r, t, sender);
    }
    r->last_leader_contact = r->now ? r->now : 1;
    r->prevote = false;
    r->listening = false;
    r->claim_after_listen = false;
    r->wait_first_ref = false;
    reset_election(r);
}

static bool append_local(raft_t *r, uint8_t entry)
{
    if (r->last >= r->cfg.log_size) {
        dbg(r, "raft: log full (%u), entry 0x%02x dropped", r->last, entry);
        return false;
    }
    uint16_t i = ++r->last;
    r->log_term[i - 1] = r->term;
    r->log_ent[i - 1] = entry;
    set_replayed(r, i, false);
    r->match[r->cfg.self_id] = i;
    char s[40];
    raft_entry_str(entry, s, sizeof(s));
    dbg(r, "raft: leader appends idx %u term %u %s", i, r->term, s);
    return true;
}

static void become_leader(raft_t *r)
{
    r->role = RAFT_LEADER;
    r->leader = r->cfg.self_id;
    r->prevote = false;
    r->listening = false;
    r->claim_after_listen = false;
    r->wait_first_ref = false;
    for (int n = 1; n <= 3; n++) {
        r->next[n] = (uint16_t)(r->last + 1);
        r->match[n] = 0;
        r->next_catchup[n] = r->now;
    }
    r->bcast_next = (uint16_t)(r->last + 1);
    r->next_bcast_time = r->now;
    r->next_append_time = r->now;
    for (int n = 0; n < 4; n++) {
        r->last_report_valid[n] = false;
    }
    dbg(r, "raft: LEADER, term %u, last %u", r->term, r->last);
    append_local(r, RAFT_ENT_MAKE_NOP());       /* commits older terms */
}

static uint8_t last_term(const raft_t *r)
{
    return term_at(r, r->last);
}

static void send_req_vote(raft_t *r, raft_out_t *out, uint8_t term, bool prevote)
{
    uint8_t d[8] = { 0 };
    d[1] = term;
    raft_put16(&d[2], r->last);
    d[4] = last_term(r);
    d[5] = prevote ? 1 : 0;
    emit(r, out, RAFT_ID_REQ_VOTE, d);
}

static void start_prevote(raft_t *r, raft_out_t *out)
{
    r->prevote = true;
    r->prevote_term = (uint8_t)(r->term + 1);
    r->votes = RAFT_NODEBIT(r->cfg.self_id);
    dbg(r, "raft: pre-vote for term %u", r->prevote_term);
    send_req_vote(r, out, r->prevote_term, true);
    reset_election(r);
}

static void start_election(raft_t *r, raft_out_t *out)
{
    r->prevote = false;
    r->role = RAFT_CANDIDATE;
    r->term = (uint8_t)(r->term + 1);
    r->voted_for = r->cfg.self_id;
    r->votes = RAFT_NODEBIT(r->cfg.self_id);
    r->leader = 0;
    dbg(r, "raft: candidate, term %u", r->term);
    send_req_vote(r, out, r->term, false);
    reset_election(r);
}

/* ------------------------------------------------------------------ */
/* Leader: sending                                                    */
/* ------------------------------------------------------------------ */

static void send_append(raft_t *r, raft_out_t *out, uint16_t idx, bool replayed)
{
    /* idx = entry to carry, or 0 for an empty APPEND (prev = last) */
    uint8_t d[8] = { 0 };
    uint16_t prev = idx ? (uint16_t)(idx - 1) : r->last;
    uint16_t ref = idx ? idx : prev;
    uint16_t c = r->commit < ref ? r->commit : ref;
    uint16_t back = (uint16_t)(ref - c);

    d[0] = replayed ? RAFT_FLAG_REPLAYED : 0;
    d[1] = r->term;
    raft_put16(&d[2], prev);
    d[4] = term_at(r, prev);
    d[5] = idx ? r->log_ent[idx - 1] : 0;
    d[6] = idx ? r->log_term[idx - 1] : 0;
    d[7] = back >= RAFT_COMMIT_BACK_NONE ? RAFT_COMMIT_BACK_NONE : (uint8_t)back;
    emit(r, out, RAFT_ID_APPEND, d);
    r->next_append_time = r->now + (uint64_t)r->cfg.append_period_ms * US_PER_MS;
    r->commit_changed = false;
}

static void advance_commit(raft_t *r)
{
    for (uint16_t n = r->last; n > r->commit; n--) {
        if (term_at(r, n) != r->term) {
            break;                              /* only current-term entries commit by counting */
        }
        int count = 0;
        for (int f = 1; f <= r->cfg.n_nodes; f++) {
            if (f == r->cfg.self_id ? r->last >= n : r->match[f] >= n) {
                count++;
            }
        }
        if (majority(r, count)) {
            r->commit = n;
            r->commit_changed = true;
            apply_committed(r);
            break;
        }
    }
}

static bool uncommitted_has(const raft_t *r, uint8_t entry)
{
    for (uint16_t i = (uint16_t)(r->applied + 1); i <= r->last; i++) {
        if (r->log_ent[i - 1] == entry) {
            return true;
        }
    }
    return false;
}

static bool recent_has(const raft_t *r, uint8_t entry)
{
    uint16_t from = r->last > EVENT_DUP_WINDOW ? (uint16_t)(r->last - EVENT_DUP_WINDOW + 1) : 1;
    for (uint16_t i = from; i <= r->last; i++) {
        if (r->log_ent[i - 1] == entry) {
            return true;
        }
    }
    return false;
}

/* Leader: append an entry only if it changes something (MEMBER) or is new (EVENT) */
static void leader_append_if_needed(raft_t *r, uint8_t e)
{
    if (RAFT_ENT_TYPE(e) == RAFT_ET_EVENT) {
        if (!recent_has(r, e)) {
            append_local(r, e);
        }
        return;
    }
    raft_net_t before, after;
    bool fb, fa;
    projected(r, &before, &fb);
    after = before;
    fa = fb;
    apply_entry(&after, &fa, e);
    if (memcmp(&before, &after, sizeof(before)) != 0 || fa != fb) {
        append_local(r, e);
    }
}

/* Leader: make the log match what this node observes */
static void leader_reconcile(raft_t *r)
{
    raft_net_t p;
    bool fail;
    uint8_t self = r->cfg.self_id;

    projected(r, &p, &fail);
    if (!(p.alive & RAFT_NODEBIT(self))) {
        append_local(r, RAFT_ENT_MAKE_NODE(self, true));
        projected(r, &p, &fail);
    }
    if (r->own_links_known && p.link[self - 1] != r->own_links) {
        for (uint8_t bus = 1; bus <= 2; bus++) {
            uint8_t b = RAFT_BUSBIT(bus);
            if ((p.link[self - 1] & b) != (r->own_links & b)) {
                append_local(r, RAFT_ENT_MAKE_LINK(self, bus, (r->own_links & b) != 0));
            }
        }
    }
    if (r->obs_alive_known) {
        for (uint8_t n = 1; n <= r->cfg.n_nodes; n++) {
            if (n == self) {
                continue;
            }
            bool obs = (r->obs_alive & RAFT_NODEBIT(n)) != 0;
            bool gr = (p.alive & RAFT_NODEBIT(n)) != 0;
            if (obs != gr) {
                append_local(r, RAFT_ENT_MAKE_NODE(n, obs));
            }
        }
    }
    if (r->obs_cockpit_known && fail == r->obs_cockpit_ok) {
        append_local(r, RAFT_ENT_MAKE_COCKPIT(r->obs_cockpit_ok));
    }
    for (uint8_t z = 1; z <= 3; z++) {
        if (r->test_pending_until[z] == 0) {
            continue;
        }
        if (r->now >= r->test_pending_until[z]) {
            r->test_pending_until[z] = 0;       /* forgotten after 2 s */
            continue;
        }
        uint8_t e = RAFT_ENT_MAKE_EVENT(z, RAFT_EVT_TEST_BUTTON);
        if (!uncommitted_has(r, e)) {
            append_local(r, e);
        }
    }
}

static void leader_tick(raft_t *r, raft_out_t *out)
{
    leader_reconcile(r);
    advance_commit(r);                          /* single-node majority never happens with N = 3 */

    bool sent = false;
    uint64_t pace = (uint64_t)r->cfg.catchup_pace_ms * US_PER_MS;

    /* New entries, broadcast in order */
    if (r->bcast_next <= r->last && r->now >= r->next_bcast_time) {
        uint16_t i = r->bcast_next++;
        send_append(r, out, i, false);
        for (int f = 1; f <= 3; f++) {
            if (f != r->cfg.self_id && r->next[f] == i) {
                r->next[f] = (uint16_t)(i + 1);  /* optimistic; a failed ACK resets it */
            }
        }
        r->next_bcast_time = r->now + pace;
        sent = true;
    }

    /* Catch-up for followers that are behind, paced, marked REPLAYED */
    for (int f = 1; f <= r->cfg.n_nodes && !sent; f++) {
        if (f == r->cfg.self_id || r->next[f] >= r->bcast_next || r->next[f] == 0) {
            continue;
        }
        if (r->now < r->next_catchup[f] || r->now < r->next_bcast_time) {
            continue;
        }
        uint16_t i = r->next[f]++;
        send_append(r, out, i, true);
        r->next_catchup[f] = r->now + pace;
        r->next_bcast_time = r->now + pace;
        sent = true;
    }

    /* Empty APPEND: every append_period, and at once when the commit index moves */
    if (!sent && (r->now >= r->next_append_time || r->commit_changed)) {
        send_append(r, out, 0, false);
    }
}

/* ------------------------------------------------------------------ */
/* Client (followers)                                                 */
/* ------------------------------------------------------------------ */

static bool queue_report(raft_t *r, uint8_t entry)
{
    for (int k = 0; k < RAFT_MAX_REPORTS; k++) {
        if (r->rep[k].used && r->rep[k].entry == entry) {
            return true;                        /* already waiting */
        }
    }
    for (int k = 0; k < RAFT_MAX_REPORTS; k++) {
        if (!r->rep[k].used) {
            r->rep[k].used = true;
            r->rep[k].entry = entry;
            r->rep[k].id = ++r->report_id;
            r->rep[k].min_index = r->applied;
            r->rep[k].next_send = r->now;
            r->rep[k].give_up = r->now + REPORT_GIVE_UP_US;
            return true;
        }
    }
    return false;
}

static void client_tick(raft_t *r, raft_out_t *out)
{
    uint8_t self = r->cfg.self_id;

    /* Own links vs the committed graph: report differences */
    if (r->own_links_known && (r->net.alive & RAFT_NODEBIT(self)) &&
        r->net.link[self - 1] != r->own_links) {
        for (uint8_t bus = 1; bus <= 2; bus++) {
            uint8_t b = RAFT_BUSBIT(bus);
            if ((r->net.link[self - 1] & b) != (r->own_links & b)) {
                uint8_t e = RAFT_ENT_MAKE_LINK(self, bus, (r->own_links & b) != 0);
                uint8_t opposite = RAFT_ENT_MAKE_LINK(self, bus, (r->own_links & b) == 0);
                for (int k = 0; k < RAFT_MAX_REPORTS; k++) {
                    if (r->rep[k].used && r->rep[k].entry == opposite) {
                        r->rep[k].used = false; /* superseded */
                    }
                }
                queue_report(r, e);
            }
        }
    }

    for (int k = 0; k < RAFT_MAX_REPORTS; k++) {
        if (!r->rep[k].used) {
            continue;
        }
        if (r->now >= r->rep[k].give_up) {
            r->rep[k].used = false;
            continue;
        }
        if (r->leader == 0 || r->leader == self || r->now < r->rep[k].next_send) {
            continue;
        }
        uint8_t d[8] = { 0 };
        d[1] = r->rep[k].entry;
        raft_put16(&d[2], r->rep[k].id);
        d[4] = r->leader;
        emit(r, out, RAFT_ID_REPORT, d);
        r->rep[k].next_send = r->now + (uint64_t)r->cfg.report_retry_ms * US_PER_MS;
    }
}

/* ------------------------------------------------------------------ */
/* Frame handlers                                                     */
/* ------------------------------------------------------------------ */

static void send_ack(raft_t *r, raft_out_t *out, uint16_t match, bool ok, uint8_t leader)
{
    uint8_t d[8] = { 0 };
    d[1] = r->term;
    raft_put16(&d[2], match);
    d[4] = ok ? 1 : 0;
    d[5] = leader;
    emit(r, out, RAFT_ID_ACK, d);
}

static void on_ref(raft_t *r, uint8_t sender, const uint8_t *d)
{
    uint8_t t = d[1];
    if (raft_term_newer(r->term, t)) {
        return;                                 /* deposed leader */
    }
    leader_contact(r, sender, t);
}

static void on_req_vote(raft_t *r, uint8_t sender, const uint8_t *d, raft_out_t *out)
{
    if (r->silent) {
        return;
    }
    uint8_t t = d[1];
    uint16_t lidx = raft_get16(&d[2]);
    uint8_t lterm = d[4];
    bool pv = d[5] != 0;
    uint8_t my_lt = last_term(r);
    bool uptodate = raft_term_newer(lterm, my_lt) || (lterm == my_lt && lidx >= r->last);
    uint8_t v[8] = { 0 };

    if (pv) {
        uint64_t min_to = (uint64_t)r->cfg.missed_heartbeats * r->cfg.heartbeat_ms * US_PER_MS;
        bool leader_recent = r->role == RAFT_LEADER ||
                             (r->last_leader_contact != 0 && r->now - r->last_leader_contact < min_to);
        bool grant = raft_term_newer(t, r->term) && uptodate && !leader_recent;
        v[1] = t;
        v[2] = sender;
        v[3] = grant ? 1 : 0;
        v[4] = 1;
        emit(r, out, RAFT_ID_VOTE, v);
        return;
    }

    if (raft_term_newer(t, r->term)) {
        become_follower(r, t, 0);
    }
    bool grant = t == r->term && r->role != RAFT_LEADER &&
                 (r->voted_for == 0 || r->voted_for == sender) && uptodate;
    if (grant) {
        r->voted_for = sender;
        reset_election(r);
        dbg(r, "raft: vote for %u in term %u", sender, t);
    }
    v[1] = r->term;
    v[2] = sender;
    v[3] = grant ? 1 : 0;
    v[4] = 0;
    emit(r, out, RAFT_ID_VOTE, v);
}

static void on_vote(raft_t *r, uint8_t sender, const uint8_t *d, raft_out_t *out)
{
    uint8_t t = d[1];
    bool granted = d[3] != 0;
    bool pv = d[4] != 0;

    if (!pv && raft_term_newer(t, r->term)) {
        become_follower(r, t, 0);
        return;
    }
    if (d[2] != r->cfg.self_id || !granted || r->silent) {
        return;
    }
    if (pv) {
        if (r->prevote && r->role == RAFT_FOLLOWER && t == r->prevote_term) {
            r->votes |= RAFT_NODEBIT(sender);
            if (majority(r, popcount3(r->votes))) {
                start_election(r, out);
            }
        }
        return;
    }
    if (r->role == RAFT_CANDIDATE && t == r->term) {
        r->votes |= RAFT_NODEBIT(sender);
        if (majority(r, popcount3(r->votes))) {
            become_leader(r);
        }
    }
}

static void on_append(raft_t *r, uint8_t sender, const uint8_t *d, raft_out_t *out)
{
    uint8_t t = d[1];
    uint16_t prev = raft_get16(&d[2]);
    uint8_t pt = d[4];
    uint8_t e = d[5];
    uint8_t et = d[6];
    uint8_t back = d[7];
    bool replayed = (d[0] & RAFT_FLAG_REPLAYED) != 0;

    if (raft_term_newer(r->term, t)) {
        send_ack(r, out, r->last, false, sender);   /* tells a deposed leader our term */
        return;
    }
    leader_contact(r, sender, t);

    uint16_t ref = e ? (uint16_t)(prev + 1) : prev;
    bool ok = prev <= r->last && term_at(r, prev) == pt;
    if (ok && e && ref > r->cfg.log_size) {
        ok = false;
    }
    if (!ok) {
        uint16_t hint = prev <= r->last ? (uint16_t)(prev ? prev - 1 : 0) : r->last;
        send_ack(r, out, hint, false, sender);
        return;
    }
    if (e) {
        if (ref <= r->last && term_at(r, ref) != et) {
            r->last = (uint16_t)(ref - 1);          /* conflicting suffix goes */
        }
        if (ref > r->last) {
            r->log_term[ref - 1] = et;
            r->log_ent[ref - 1] = e;
            set_replayed(r, ref, replayed);
            r->last = ref;
        }
    }
    if (back != RAFT_COMMIT_BACK_NONE && back <= ref) {
        uint16_t c = (uint16_t)(ref - back);
        if (c > r->commit) {
            r->commit = c;
        }
    }
    apply_committed(r);
    if (e) {
        send_ack(r, out, ref, true, sender);
    }
}

static void on_ack(raft_t *r, uint8_t sender, const uint8_t *d)
{
    uint8_t t = d[1];
    uint16_t m = raft_get16(&d[2]);
    bool ok = d[4] != 0;

    if (raft_term_newer(t, r->term)) {
        become_follower(r, t, 0);
        return;
    }
    if (r->role != RAFT_LEADER || t != r->term || d[5] != r->cfg.self_id) {
        return;
    }
    if (ok) {
        if (m > r->match[sender]) {
            r->match[sender] = m;
        }
        if (r->next[sender] < m + 1) {
            r->next[sender] = (uint16_t)(m + 1);
        }
        advance_commit(r);
    } else {
        uint16_t n = (uint16_t)(m + 1);
        if (n > r->last + 1) {
            n = (uint16_t)(r->last + 1);
        }
        if (n < 1) {
            n = 1;
        }
        r->next[sender] = n;
        if (r->match[sender] >= n) {
            r->match[sender] = (uint16_t)(n - 1);
        }
        r->next_catchup[sender] = r->now;
    }
}

static void on_report(raft_t *r, uint8_t sender, const uint8_t *d, raft_out_t *out)
{
    uint8_t e = d[1];
    uint16_t rid = raft_get16(&d[2]);

    if (r->role == RAFT_LEADER) {
        if (r->last_report_valid[sender] && r->last_report_id[sender] == rid) {
            return;                             /* a retry of a report already appended */
        }
        uint8_t ty = RAFT_ENT_TYPE(e);
        if (ty < RAFT_ET_NOP || ty > RAFT_ET_EVENT) {
            return;
        }
        r->last_report_id[sender] = rid;
        r->last_report_valid[sender] = true;
        leader_append_if_needed(r, e);
        return;
    }
    if (d[4] == r->cfg.self_id) {
        uint8_t v[8] = { 0 };
        v[1] = r->leader;
        raft_put16(&v[2], rid);
        v[4] = sender;
        emit(r, out, RAFT_ID_REDIRECT, v);
    }
}

static void on_redirect(raft_t *r, const uint8_t *d)
{
    if (d[4] != r->cfg.self_id || r->role == RAFT_LEADER) {
        return;
    }
    uint8_t l = d[1];
    if (l >= 1 && l <= r->cfg.n_nodes && l != r->cfg.self_id) {
        r->leader = l;
        for (int k = 0; k < RAFT_MAX_REPORTS; k++) {
            if (r->rep[k].used && r->rep[k].id == raft_get16(&d[2])) {
                r->rep[k].next_send = r->now;   /* resend to the real leader now */
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public                                                             */
/* ------------------------------------------------------------------ */

void raft_init(raft_t *r, const raft_config_t *cfg, void *log_mem, uint64_t now_us,
               raft_apply_cb cb, raft_debug_cb dbgcb, void *ctx)
{
    memset(r, 0, sizeof(*r));
    r->cfg = *cfg;
    if (r->cfg.n_nodes == 0) {
        r->cfg.n_nodes = 3;
    }
    uint8_t *m = (uint8_t *)log_mem;
    r->log_term = m;
    r->log_ent = m + cfg->log_size;
    r->log_replayed = m + 2u * cfg->log_size;
    memset(r->log_replayed, 0, RAFT_REPLAY_RING / 8u);
    r->apply_cb = cb;
    r->debug_cb = dbgcb;
    r->ctx = ctx;
    r->now = now_us;
    r->rng = cfg->election_seed ^ (0x9E3779B1u * cfg->self_id) ^ 0x2545F491u;
    if (r->rng == 0) {
        r->rng = 0x1234567u;
    }

    r->net.alive = 0x07;                        /* every node starts from the full graph */
    for (int n = 0; n < 3; n++) {
        r->net.link[n] = 0x03;
    }
    r->role = RAFT_FOLLOWER;

    if (cfg->default_leader != 0) {
        if (cfg->default_leader == cfg->self_id) {
            r->listening = true;                /* listen before claiming term 1 */
            r->claim_after_listen = true;
            r->listen_until = now_us + (uint64_t)cfg->boot_listen_ms * US_PER_MS;
        } else {
            r->term = 1;
            r->leader = cfg->default_leader;
            r->wait_first_ref = true;           /* no campaigning before any leader is heard */
        }
    }
    reset_election(r);
}

void raft_on_frame(raft_t *r, uint8_t bus, uint16_t id, const uint8_t data[8],
                   uint64_t rx_time_us, raft_out_t *out)
{
    (void)bus;
    uint16_t base = raft_id_base(id);
    uint8_t sender = raft_id_sender(id);

    if (base == 0 || sender == r->cfg.self_id || sender > r->cfg.n_nodes) {
        return;
    }
    if (rx_time_us > r->now) {
        r->now = rx_time_us;
    }
    switch (base) {
    case RAFT_ID_REF:      on_ref(r, sender, data); break;
    case RAFT_ID_REQ_VOTE: on_req_vote(r, sender, data, out); break;
    case RAFT_ID_VOTE:     on_vote(r, sender, data, out); break;
    case RAFT_ID_APPEND:   on_append(r, sender, data, out); break;
    case RAFT_ID_ACK:      on_ack(r, sender, data); break;
    case RAFT_ID_REPORT:   if (!r->silent) { on_report(r, sender, data, out); } break;
    case RAFT_ID_REDIRECT: on_redirect(r, data); break;
    default: break;
    }
}

void raft_on_tick(raft_t *r, uint64_t now_us, raft_out_t *out)
{
    if (now_us > r->now) {
        r->now = now_us;
    }
    if (r->silent) {
        return;
    }
    if (r->role == RAFT_LEADER) {
        leader_tick(r, out);
        return;
    }
    if (r->listening) {
        if (r->now < r->listen_until) {
            return;
        }
        r->listening = false;
        if (r->claim_after_listen) {
            r->claim_after_listen = false;
            if (r->term == 0) {
                r->term = 1;
            }
            r->voted_for = r->cfg.self_id;
            become_leader(r);
            leader_tick(r, out);
            return;
        }
        reset_election(r);
    }
    if (!r->wait_first_ref && r->now >= r->election_deadline) {
        if (r->own_links_known && r->own_links == 0) {
            reset_election(r);                  /* isolated: cannot win, must not bump the term */
        } else {
            start_prevote(r, out);
        }
    }
    client_tick(r, out);
}

void raft_set_own_links(raft_t *r, uint8_t link_mask)
{
    r->own_links = link_mask & 0x03;
    r->own_links_known = true;
}

void raft_set_silent(raft_t *r, bool silent)
{
    if (silent == r->silent) {
        return;
    }
    r->silent = silent;
    if (silent) {
        if (r->role != RAFT_FOLLOWER) {
            dbg(r, "raft: silent (was %s), term %u", r->role == RAFT_LEADER ? "leader" : "candidate", r->term);
        }
        r->role = RAFT_FOLLOWER;
        r->prevote = false;
        if (r->leader == r->cfg.self_id) {
            r->leader = 0;
        }
        return;
    }
    /* Rejoin as a follower: listen first, adopt the current term, catch up */
    r->listening = true;
    r->claim_after_listen = false;
    r->listen_until = r->now + (uint64_t)r->cfg.boot_listen_ms * US_PER_MS;
    r->leader = 0;
    r->last_leader_contact = 0;
    dbg(r, "raft: rejoining, listening %u ms", r->cfg.boot_listen_ms);
}

void raft_observe_peers(raft_t *r, uint8_t alive_mask)
{
    r->obs_alive = (uint8_t)((alive_mask | RAFT_NODEBIT(r->cfg.self_id)) & 0x07);
    r->obs_alive_known = true;
}

void raft_observe_cockpit(raft_t *r, bool ok)
{
    r->obs_cockpit_ok = ok;
    r->obs_cockpit_known = true;
}

void raft_note_test_fault(raft_t *r, uint8_t zone)
{
    if (zone >= 1 && zone <= 3) {
        r->test_pending_until[zone] = r->now + TEST_PENDING_US;
    }
}

bool raft_report(raft_t *r, uint8_t entry)
{
    if (r->silent) {
        return false;
    }
    if (r->role == RAFT_LEADER) {
        leader_append_if_needed(r, entry);
        return true;
    }
    return queue_report(r, entry);
}

raft_net_t raft_get_network(const raft_t *r) { return r->net; }
bool raft_failure_mode(const raft_t *r) { return r->failure; }
bool raft_is_leader(const raft_t *r) { return r->role == RAFT_LEADER && !r->silent; }
uint8_t raft_leader(const raft_t *r) { return r->leader; }
uint8_t raft_term(const raft_t *r) { return r->term; }
uint8_t raft_role(const raft_t *r) { return r->role; }
uint16_t raft_last_index(const raft_t *r) { return r->last; }
uint16_t raft_commit_index(const raft_t *r) { return r->commit; }

bool raft_log_entry(const raft_t *r, uint16_t index, uint8_t *term, uint8_t *entry)
{
    if (index == 0 || index > r->last) {
        return false;
    }
    *term = r->log_term[index - 1];
    *entry = r->log_ent[index - 1];
    return true;
}
