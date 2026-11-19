/*
 * host_test.c - Pi-side checks that run on any PC (make host-test):
 *   - catalog.h / raft_proto.h agree on IDs, senders and the entry byte (catalog v0.8)
 *   - config.c reads raft_config.json in the v0.8 layout, and the LEADER override works
 * The Raft core itself is covered by Aniket's simulator: make sim
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "catalog.h"
#include "config.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static void test_catalog(void)
{
    CHECK(RAFT_ENT_MAKE_COCKPIT(0) == 0x40 && RAFT_ENT_MAKE_COCKPIT(1) == 0x41);
    CHECK(RAFT_ENT_MAKE_NODE(3, 0) == 0x78);
    CHECK(RAFT_ENT_MAKE_LINK(2, 1, 0) == 0x90);
    CHECK(RAFT_ENT_MAKE_EVENT(2, 3) == 0xB3);
    CHECK(RAFT_ENT_MAKE_NOP() == 0x20);
    for (uint16_t s = 1; s <= 3; s++) {
        uint16_t ids[] = { RAFT_ID_REF, RAFT_ID_REQ_VOTE, RAFT_ID_VOTE, RAFT_ID_APPEND,
                           RAFT_ID_ACK, RAFT_ID_REPORT, RAFT_ID_REDIRECT };
        for (unsigned i = 0; i < sizeof ids / sizeof ids[0]; i++) {
            CHECK(cat_is_raft(ids[i] + s));
            CHECK(cat_source(ids[i] + s, NULL) == s);
            CHECK(raft_id_base(ids[i] + s) == ids[i]);
        }
    }
    uint8_t fault[8] = { 0, 2, FAULT_TEST_BUTTON, 3, 0, 0, 0, 0 };
    CHECK(!cat_is_raft(ID_CTRL) && !cat_is_raft(ID_FAULT) && !cat_is_raft(ID_HB_N1) &&
          !cat_is_raft(ID_BLINK) && !cat_is_raft(ID_FORCE) && !cat_is_raft(RAFT_ID_REF));
    CHECK(cat_source(ID_HB_N2, NULL) == 2 && cat_source(ID_FAULT, fault) == 3);
    char s[48];
    raft_entry_str(0x90, s, sizeof s);
    CHECK(strcmp(s, "MEMBER LINK steering A DOWN") == 0);
}

static void test_config(const char *path)
{
    raft_config_t rc;
    pi_extra_t px;
    char err[160];
    CHECK(config_load(path, &rc, &px, err, sizeof err) == 0);
    CHECK(rc.n_nodes == 3 && rc.self_id == 1 && rc.default_leader == 1);
    CHECK(rc.heartbeat_ms == 20 && rc.missed_heartbeats == 3 && rc.election_jitter_ms == 39);
    CHECK(rc.log_size == 30000 && rc.debug && rc.append_period_ms == 100 && rc.boot_listen_ms == 100);
    CHECK(rc.catchup_pace_ms == 2 && rc.report_retry_ms == 50);
    CHECK(px.relay_delay_us == 550 && px.pps_gpio == 16);
#ifndef _WIN32
    setenv("LEADER", "random", 1);
    setenv("ELECTION_SEED", "3", 1);
    CHECK(config_load(path, &rc, &px, err, sizeof err) == 0 && rc.default_leader == 0 && rc.election_seed == 3);
    setenv("LEADER", "drivetrain", 1);
    CHECK(config_load(path, &rc, &px, err, sizeof err) == 0 && rc.default_leader == 3);
    setenv("LEADER", "bogus", 1);
    CHECK(config_load(path, &rc, &px, err, sizeof err) != 0);
    unsetenv("LEADER");
    unsetenv("ELECTION_SEED");
#endif
}

int main(int argc, char **argv)
{
    test_catalog();
    test_config(argc > 1 ? argv[1] : "raft_config.json");
    printf(fails ? "%d check(s) FAILED\n" : "all checks passed\n", fails);
    return fails != 0;
}
