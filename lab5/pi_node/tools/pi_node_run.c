/*
 * pi_node_run.c - runs pi_node on its own (no wheel, no cockpit.py) for bench tests.
 *
 *   sudo ./pi_node_run [raft_config.json] [--ok]
 *
 * Prints the status once a second and every applied log entry. Console keys (then Enter):
 *   o   toggle cockpit OK / ERROR (starts in ERROR unless --ok)
 *   t   toggle the cockpit test-button fault (node goes silent in Raft, HB_N1 stops)
 *   e N report APP_EVENT code N (0..7) from the cockpit
 *   q   quit
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>
#include "pi_node.h"
#include "catalog.h"

static volatile sig_atomic_t stop;
static void on_sig(int s) { (void)s; stop = 1; }

static const char *const NAME[4] = { "-", "cockpit", "steering", "drivetrain" };

static void print_status(void)
{
    raft_net_t net;
    uint8_t leader = 0, term = 0, bus = 0;
    bool failure = false;
    int32_t drift = 0;
    pi_node_status(&net, &leader, &term, &failure, &bus, &drift);
    printf("[status] term %u leader %s failure %s buses %c%c | graph:", term, NAME[leader & 3],
           failure ? "YES" : "no", (bus & 1) ? 'A' : '-', (bus & 2) ? 'B' : '-');
    for (int n = 1; n <= 3; n++)
        printf(" %s=%s/%c%c", NAME[n], (net.alive & (1u << (n - 1))) ? "up" : "down",
               (net.link[n - 1] & 1) ? 'A' : '-', (net.link[n - 1] & 2) ? 'B' : '-');
    printf(" drift %.3f ppm\n", drift / 1000.0);
    fflush(stdout);
}

int main(int argc, char **argv)
{
    const char *cfg = "raft_config.json";
    bool ok = false, test = false;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--ok")) ok = true;
        else cfg = argv[i];
    }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    if (pi_node_start(cfg) != 0) {
        fprintf(stderr, "pi_node_start failed\n");
        return 1;
    }
    pi_node_set_cockpit_state(ok, ok ? 0 : ERR_CTRL_LOST);

    time_t last = 0;
    unsigned app_frames = 0;
    bool console = true;                     /* false once stdin is closed (unattended run) */
    while (!stop) {
        fd_set rf;
        FD_ZERO(&rf);
        if (console) FD_SET(0, &rf);
        struct timeval tv = { 0, 50000 };
        if (select(console ? 1 : 0, &rf, NULL, NULL, &tv) > 0) {
            char line[64];
            if (!fgets(line, sizeof line, stdin)) { console = false; continue; }
            if (line[0] == 'q') break;
            if (line[0] == 'o') ok = !ok;
            if (line[0] == 't') test = !test;
            if (line[0] == 'e') printf("[event] report %s\n",
                                       pi_node_report_event((uint8_t)atoi(line + 1)) ? "refused" : "queued");
            uint8_t err = (uint8_t)((ok ? 0 : ERR_CTRL_LOST) | (test ? ERR_TEST : 0));
            pi_node_set_cockpit_state(ok && !test, err);
        }
        uint16_t id;
        uint8_t d[8], bus;
        while (pi_node_poll_app(&id, d, &bus, 0) == 0) app_frames++;
        uint16_t idx;
        uint8_t term, entry;
        bool rep;
        while (pi_node_poll_log(&idx, &term, &entry, &rep, 0) == 0) { /* printed by pi_node (debug) */ }
        time_t now = time(NULL);
        if (now != last) {
            last = now;
            print_status();
            printf("[status] app frames received in the last second: %u\n", app_frames);
            app_frames = 0;
        }
    }
    pi_node_stop();
    return 0;
}
