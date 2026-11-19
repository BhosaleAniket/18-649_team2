/*
 * ref_stamp_test.c - is the CAN_RAW_RECV_OWN_MSGS kernel stamp the real end of the frame?
 * (catalog v0.7 open question; time sync on the Pi leader depends on it)
 *
 * Every period it raises a GPIO, sends one frame, waits for the frame's own echo (MSG_CONFIRM)
 * and lowers the GPIO. It prints, per frame, how long after the GPIO edge the kernel stamped
 * the echo. On the scope, measure the same thing for real: GPIO rising edge -> end of the frame
 * on the CAN line. The difference is the stamp error; the spread is its jitter.
 *
 *   sudo ./ref_stamp_test [-i canA] [-g 26] [-n 500] [-p 20] [-I 0x7E0]
 *
 * Run it with pi_node and cockpit.py STOPPED, and at least one Nucleo powered on that bus to
 * ACK the frame (otherwise the MCP2515 retransmits and there is no echo).
 */
#define _GNU_SOURCE
#include <errno.h>
#include <net/if.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "gpio.h"

static int64_t mono_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static void report(const char *what, double *v, int n)
{
    if (n == 0) { fprintf(stderr, "%s: no samples\n", what); return; }
    qsort(v, (size_t)n, sizeof *v, cmp_d);
    double sum = 0;
    for (int i = 0; i < n; i++) sum += v[i];
    fprintf(stderr, "%-34s n=%d  min %.1f  mean %.1f  p99 %.1f  max %.1f us\n",
           what, n, v[0], sum / n, v[(int)(0.99 * (n - 1))], v[n - 1]);
}

int main(int argc, char **argv)
{
    const char *iface = "canA";
    int pin = 26, count = 500, period_ms = 20;
    unsigned id = 0x7E0;
    int opt;
    while ((opt = getopt(argc, argv, "i:g:n:p:I:")) != -1) {
        switch (opt) {
        case 'i': iface = optarg; break;
        case 'g': pin = atoi(optarg); break;
        case 'n': count = atoi(optarg); break;
        case 'p': period_ms = atoi(optarg); break;
        case 'I': id = (unsigned)strtoul(optarg, NULL, 0); break;
        default:
            fprintf(stderr, "usage: %s [-i canA] [-g gpio] [-n count] [-p period_ms] [-I id]\n", argv[0]);
            return 2;
        }
    }

    struct sched_param sp = { .sched_priority = 70 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "warning: no SCHED_FIFO (%s); numbers will be noisier\n", strerror(errno));
    if (gpio_init() != 0) { fprintf(stderr, "cannot open /dev/gpiomem\n"); return 1; }
    gpio_output(pin);
    gpio_write(pin, 0);

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct ifreq ifr;
    memset(&ifr, 0, sizeof ifr);
    snprintf(ifr.ifr_name, sizeof ifr.ifr_name, "%s", iface);
    if (s < 0 || ioctl(s, SIOCGIFINDEX, &ifr) < 0) { fprintf(stderr, "no interface %s\n", iface); return 1; }
    struct sockaddr_can addr = { .can_family = AF_CAN, .can_ifindex = ifr.ifr_ifindex };
    int one = 1;
    struct timeval tv = { 0, 50000 };
    struct can_filter flt = { id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG };
    if (bind(s, (struct sockaddr *)&addr, sizeof addr) < 0 ||
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof flt) < 0 ||
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &one, sizeof one) < 0 ||
        setsockopt(s, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof one) < 0 ||
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv) < 0) {
        perror("socket setup");
        return 1;
    }

    double *stamp = calloc((size_t)count, sizeof(double));
    double *seen = calloc((size_t)count, sizeof(double));
    int ns = 0, lost = 0;
    printf("frame,stamp_after_edge_us,echo_read_after_edge_us\n");
    int64_t next = mono_ns();
    for (int i = 0; i < count; i++) {
        next += (int64_t)period_ms * 1000000LL;
        struct timespec ts = { (time_t)(next / 1000000000LL), (long)(next % 1000000000LL) };
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        struct can_frame f;
        memset(&f, 0, sizeof f);
        f.can_id = id;
        f.can_dlc = 8;
        memcpy(f.data, &i, sizeof i);

        gpio_write(pin, 1);                                   /* scope trigger: CH1 rising */
        int64_t t_edge = mono_ns();
        if (write(s, &f, sizeof f) != (ssize_t)sizeof f) {
            gpio_write(pin, 0);
            lost++;
            continue;
        }
        bool got = false;
        int64_t t_stamp = 0;
        while (!got) {
            struct can_frame r;
            char ctrl[CMSG_SPACE(sizeof(struct timespec)) + 64];
            struct iovec iov = { &r, sizeof r };
            struct msghdr m;
            memset(&m, 0, sizeof m);
            m.msg_iov = &iov;
            m.msg_iovlen = 1;
            m.msg_control = ctrl;
            m.msg_controllen = sizeof ctrl;
            if (recvmsg(s, &m, 0) < 0) break;                 /* 50 ms timeout: no echo */
            if (!(m.msg_flags & MSG_CONFIRM) || memcmp(r.data, &i, sizeof i)) continue;
            struct timespec ks, rt, mo;
            clock_gettime(CLOCK_REALTIME, &rt);
            clock_gettime(CLOCK_MONOTONIC, &mo);
            for (struct cmsghdr *c = CMSG_FIRSTHDR(&m); c; c = CMSG_NXTHDR(&m, c))
                if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
                    memcpy(&ks, CMSG_DATA(c), sizeof ks);
                    int64_t off = ((int64_t)rt.tv_sec - mo.tv_sec) * 1000000000LL + (rt.tv_nsec - mo.tv_nsec);
                    t_stamp = (int64_t)ks.tv_sec * 1000000000LL + ks.tv_nsec - off;
                    got = true;
                }
        }
        int64_t t_read = mono_ns();
        gpio_write(pin, 0);
        if (!got) { lost++; continue; }
        stamp[ns] = (double)(t_stamp - t_edge) / 1000.0;
        seen[ns] = (double)(t_read - t_edge) / 1000.0;
        printf("%d,%.1f,%.1f\n", i, stamp[ns], seen[ns]);
        ns++;
    }
    fflush(stdout);
    fprintf(stderr, "\n");
    report("kernel stamp after GPIO edge", stamp, ns);
    report("echo read after GPIO edge", seen, ns);
    fprintf(stderr, "frames without an echo: %d of %d\n", lost, count);
    fprintf(stderr, "Compare 'kernel stamp' with the scope: GPIO %d rising -> end of frame on %s.\n"
                    "If they differ, put (stamp mean - scope mean) into pi.ref_stamp_adjust_us in\n"
                    "raft_config.json. A wide min..max spread means the stamp is too jittery to use.\n",
            pin, iface);
    return 0;
}
