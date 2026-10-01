/*
 * bridge.c - UDP wheel stream -> UART command link to the STM32 (Lab 2, Team 2)
 *
 *   Laptop proxy --UDP:8000--> [bridge on Pi] --/dev/serial0 115200--> STM32
 *                                             <--- status frames -----
 *
 * Build:  gcc -O2 -Wall -o bridge bridge.c
 * Run:    ./bridge            (stop wheel_monitor first - both use port 8000)
 *
 * While running, type in the terminal and press Enter:
 *   b   send one frame with a reserved flag bit set  (out-of-range command test)
 *   c   send one frame with a wrong CRC              (malformed frame test)
 *   q   quit
 *
 * Options:
 *   -d DEV   serial device            (default /dev/serial0)
 *   -L N     left-blinker button idx  (default 5, left paddle)
 *   -R N     right-blinker button idx (default 4, right paddle)
 *   -T N     self-test button idx     (default 0, A)
 *   -v       print every command and status frame
 *   -g       disable the GPIO test points
 *
 * Test points (BCM numbering, toggled on each event):
 *   UDP_RX  GPIO23  (header pin 16)  - each UDP state update received
 *   CMD_TX  GPIO24  (header pin 18)  - each command written to the UART
 *
 * The protocol itself is documented in link_proto.h.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "link_proto.h"

/* ---- wheel packet (276 bytes: uint32 counter + DIJOYSTATE2) ---- */
#define UDP_PORT    8000
#define PKT_LEN     276
#define OFF_COUNTER 0
#define OFF_LX      4    /* steering */
#define OFF_LY      8    /* throttle */
#define OFF_LRZ     24   /* brake    */
#define OFF_BTN     52   /* rgbButtons[128] */
#define NUM_BTN     128

/* ---- team calibration (measured 9/24 on the Windows proxy) ---- */
#define PEDAL_RELEASED  32767   /* raw value with the pedal up         */
#define PEDAL_PRESSED  -32768   /* raw value with the pedal floored    */

/* ---- timing ---- */
#define REFRESH_US        20000  /* resend last command if no UDP for 20 ms */
#define UDP_TIMEOUT_US   500000  /* proxy silent 500 ms -> stop sending     */
#define STATUS_TIMEOUT_US 66000  /* 3 missed STM32 heartbeats (3 x 22 ms)   */

/* ---- test point pins ---- */
#define TP_UDP_RX 23
#define TP_CMD_TX 24

static int btn_left = 5, btn_right = 4, btn_test = 0;
static int verbose = 0;
static volatile sig_atomic_t stop = 0;

/* ------------------------------------------------------------------ */
/* time                                                                */
/* ------------------------------------------------------------------ */
static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* ------------------------------------------------------------------ */
/* GPIO test points: direct BCM2711 register access via /dev/gpiomem   */
/* (fast, and works without root for users in the gpio group)          */
/* ------------------------------------------------------------------ */
static volatile uint32_t *gpio = NULL;
#define GPSET0 7    /* 0x1C / 4 */
#define GPCLR0 10   /* 0x28 / 4 */
#define GPLEV0 13   /* 0x34 / 4 */

static void gpio_make_output(int pin)
{
    int reg = pin / 10, sh = (pin % 10) * 3;
    gpio[reg] = (gpio[reg] & ~(7u << sh)) | (1u << sh);
    gpio[GPCLR0] = 1u << pin;
}

static int gpio_init(void)
{
    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (fd < 0)
        return -1;
    void *m = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED)
        return -1;
    gpio = (volatile uint32_t *)m;
    gpio_make_output(TP_UDP_RX);
    gpio_make_output(TP_CMD_TX);
    return 0;
}

static inline void tp_toggle(int pin)
{
    if (!gpio)
        return;
    if (gpio[GPLEV0] & (1u << pin))
        gpio[GPCLR0] = 1u << pin;
    else
        gpio[GPSET0] = 1u << pin;
}

/* ------------------------------------------------------------------ */
/* serial                                                              */
/* ------------------------------------------------------------------ */
static int serial_open(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        return -1;
    struct termios t;
    if (tcgetattr(fd, &t) < 0) {
        close(fd);
        return -1;
    }
    cfmakeraw(&t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &t) < 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* ------------------------------------------------------------------ */
/* command frames                                                      */
/* ------------------------------------------------------------------ */
struct cmd {
    int16_t steer;
    uint8_t thr, brk, flags;
};

static uint8_t tx_seq = 0;
static unsigned long n_cmd_sent = 0, n_cmd_dropped = 0, n_stale_flush = 0;

static void build_cmd(uint8_t *f, const struct cmd *c, uint8_t seq)
{
    f[0] = LINK_SYNC0;
    f[1] = LINK_SYNC1;
    f[2] = CMD_TYPE;
    f[3] = seq;
    f[4] = (uint8_t)((uint16_t)c->steer & 0xFF);
    f[5] = (uint8_t)((uint16_t)c->steer >> 8);
    f[6] = c->thr;
    f[7] = c->brk;
    f[8] = c->flags;
    f[9] = link_crc8(&f[2], CMD_LEN - 3);
}

/*
 * Write one frame. If the UART is backed up, older queued commands are
 * stale (an old throttle value is worse than none), so flush them and send
 * the newest. Never block.
 */
static int write_frame(int fd, const uint8_t *f, int len)
{
    int queued = 0;
    if (ioctl(fd, TIOCOUTQ, &queued) == 0 && queued > len) {
        tcflush(fd, TCOFLUSH);
        n_stale_flush++;
    }
    ssize_t w = write(fd, f, len);
    if (w != len) {
        n_cmd_dropped++;
        return -1;
    }
    return 0;
}

static void send_cmd(int fd, const struct cmd *c, const char *why)
{
    uint8_t f[CMD_LEN];
    build_cmd(f, c, tx_seq);
    if (write_frame(fd, f, CMD_LEN) == 0) {
        tp_toggle(TP_CMD_TX);
        n_cmd_sent++;
        if (verbose)
            printf("TX %-7s seq=%3u steer=%6d thr=%3u brk=%3u flags=0x%02x\n",
                   why, tx_seq, c->steer, c->thr, c->brk, c->flags);
    }
    tx_seq++;
}

/* ------------------------------------------------------------------ */
/* UDP packet -> command                                               */
/* ------------------------------------------------------------------ */
static int32_t rd32(const uint8_t *p)
{
    int32_t v;
    memcpy(&v, p, 4);
    return v;
}

static int in_int16(int32_t v) { return v >= -32768 && v <= 32767; }

static uint8_t pedal_to_u8(int32_t raw)
{
    int64_t num = (int64_t)(raw - PEDAL_RELEASED) * 255;
    int64_t den = (int64_t)PEDAL_PRESSED - PEDAL_RELEASED;
    int64_t p = (num + den / 2) / den;     /* rounded */
    if (p < 0) p = 0;
    if (p > 255) p = 255;
    return (uint8_t)p;
}

/* returns 0 on success, -1 if the packet is malformed / out of range */
static int decode_packet(const uint8_t *b, ssize_t n, struct cmd *c)
{
    if (n != PKT_LEN)
        return -1;
    int32_t lx = rd32(b + OFF_LX), ly = rd32(b + OFF_LY), lrz = rd32(b + OFF_LRZ);
    if (!in_int16(lx) || !in_int16(ly) || !in_int16(lrz))
        return -1;   /* reject, never wrap or clamp bad source data */

    c->steer = (int16_t)lx;
    c->thr = pedal_to_u8(ly);
    c->brk = pedal_to_u8(lrz);
    c->flags = 0;
    const uint8_t *btn = b + OFF_BTN;
    if (btn[btn_left])  c->flags |= CMD_FLAG_LEFT;
    if (btn[btn_right]) c->flags |= CMD_FLAG_RIGHT;
    if (btn[btn_test])  c->flags |= CMD_FLAG_TEST;
    return 0;
}

/* ------------------------------------------------------------------ */
/* status frames from the STM32                                        */
/* ------------------------------------------------------------------ */
static uint8_t sbuf[STATUS_LEN];
static int sidx = 0;
static unsigned long n_status = 0, n_status_bad = 0, n_rx_bytes = 0;
static uint64_t last_status_us = 0, max_status_gap_us = 0;
static int status_lost = 1;
static int last_state = -1, last_faults = -1;
static unsigned last_i[3] = {0, 0, 0};
static uint8_t last_seq = 0;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void handle_status(const uint8_t *f, uint64_t t)
{
    if (f[5] & ST_FAULT_RESERVED_MASK) {
        n_status_bad++;
        return;
    }
    n_status++;
    if (last_status_us && !status_lost) {
        uint64_t gap = t - last_status_us;
        if (gap > max_status_gap_us)
            max_status_gap_us = gap;
    }
    last_status_us = t;
    if (status_lost) {
        printf(">> STM32 heartbeat OK\n");
        status_lost = 0;
    }
    last_i[0] = rd16(f + 6);
    last_i[1] = rd16(f + 8);
    last_i[2] = rd16(f + 10);
    last_seq = f[3];

    int state = f[4], faults = f[5];
    if (state != last_state || faults != last_faults || verbose) {
        printf("RX status seq=%3u state=%s faults=0x%02x%s%s%s%s  current(mA) left_motor=%u right_motor=%u servo=%u\n",
               f[3], state == ST_STATE_NORMAL ? "NORMAL" : "ERROR ",
               faults,
               (faults & ST_FAULT_LINK) ? " [link]" : "",
               (faults & ST_FAULT_SELFTEST) ? " [selftest]" : "",
               (faults & ST_FAULT_BADCMD) ? " [badcmd]" : "",
               (faults & ST_FAULT_POWERUP) ? " [powerup]" : "",
               last_i[0], last_i[1], last_i[2]);
        last_state = state;
        last_faults = faults;
    }
}

static void status_byte(uint8_t b, uint64_t t)
{
    switch (sidx) {
    case 0:
        if (b == LINK_SYNC0)
            sbuf[sidx++] = b;
        return;
    case 1:
        if (b == LINK_SYNC1)
            sbuf[sidx++] = b;
        else
            sidx = (b == LINK_SYNC0) ? 1 : 0;
        return;
    case 2:
        if (b != STATUS_TYPE) {
            n_status_bad++;
            sidx = (b == LINK_SYNC0) ? 1 : 0;
            return;
        }
        break;
    default:
        break;
    }
    sbuf[sidx++] = b;
    if (sidx == STATUS_LEN) {
        sidx = 0;
        if (link_crc8(&sbuf[2], STATUS_LEN - 3) == sbuf[STATUS_LEN - 1])
            handle_status(sbuf, t);
        else
            n_status_bad++;
    }
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */
static void on_sigint(int s) { (void)s; stop = 1; }

static void usage(const char *p)
{
    fprintf(stderr, "usage: %s [-d dev] [-L idx] [-R idx] [-T idx] [-v] [-g]\n", p);
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/serial0";
    int use_gpio = 1, opt;

    while ((opt = getopt(argc, argv, "d:L:R:T:vgh")) != -1) {
        switch (opt) {
        case 'd': dev = optarg; break;
        case 'L': btn_left = atoi(optarg); break;
        case 'R': btn_right = atoi(optarg); break;
        case 'T': btn_test = atoi(optarg); break;
        case 'v': verbose = 1; break;
        case 'g': use_gpio = 0; break;
        default: usage(argv[0]); return 1;
        }
    }
    if (btn_left < 0 || btn_left >= NUM_BTN || btn_right < 0 || btn_right >= NUM_BTN ||
        btn_test < 0 || btn_test >= NUM_BTN) {
        fprintf(stderr, "button index must be 0..%d\n", NUM_BTN - 1);
        return 1;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    int ser = serial_open(dev);
    if (ser < 0) {
        fprintf(stderr, "cannot open %s: %s\n", dev, strerror(errno));
        return 1;
    }

    int udp = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_port = htons(UDP_PORT);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (udp < 0 || bind(udp, (struct sockaddr *)&a, sizeof a) < 0) {
        perror("UDP bind (is wheel_monitor still running?)");
        return 1;
    }

    if (use_gpio) {
        if (gpio_init() == 0)
            printf("test points: UDP_RX=GPIO%d (pin 16), CMD_TX=GPIO%d (pin 18)\n",
                   TP_UDP_RX, TP_CMD_TX);
        else
            printf("warning: /dev/gpiomem unavailable, test points disabled\n");
    }

    printf("bridge: UDP :%d -> %s @ %d baud | buttons L=%d R=%d test=%d\n",
           UDP_PORT, dev, LINK_BAUD, btn_left, btn_right, btn_test);
    printf("type b+Enter = bad-range frame, c+Enter = bad-CRC frame, q+Enter = quit\n");

    struct pollfd pf[3] = {
        {.fd = udp, .events = POLLIN},
        {.fd = ser, .events = POLLIN},
        {.fd = STDIN_FILENO, .events = POLLIN},
    };

    struct cmd cur = {0};
    int have_cmd = 0, link_active = 0;
    uint64_t last_udp_us = 0, last_send_us = 0, last_report_us = now_us();
    unsigned long n_udp = 0, n_udp_bad = 0, n_udp_prev = 0;
    uint8_t pkt[1024], rx[256];

    while (!stop) {
        uint64_t t = now_us();

        /* how long can we sleep before the next refresh is due? */
        int timeout_ms = 100;
        if (link_active) {
            uint64_t due = last_send_us + REFRESH_US;
            timeout_ms = (due > t) ? (int)((due - t + 999) / 1000) : 0;
        }

        int r = poll(pf, 3, timeout_ms);
        if (r < 0 && errno != EINTR) {
            perror("poll");
            break;
        }
        t = now_us();

        /* --- UDP wheel packet --- */
        if (pf[0].revents & POLLIN) {
            ssize_t n = recv(udp, pkt, sizeof pkt, 0);
            tp_toggle(TP_UDP_RX);
            struct cmd c;
            if (n > 0 && decode_packet(pkt, n, &c) == 0) {
                n_udp++;
                cur = c;
                have_cmd = 1;
                last_udp_us = t;
                if (!link_active)
                    printf(">> proxy stream up, sending commands\n");
                link_active = 1;
                send_cmd(ser, &cur, "udp");
                last_send_us = t;
            } else {
                n_udp_bad++;
            }
        }

        /* --- status bytes from the STM32 --- */
        if (pf[1].revents & POLLIN) {
            ssize_t n;
            while ((n = read(ser, rx, sizeof rx)) > 0) {
                n_rx_bytes += (unsigned long)n;
                for (ssize_t i = 0; i < n; i++)
                    status_byte(rx[i], t);
            }
        }

        /* --- keyboard: fault injection --- */
        if (pf[2].revents & POLLIN) {
            char line[64];
            if (!fgets(line, sizeof line, stdin)) {
                pf[2].fd = -1;   /* stdin closed, stop polling it */
            } else if (line[0] == 'q') {
                stop = 1;
            } else if (line[0] == 'b') {
                struct cmd bad = cur;
                bad.flags |= 0x80;   /* reserved bit -> out-of-range */
                send_cmd(ser, &bad, "BADRNG");
                printf("<< injected out-of-range command (reserved flag bit set)\n");
            } else if (line[0] == 'c') {
                uint8_t f[CMD_LEN];
                build_cmd(f, &cur, tx_seq++);
                f[9] ^= 0xFF;        /* corrupt CRC */
                write_frame(ser, f, CMD_LEN);
                tp_toggle(TP_CMD_TX);
                printf("<< injected bad-CRC command\n");
            }
        }

        /* --- proxy went silent: stop commanding so the STM32 fails safe --- */
        if (link_active && t - last_udp_us > UDP_TIMEOUT_US) {
            link_active = 0;
            printf(">> no UDP for %d ms, STOPPED sending (STM32 should fail safe)\n",
                   UDP_TIMEOUT_US / 1000);
        }

        /* --- periodic refresh (keeps commands <= 20 ms apart) --- */
        if (link_active && have_cmd && t - last_send_us >= REFRESH_US) {
            send_cmd(ser, &cur, "refresh");
            last_send_us = t;
        }

        /* --- STM32 heartbeat watch --- */
        if (!status_lost && t - last_status_us > STATUS_TIMEOUT_US) {
            status_lost = 1;
            printf(">> STM32 heartbeat LOST (no status for %d ms)\n",
                   STATUS_TIMEOUT_US / 1000);
        }

        /* --- 1 s summary --- */
        if (t - last_report_us >= 1000000) {
            printf("[1s] TX: udp=%lu/s bad_udp=%lu cmd_tx=%lu drop=%lu flush=%lu | "
                   "steer=%d thr=%u brk=%u flags=0x%02x\n",
                   n_udp - n_udp_prev, n_udp_bad, n_cmd_sent, n_cmd_dropped, n_stale_flush,
                   cur.steer, cur.thr, cur.brk, cur.flags);
            if (status_lost)
                printf("     RX: NO STATUS  rx_bytes=%lu status=%lu bad=%lu%s\n",
                       n_rx_bytes, n_status, n_status_bad,
                       n_rx_bytes == 0 ? "  <- nothing arriving on Pi RX (pin 10)" : "");
            else
                printf("     RX: seq=%3u state=%s faults=0x%02x%s%s%s%s | current(mA) left_motor=%u right_motor=%u servo=%u"
                       " | status=%lu bad=%lu maxgap=%.1fms\n",
                       last_seq, last_state == ST_STATE_NORMAL ? "NORMAL" : "ERROR",
                       last_faults,
                       (last_faults & ST_FAULT_LINK) ? " [link]" : "",
                       (last_faults & ST_FAULT_SELFTEST) ? " [selftest]" : "",
                       (last_faults & ST_FAULT_BADCMD) ? " [badcmd]" : "",
                       (last_faults & ST_FAULT_POWERUP) ? " [powerup]" : "",
                       last_i[0], last_i[1], last_i[2],
                       n_status, n_status_bad, max_status_gap_us / 1000.0);
            n_udp_prev = n_udp;
            max_status_gap_us = 0;
            last_report_us = t;
        }
    }

    printf("bridge: exiting\n");
    close(ser);
    close(udp);
    return 0;
}
