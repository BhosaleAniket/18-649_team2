

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/atomic.h>

#include "canbus.h"
#include "catalog.h"
#include "config.h"
#include "node.h"
#include "state.h"
#include "wcet.h"

static const struct gpio_dt_spec led_deg[NUM_BUSES] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_deg_a), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_deg_b), gpios),
};

static const struct device *const wdt_dev = DEVICE_DT_GET(DT_NODELABEL(iwdg));
static int wdt_channel = -1;
static atomic_t wdt_bits;

void wdt_checkin(uint32_t bit)
{
    atomic_or(&wdt_bits, (atomic_val_t)bit);
}

#if NODE_ID == 2
static const uint8_t other_zones[2] = { ZONE_N1, ZONE_N3 };
#else
static const uint8_t other_zones[2] = { ZONE_N1, ZONE_N2 };
#endif

/* ------------------------------------------------------------------ */
/* HB_TX                                                              */
/* ------------------------------------------------------------------ */

static uint8_t zones_heard_bits(uint32_t now)
{
    uint8_t bits = (uint8_t)(1u << NODE_ID);
    for (int i = 0; i < 2; i++) {
        struct peer_info p;
        state_get_peer(other_zones[i], &p);
        if (p.last_hb_ms != 0 && (now - p.last_hb_ms) < ZONE_TIMEOUT_MS) {
            bits |= (uint8_t)(1u << other_zones[i]);
        }
    }
    return bits;
}

K_TIMER_DEFINE(hb_timer, NULL, NULL);
K_THREAD_STACK_DEFINE(hb_tx_stack, STACK_SMALL);
static struct k_thread hb_tx_thread;

static void hb_tx_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    const uint16_t my_id = (NODE_ID == 2) ? ID_HB_N2 : ID_HB_N3;

    k_timer_start(&hb_timer, K_MSEC(HB_PERIOD_MS), K_MSEC(HB_PERIOD_MS));
    while (1) {
        k_timer_status_sync(&hb_timer);
        WCET_BEGIN(hbtx);

        if (state_reasons() & ERR_TEST) {
            /* Test button: this node has failed itself. It sent one FAULT
             * and goes silent so the others also see missed heartbeats. */
            WCET_END(W_HB_TX, hbtx);
            wdt_checkin(WDT_BIT_HB_TX);
            continue;
        }
        uint32_t now = k_uptime_get_32();
        uint8_t d[8] = { 0 };
        d[1] = state_in_error() ? 0 : 1;
        d[2] = (canbus_is_degraded(BUS_A) ? 0 : 0x01) | (canbus_is_degraded(BUS_B) ? 0 : 0x02);
        d[3] = (uint8_t)state_reasons();
        d[4] = zones_heard_bits(now);
#if NODE_ID == 2
        blink_sync_info(&d[5]);                 /* local hazard timing for N3 */
#endif
        canbus_send_both(my_id, d, true);
        WCET_END(W_HB_TX, hbtx);

        wdt_checkin(WDT_BIT_HB_TX);
    }
}

/* ------------------------------------------------------------------ */
/* HB_MON                                                             */
/* ------------------------------------------------------------------ */

K_THREAD_STACK_DEFINE(hb_mon_stack, STACK_SMALL);
static struct k_thread hb_mon_thread;

static void hb_mon_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    bool zone_alive[2] = { false, false };

    while (1) {
        k_msleep(HB_MON_PERIOD_MS);
        WCET_BEGIN(hbmon);
        uint32_t now = k_uptime_get_32();

        /* Per-bus health (R4.7): any frame from anyone on that bus */
        for (int bus = 0; bus < NUM_BUSES; bus++) {
            /* signed difference: a frame can arrive (rx_cb, cooperative thread) between
             * reading 'now' and last_rx, which made the unsigned subtraction wrap and look
             * like 4 billion ms of silence (spurious degrade + TX flush) */
            bool deg = (int32_t)(now - canbus_last_rx_ms(bus)) >= (int32_t)DEG_TIMEOUT_MS;
            if (deg != canbus_is_degraded(bus)) {
                /* LED FIRST: it must never wait for the controller flush below */
                gpio_pin_set_dt(&led_deg[bus], deg);
                k_busy_wait(200);                       /* let the pin settle, then read the real level */
                printk("DEG %c %s at %u ms (last rx %u ms ago); pin set to %d, reads back %d\n",
                       'A' + bus, deg ? "ON " : "off", now, now - canbus_last_rx_ms(bus),
                       deg ? 1 : 0, gpio_pin_get_dt(&led_deg[bus]));
                canbus_set_degraded(bus, deg);          /* can_stop()/can_start() when entering */
                printk("DEG %c flush done, took %u ms\n", 'A' + bus, k_uptime_get_32() - now);
            }
        }

        /* Zone liveness (R4.5): heartbeat on at least one bus, relayed counts */
        bool all_alive = true;
        bool remote_still_reported = false;
        for (int i = 0; i < 2; i++) {
            struct peer_info p;
            state_get_peer(other_zones[i], &p);
            bool alive = p.last_hb_ms != 0 && (now - p.last_hb_ms) < ZONE_TIMEOUT_MS;
            if (zone_alive[i] && !alive) {
                state_send_fault(other_zones[i], FAULT_ZONE_LOST);
            }
            zone_alive[i] = alive;
            all_alive &= alive;
            if (alive && (p.reasons & ERR_PROPAGATE)) {
                remote_still_reported = true;
            }
        }
        if (all_alive) {
            state_clear(ERR_ZONE_LOST);
        } else {
            state_set(ERR_ZONE_LOST);
        }

        /* CTRL stream */
        struct ctrl_cmd c;
        state_get_ctrl(&c);
        if (c.valid && (now - c.t_ms) < CTRL_TIMEOUT_MS) {
            state_clear(ERR_CTRL_LOST);
        } else {
            state_set(ERR_CTRL_LOST);
        }

        /* Holds */
        uint32_t r = state_reasons();
        if ((r & ERR_BADCMD) && (now - state_last_badcmd_ms()) >= BADCMD_HOLD_MS) {
            state_clear(ERR_BADCMD);
        }
        if ((r & ERR_REMOTE) && !remote_still_reported &&
            (now - state_last_remote_fault_ms()) >= REMOTE_HOLD_MS) {
            state_clear(ERR_REMOTE);
        }

        WCET_END(W_HB_MON, hbmon);
        wdt_checkin(WDT_BIT_HB_MON);
    }
}

/* ------------------------------------------------------------------ */
/* Report (low priority, printk only here)                             */
/* ------------------------------------------------------------------ */

K_THREAD_STACK_DEFINE(report_stack, STACK_REPORT);
static struct k_thread report_thread;

static void report_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    uint32_t passes = 0;

    while (1) {
        k_msleep(REPORT_PERIOD_MS);
        if ((++passes % 5) == 0) {
            wcet_report();                      /* no-op unless built with -DWCET=1 */
        }
        uint32_t now = k_uptime_get_32();
        struct canbus_stats s;
        struct ctrl_cmd ctl;
        canbus_get_stats(&s);
        state_get_ctrl(&ctl);
        uint32_t r = state_reasons();

        printk("[%u] N%d %s reasons=0x%02x%s%s%s%s%s | DEG A=%d B=%d | last rx age A=%d B=%d ms | LED pin reads A=%d B=%d | heard=0x%02x\n",
               now, NODE_ID, r ? "ERROR" : "NORMAL", r,
               (r & ERR_CTRL_LOST) ? " CTRL_LOST" : "",
               (r & ERR_ZONE_LOST) ? " ZONE_LOST" : "",
               (r & ERR_TEST) ? " TEST" : "",
               (r & ERR_BADCMD) ? " BADCMD" : "",
               (r & ERR_REMOTE) ? " REMOTE" : "",
               canbus_is_degraded(BUS_A), canbus_is_degraded(BUS_B),
               (int32_t)(now - canbus_last_rx_ms(BUS_A)), (int32_t)(now - canbus_last_rx_ms(BUS_B)),
               gpio_pin_get_dt(&led_deg[BUS_A]), gpio_pin_get_dt(&led_deg[BUS_B]),
               zones_heard_bits(now));
        printk("    rx A/B %u/%u  tx ok A/B %u/%u  err %u/%u  refused %u/%u  dup %u stale %u ovf %u\n",
               s.rx[0], s.rx[1], s.tx_ok[0], s.tx_ok[1], s.tx_err[0], s.tx_err[1],
               s.tx_refused[0], s.tx_refused[1], s.dup_dropped, s.stale_dropped, s.rxq_overflow);
        uint32_t entries, cnt[5];
        state_get_diag(&entries, cnt);
        printk("    error entries=%u  set counts: CTRL_LOST=%u ZONE_LOST=%u TEST=%u BADCMD=%u REMOTE=%u\n",
               entries, cnt[0], cnt[1], cnt[2], cnt[3], cnt[4]);
        if (ctl.valid) {
            printk("    ctrl steer=%d thr=%u brk=%u btn=0x%02x age=%u ms\n",
                   ctl.steer, ctl.thr, ctl.brk, ctl.buttons, now - ctl.t_ms);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Watchdog                                                           */
/* ------------------------------------------------------------------ */

K_THREAD_STACK_DEFINE(wdt_stack, STACK_SMALL);
static struct k_thread wdt_thread;

static void wdt_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        k_msleep(WDT_FEED_PERIOD_MS);
        if (wdt_channel >= 0 &&
            (atomic_get(&wdt_bits) & WDT_ALL_BITS) == WDT_ALL_BITS) {
            atomic_clear(&wdt_bits);
            wdt_feed(wdt_dev, wdt_channel);
        }
    }
}

static int watchdog_start(void)
{
    if (!device_is_ready(wdt_dev)) {
        return -ENODEV;
    }
    struct wdt_timeout_cfg cfg = {
        .window = { .min = 0, .max = WDT_TIMEOUT_MS },
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };
    wdt_channel = wdt_install_timeout(wdt_dev, &cfg);
    if (wdt_channel < 0) {
        return wdt_channel;
    }
    /* Pause while halted by the debugger if the hardware allows it */
    int err = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (err != 0) {
        err = wdt_setup(wdt_dev, 0);
    }
    return err;
}

/* ------------------------------------------------------------------ */

int monitor_init(void)
{
    for (int bus = 0; bus < NUM_BUSES; bus++) {
        if (!gpio_is_ready_dt(&led_deg[bus])) {
            return -ENODEV;
        }
        gpio_pin_configure_dt(&led_deg[bus], GPIO_OUTPUT_INACTIVE);
    }

    /* LED self-test at boot: both DEG LEDs on for 500 ms. If an LED does not flash here,
     * the problem is its pin, wiring or polarity, not the degrade logic. */
    for (int bus = 0; bus < NUM_BUSES; bus++) {
        gpio_pin_set_dt(&led_deg[bus], 1);
    }
    k_msleep(500);
    for (int bus = 0; bus < NUM_BUSES; bus++) {
        gpio_pin_set_dt(&led_deg[bus], 0);
    }
    printk("DEG LED self-test done (A=PA4, B=PB0)\n");

    k_thread_create(&hb_tx_thread, hb_tx_stack, K_THREAD_STACK_SIZEOF(hb_tx_stack),
                    hb_tx_fn, NULL, NULL, NULL, PRIO_HB_TX, 0, K_NO_WAIT);
    k_thread_name_set(&hb_tx_thread, "hb_tx");
    k_thread_create(&hb_mon_thread, hb_mon_stack, K_THREAD_STACK_SIZEOF(hb_mon_stack),
                    hb_mon_fn, NULL, NULL, NULL, PRIO_HB_MON, 0, K_NO_WAIT);
    k_thread_name_set(&hb_mon_thread, "hb_mon");
    k_thread_create(&report_thread, report_stack, K_THREAD_STACK_SIZEOF(report_stack),
                    report_fn, NULL, NULL, NULL, PRIO_REPORT, 0, K_NO_WAIT);
    k_thread_name_set(&report_thread, "report");

    int err = watchdog_start();
    if (err != 0) {
        printk("watchdog not started (%d)\n", err);
    }
    k_thread_create(&wdt_thread, wdt_stack, K_THREAD_STACK_SIZEOF(wdt_stack),
                    wdt_fn, NULL, NULL, NULL, PRIO_WATCHDOG, 0, K_NO_WAIT);
    k_thread_name_set(&wdt_thread, "wdt");
    return 0;
}