/*
 * wcet.c - see wcet.h. Compiled only with -DWCET=1.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/spinlock.h>
#include <string.h>

#include "wcet.h"

static const char *const names[W_COUNT] = {
    [W_RX_PATH] = "RX_PATH",   [W_DISPATCH] = "DISPATCH", [W_DRIVE] = "DRIVE",
    [W_STEER] = "STEER",       [W_BLINK] = "BLINK",       [W_TEST_BTN] = "TEST_BTN",
    [W_PID] = "PID",           [W_HB_MON] = "HB_MON",     [W_HB_TX] = "HB_TX",
    [W_FORCE_TX] = "FORCE_TX", [W_RELAY] = "RELAY",       [W_SPI_TX] = "SPI_TX",
    [W_ACT_CS] = "ACT_CS",
};

struct wcet_stat {
    uint32_t n;
    uint32_t pure_max;
    uint32_t wall_max;
};

static struct k_spinlock lock;
static struct wcet_stat stats[W_COUNT];
static uint32_t overhead;               /* cycles of one empty begin/end pair */

/* INT edge timestamps, one per bus (written in ISR context) */
static const struct gpio_dt_spec int_pin[2] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(can_a), int_gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(can_b), int_gpios),
};
static struct gpio_callback int_cb[2];
static volatile uint32_t int_t[2];      /* single word, ISR writer, one reader: ok */
static volatile bool int_pending[2];

static uint64_t exec_now(void)
{
    k_thread_runtime_stats_t s;
    k_thread_runtime_stats_get(k_current_get(), &s);
    return s.execution_cycles;
}

void wcet_begin(struct wcet_mark *m)
{
    m->exec0 = exec_now();
    m->t0 = k_cycle_get_32();
}

static void record(enum wcet_id id, uint32_t pure, uint32_t wall)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    struct wcet_stat *s = &stats[id];
    s->n++;
    if (pure > s->pure_max) {
        s->pure_max = pure;
    }
    if (wall > s->wall_max) {
        s->wall_max = wall;
    }
    k_spin_unlock(&lock, key);
}

void wcet_end(enum wcet_id id, const struct wcet_mark *m)
{
    uint32_t wall = k_cycle_get_32() - m->t0;
    uint64_t exec = exec_now() - m->exec0;
    uint32_t pure = (exec > overhead) ? (uint32_t)(exec - overhead) : 0;
    record(id, pure, wall);
}

void wcet_record_wall(enum wcet_id id, uint32_t cycles)
{
    record(id, 0, cycles);
}

static void int_edge_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(pins);
    int bus = (cb == &int_cb[0]) ? 0 : 1;
    if (!int_pending[bus]) {            /* first edge of a burst: start of the wait */
        int_t[bus] = k_cycle_get_32();
        int_pending[bus] = true;
    }
}

void wcet_tx_delivered(int bus)
{
    int_pending[bus] = false;           /* that INT was a transmit-complete */
}

void wcet_rx_delivered(int bus)
{
    if (int_pending[bus]) {
        uint32_t d = k_cycle_get_32() - int_t[bus];
        /* > 5 ms = a stale edge (error interrupt, missed clear): not a sample */
        if (d < sys_clock_hw_cycles_per_sec() / 200) {
            wcet_record_wall(W_RX_PATH, d);
        }
        int_pending[bus] = false;
    }
}

static uint32_t to_us_x10(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * 10000000ULL) / sys_clock_hw_cycles_per_sec());
}

void wcet_report(void)
{
    struct wcet_stat copy[W_COUNT];
    k_spinlock_key_t key = k_spin_lock(&lock);
    memcpy(copy, stats, sizeof(copy));
    k_spin_unlock(&lock, key);

    printk("WCET-BEGIN N%d clock=%u Hz overhead=%u cyc\n", NODE_ID,
           sys_clock_hw_cycles_per_sec(), overhead);
    for (int i = 0; i < W_COUNT; i++) {
        if (copy[i].n == 0) {
            continue;
        }
        uint32_t p = to_us_x10(copy[i].pure_max), w = to_us_x10(copy[i].wall_max);
        printk("WCET %s n=%u pure_max=%u.%u wall_max=%u.%u\n", names[i], copy[i].n,
               p / 10, p % 10, w / 10, w % 10);
    }
    printk("WCET-END\n");
}

void wcet_init(void)
{
    /* Calibrate: the cheapest of many empty begin/end pairs */
    uint32_t best = UINT32_MAX;
    for (int i = 0; i < 64; i++) {
        struct wcet_mark m;
        wcet_begin(&m);
        uint64_t e = exec_now() - m.exec0;
        if (e < best) {
            best = (uint32_t)e;
        }
    }
    overhead = best;

    /* Second callback on each INT pin, next to the driver's own */
    for (int bus = 0; bus < 2; bus++) {
        gpio_init_callback(&int_cb[bus], int_edge_cb, BIT(int_pin[bus].pin));
        gpio_add_callback(int_pin[bus].port, &int_cb[bus]);
    }
}
