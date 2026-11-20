/*
 * node3.c - N3 rear / drivetrain zone: DRIVE and PID
 *
 * Outputs are test points this lab (scoped, not wired to the H-bridge):
 *   PWM_OUT  PA8  TIM1_CH1 20 kHz motor PWM (L298N EN)
 *   DIR_A    PC0  1 = drive forward (IN1=1, IN2=0); 0 = brake (IN1=IN2,
 *                 EN held at 100%, the L298N fast stop from Lab 2)
 *   PWM_SET  PA12 toggles right after every duty write
 *
 * DRIVE (prio 2): woken by every valid CTRL. Brake has priority: brake
 *   pressed -> DIR_A low and PWM 100% immediately (this is the edge the
 *   brake chain is measured to). Otherwise DIR_A high, a feed-forward duty
 *   from the throttle is written at once, and the target goes to PID.
 * PID (prio 6, 10 ms): PI speed loop. No encoders in this lab, so the
 *   measured speed comes from a first-order motor model; the task, its
 *   period and its WCET are the template's, the plant is simulated.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/spinlock.h>

#include "catalog.h"
#include "config.h"
#include "node.h"
#include "state.h"
#include "wcet.h"

static const struct device *const motor = DEVICE_DT_GET(DT_NODELABEL(motor_pwm));
static const struct gpio_dt_spec tp_dir = GPIO_DT_SPEC_GET(DT_NODELABEL(tp_dir_a), gpios);
static const struct gpio_dt_spec tp_pwm_set = GPIO_DT_SPEC_GET(DT_NODELABEL(tp_pwm_set), gpios);

#define MOTOR_CHANNEL   1           /* TIM1_CH1 */

/* PI gains (Lab 2 values) and the simulated plant */
#define KP              0.010f
#define KI              0.050f
#define MAX_RPM         200.0f      /* full throttle target */
#define PLANT_TAU_S     0.15f       /* motor time constant, model only */

K_MUTEX_DEFINE(act_lock);           /* DRIVE, PID and fail-safe all write the bridge */

static bool braking = true;         /* protected by act_lock */
static float target_rpm;            /* protected by act_lock */
static float sim_rpm, integ;        /* PID only */

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static void write_duty(float duty)
{
    duty = clampf(duty, 0.0f, 1.0f);
    uint32_t pulse = (uint32_t)(duty * (float)MOTOR_PERIOD_NS);
    pwm_set(motor, MOTOR_CHANNEL, MOTOR_PERIOD_NS, pulse, PWM_POLARITY_NORMAL);
    gpio_pin_toggle_dt(&tp_pwm_set);            /* PWM_SET */
}

/* Caller holds act_lock */
static void brake_now(void)
{
    gpio_pin_set_dt(&tp_dir, 0);                /* DIR_A low = brake: the measured edge */
    write_duty(1.0f);
    braking = true;
    target_rpm = 0.0f;
}

void node_apply_fail_safe(void)
{
    k_mutex_lock(&act_lock, K_FOREVER);
    brake_now();
    k_mutex_unlock(&act_lock);
}

/* ------------------------------------------------------------------ */
/* DRIVE                                                              */
/* ------------------------------------------------------------------ */

K_SEM_DEFINE(ctrl_sem, 0, 1);

void node_on_ctrl(void)
{
    k_sem_give(&ctrl_sem);
}

K_THREAD_STACK_DEFINE(drive_stack, STACK_MEDIUM);
static struct k_thread drive_thread;

static void drive_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        k_sem_take(&ctrl_sem, K_FOREVER);
        WCET_BEGIN(drive);
        struct ctrl_cmd cmd;
        state_get_ctrl(&cmd);

        k_mutex_lock(&act_lock, K_FOREVER);
        WCET_BEGIN(cs);
        if (state_in_error() || !cmd.valid || cmd.brk > BRAKE_ON_THRESHOLD) {
            brake_now();                        /* brake first */
        } else {
            float thr = cmd.thr / 255.0f;
            if (braking) {
                integ = 0.0f;                   /* clean start after a brake */
            }
            braking = false;
            target_rpm = thr * MAX_RPM;
            gpio_pin_set_dt(&tp_dir, 1);        /* forward */
            write_duty(thr * MAX_DUTY);         /* feed-forward now; PID trims */
        }
        WCET_END(W_ACT_CS, cs);
        k_mutex_unlock(&act_lock);
        WCET_END(W_DRIVE, drive);
    }
}

/* ------------------------------------------------------------------ */
/* PID                                                                */
/* ------------------------------------------------------------------ */

K_TIMER_DEFINE(pid_timer, NULL, NULL);
K_THREAD_STACK_DEFINE(pid_stack, STACK_MEDIUM);
static struct k_thread pid_thread;

static void pid_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    const float dt = PID_PERIOD_MS / 1000.0f;
    float duty = 0.0f;

    k_timer_start(&pid_timer, K_MSEC(PID_PERIOD_MS), K_MSEC(PID_PERIOD_MS));
    while (1) {
        k_timer_status_sync(&pid_timer);
        WCET_BEGIN(pid);

        /* Snapshot under the lock, compute outside it, write under it, so
         * DRIVE is never blocked for the length of the PI computation. */
        k_mutex_lock(&act_lock, K_FOREVER);
        bool brk_now = braking;
        float target = target_rpm;
        k_mutex_unlock(&act_lock);

        if (brk_now) {
            duty = 0.0f;
            integ = 0.0f;
        } else {
            float err = target - sim_rpm;
            float ff = (target / MAX_RPM) * MAX_DUTY;
            float unclamped = ff + KP * err + KI * (integ + err * dt);
            if (unclamped > 0.0f && unclamped < MAX_DUTY) {
                integ += err * dt;              /* anti-windup: integrate only when not saturated */
            }
            duty = clampf(ff + KP * err + KI * integ, 0.0f, MAX_DUTY);

            k_mutex_lock(&act_lock, K_FOREVER);
            if (!braking) {                     /* DRIVE may have braked meanwhile */
                write_duty(duty);
            }
            k_mutex_unlock(&act_lock);
        }

        /* Simulated motor: first-order response to the applied duty */
        float applied_rpm = braking ? 0.0f : (duty / MAX_DUTY) * MAX_RPM;
        sim_rpm += (applied_rpm - sim_rpm) * (dt / PLANT_TAU_S);

        WCET_END(W_PID, pid);
        wdt_checkin(WDT_BIT_NODE);
    }
}

/* ------------------------------------------------------------------ */

int node_init(void)
{
    if (!device_is_ready(motor) || !gpio_is_ready_dt(&tp_dir) || !gpio_is_ready_dt(&tp_pwm_set)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&tp_dir, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&tp_pwm_set, GPIO_OUTPUT_INACTIVE);
    node_apply_fail_safe();                     /* power-up = error = braked */

    k_thread_create(&drive_thread, drive_stack, K_THREAD_STACK_SIZEOF(drive_stack),
                    drive_fn, NULL, NULL, NULL, PRIO_ACTUATE, 0, K_NO_WAIT);
    k_thread_name_set(&drive_thread, "drive");
    k_thread_create(&pid_thread, pid_stack, K_THREAD_STACK_SIZEOF(pid_stack),
                    pid_fn, NULL, NULL, NULL, PRIO_PID, 0, K_NO_WAIT);
    k_thread_name_set(&pid_thread, "pid");
    return 0;
}
