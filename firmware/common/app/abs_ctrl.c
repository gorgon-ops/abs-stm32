#include "abs_ctrl.h"
#include "abs_config.h"

const abs_ctrl_cfg_t abs_ctrl_cfg_default = {
    ABS_SLIP_DUMP,
    ABS_SLIP_RECOVER,
    ABS_WHEEL_DECEL_DUMP,
    ABS_WHEEL_ACCEL_RECOVER,
    ABS_RATE_APPLY,
    ABS_RATE_DUMP,
    ABS_RATE_REAPPLY,
    ABS_DUMP_FLOOR_FRAC,
    ABS_REAPPLY_KNEE_FRAC,
    ABS_DUMP_MAX_S,
    ABS_HOLD_MAX_S,
    ABS_CUTOUT_MPS,
    ABS_LOCK_DECAY,
    ABS_LOCK_MIN
};

static void enter(abs_ctrl_t *c, abs_state_t s)
{
    if (c->state != s) {
        c->state      = s;
        c->t_in_state = 0.0f;
    }
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float abs_slip_ratio(float v_ref, float v_wheel)
{
    float s;

    /* Below a few cm/s the division is meaningless and the result would swing
     * between 0 and 1 on sensor noise alone. */
    if (v_ref < 0.05f) {
        return 0.0f;
    }
    s = (v_ref - v_wheel) / v_ref;
    return clampf(s, 0.0f, 1.0f);
}

void abs_ctrl_init(abs_ctrl_t *c, const abs_ctrl_cfg_t *cfg)
{
    c->cfg          = cfg;
    c->state        = ABS_STATE_IDLE;
    c->duty         = 0.0f;
    c->slip         = 0.0f;
    c->duty_lock    = 0.0f;
    c->t_in_state   = 0.0f;
    c->active       = 0;
    c->engaged      = 0;
    c->abdicated    = 0;
    c->cycles       = 0;
    c->cycles_event = 0;
    c->event_time_s = 0.0f;
}

float abs_ctrl_update(abs_ctrl_t *c, float brake_demand, float v_ref,
                      float v_wheel, float omega_dot, float dt)
{
    int dump_trigger, recovered;

    if (dt <= 0.0f) {
        return c->duty;
    }

    brake_demand   = clampf(brake_demand, 0.0f, 1.0f);
    c->slip        = abs_slip_ratio(v_ref, v_wheel);
    c->t_in_state += dt;

    /* --- no demand: release, and reset the per-event counters ------------- */
    if (brake_demand <= 0.0f) {
        enter(c, ABS_STATE_IDLE);
        c->duty         = 0.0f;
        c->active       = 0;
        c->engaged      = 0;
        c->abdicated    = 0;
        c->cycles_event = 0;
        c->event_time_s = 0.0f;
        return c->duty;
    }

    c->event_time_s += dt;

    /* --- gave up on this surface: base braking, and stay out of the way --- */
    if (c->abdicated) {
        enter(c, ABS_STATE_ABDICATE);
        c->duty   = brake_demand;
        c->active = 0;
        return c->duty;
    }

    /* --- too slow to modulate: hand the demand straight through ----------- */
    if (v_ref < c->cfg->cutout_mps) {
        enter(c, ABS_STATE_CUTOUT);
        c->duty   = brake_demand;
        c->active = 0;
        return c->duty;
    }

    dump_trigger = (c->slip > c->cfg->slip_dump) ||
                   (omega_dot < c->cfg->wheel_decel_dump);

    recovered    = (c->slip < c->cfg->slip_recover) &&
                   (omega_dot > c->cfg->wheel_accel_recover);

    switch (c->state) {

    case ABS_STATE_IDLE:
    case ABS_STATE_CUTOUT:
        enter(c, ABS_STATE_APPLY);
        break;

    case ABS_STATE_APPLY:
        if (dump_trigger) {
            c->duty_lock = c->duty;
            enter(c, ABS_STATE_DUMP);
            c->cycles++;
            c->cycles_event++;
            c->engaged = 1;
        } else {
            c->duty += c->cfg->rate_apply * dt;
        }
        break;

    case ABS_STATE_DUMP: {
        float floor_duty = c->duty_lock * c->cfg->dump_floor_frac;
        c->duty -= c->cfg->rate_dump * dt;
        if (c->duty < floor_duty) {
            c->duty = floor_duty;
        }
        if (recovered) {
            enter(c, ABS_STATE_HOLD);
        } else if (c->t_in_state > c->cfg->dump_max_s) {
            if (c->slip <= c->cfg->slip_dump) {
                enter(c, ABS_STATE_HOLD);
            } else if (c->duty > c->cfg->lock_min) {
                c->duty_lock *= c->cfg->lock_decay;
                c->t_in_state = 0.0f;
            } else {
                c->duty_lock = 0.0f;
                c->abdicated = 1;
            }
        }
        break;
    }

    case ABS_STATE_HOLD:
        if (dump_trigger) {
            c->duty_lock = c->duty;
            enter(c, ABS_STATE_DUMP);
            c->cycles++;
            c->cycles_event++;
        } else if (c->t_in_state > c->cfg->hold_max_s) {
            enter(c, ABS_STATE_REAPPLY);
        }
        break;

    case ABS_STATE_REAPPLY:
        if (dump_trigger) {
            /* The limit moved - it always does, as speed and load change.
             * Relearn it from where we are now rather than from the old one. */
            c->duty_lock = c->duty;
            enter(c, ABS_STATE_DUMP);
            c->cycles++;
            c->cycles_event++;
        } else if (c->duty < c->duty_lock * c->cfg->reapply_knee_frac) {
            /* Below the knee the answer is already known to be safe: it is
             * less brake than the wheel tolerated a moment ago. */
            c->duty += c->cfg->rate_apply * dt;
        } else {
            /* Above it, creep - this is where the limit is, and the point is
             * to sit just under it rather than to find it again. */
            c->duty += c->cfg->rate_reapply * dt;
        }
        break;

    default:
        enter(c, ABS_STATE_IDLE);
        break;
    }

    /* The demand from the driver is a ceiling the modulator may sit under but
     * never exceed. ABS takes brake away; it never adds any. */
    c->duty   = clampf(c->duty, 0.0f, brake_demand);
    c->active = (c->state == ABS_STATE_DUMP)  ||
                (c->state == ABS_STATE_HOLD)  ||
                (c->state == ABS_STATE_REAPPLY);

    return c->duty;
}

abs_led_mode_t abs_ctrl_led_mode(const abs_ctrl_t *c)
{
    if (c->active) {
        return ABS_LED_FLASH;
    }
    if (c->duty > 0.0f) {
        return c->abdicated ? ABS_LED_ABDICATED : ABS_LED_SOLID;
    }
    return ABS_LED_OFF;
}

int abs_ctrl_led_level(abs_led_mode_t mode, uint32_t now_ms)
{
    uint32_t period_ms, half;

    switch (mode) {
    case ABS_LED_OFF:
        return 0;
    case ABS_LED_SOLID:
        return 1;
    case ABS_LED_ABDICATED:
    case ABS_LED_FLASH:
    default:
        period_ms = (mode == ABS_LED_ABDICATED)
                    ? (uint32_t)(1000.0f / ABS_LED_SLOW_HZ)
                    : (uint32_t)(1000.0f / ABS_LED_FLASH_HZ);
        if (period_ms == 0u) {
            period_ms = 1u;
        }
        half = period_ms / 2u;
        if (half == 0u) {
            half = 1u;
        }
        return (now_ms % period_ms) < half;
    }
}

float abs_ctrl_cycle_hz(const abs_ctrl_t *c)
{
    if (c->event_time_s <= 0.0f) {
        return 0.0f;
    }
    return (float)c->cycles_event / c->event_time_s;
}
