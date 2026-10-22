#include "wheel_speed.h"
#include "abs_config.h"

const wheel_speed_cfg_t wheel_speed_cfg_default = {
    ABS_ENCODER_CPR,
    ABS_WHEEL_RADIUS_M,
    ABS_WS_TAU_SPEED_S,
    ABS_WS_TAU_ACCEL_S,
    ABS_WS_MIN_VALID_MPS,
    16
};

/* One-pole low pass expressed by time constant rather than by coefficient, so
 * the behaviour is unchanged if the loop rate changes. alpha -> 1 as dt -> 0. */
static float lp(float prev, float x, float tau, float dt)
{
    float a;
    if (tau <= 0.0f) {
        return x;
    }
    a = dt / (tau + dt);
    return prev + a * (x - prev);
}

int32_t wheel_speed_delta(uint32_t now, uint32_t prev, uint8_t bits)
{
    uint32_t mask, diff, sign;

    if (bits >= 32) {
        return (int32_t)(now - prev);
    }

    mask = (1u << bits) - 1u;
    diff = (now - prev) & mask;
    sign = 1u << (bits - 1);
    if (diff & sign) {
        return (int32_t)diff - (int32_t)(mask + 1u);
    }
    return (int32_t)diff;
}

void wheel_speed_init(wheel_speed_t *ws, const wheel_speed_cfg_t *cfg)
{
    ws->cfg       = cfg;
    ws->last_raw  = 0;
    ws->primed    = 0;
    ws->seeded    = 0;
    ws->position  = 0;
    ws->omega     = 0.0f;
    ws->omega_dot = 0.0f;
    ws->v_wheel   = 0.0f;
    ws->valid     = 0;
}

void wheel_speed_update(wheel_speed_t *ws, uint32_t raw, float dt)
{
    int32_t d;
    float omega_raw, omega_prev, dot_raw;

    if (dt <= 0.0f) {
        return;
    }

    if (!ws->primed) {
        ws->last_raw = raw;
        ws->primed   = 1;
        return;
    }

    d = wheel_speed_delta(raw, ws->last_raw, ws->cfg->counter_bits);
    ws->last_raw  = raw;
    ws->position += d;

    /* counts -> revolutions -> radians, per second. */
    omega_raw = ((float)d / ws->cfg->counts_per_rev) * (2.0f * ABS_PI) / dt;

    if (!ws->seeded) {
        ws->omega     = omega_raw;
        ws->omega_dot = 0.0f;
        ws->seeded    = 1;
        ws->v_wheel   = ws->omega * ws->cfg->wheel_radius_m;
        ws->valid     = (ws->v_wheel >= ws->cfg->min_valid_mps) ||
                        (ws->v_wheel <= -ws->cfg->min_valid_mps);
        return;
    }

    omega_prev = ws->omega;
    ws->omega  = lp(ws->omega, omega_raw, ws->cfg->tau_speed_s, dt);
    dot_raw       = (ws->omega - omega_prev) / dt;
    ws->omega_dot = lp(ws->omega_dot, dot_raw, ws->cfg->tau_accel_s, dt);

    ws->v_wheel = ws->omega * ws->cfg->wheel_radius_m;
    ws->valid   = (ws->v_wheel >= ws->cfg->min_valid_mps) ||
                  (ws->v_wheel <= -ws->cfg->min_valid_mps);
}
