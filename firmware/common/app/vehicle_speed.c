#include "vehicle_speed.h"
#include "abs_config.h"

const vehicle_speed_cfg_t vehicle_speed_cfg_default = {
    ABS_VS_Q_VEL,
    ABS_VS_Q_BIAS,
    ABS_VS_R_WHEEL,
    ABS_VS_MAX_DECEL_MPS2,
    ABS_VS_INIT_VAR_V,
    ABS_VS_INIT_VAR_B
};

void vehicle_speed_init(vehicle_speed_t *vs, const vehicle_speed_cfg_t *cfg)
{
    vs->cfg     = cfg;
    vs->v       = 0.0f;
    vs->bias    = 0.0f;
    vs->P[0][0] = cfg->init_var_v;
    vs->P[0][1] = 0.0f;
    vs->P[1][0] = 0.0f;
    vs->P[1][1] = cfg->init_var_b;
    vs->innovation = 0.0f;
    vs->corrected  = 0;
}

void vehicle_speed_reset(vehicle_speed_t *vs, float v)
{
    vs->v       = v;
    vs->P[0][0] = vs->cfg->init_var_v;
    vs->P[0][1] = 0.0f;
    vs->P[1][0] = 0.0f;
    /* Deliberately not resetting bias: it is a property of the sensor, not of
     * the run, and it took a while to learn. */
}

void vehicle_speed_predict(vehicle_speed_t *vs, float a_long, float dt)
{
    float v_prev, a_eff, floor_v;
    float p00, p01, p10, p11;

    if (dt <= 0.0f) {
        return;
    }

    v_prev = vs->v;
    a_eff  = a_long - vs->bias;

    vs->v = v_prev + a_eff * dt;

    /* P = F P F' + Q, with F = [[1, -dt], [0, 1]].
     * Written out rather than looped: it is four lines, it is branch free, and
     * it runs 1000 times a second. */
    p00 = vs->P[0][0];
    p01 = vs->P[0][1];
    p10 = vs->P[1][0];
    p11 = vs->P[1][1];

    vs->P[0][0] = p00 - dt * (p01 + p10) + dt * dt * p11 + vs->cfg->q_vel * dt;
    vs->P[0][1] = p01 - dt * p11;
    vs->P[1][0] = p10 - dt * p11;
    vs->P[1][1] = p11 + vs->cfg->q_bias * dt;

    /* Physical limit. See ABS_VS_MAX_DECEL_MPS2 - this is the guard rail that
     * keeps a bad bias or a dragging wheel from walking the reference to zero. */
    floor_v = v_prev - vs->cfg->max_decel * dt;
    if (vs->v < floor_v) {
        vs->v = floor_v;
    }
    if (vs->v < 0.0f) {
        vs->v = 0.0f;
    }

    vs->corrected = 0;
}

float vehicle_speed_trust(float slip, int braking, float v_wheel, float v_est)
{
    float t;

    if (braking) {
        /* One-sided: pull the estimate up to a faster wheel, never down to a
         * slower one. See the header - this is the difference between an ABS
         * that fires and one that watches. */
        if (v_wheel > v_est) {
            return 0.25f;
        }
        return 0.0f;
    }

    if (slip < 0.0f) {
        slip = -slip;
    }

    /* Full trust up to a slip of 2%, none past 10%, linear between. A rolling
     * wheel carries a little slip even without braking, so the deadband is not
     * optional. */
    if (slip <= 0.02f) {
        return 1.0f;
    }
    if (slip >= 0.10f) {
        return 0.0f;
    }
    t = (0.10f - slip) / 0.08f;
    return t;
}

void vehicle_speed_correct(vehicle_speed_t *vs, float v_wheel, float trust)
{
    float r, s, k0, k1, y;
    float p00, p01, p10, p11;

    if (trust <= 0.0f) {
        return;
    }
    if (trust > 1.0f) {
        trust = 1.0f;
    }

    r = vs->cfg->r_wheel / trust;

    p00 = vs->P[0][0];
    p01 = vs->P[0][1];
    p10 = vs->P[1][0];
    p11 = vs->P[1][1];

    s  = p00 + r;               /* H P H' + R, with H = [1 0] */
    k0 = p00 / s;
    k1 = p10 / s;

    y = v_wheel - vs->v;
    vs->innovation = y;

    vs->v    += k0 * y;
    vs->bias += k1 * y;

    /* P = (I - K H) P */
    vs->P[0][0] = p00 - k0 * p00;
    vs->P[0][1] = p01 - k0 * p01;
    vs->P[1][0] = p10 - k1 * p00;
    vs->P[1][1] = p11 - k1 * p01;

    if (vs->v < 0.0f) {
        vs->v = 0.0f;
    }
    vs->corrected = 1;
}

void vehicle_speed_correct_vel(vehicle_speed_t *vs, float v_wheel, float trust)
{
    float r, s_inn, k0, y, p00, p01;

    if (trust <= 0.0f) {
        return;
    }
    if (trust > 1.0f) {
        trust = 1.0f;
    }

    r   = vs->cfg->r_wheel / trust;
    p00 = vs->P[0][0];
    p01 = vs->P[0][1];

    s_inn = p00 + r;
    k0    = p00 / s_inn;

    y = v_wheel - vs->v;
    vs->innovation = y;

    vs->v += k0 * y;

    vs->P[0][0] = p00 - k0 * p00;
    vs->P[0][1] = p01 - k0 * p01;

    if (vs->v < 0.0f) {
        vs->v = 0.0f;
    }
    vs->corrected = 1;
}
