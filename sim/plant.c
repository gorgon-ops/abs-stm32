#include "plant.h"
#include "abs_config.h"
#include <math.h>

const plant_cfg_t plant_cfg_default = {
    0.50f,        /* m           - mass share on this wheel */
    2.0e-4f,      /* J           - wheel plus reflected motor/gearbox inertia */
    0.0325f,      /* R                                                        */

    1.28f,        /* c1          - Burckhardt, dry asphalt                    */
    23.99f,       /* c2                                                       */
    0.52f,        /* c3                                                       */

    0.0120f,      /* k_brake     - a brake this rig can                       */
    1.50f,        /* t_brake_max   lock, which is the premise ABS rests on     */

    0.0040f,      /* pitch_gain  - 2.7 deg of dive at 1.2 g                   */
    0.120f,       /* pitch_tau                                                */

    422.4f,       /* cpr         - 11 ppr hall x4 quadrature x 9.6:1 gearbox  */

    0.180f,       /* accel_bias  - MPU-6500 zero-g offset, well within spec   */
    0.060f,       /* accel_noise                                              */
    0.0000f,      /* gyro_bias   - removed by the levelling step              */
    0.0030f       /* gyro_noise                                               */
};

const surface_t plant_surfaces[4] = {
    { "dry asphalt", 1.2801f,  23.99f,  0.52f   },
    { "wet asphalt", 0.8570f,  33.822f, 0.347f  },
    { "snow",        0.1946f,  94.129f, 0.0646f },
    { "ice",         0.0500f, 306.39f,  0.0f    }
};

/* Deterministic noise. A test that asserts on stopping distance must not
 * depend on the C library's random number generator, or on the platform. */
static float rng_uniform(plant_t *p)
{
    p->rng = p->rng * 1664525u + 1013904223u;
    return ((float)((p->rng >> 8) & 0xFFFFu) / 32768.0f) - 1.0f;   /* -1..1 */
}

float plant_mu(const plant_cfg_t *cfg, float lambda)
{
    if (lambda < 0.0f) {
        lambda = 0.0f;
    }
    if (lambda > 1.0f) {
        lambda = 1.0f;
    }
    return cfg->c1 * (1.0f - expf(-cfg->c2 * lambda)) - cfg->c3 * lambda;
}

void sim_init(plant_t *p, const plant_cfg_t *cfg, float v0, uint32_t seed)
{
    p->cfg        = cfg;
    p->v          = v0;
    p->omega      = v0 / cfg->R;      /* starts rolling freely, no slip */
    p->angle      = 0.0f;
    p->x          = 0.0f;
    p->pitch      = 0.0f;
    p->pitch_rate = 0.0f;
    p->a_long     = 0.0f;
    p->lambda     = 0.0f;
    p->mu         = 0.0f;
    p->t_brake    = 0.0f;
    p->rng        = seed ? seed : 1u;
}

void step(plant_t *p, float duty, float dt)
{
    const plant_cfg_t *c = p->cfg;
    float fz, fx, omega_dot, a, pitch_target, t_cap;

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    /* Slip */
    if (p->v > 0.02f) {
        p->lambda = (p->v - p->omega * c->R) / p->v;
        if (p->lambda < 0.0f) p->lambda = 0.0f;
        if (p->lambda > 1.0f) p->lambda = 1.0f;
    } else {
        p->lambda = 0.0f;
    }

    p->mu = plant_mu(c, p->lambda);

    fz = c->m * ABS_G_MPS2;
    fx = p->mu * fz;
    if (p->v <= 0.0f) {
        fx = 0.0f;
    }

    t_cap = c->k_brake * p->omega;
    if (t_cap > c->t_brake_max) {
        t_cap = c->t_brake_max;
    }
    if (t_cap < 0.0f) {
        t_cap = 0.0f;
    }
    p->t_brake = duty * t_cap;
    a = -fx / c->m;
    p->a_long = a;
    p->v += a * dt;
    if (p->v < 0.0f) {
        p->v = 0.0f;
    }

    omega_dot = (fx * c->R - p->t_brake) / c->J;
    p->omega += omega_dot * dt;
    if (p->omega < 0.0f) {
        p->omega = 0.0f;
    }

    p->angle += p->omega * dt;
    p->x     += p->v * dt;
    pitch_target  = c->pitch_gain * a;
    p->pitch_rate = (pitch_target - p->pitch) / c->pitch_tau;
    p->pitch     += p->pitch_rate * dt;
}

uint32_t plant_encoder_raw(const plant_t *p)
{
    float revs   = p->angle / (2.0f * ABS_PI);
    long  counts = (long)floorf(revs * p->cfg->cpr);
    return (uint32_t)(counts & 0xFFFFL);
}

void plant_imu_sample(plant_t *p, imu_sample_t *out)
{
    const plant_cfg_t *c = p->cfg;

    out->ax = p->a_long + ABS_G_MPS2 * sinf(p->pitch)
              + c->accel_bias + c->accel_noise * rng_uniform(p);
    out->ay = c->accel_noise * rng_uniform(p);
    out->az = ABS_G_MPS2 * cosf(p->pitch)
              + c->accel_noise * rng_uniform(p);

    out->gx = c->gyro_noise * rng_uniform(p);
    out->gy = p->pitch_rate + c->gyro_bias + c->gyro_noise * rng_uniform(p);
    out->gz = c->gyro_noise * rng_uniform(p);
}
