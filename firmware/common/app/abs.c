#include "abs.h"

void abs_system_init(abs_system_t *s)
{
    wheel_speed_init(&s->ws,    &wheel_speed_cfg_default);
    imu_fusion_init(&s->imu,    &imu_fusion_cfg_default);
    vehicle_speed_init(&s->vs,  &vehicle_speed_cfg_default);
    abs_ctrl_init(&s->ctrl,     &abs_ctrl_cfg_default);

    s->duty   = 0.0f;
    s->led    = 0;
    s->primed = 0;
    s->ticks  = 0;
}

void abs_system_level_sample(abs_system_t *s, const imu_sample_t *imu)
{
    imu_fusion_level_sample(&s->imu, imu);
}

int abs_system_level_finish(abs_system_t *s)
{
    return imu_fusion_level_finish(&s->imu);
}

float abs_system_update(abs_system_t *s, uint32_t enc_raw,
                        const imu_sample_t *imu, float brake_demand,
                        float dt, uint32_t now_ms)
{
    float trust;
    int   braking;
    float prev_slip;

    s->ticks++;

    /* The trust decision has to be made before the observer is corrected, but
     * slip cannot be known until the observer has been corrected. Breaking the
     * loop with last tick's values costs one tick of lag - one millisecond at
     * the default rate, against ABS cycles lasting fifty times that. */
    prev_slip = s->ctrl.slip;
    braking   = (brake_demand > 0.0f);

    wheel_speed_update(&s->ws, enc_raw, dt);
    imu_fusion_update(&s->imu, imu, dt);

    if (!s->primed && s->ws.valid && brake_demand <= 0.0f) {
        vehicle_speed_reset(&s->vs, s->ws.v_wheel);
        s->primed = 1;
    }

    vehicle_speed_predict(&s->vs, s->imu.a_long, dt);
    trust = vehicle_speed_trust(prev_slip, braking, s->ws.v_wheel, s->vs.v);
    if (!s->ws.valid) {
        trust = 0.0f;
    }
    if (braking) {
        vehicle_speed_correct_vel(&s->vs, s->ws.v_wheel, trust);
    } else {
        vehicle_speed_correct(&s->vs, s->ws.v_wheel, trust);
    }

    s->duty = abs_ctrl_update(&s->ctrl, brake_demand, s->vs.v,
                              s->ws.v_wheel, s->ws.omega_dot, dt);

    s->led = abs_ctrl_led_level(abs_ctrl_led_mode(&s->ctrl), now_ms);

    return s->duty;
}

float abs_system_v_ref(const abs_system_t *s)   { return s->vs.v; }
float abs_system_v_wheel(const abs_system_t *s) { return s->ws.v_wheel; }
float abs_system_slip(const abs_system_t *s)    { return s->ctrl.slip; }
