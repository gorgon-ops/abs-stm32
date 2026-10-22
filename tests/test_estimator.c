#include "test_util.h"

#include "abs_config.h"
#include "wheel_speed.h"
#include "imu_fusion.h"
#include "vehicle_speed.h"

#include <math.h>
#include <string.h>

/* ================================================================== */
/* Encoder counter arithmetic                                          */
/* ================================================================== */

static void test_delta_plain(void)
{
    CHECK_EQ(wheel_speed_delta(100u, 40u, 16), 60);
    CHECK_EQ(wheel_speed_delta(40u, 100u, 16), -60);
    CHECK_EQ(wheel_speed_delta(7u, 7u, 16), 0);
}

static void test_delta_wraps_forward(void)
{
    CHECK_EQ(wheel_speed_delta(3u, 65533u, 16), 6);
    CHECK_EQ(wheel_speed_delta(0u, 65535u, 16), 1);
}

static void test_delta_wraps_backward(void)
{
    CHECK_EQ(wheel_speed_delta(65533u, 3u, 16), -6);
    CHECK_EQ(wheel_speed_delta(65535u, 0u, 16), -1);
}

static void test_delta_half_range_is_the_limit(void)
{
    CHECK_EQ(wheel_speed_delta(32768u, 0u, 16), -32768);
    CHECK_EQ(wheel_speed_delta(32767u, 0u, 16), 32767);
}

static void test_delta_32bit(void)
{
    CHECK_EQ(wheel_speed_delta(5u, 0xFFFFFFFBu, 32), 10);
    CHECK_EQ(wheel_speed_delta(0xFFFFFFFBu, 5u, 32), -10);
}

/* ================================================================== */
/* Wheel speed                                                         */
/* ================================================================== */

/* Counter value for a wheel that has turned at omega for n ticks. */
static uint32_t counts_for(float omega, float dt, int n, const wheel_speed_cfg_t *c)
{
    double revs = ((double)omega * dt * n) / (2.0 * ABS_PI);
    long   ct   = (long)(revs * c->counts_per_rev);
    return (uint32_t)(ct & 0xFFFFL);
}

static void test_wheel_speed_seeds_at_the_measurement(void)
{
    wheel_speed_t ws;
    wheel_speed_cfg_t cfg = wheel_speed_cfg_default;
    float dt = ABS_LOOP_DT_S;
    float omega = 120.0f;

    wheel_speed_init(&ws, &cfg);
    wheel_speed_update(&ws, counts_for(omega, dt, 0, &cfg), dt); /* primes */
    wheel_speed_update(&ws, counts_for(omega, dt, 1, &cfg), dt); /* seeds  */
    CHECK_NEAR(ws.omega, omega, 20.0f);
    CHECK_NEAR(ws.omega_dot, 0.0f, 1.0f);
    CHECK(ws.valid);
}

static void test_wheel_speed_converges_at_constant_speed(void)
{
    wheel_speed_t ws;
    wheel_speed_cfg_t cfg = wheel_speed_cfg_default;
    float dt = ABS_LOOP_DT_S, omega = 90.0f;
    int i;

    wheel_speed_init(&ws, &cfg);
    for (i = 0; i < 400; i++) {
        wheel_speed_update(&ws, counts_for(omega, dt, i, &cfg), dt);
    }

    CHECK_NEAR(ws.omega, omega, 3.0f);
    CHECK_NEAR(ws.v_wheel, omega * cfg.wheel_radius_m, 0.12f);
    CHECK_NEAR(ws.omega_dot, 0.0f, 150.0f);
}

static void test_wheel_speed_survives_counter_rollover(void)
{
    wheel_speed_t ws;
    wheel_speed_cfg_t cfg = wheel_speed_cfg_default;
    float dt = ABS_LOOP_DT_S, omega = 200.0f;
    int i;
    float worst = 0.0f;

    wheel_speed_init(&ws, &cfg);
    for (i = 0; i < 6000; i++) {
        wheel_speed_update(&ws, counts_for(omega, dt, i, &cfg), dt);
        if (i > 100) {
            float e = fabsf(ws.omega - omega);
            if (e > worst) {
                worst = e;
            }
        }
    }
    CHECK_NEAR(ws.omega, omega, 5.0f);
    CHECK(worst < 15.0f);
}

static void test_wheel_speed_handles_reverse_rotation(void)
{
    wheel_speed_t ws;
    wheel_speed_cfg_t cfg = wheel_speed_cfg_default;
    float dt = ABS_LOOP_DT_S, omega = -80.0f;
    int i;

    wheel_speed_init(&ws, &cfg);
    for (i = 0; i < 400; i++) {
        wheel_speed_update(&ws, counts_for(omega, dt, i, &cfg), dt);
    }
    CHECK_NEAR(ws.omega, omega, 3.0f);
    CHECK(ws.v_wheel < 0.0f);
    CHECK(ws.valid);
}

static void test_wheel_speed_flags_low_speed_invalid(void)
{
    wheel_speed_t ws;
    wheel_speed_cfg_t cfg = wheel_speed_cfg_default;
    int i;

    wheel_speed_init(&ws, &cfg);
    for (i = 0; i < 50; i++) {
        wheel_speed_update(&ws, 0u, ABS_LOOP_DT_S);
    }
    CHECK(!ws.valid);
    CHECK_NEAR(ws.v_wheel, 0.0f, 0.01f);
}

/* ================================================================== */
/* IMU fusion                                                          */
/* ================================================================== */

static imu_sample_t level_sample(float bias_ax, float gyro_bias)
{
    imu_sample_t s;
    memset(&s, 0, sizeof(s));
    s.ax = bias_ax;
    s.ay = 0.0f;
    s.az = ABS_G_MPS2;
    s.gy = gyro_bias;
    return s;
}

static void test_level_learns_gyro_bias(void)
{
    imu_fusion_t f;
    imu_sample_t s = level_sample(0.0f, 0.021f);
    int i;

    imu_fusion_init(&f, &imu_fusion_cfg_default);
    for (i = 0; i < 256; i++) {
        imu_fusion_level_sample(&f, &s);
    }
    CHECK(imu_fusion_level_finish(&f));
    CHECK_NEAR(f.gyro_bias_y, 0.021f, 1e-4f);
    CHECK(f.levelled);
}

static void test_level_rejects_too_few_samples(void)
{
    imu_fusion_t f;
    imu_sample_t s = level_sample(0.0f, 0.0f);
    int i;

    imu_fusion_init(&f, &imu_fusion_cfg_default);
    for (i = 0; i < 4; i++) {
        imu_fusion_level_sample(&f, &s);
    }
    CHECK_EQ(imu_fusion_level_finish(&f), 0);
    CHECK(!f.levelled);
}

static void test_level_absorbs_accel_offset_as_tilt(void)
{
    imu_fusion_t f;
    imu_sample_t s = level_sample(0.18f, 0.0f);
    int i;

    imu_fusion_init(&f, &imu_fusion_cfg_default);
    for (i = 0; i < 256; i++) {
        imu_fusion_level_sample(&f, &s);
    }
    CHECK(imu_fusion_level_finish(&f));
    CHECK_NEAR(f.pitch_rad, atan2f(0.18f, ABS_G_MPS2), 1e-4f);
    imu_fusion_update(&f, &s, ABS_LOOP_DT_S);
    CHECK_NEAR(f.a_long, 0.0f, 0.01f);
}

static void test_gravity_is_removed_when_pitched(void)
{
    imu_fusion_t f;
    imu_sample_t s;
    float pitch = 0.05f;
    int i;

    imu_fusion_init(&f, &imu_fusion_cfg_default);
    memset(&s, 0, sizeof(s));
    s.ax = ABS_G_MPS2 * sinf(pitch);
    s.az = ABS_G_MPS2 * cosf(pitch);

    for (i = 0; i < 256; i++) {
        imu_fusion_level_sample(&f, &s);
    }
    CHECK(imu_fusion_level_finish(&f));
    CHECK_NEAR(f.pitch_rad, pitch, 1e-3f);

    for (i = 0; i < 100; i++) {
        imu_fusion_update(&f, &s, ABS_LOOP_DT_S);
    }
    CHECK_NEAR(f.a_long, 0.0f, 0.02f);
}

static void test_accel_is_rejected_as_attitude_reference_under_braking(void)
{
    imu_fusion_t f;
    imu_sample_t rest, braking;
    int i;

    imu_fusion_init(&f, &imu_fusion_cfg_default);
    rest = level_sample(0.0f, 0.0f);
    for (i = 0; i < 256; i++) {
        imu_fusion_level_sample(&f, &rest);
    }
    CHECK(imu_fusion_level_finish(&f));
    memset(&braking, 0, sizeof(braking));
    braking.ax = -1.2f * ABS_G_MPS2;
    braking.az = ABS_G_MPS2;
    braking.gy = 0.0f;

    for (i = 0; i < 400; i++) {
        imu_fusion_update(&f, &braking, ABS_LOOP_DT_S);
    }
    CHECK(!f.accel_valid);
    CHECK_NEAR(f.pitch_rad, 0.0f, 0.01f);
    CHECK_NEAR(f.a_long, -1.2f * ABS_G_MPS2, 0.1f);
}

static void test_accel_is_accepted_when_coasting(void)
{
    imu_fusion_t f;
    imu_sample_t s;
    int i;

    imu_fusion_init(&f, &imu_fusion_cfg_default);
    s = level_sample(0.0f, 0.0f);
    for (i = 0; i < 256; i++) {
        imu_fusion_level_sample(&f, &s);
    }
    CHECK(imu_fusion_level_finish(&f));

    imu_fusion_update(&f, &s, ABS_LOOP_DT_S);
    CHECK(f.accel_valid);
}

static void test_gyro_drift_is_bounded_over_a_stop(void)
{
    imu_fusion_t f;
    imu_sample_t rest, braking;
    int i;
    float residual = 0.002f;

    imu_fusion_init(&f, &imu_fusion_cfg_default);
    rest = level_sample(0.0f, 0.0f);
    for (i = 0; i < 256; i++) {
        imu_fusion_level_sample(&f, &rest);
    }
    CHECK(imu_fusion_level_finish(&f));

    memset(&braking, 0, sizeof(braking));
    braking.ax = -1.1f * ABS_G_MPS2;
    braking.az = ABS_G_MPS2;
    braking.gy = residual;
    for (i = 0; i < 1000; i++) {
        imu_fusion_update(&f, &braking, ABS_LOOP_DT_S);
    }
    CHECK_NEAR(f.pitch_rad, residual * 1.0f, 5e-4f);
    CHECK(fabsf(ABS_G_MPS2 * sinf(f.pitch_rad)) < 0.05f);
}

/* ================================================================== */
/* Velocity observer                                                   */
/* ================================================================== */

static void test_trust_full_when_rolling_free(void)
{
    CHECK_NEAR(vehicle_speed_trust(0.0f, 0, 4.0f, 4.0f), 1.0f, 1e-6f);
    CHECK_NEAR(vehicle_speed_trust(0.01f, 0, 4.0f, 4.0f), 1.0f, 1e-6f);
}

static void test_trust_fades_with_slip(void)
{
    float mid = vehicle_speed_trust(0.06f, 0, 4.0f, 4.0f);
    CHECK(mid > 0.0f && mid < 1.0f);
    CHECK_NEAR(vehicle_speed_trust(0.10f, 0, 4.0f, 4.0f), 0.0f, 1e-6f);
    CHECK_NEAR(vehicle_speed_trust(0.50f, 0, 4.0f, 4.0f), 0.0f, 1e-6f);
}

static void test_trust_is_one_sided_while_braking(void)
{
    CHECK_NEAR(vehicle_speed_trust(0.30f, 1, 3.0f, 4.0f), 0.0f, 1e-6f);
    CHECK_NEAR(vehicle_speed_trust(0.00f, 1, 3.9f, 4.0f), 0.0f, 1e-6f);
    CHECK(vehicle_speed_trust(0.0f, 1, 4.5f, 4.0f) > 0.0f);
}

static void test_predict_integrates_acceleration(void)
{
    vehicle_speed_t vs;
    int i;

    vehicle_speed_init(&vs, &vehicle_speed_cfg_default);
    vehicle_speed_reset(&vs, 4.0f);
    for (i = 0; i < 100; i++) {
        vehicle_speed_predict(&vs, -5.0f, ABS_LOOP_DT_S);
    }
    CHECK_NEAR(vs.v, 4.0f - 5.0f * 0.1f, 0.01f);
}

static void test_predict_clamps_impossible_deceleration(void)
{
    vehicle_speed_t vs;
    int i;

    vehicle_speed_init(&vs, &vehicle_speed_cfg_default);
    vehicle_speed_reset(&vs, 4.0f);
    for (i = 0; i < 100; i++) {
        vehicle_speed_predict(&vs, -5.0f * ABS_G_MPS2, ABS_LOOP_DT_S);
    }
    CHECK_NEAR(vs.v, 4.0f - ABS_VS_MAX_DECEL_MPS2 * 0.1f, 0.02f);
}

static void test_predict_never_goes_negative(void)
{
    vehicle_speed_t vs;
    int i;

    vehicle_speed_init(&vs, &vehicle_speed_cfg_default);
    vehicle_speed_reset(&vs, 0.2f);
    for (i = 0; i < 500; i++) {
        vehicle_speed_predict(&vs, -9.0f, ABS_LOOP_DT_S);
    }
    CHECK(vs.v >= 0.0f);
}

static void test_observer_learns_accelerometer_bias(void)
{
    vehicle_speed_t vs;
    int i;
    float bias = 0.25f;

    vehicle_speed_init(&vs, &vehicle_speed_cfg_default);
    vehicle_speed_reset(&vs, 4.0f);
    for (i = 0; i < 2000; i++) {
        vehicle_speed_predict(&vs, bias, ABS_LOOP_DT_S);
        vehicle_speed_correct(&vs, 4.0f, 1.0f);
    }
    CHECK(vs.bias > 0.4f * bias);

    for (i = 0; i < 18000; i++) {
        vehicle_speed_predict(&vs, bias, ABS_LOOP_DT_S);
        vehicle_speed_correct(&vs, 4.0f, 1.0f);
    }
    CHECK_NEAR(vs.bias, bias, 0.05f);
    CHECK_NEAR(vs.v, 4.0f, 0.02f);
    CHECK(fabsf(vs.bias - bias) * 0.5f < 0.05f);
}

static void test_velocity_only_correction_leaves_bias_alone(void)
{
    vehicle_speed_t vs;
    float bias_before;
    int i;

    vehicle_speed_init(&vs, &vehicle_speed_cfg_default);
    vehicle_speed_reset(&vs, 4.0f);
    for (i = 0; i < 2000; i++) {
        vehicle_speed_predict(&vs, 0.20f, ABS_LOOP_DT_S);
        vehicle_speed_correct(&vs, 4.0f, 1.0f);
    }
    bias_before = vs.bias;
    for (i = 0; i < 500; i++) {
        vehicle_speed_predict(&vs, -8.0f, ABS_LOOP_DT_S);
        vehicle_speed_correct_vel(&vs, vs.v + 0.5f, 0.25f);
    }
    CHECK_NEAR(vs.bias, bias_before, 1e-3f);
}

static void test_zero_trust_is_a_no_op(void)
{
    vehicle_speed_t vs;
    float v, b;
    vehicle_speed_init(&vs, &vehicle_speed_cfg_default);
    vehicle_speed_reset(&vs, 4.0f);
    v = vs.v;
    b = vs.bias;
    vehicle_speed_correct(&vs, 0.1f, 0.0f);
    vehicle_speed_correct_vel(&vs, 0.1f, 0.0f);
    CHECK_NEAR(vs.v, v, 1e-9f);
    CHECK_NEAR(vs.bias, b, 1e-9f);
}

int main(void)
{
    test_delta_plain();
    test_delta_wraps_forward();
    test_delta_wraps_backward();
    test_delta_half_range_is_the_limit();
    test_delta_32bit();

    test_wheel_speed_seeds_at_the_measurement();
    test_wheel_speed_converges_at_constant_speed();
    test_wheel_speed_survives_counter_rollover();
    test_wheel_speed_handles_reverse_rotation();
    test_wheel_speed_flags_low_speed_invalid();

    test_level_learns_gyro_bias();
    test_level_rejects_too_few_samples();
    test_level_absorbs_accel_offset_as_tilt();
    test_gravity_is_removed_when_pitched();
    test_accel_is_rejected_as_attitude_reference_under_braking();
    test_accel_is_accepted_when_coasting();
    test_gyro_drift_is_bounded_over_a_stop();

    test_trust_full_when_rolling_free();
    test_trust_fades_with_slip();
    test_trust_is_one_sided_while_braking();
    test_predict_integrates_acceleration();
    test_predict_clamps_impossible_deceleration();
    test_predict_never_goes_negative();
    test_observer_learns_accelerometer_bias();
    test_velocity_only_correction_leaves_bias_alone();
    test_zero_trust_is_a_no_op();

    return test_report();
}
