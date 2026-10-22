#include "imu_fusion.h"
#include "abs_config.h"
#include <math.h>

const imu_fusion_cfg_t imu_fusion_cfg_default = {
    ABS_IMU_PITCH_TAU_S,
    ABS_IMU_ACCEL_TAU_S,
    1.0f,
    1.0f
};

static float lp(float prev, float x, float tau, float dt)
{
    float a;
    if (tau <= 0.0f) {
        return x;
    }
    a = dt / (tau + dt);
    return prev + a * (x - prev);
}

float imu_fusion_pitch_from_accel(const imu_fusion_cfg_t *cfg,
                                  const imu_sample_t *s)
{
    float ax = cfg->sign_ax * s->ax;
    float lateral = sqrtf(s->ay * s->ay + s->az * s->az);
    /* atan2 rather than asin(ax/g): it stays well conditioned past 90 degrees
     * and does not blow up if the magnitude drifts from exactly 1 g. */
    return atan2f(ax, lateral);
}

void imu_fusion_init(imu_fusion_t *f, const imu_fusion_cfg_t *cfg)
{
    f->cfg         = cfg;
    f->pitch_rad   = 0.0f;
    f->gyro_bias_y = 0.0f;
    f->a_long      = 0.0f;
    f->a_long_raw  = 0.0f;
    f->levelled    = 0;
    f->acc_ax = f->acc_ay = f->acc_az = f->acc_gy = 0.0f;
    f->acc_n  = 0;
}

void imu_fusion_level_sample(imu_fusion_t *f, const imu_sample_t *s)
{
    f->acc_ax += s->ax;
    f->acc_ay += s->ay;
    f->acc_az += s->az;
    f->acc_gy += s->gy;
    f->acc_n  += 1;
}

int imu_fusion_level_finish(imu_fusion_t *f)
{
    imu_sample_t mean;
    float n;

    if (f->acc_n < 16) {
        return 0;
    }
    n = (float)f->acc_n;

    mean.ax = f->acc_ax / n;
    mean.ay = f->acc_ay / n;
    mean.az = f->acc_az / n;
    mean.gx = 0.0f;
    mean.gy = f->acc_gy / n;
    mean.gz = 0.0f;

    /* At rest the gyro should read zero, so whatever it reads is bias. */
    f->gyro_bias_y = mean.gy;

    /* And the accelerometer is seeing nothing but gravity, so it defines the
     * true starting pitch - including however crookedly the board is glued on. */
    f->pitch_rad = imu_fusion_pitch_from_accel(f->cfg, &mean);

    f->a_long     = 0.0f;
    f->a_long_raw = 0.0f;
    f->levelled   = 1;

    f->acc_ax = f->acc_ay = f->acc_az = f->acc_gy = 0.0f;
    f->acc_n  = 0;
    return 1;
}

void imu_fusion_update(imu_fusion_t *f, const imu_sample_t *s, float dt)
{
    float rate, pitch_gyro, pitch_acc, alpha, ax, mag, excess;

    if (dt <= 0.0f) {
        return;
    }

    /* --- pitch: gyro for the fast truth, accelerometer for the slow truth -- */

    rate       = f->cfg->sign_gy * (s->gy - f->gyro_bias_y);
    pitch_gyro = f->pitch_rad + rate * dt;

    f->accel_valid = 0;
    mag    = sqrtf(s->ax * s->ax + s->ay * s->ay + s->az * s->az);
    excess = mag - ABS_G_MPS2;
    if (excess < 0.0f) {
        excess = -excess;
    }

    if (excess <= ABS_IMU_ACCEL_GATE_MPS2) {
        pitch_acc = imu_fusion_pitch_from_accel(f->cfg, s);
        /* alpha near 1 means "believe the gyro". With tau = 1.2 s at 1 kHz
         * that is 0.99917: the accelerometer only trims slow drift. */
        alpha = f->cfg->tau_pitch_s / (f->cfg->tau_pitch_s + dt);
        f->pitch_rad = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
        f->accel_valid = 1;
    } else {
        /* Dead reckoning on the gyro. Bounded by how long a stop lasts: at the
         * levelled bias residual this drifts far less than the pitch angles we
         * care about, but it is why imu_fusion_level() is not optional. */
        f->pitch_rad = pitch_gyro;
    }

    /* --- longitudinal acceleration with gravity taken back out ------------ */

    ax            = f->cfg->sign_ax * s->ax;
    f->a_long_raw = ax - ABS_G_MPS2 * sinf(f->pitch_rad);
    f->a_long     = lp(f->a_long, f->a_long_raw, f->cfg->tau_accel_s, dt);
}
