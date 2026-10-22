#ifndef IMU_FUSION_H
#define IMU_FUSION_H

typedef struct {
    float tau_pitch_s;
    float tau_accel_s;
    float sign_ax;
    float sign_gy;
} imu_fusion_cfg_t;

typedef struct {
    float ax, ay, az;      /* m/s^2, as reported by the chip */
    float gx, gy, gz;      /* rad/s, as reported by the chip */
} imu_sample_t;

typedef struct {
    const imu_fusion_cfg_t *cfg;
    float pitch_rad;       /* fused, positive nose up */
    float gyro_bias_y;     /* rad/s, removed from every sample */
    float a_long;          /* m/s^2, gravity removed, forward positive */
    float a_long_raw;      /* m/s^2, before the low pass, for diagnostics */
    int   levelled;        /* has level() been run */
    int   accel_valid;     /* was the accelerometer trusted for pitch this tick */
    /* accumulators used by imu_fusion_level() */
    float acc_ax, acc_ay, acc_az, acc_gy;
    int   acc_n;
} imu_fusion_t;

void imu_fusion_init(imu_fusion_t *f, const imu_fusion_cfg_t *cfg);


void imu_fusion_level_sample(imu_fusion_t *f, const imu_sample_t *s);
int  imu_fusion_level_finish(imu_fusion_t *f);   /* 0 if too few samples */

/* Pitch angle implied by the accelerometer alone. Only valid at rest. */
float imu_fusion_pitch_from_accel(const imu_fusion_cfg_t *cfg,
                                  const imu_sample_t *s);

void imu_fusion_update(imu_fusion_t *f, const imu_sample_t *s, float dt);

extern const imu_fusion_cfg_t imu_fusion_cfg_default;

#endif 