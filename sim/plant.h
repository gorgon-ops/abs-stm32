#ifndef PLANT_H
#define PLANT_H

#include <stdint.h>
#include "imu_fusion.h"

typedef struct {
    float m;            /* sprung mass carried by this wheel, kg */
    float J;            /* wheel + reflected drivetrain inertia, kg m^2 */
    float R;            /* rolling radius, m */

    float c1, c2, c3;   /* Burckhardt coefficients for the surface */

    float k_brake;      /* short-brake torque per rad/s, Nm/(rad/s) */
    float t_brake_max;  /* saturation torque, Nm */

    float pitch_gain;   /* rad of nose-down dive per m/s^2 of deceleration */
    float pitch_tau;    /* suspension/tyre compliance time constant, s */

    float cpr;          /* encoder counts per wheel revolution, after X4 */

    float accel_bias;   /* m/s^2, the error the observer has to learn */
    float accel_noise;  /* m/s^2, peak uniform */
    float gyro_bias;    /* rad/s */
    float gyro_noise;   /* rad/s, peak uniform */
} plant_cfg_t;

typedef struct {
    const plant_cfg_t *cfg;

    float v;            /* vehicle speed, m/s */
    float omega;        /* wheel angular velocity, rad/s */
    float angle;        /* accumulated wheel angle, rad */
    float x;            /* distance travelled, m */
    float pitch;        /* body pitch, rad, negative is nose down */
    float pitch_rate;   /* rad/s */

    float a_long;       /* true longitudinal acceleration, m/s^2 */
    float lambda;       /* true slip ratio */
    float mu;           /* current friction coefficient */
    float t_brake;      /* torque actually applied, Nm */

    uint32_t rng;
} plant_t;

extern const plant_cfg_t plant_cfg_default;
typedef struct {
    const char *name;
    float c1, c2, c3;
} surface_t;

extern const surface_t plant_surfaces[4];
#define PLANT_N_SURFACES 4

void sim_init(plant_t *p, const plant_cfg_t *cfg, float v0, uint32_t seed);
void step(plant_t *p, float duty, float dt);
uint32_t plant_encoder_raw(const plant_t *p);
void     plant_imu_sample(plant_t *p, imu_sample_t *out);

/* Friction coefficient at a given slip, exposed for tests and for finding the
 * peak of the curve. */
float plant_mu(const plant_cfg_t *cfg, float lambda);

#endif /* PLANT_H */
