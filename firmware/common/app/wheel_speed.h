#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>

typedef struct {
    float counts_per_rev;    /* after X4 quadrature and gearing */
    float wheel_radius_m;
    float tau_speed_s;       /* IIR time constant, angular velocity */
    float tau_accel_s;       /* IIR time constant, angular acceleration */
    float min_valid_mps;     /* below this the estimate is flagged invalid */
    uint8_t counter_bits;    /* 16 for most TIMs, 32 for TIM2/TIM5 on F4 */
} wheel_speed_cfg_t;

typedef struct {
    const wheel_speed_cfg_t *cfg;
    uint32_t last_raw;
    int      primed;         /* first sample only establishes a baseline */
    int      seeded;         /* filters have been given a starting value */
    int32_t  position;       /* unwrapped cumulative count, for odometry */
    float    omega;          /* rad/s, filtered, signed */
    float    omega_dot;      /* rad/s^2, filtered, signed */
    float    v_wheel;        /* m/s = omega * radius */
    int      valid;
} wheel_speed_t;

void wheel_speed_init(wheel_speed_t *ws, const wheel_speed_cfg_t *cfg);
void wheel_speed_update(wheel_speed_t *ws, uint32_t raw, float dt);
int32_t wheel_speed_delta(uint32_t now, uint32_t prev, uint8_t bits);
extern const wheel_speed_cfg_t wheel_speed_cfg_default;

#endif