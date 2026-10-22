
#ifndef VEHICLE_SPEED_H
#define VEHICLE_SPEED_H

typedef struct {
    float q_vel;        /* process noise on velocity, (m/s)^2 per second */
    float q_bias;       /* process noise on bias, (m/s^2)^2 per second */
    float r_wheel;      /* wheel speed measurement variance when rolling clean */
    float max_decel;    /* m/s^2, hard limit on how fast the estimate may fall */
    float init_var_v;
    float init_var_b;
} vehicle_speed_cfg_t;

typedef struct {
    const vehicle_speed_cfg_t *cfg;
    float v;            /* m/s */
    float bias;         /* m/s^2 */
    float P[2][2];
    float innovation;   /* last (z - v), for diagnostics */
    int   corrected;    /* was the last update actually corrected by the wheel */
} vehicle_speed_t;

void vehicle_speed_init(vehicle_speed_t *vs, const vehicle_speed_cfg_t *cfg);

void vehicle_speed_reset(vehicle_speed_t *vs, float v);
void vehicle_speed_predict(vehicle_speed_t *vs, float a_long, float dt);
void vehicle_speed_correct(vehicle_speed_t *vs, float v_wheel, float trust);
void vehicle_speed_correct_vel(vehicle_speed_t *vs, float v_wheel, float trust);
float vehicle_speed_trust(float slip, int braking, float v_wheel, float v_est);

extern const vehicle_speed_cfg_t vehicle_speed_cfg_default;

#endif
