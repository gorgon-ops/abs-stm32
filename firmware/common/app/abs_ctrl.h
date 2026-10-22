#ifndef ABS_CTRL_H
#define ABS_CTRL_H

#include <stdint.h>

typedef enum {
    ABS_STATE_IDLE = 0,
    ABS_STATE_APPLY,
    ABS_STATE_DUMP,
    ABS_STATE_HOLD,
    ABS_STATE_REAPPLY,
    ABS_STATE_CUTOUT,
    ABS_STATE_ABDICATE
} abs_state_t;

typedef enum {
    ABS_LED_OFF = 0,
    ABS_LED_SOLID,
    ABS_LED_FLASH,
    ABS_LED_ABDICATED
} abs_led_mode_t;

typedef struct {
    float slip_dump;
    float slip_recover;
    float wheel_decel_dump;
    float wheel_accel_recover;
    float rate_apply;
    float rate_dump;
    float rate_reapply;
    float dump_floor_frac;
    float reapply_knee_frac;
    float dump_max_s;
    float hold_max_s;
    float cutout_mps;
    float lock_decay;
    float lock_min;
} abs_ctrl_cfg_t;

typedef struct {
    const abs_ctrl_cfg_t *cfg;

    abs_state_t state;
    float  duty;
    float  slip;
    float  duty_lock;
    float  t_in_state;

    int    active;
    int    engaged;
    int    abdicated;
    uint32_t cycles;
    uint32_t cycles_event;
    float  event_time_s;
} abs_ctrl_t;

void abs_ctrl_init(abs_ctrl_t *c, const abs_ctrl_cfg_t *cfg);

/* Slip ratio (v_ref - v_wheel) / v_ref, clamped to 0..1 and defined as 0 when
 * the reference is too small to divide by.*/
float abs_slip_ratio(float v_ref, float v_wheel);

float abs_ctrl_update(abs_ctrl_t *c, float brake_demand, float v_ref,
                      float v_wheel, float omega_dot, float dt);

abs_led_mode_t abs_ctrl_led_mode(const abs_ctrl_t *c);
int abs_ctrl_led_level(abs_led_mode_t mode, uint32_t now_ms);

/* Average modulation frequency of the current event, Hz. Real ABS lands
 * between 4 and 20 Hz. */
float abs_ctrl_cycle_hz(const abs_ctrl_t *c);

extern const abs_ctrl_cfg_t abs_ctrl_cfg_default;

#endif 
