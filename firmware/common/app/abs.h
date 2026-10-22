#ifndef ABS_H
#define ABS_H

#include <stdint.h>
#include "wheel_speed.h"
#include "imu_fusion.h"
#include "vehicle_speed.h"
#include "abs_ctrl.h"

typedef struct {
    wheel_speed_t   ws;
    imu_fusion_t    imu;
    vehicle_speed_t vs;
    abs_ctrl_t      ctrl;

    float    duty;
    int      led;
    int      primed;
    uint32_t ticks;
} abs_system_t;

void abs_system_init(abs_system_t *s);

void abs_system_level_sample(abs_system_t *s, const imu_sample_t *imu);
int  abs_system_level_finish(abs_system_t *s);

float abs_system_update(abs_system_t *s, uint32_t enc_raw,
                        const imu_sample_t *imu, float brake_demand,
                        float dt, uint32_t now_ms);

float abs_system_v_ref(const abs_system_t *s);
float abs_system_v_wheel(const abs_system_t *s);
float abs_system_slip(const abs_system_t *s);

#endif /* ABS_H */
