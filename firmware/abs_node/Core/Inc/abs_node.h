#ifndef ABS_NODE_H
#define ABS_NODE_H

#include <stdint.h>
#include "abs.h"
#include "mpu6500.h"

typedef enum {
    NODE_BOOT = 0,
    NODE_IMU_FAULT,
    NODE_LEVELLING,
    NODE_RUN
} node_state_t;

typedef struct {
    node_state_t  state;

    abs_system_t  abs;
    mpu6500_t     imu;
    imu_sample_t  sample;     
    int           have_sample;
    int           sample_is_new;  

    uint32_t      level_count;
    uint32_t      last_encoder;

    /* brake input debounce */
    uint8_t       brake_raw;
    uint8_t       brake_stable;
    uint16_t      brake_count;

 
    uint32_t      ticks;
    uint32_t      overruns;
    uint32_t      imu_missed;
    uint32_t      last_cycles;
    uint32_t      worst_cycles;
} abs_node_t;

uint32_t worst_delay_Us(const abs_node_t *n);

int  ABS_initialization(abs_node_t *n);

void abs_handler(abs_node_t *n);

void abs_node_poll(abs_node_t *n);

#endif 
