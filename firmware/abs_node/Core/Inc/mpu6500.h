#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "imu_fusion.h"

typedef enum {
    MPU6500_OK = 0,
    MPU6500_ERR_BUS,
    MPU6500_ERR_WHOAMI,
    MPU6500_ERR_BUSY,
    MPU6500_ERR_STALE
} mpu6500_status_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t           addr;

    uint8_t  raw[14];          /* DMA  */
    volatile uint8_t busy;
    volatile uint8_t fresh;
    volatile uint32_t errors;  
    uint32_t samples;         

    float accel_scale;        
    float gyro_scale;          
} mpu6500_t;

mpu6500_status_t mpu6500_init(mpu6500_t *dev, I2C_HandleTypeDef *hi2c,
                              uint16_t addr_8bit);


mpu6500_status_t start_read(mpu6500_t *dev);
mpu6500_status_t get_dma(mpu6500_t *dev, imu_sample_t *out);
void dma_complete(mpu6500_t *dev, I2C_HandleTypeDef *hi2c);
void i2c_error(mpu6500_t *dev, I2C_HandleTypeDef *hi2c);

#endif 