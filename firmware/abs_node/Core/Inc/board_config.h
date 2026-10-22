#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef  htim1;   /* L9110S        */
extern TIM_HandleTypeDef  htim3;   /* encoder          */
extern TIM_HandleTypeDef  htim6;   /* 1 kHz loop tick  */
extern I2C_HandleTypeDef  hi2c1;   /* MPU-6500         */
extern UART_HandleTypeDef huart2;  /* telemetry        */

#define BOARD_TIM_PWM      (&htim1)
#define BOARD_TIM_ENCODER  (&htim3)
#define BOARD_TIM_TICK     (&htim6)
#define BOARD_I2C_IMU      (&hi2c1)
#define BOARD_UART_TELEM   (&huart2)

/* ---- PWM ---------------------------------------------------------------- */

#define BOARD_PWM_CH_IA    TIM_CHANNEL_1
#define BOARD_PWM_CH_IB    TIM_CHANNEL_2


#define BOARD_PWM_ARR      8999u

/* ---- encoder ------------------------------------------------------------ */

/* TIM3 is a 16-bit timer. wheel_speed.c needs this to unwrap the counter. */
#define BOARD_ENCODER_BITS 16

/* ---- LED and brake input ------------------------------------------------ */

#define BOARD_LED_PORT     GPIOA
#define BOARD_LED_PIN      GPIO_PIN_5

#define BOARD_BRAKE_PORT   GPIOC
#define BOARD_BRAKE_PIN    GPIO_PIN_13
#define BOARD_BRAKE_ACTIVE GPIO_PIN_RESET

/* ---- MPU-6500 ----------------------------------------------------------- */
#define BOARD_IMU_ADDR     (0x68u << 1)

/* ---- clock -------------------------------------------------------------- */

#define BOARD_SYSCLK_MHZ   180u

/* ---- rig behaviour ------------------------------------------------------ */

#define BOARD_DRIVE_DUTY   0.45f

#define BOARD_TELEM_HZ     50u

#endif 
