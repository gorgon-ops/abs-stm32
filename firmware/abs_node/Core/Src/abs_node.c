#include "abs_node.h"
#include "board_config.h"
#include "l9110s.h"
#include "abs_config.h"

#include <stdio.h>
#include <string.h>

/* Twenty milliseconds of agreement before a button edge is believed. B1 has no
 * hardware debounce, and at a 1 kHz sample rate a single bounce would look
 * like a brake application and release. */
#define BRAKE_DEBOUNCE_TICKS  20u

/* The wheel must be genuinely still while the gyro bias is measured, so the
 * levelling counter resets if the encoder moves at all. */
#define LEVEL_MAX_COUNT_DRIFT 2

static void cycle_counter_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t worst_delay_Us(const abs_node_t *n)
{
    return n->worst_cycles / BOARD_SYSCLK_MHZ;
}

static void led_write(int on)
{
    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t encoder_raw(void)
{
    return (uint32_t)__HAL_TIM_GET_COUNTER(BOARD_TIM_ENCODER);
}

static int32_t encoder_moved(abs_node_t *n, uint32_t now)
{
    int32_t d = wheel_speed_delta(now, n->last_encoder, BOARD_ENCODER_BITS);
    return (d < 0) ? -d : d;
}

int ABS_initialization(abs_node_t *n)
{
    memset(n, 0, sizeof(*n));

    cycle_counter_init();

    abs_system_init(&n->abs);
    l9110s_init();
    led_write(0);

    if (mpu6500_init(&n->imu, BOARD_I2C_IMU, BOARD_IMU_ADDR) != MPU6500_OK) {

        n->state = NODE_IMU_FAULT;
        l9110s_stop();
        return 0;
    }

    HAL_TIM_Encoder_Start(BOARD_TIM_ENCODER, TIM_CHANNEL_ALL);
    n->last_encoder = encoder_raw();

    n->state = NODE_LEVELLING;
    start_read(&n->imu);
    return 1;
}

static void tick_levelling(abs_node_t *n, uint32_t enc)
{

    led_write((n->ticks & 0x100u) != 0u);
    l9110s_stop();

    if (encoder_moved(n, enc) > LEVEL_MAX_COUNT_DRIFT) {
        n->level_count = 0;
        imu_fusion_init(&n->abs.imu, &imu_fusion_cfg_default);
        return;
    }

    if (!n->sample_is_new) {
        return;
    }

    abs_system_level_sample(&n->abs, &n->sample);
    n->level_count++;

    if (n->level_count >= (uint32_t)ABS_IMU_LEVEL_SAMPLES) {
        if (abs_system_level_finish(&n->abs)) {
            n->state = NODE_RUN;
        } else {
            n->level_count = 0;
        }
    }
}

static void tick_run(abs_node_t *n, uint32_t enc)
{
    float demand, duty;

    /* --- brake input, debounced ------------------------------------------ */
    {
        uint8_t raw = (HAL_GPIO_ReadPin(BOARD_BRAKE_PORT, BOARD_BRAKE_PIN)
                       == BOARD_BRAKE_ACTIVE) ? 1u : 0u;
        if (raw == n->brake_raw) {
            if (n->brake_count < BRAKE_DEBOUNCE_TICKS) {
                n->brake_count++;
                if (n->brake_count == BRAKE_DEBOUNCE_TICKS) {
                    n->brake_stable = raw;
                }
            }
        } else {
            n->brake_raw   = raw;
            n->brake_count = 0;
        }
    }

    demand = n->brake_stable ? 1.0f : 0.0f;

    /* --- the control law ------------------------------------------------- */
    duty = abs_system_update(&n->abs, enc, &n->sample, demand,
                             ABS_LOOP_DT_S, HAL_GetTick());

    /* --- actuation ------------------------------------------------------- */
    if (demand > 0.0f) {
        l9110s_brake(duty);
    } else {

        l9110s_drive(BOARD_DRIVE_DUTY);
    }

    led_write(n->abs.led);
}

void abs_handler(abs_node_t *n)
{
    uint32_t enc;
    uint32_t t_start = DWT->CYCCNT;

    n->ticks++;

    if (n->state == NODE_IMU_FAULT) {
        l9110s_stop();
        led_write((n->ticks & 0x40u) != 0u);   /* fast blink: something is wrong */
        return;
    }

    /* Collect the transfer started last tick, then immediately start the next
     * one so it has a full tick to complete. */
    if (get_dma(&n->imu, &n->sample) == MPU6500_OK) {
        n->have_sample   = 1;
        n->sample_is_new = 1;
    } else if (n->have_sample) {
        n->sample_is_new = 0;

        n->imu_missed++;
    }
    start_read(&n->imu);

    enc = encoder_raw();

    switch (n->state) {
    case NODE_LEVELLING:
        tick_levelling(n, enc);
        break;
    case NODE_RUN:
        tick_run(n, enc);
        break;
    default:
        l9110s_stop();
        break;
    }

    n->last_encoder = enc;

    /* --- did this tick fit in its period? -------------------------------- */
    n->last_cycles = DWT->CYCCNT - t_start;
    if (n->last_cycles > n->worst_cycles) {
        n->worst_cycles = n->last_cycles;
    }
    if (__HAL_TIM_GET_FLAG(BOARD_TIM_TICK, TIM_FLAG_UPDATE) != RESET) {
        n->overruns++;
    }
}

/* ------------------------------------------------------------------------- */
/* Telemetry                                                                  */
/* ------------------------------------------------------------------------- */

#if BOARD_TELEM_HZ > 0u

static char     telem_buf[96];
static uint32_t telem_last;

void abs_node_poll(abs_node_t *n)
{
    uint32_t now = HAL_GetTick();
    int      len;

    if ((now - telem_last) < (1000u / BOARD_TELEM_HZ)) {
        return;
    }
    telem_last = now;

    if (BOARD_UART_TELEM->gState != HAL_UART_STATE_READY) {
        return;   
    }

    len = snprintf(telem_buf, sizeof(telem_buf),
                   "%lu,%d,%d,%d,%d,%d,%d,%d,%lu\r\n",
                   (unsigned long)now,
                   (int)(abs_system_v_ref(&n->abs)   * 1000.0f),
                   (int)(abs_system_v_wheel(&n->abs) * 1000.0f),
                   (int)(abs_system_slip(&n->abs)    * 1000.0f),
                   (int)(n->abs.duty                 * 1000.0f),
                   (int)n->abs.ws.omega_dot,
                   (int)n->abs.ctrl.state,
                   (int)n->abs.led,
                   (unsigned long)n->abs.ctrl.cycles);

    if (len > 0) {
        HAL_UART_Transmit_DMA(BOARD_UART_TELEM, (uint8_t *)telem_buf,
                              (uint16_t)len);
    }
}

#else

void abs_node_poll(abs_node_t *n)
{
    (void)n;
}

#endif
