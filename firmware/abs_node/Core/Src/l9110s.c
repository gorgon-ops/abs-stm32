#include "l9110s.h"

static float between_0_and_1(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static void set_register_ccr(uint32_t channel, float duty)
{
    uint32_t ccr = (uint32_t)(between_0_and_1(duty) * (float)BOARD_PWM_ARR + 0.5f);
    __HAL_TIM_SET_COMPARE(BOARD_TIM_PWM, channel, ccr);
}

void l9110s_init(void)
{
    /* Start both channels before setting anything, so the outputs are driven
     * rather than floating while the rest of the system boots. */
    HAL_TIM_PWM_Start(BOARD_TIM_PWM, BOARD_PWM_CH_IA);
    HAL_TIM_PWM_Start(BOARD_TIM_PWM, BOARD_PWM_CH_IB);
    l9110s_stop();
}

void l9110s_drive(float duty)
{
    /* IA carries the PWM, IB stays low. During the low phase of IA both inputs
     * are low, which shorts the motor - so this is brake-mode PWM, and the
     * torque is the drive torque minus the short-brake torque. That is exactly
     * why it is also the release mechanism: see l9110s.h. */
    set_register_ccr(BOARD_PWM_CH_IB, 0.0f);
    set_register_ccr(BOARD_PWM_CH_IA, duty);
}

void l9110s_brake(float duty)
{
    duty = between_0_and_1(duty);

#if BOARD_L9110S_HAS_COAST
    /* A bridge that really does tri-state: alternate short and high-Z, which
     * is the textbook mapping and needs no forward drive. */
    set_register_ccr(BOARD_PWM_CH_IA, duty);
    set_register_ccr(BOARD_PWM_CH_IB, duty);
#else
    /* The real L9110S. Full brake is the bridge doing nothing, so braking less
     * means driving more. The inversion lives here, in one line, in the only
     * file that knows what the hardware actually does. */
    l9110s_drive(1.0f - duty);
#endif
}

void l9110s_stop(void)
{
    set_register_ccr(BOARD_PWM_CH_IA, 0.0f);
    set_register_ccr(BOARD_PWM_CH_IB, 0.0f);
}
