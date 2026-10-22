#include "test_util.h"

#include "abs_config.h"
#include "abs_ctrl.h"

#define DT ABS_LOOP_DT_S

static float drive(abs_ctrl_t *c, float demand, float v_ref, float v_wheel,
                   float omega_dot, int ticks)
{
    float duty = c->duty;
    int i;
    for (i = 0; i < ticks; i++) {
        duty = abs_ctrl_update(c, demand, v_ref, v_wheel, omega_dot, DT);
    }
    return duty;
}

/* ================================================================== */
/* Slip ratio                                                          */
/* ================================================================== */

static void test_slip_basic(void)
{
    CHECK_NEAR(abs_slip_ratio(4.0f, 4.0f), 0.0f, 1e-6f);
    CHECK_NEAR(abs_slip_ratio(4.0f, 3.2f), 0.2f, 1e-6f);
    CHECK_NEAR(abs_slip_ratio(4.0f, 0.0f), 1.0f, 1e-6f);
}

static void test_slip_clamps(void)
{
    /* A wheel turning faster than the vehicle is traction, not braking slip,
       and this controller has nothing to say about it. */
    CHECK_NEAR(abs_slip_ratio(4.0f, 5.0f), 0.0f, 1e-6f);
    CHECK_NEAR(abs_slip_ratio(4.0f, -1.0f), 1.0f, 1e-6f);
}

static void test_slip_undefined_at_rest(void)
{
    CHECK_NEAR(abs_slip_ratio(0.0f, 0.0f), 0.0f, 1e-6f);
    CHECK_NEAR(abs_slip_ratio(0.01f, 0.0f), 0.0f, 1e-6f);
}

/* ================================================================== */
/* Basic application                                                   */
/* ================================================================== */

static void test_idle_with_no_demand(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    CHECK_NEAR(drive(&c, 0.0f, 4.0f, 4.0f, 0.0f, 50), 0.0f, 1e-6f);
    CHECK_EQ(c.state, ABS_STATE_IDLE);
    CHECK(!c.active);
}

static void test_apply_ramps_up(void)
{
    abs_ctrl_t c;
    float duty;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    duty = drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 50);
    CHECK_EQ(c.state, ABS_STATE_APPLY);
    CHECK_NEAR(duty, ABS_RATE_APPLY * 50.0f * DT, 0.02f);
    CHECK(!c.engaged);
}

static void test_duty_never_exceeds_demand(void)
{
    abs_ctrl_t c;
    float duty;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    duty = drive(&c, 0.30f, 4.0f, 4.0f, 0.0f, 2000);
    CHECK_NEAR(duty, 0.30f, 1e-5f);
}

static void test_release_resets_the_event(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 2.0f, -3000.0f, 40);
    CHECK(c.engaged);
    CHECK(c.cycles_event > 0u);

    drive(&c, 0.0f, 4.0f, 4.0f, 0.0f, 5);
    CHECK_EQ(c.state, ABS_STATE_IDLE);
    CHECK_EQ(c.cycles_event, 0);
    CHECK(!c.engaged);
    /* The lifetime counter is diagnostics and must survive. */
    CHECK(c.cycles > 0u);
}

/* ================================================================== */
/* The two dump triggers                                               */
/* ================================================================== */

static void test_dump_on_slip(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 30);
    CHECK_EQ(c.state, ABS_STATE_APPLY);
    drive(&c, 1.0f, 4.0f, 2.8f, 0.0f, 2);
    CHECK_EQ(c.state, ABS_STATE_DUMP);
    CHECK(c.engaged);
}

static void test_dump_on_wheel_deceleration_without_a_reference(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 30);
    drive(&c, 1.0f, 4.0f, 4.0f, -3000.0f, 2);
    CHECK_EQ(c.state, ABS_STATE_DUMP);
}

static void test_no_dump_under_normal_hard_braking(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    drive(&c, 1.0f, 4.0f, 4.0f, -353.0f, 100);
    CHECK_EQ(c.state, ABS_STATE_APPLY);
    CHECK(!c.engaged);
}

/* ================================================================== */
/* The modulation cycle                                                */
/* ================================================================== */

static void test_dump_respects_its_floor(void)
{
    abs_ctrl_t c;
    float duty;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    CHECK_NEAR(c.duty, 1.0f, 1e-5f);
    duty = drive(&c, 1.0f, 4.0f, 2.8f, -3000.0f, 20);
    CHECK_EQ(c.state, ABS_STATE_DUMP);
    CHECK(duty >= ABS_DUMP_FLOOR_FRAC * 1.0f - 1e-4f);
}

static void test_recovery_leads_to_hold_then_reapply(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    drive(&c, 1.0f, 4.0f, 2.8f, -3000.0f, 10);
    CHECK_EQ(c.state, ABS_STATE_DUMP);
    drive(&c, 1.0f, 4.0f, 3.95f, 800.0f, 2);
    CHECK_EQ(c.state, ABS_STATE_HOLD);
    drive(&c, 1.0f, 4.0f, 3.95f, 800.0f, 20);
    CHECK_EQ(c.state, ABS_STATE_REAPPLY);
}

static void test_reapply_is_slower_near_the_known_limit(void)
{
    abs_ctrl_t c;
    float d0, d1, fast_rate, slow_rate;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    drive(&c, 1.0f, 4.0f, 2.8f, -3000.0f, 10);
    drive(&c, 1.0f, 4.0f, 3.95f, 800.0f, 25);
    CHECK_EQ(c.state, ABS_STATE_REAPPLY);

    d0 = c.duty;
    drive(&c, 1.0f, 4.0f, 3.95f, 800.0f, 10);
    fast_rate = (c.duty - d0) / (10.0f * DT);
    drive(&c, 1.0f, 4.0f, 3.95f, 800.0f, 200);
    d1 = c.duty;
    drive(&c, 1.0f, 4.0f, 3.95f, 800.0f, 10);
    slow_rate = (c.duty - d1) / (10.0f * DT);
    CHECK(slow_rate < fast_rate);
}

static void test_cycles_are_counted(void)
{
    abs_ctrl_t c;
    int i;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    for (i = 0; i < 4; i++) {
        drive(&c, 1.0f, 4.0f, 2.8f, -3000.0f, 10);   /* lock   */
        drive(&c, 1.0f, 4.0f, 3.95f, 800.0f, 40);    /* recover */
    }
    CHECK_EQ(c.cycles_event, 4);
    CHECK(abs_ctrl_cycle_hz(&c) > 0.0f);
}

/* ================================================================== */
/* Giving up, on purpose                                               */
/* ================================================================== */

static void test_low_speed_cutout_passes_demand_through(void)
{
    abs_ctrl_t c;
    float duty;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    duty = drive(&c, 0.8f, 0.2f, 0.0f, -3000.0f, 50);
    CHECK_EQ(c.state, ABS_STATE_CUTOUT);
    CHECK_NEAR(duty, 0.8f, 1e-5f);
    CHECK(!c.active);
}

static void test_abdicates_when_the_surface_cannot_be_regulated(void)
{
    abs_ctrl_t c;
    float duty;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    duty = drive(&c, 1.0f, 4.0f, 0.0f, -6000.0f, 3000);

    CHECK(c.abdicated);
    CHECK_EQ(c.state, ABS_STATE_ABDICATE);
    CHECK_NEAR(duty, 1.0f, 1e-5f);
    CHECK(!c.active);
}

static void test_abdication_clears_on_the_next_brake_application(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    drive(&c, 1.0f, 4.0f, 0.0f, -6000.0f, 3000);
    CHECK(c.abdicated);
    drive(&c, 0.0f, 4.0f, 4.0f, 0.0f, 5);
    CHECK(!c.abdicated);
    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 10);
    CHECK_EQ(c.state, ABS_STATE_APPLY);
}

static void test_a_normal_stop_does_not_abdicate(void)
{
    abs_ctrl_t c;
    int i;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    for (i = 0; i < 8; i++) {
        drive(&c, 1.0f, 4.0f, 2.8f, -3000.0f, 10);
        drive(&c, 1.0f, 4.0f, 3.95f, 800.0f, 40);
    }
    CHECK(!c.abdicated);
}


static void test_dead_encoder_degrades_to_base_braking(void)
{
    abs_ctrl_t c;
    float duty;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    duty = drive(&c, 1.0f, 4.0f, 0.0f, 0.0f, 2000);

    CHECK(c.abdicated);
    CHECK_EQ(c.state, ABS_STATE_ABDICATE);
    CHECK_NEAR(duty, 1.0f, 1e-5f);
}

static void test_abdication_is_judged_on_output_not_estimate(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 3);
    drive(&c, 1.0f, 4.0f, 2.0f, -4000.0f, 2);
    CHECK_EQ(c.state, ABS_STATE_DUMP);
    CHECK(!c.abdicated);
    drive(&c, 1.0f, 4.0f, 0.0f, -4000.0f, 2000);
    CHECK(c.abdicated);
}

static void test_led_distinguishes_abdicated_from_solid(void)
{
    abs_ctrl_t c;
    uint32_t t;
    int flash_edges = 0, slow_edges = 0;
    int prev;

    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    drive(&c, 1.0f, 4.0f, 0.0f, -6000.0f, 3000);
    CHECK(c.abdicated);
    CHECK_EQ(abs_ctrl_led_mode(&c), ABS_LED_ABDICATED);
    prev = abs_ctrl_led_level(ABS_LED_FLASH, 0u);
    for (t = 1u; t < 1000u; t++) {
        int now = abs_ctrl_led_level(ABS_LED_FLASH, t);
        if (now != prev) {
            flash_edges++;
        }
        prev = now;
    }
    prev = abs_ctrl_led_level(ABS_LED_ABDICATED, 0u);
    for (t = 1u; t < 1000u; t++) {
        int now = abs_ctrl_led_level(ABS_LED_ABDICATED, t);
        if (now != prev) {
            slow_edges++;
        }
        prev = now;
    }
    CHECK(slow_edges > 0);
    CHECK(flash_edges > slow_edges * 3);
}

/* ================================================================== */
/* LED                                                                 */
/* ================================================================== */

static void test_led_off_when_not_braking(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    drive(&c, 0.0f, 4.0f, 4.0f, 0.0f, 10);
    CHECK_EQ(abs_ctrl_led_mode(&c), ABS_LED_OFF);
    CHECK_EQ(abs_ctrl_led_level(ABS_LED_OFF, 0u), 0);
    CHECK_EQ(abs_ctrl_led_level(ABS_LED_OFF, 12345u), 0);
}

static void test_led_solid_when_braking_normally(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 30);
    CHECK_EQ(abs_ctrl_led_mode(&c), ABS_LED_SOLID);
    CHECK_EQ(abs_ctrl_led_level(ABS_LED_SOLID, 0u), 1);
    CHECK_EQ(abs_ctrl_led_level(ABS_LED_SOLID, 999u), 1);
}

static void test_led_flashes_while_modulating(void)
{
    abs_ctrl_t c;
    uint32_t t;
    int on = 0, off = 0;

    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 200);
    drive(&c, 1.0f, 4.0f, 2.8f, -3000.0f, 5);
    CHECK_EQ(abs_ctrl_led_mode(&c), ABS_LED_FLASH);

    /* One full second must contain both states, so the driver can tell
       "braking" from "ABS is working" at a glance. */
    for (t = 0u; t < 1000u; t++) {
        if (abs_ctrl_led_level(ABS_LED_FLASH, t)) {
            on++;
        } else {
            off++;
        }
    }
    CHECK(on > 0 && off > 0);
    CHECK_NEAR((float)on, 500.0f, 60.0f);
}

/* ================================================================== */
/* Degenerate inputs                                                   */
/* ================================================================== */

static void test_zero_dt_is_a_no_op(void)
{
    abs_ctrl_t c;
    float before;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);

    drive(&c, 1.0f, 4.0f, 4.0f, 0.0f, 30);
    before = c.duty;
    CHECK_NEAR(abs_ctrl_update(&c, 1.0f, 4.0f, 2.0f, -9000.0f, 0.0f), before, 1e-9f);
    CHECK_NEAR(c.duty, before, 1e-9f);
}

static void test_demand_is_clamped(void)
{
    abs_ctrl_t c;
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    /* A nonsense demand from a miscalibrated input must not produce a duty the
       PWM layer will wrap around. */
    CHECK(drive(&c, 5.0f, 4.0f, 4.0f, 0.0f, 500) <= 1.0f);
    abs_ctrl_init(&c, &abs_ctrl_cfg_default);
    CHECK(drive(&c, -2.0f, 4.0f, 4.0f, 0.0f, 50) >= 0.0f);
}

int main(void)
{
    test_slip_basic();
    test_slip_clamps();
    test_slip_undefined_at_rest();

    test_idle_with_no_demand();
    test_apply_ramps_up();
    test_duty_never_exceeds_demand();
    test_release_resets_the_event();

    test_dump_on_slip();
    test_dump_on_wheel_deceleration_without_a_reference();
    test_no_dump_under_normal_hard_braking();

    test_dump_respects_its_floor();
    test_recovery_leads_to_hold_then_reapply();
    test_reapply_is_slower_near_the_known_limit();
    test_cycles_are_counted();

    test_low_speed_cutout_passes_demand_through();
    test_abdicates_when_the_surface_cannot_be_regulated();
    test_abdication_clears_on_the_next_brake_application();
    test_a_normal_stop_does_not_abdicate();
    test_dead_encoder_degrades_to_base_braking();
    test_abdication_is_judged_on_output_not_estimate();
    test_led_distinguishes_abdicated_from_solid();

    test_led_off_when_not_braking();
    test_led_solid_when_braking_normally();
    test_led_flashes_while_modulating();

    test_zero_dt_is_a_no_op();
    test_demand_is_clamped();

    return test_report();
}
