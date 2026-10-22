#include "test_util.h"

#include "abs.h"
#include "abs_config.h"
#include "plant.h"

#include <string.h>

#define V0            4.0f
#define time_to_coast       0.30f
#define V_END         0.50f
#define TIMEOUT_S    12.0f

typedef struct {
    float    distance;
    float    time;
    uint32_t cycles;
    float    cycle_hz;
    float    max_vref_error;
    float    mean_slip;
    int      abdicated;
    int      led_changed;
    int      reached_end;
} result_t;

static void run(const surface_t *surf, int abs_enabled, result_t *r)
{
    abs_system_t sys;
    plant_t      p, still;
    plant_cfg_t  pcfg;
    imu_sample_t s;
    float        t, x0 = 0.0f, t0 = 0.0f;
    double       slip_sum = 0.0;
    int          n = 0, braking = 0, i;
    int          led_on = 0, led_off_while_braking = 0;

    memset(r, 0, sizeof(*r));

    pcfg    = plant_cfg_default;
    pcfg.c1 = surf->c1;
    pcfg.c2 = surf->c2;
    pcfg.c3 = surf->c3;

    abs_system_init(&sys);
    sim_init(&still, &pcfg, 0.0f, 12345u);
    for (i = 0; i < ABS_IMU_LEVEL_SAMPLES; i++) {
        step(&still, 0.0f, ABS_LOOP_DT_S);
        plant_imu_sample(&still, &s);
        abs_system_level_sample(&sys, &s);
    }
    abs_system_level_finish(&sys);

    sim_init(&p, &pcfg, V0, 999u);

    for (t = 0.0f; t < TIMEOUT_S; t += ABS_LOOP_DT_S) {
        float demand = (t >= time_to_coast) ? 1.0f : 0.0f;
        float duty;

        if (demand > 0.0f && !braking) {
            braking = 1;
            x0 = p.x;
            t0 = t;
        }

        plant_imu_sample(&p, &s);
        duty = abs_system_update(&sys, plant_encoder_raw(&p), &s, demand,
                                 ABS_LOOP_DT_S, (uint32_t)(t * 1000.0f));
        if (!abs_enabled) {
            duty = demand;
        }

        if (braking) {
            float err = sys.vs.v - p.v;
            if (err < 0.0f) {
                err = -err;
            }
            if (err > r->max_vref_error) {
                r->max_vref_error = err;
            }
            slip_sum += p.lambda;
            n++;

            if (sys.led) {
                led_on = 1;
            } else {
                led_off_while_braking = 1;
            }
        }

        step(&p, duty, ABS_LOOP_DT_S);

        if (braking && p.v <= V_END) {
            break;
        }
    }

    r->reached_end  = (p.v <= V_END);
    r->distance     = p.x - x0;
    r->time         = t - t0;
    r->cycles       = sys.ctrl.cycles;
    r->cycle_hz     = abs_ctrl_cycle_hz(&sys.ctrl);
    r->mean_slip    = n ? (float)(slip_sum / n) : 0.0f;
    r->abdicated    = sys.ctrl.abdicated;
    r->led_changed  = led_on && led_off_while_braking;
}

/* ================================================================== */

static void test_abs_stops_shorter_on_dry_asphalt(void)
{
    result_t a, l;

    run(&plant_surfaces[0], 1, &a);
    run(&plant_surfaces[0], 0, &l);

    CHECK(a.reached_end);
    CHECK(l.reached_end);
    CHECK(a.distance < l.distance * 0.92f);


    CHECK(!a.abdicated);
    CHECK(a.cycles >= 2u);
}

static void test_abs_stops_shorter_on_wet_asphalt(void)
{
    result_t a, l;

    run(&plant_surfaces[1], 1, &a);
    run(&plant_surfaces[1], 0, &l);
    CHECK(a.reached_end);
    CHECK(a.distance < l.distance * 0.90f);
    CHECK(!a.abdicated);
}

static void test_abs_regulates_near_the_friction_peak(void)
{
    result_t a;
    run(&plant_surfaces[0], 1, &a);
    CHECK(a.mean_slip > 0.05f);
    CHECK(a.mean_slip < 0.30f);
}

static void test_modulation_rate_is_physically_sane(void)
{
    result_t a;
    run(&plant_surfaces[0], 1, &a);
    CHECK(a.cycle_hz > 3.0f);
    CHECK(a.cycle_hz < 25.0f);
}

static void test_reference_velocity_tracks_the_vehicle(void)
{
    result_t a;
    run(&plant_surfaces[0], 1, &a);
    CHECK(a.max_vref_error < 0.20f);
}

static void test_led_shows_braking_and_intervention(void)
{
    result_t a;
    run(&plant_surfaces[0], 1, &a);
    CHECK(a.led_changed);
}

static void test_abs_does_no_harm_on_surfaces_it_cannot_regulate(void)
{
    int i;
    for (i = 2; i < PLANT_N_SURFACES; i++) {
        result_t a, l;
        run(&plant_surfaces[i], 1, &a);
        run(&plant_surfaces[i], 0, &l);

        CHECK(a.reached_end);
        CHECK(a.abdicated);
        CHECK(a.distance < l.distance * 1.02f);
    }
}

static void test_no_braking_means_no_intervention(void)
{
    abs_system_t sys;
    plant_t p;
    imu_sample_t s;
    int i;

    abs_system_init(&sys);
    sim_init(&p, &plant_cfg_default, V0, 31337u);
    for (i = 0; i < 3000; i++) {
        plant_imu_sample(&p, &s);
        abs_system_update(&sys, plant_encoder_raw(&p), &s, 0.0f,
                          ABS_LOOP_DT_S, (uint32_t)i);
        step(&p, 0.0f, ABS_LOOP_DT_S);
    }
    CHECK_EQ(sys.ctrl.cycles, 0);
    CHECK_EQ(sys.ctrl.state, ABS_STATE_IDLE);
    CHECK_EQ(sys.led, 0);
    CHECK_NEAR(sys.duty, 0.0f, 1e-6f);
}

int main(void)
{
    test_abs_stops_shorter_on_dry_asphalt();
    test_abs_stops_shorter_on_wet_asphalt();
    test_abs_regulates_near_the_friction_peak();
    test_modulation_rate_is_physically_sane();
    test_reference_velocity_tracks_the_vehicle();
    test_led_shows_braking_and_intervention();
    test_abs_does_no_harm_on_surfaces_it_cannot_regulate();
    test_no_braking_means_no_intervention();

    return test_report();
}
