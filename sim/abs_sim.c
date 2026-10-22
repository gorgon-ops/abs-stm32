#include <stdio.h>
#include <string.h>
#include <math.h>

#include "abs.h"
#include "abs_config.h"
#include "plant.h"

#define speed0        4.0f
#define time_to_coast       0.30f
#define TIMEOUT_S    12.00f
#define V_MEASURE_END 0.50f

typedef struct {
    float    distance_m;
    float    time_s;
    uint32_t cycles;
    float    cycle_hz;
    float    peak_slip;
    float    mean_slip;
    float    max_vref_error;
    float    learned_bias;
    int      abdicated;
    int      timed_out;      /* never reached V_MEASURE_END - distance is not a result */
} sim_result;

/* Stationary levelling, exactly as the firmware does it at power-up. */
static void level_imu(abs_system_t *sys, const plant_cfg_t *pcfg)
{
    plant_t      still;
    imu_sample_t s;
    int          i;

    sim_init(&still, pcfg, 0.0f, 12345u);
    for (i = 0; i < ABS_IMU_LEVEL_SAMPLES; i++) {
        step(&still, 0.0f, ABS_LOOP_DT_S);
        plant_imu_sample(&still, &s);
        abs_system_level_sample(sys, &s);
    }
    abs_system_level_finish(sys);
}

static void run(const surface_t *surf, int abs_enabled, int csv,
                sim_result *r)
{
    abs_system_t sys;
    plant_t      p;
    plant_cfg_t  pcfg;
    imu_sample_t imu;
    float        dt = ABS_LOOP_DT_S;
    float        t = 0.0f, x0 = 0.0f, t0 = 0.0f;
    double       slip_sum = 0.0;
    int          slip_n = 0;
    int          braking = 0;

    memset(r, 0, sizeof(*r));

    pcfg    = plant_cfg_default;
    pcfg.c1 = surf->c1;
    pcfg.c2 = surf->c2;
    pcfg.c3 = surf->c3;

    abs_system_init(&sys);
    level_imu(&sys, &pcfg);
    sim_init(&p, &pcfg, speed0, 999u);

    if (csv) {
        printf("t,v_true,v_ref,v_wheel,slip_true,slip_est,omega,omega_dot,"
               "duty,state,a_true,a_est,pitch_true,pitch_est,led\n");
    }

    while (t < TIMEOUT_S) {
        float demand, duty;

        /* Coast first so the filters settle and the observer primes and learns
         * the accelerometer bias, then ask for everything the brake has. */
        demand = (t >= time_to_coast) ? 1.0f : 0.0f;

        if (demand > 0.0f && !braking) {
            braking = 1;
            x0 = p.x;
            t0 = t;
        }

        plant_imu_sample(&p, &imu);
        duty = abs_system_update(&sys, plant_encoder_raw(&p), &imu, demand, dt,
                                 (uint32_t)(t * 1000.0f));
        if (!abs_enabled) {
            duty = demand;
        }

        if (csv) {
            printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.4f,%d,"
                   "%.4f,%.4f,%.5f,%.5f,%d\n",
                   t, p.v, sys.vs.v, sys.ws.v_wheel, p.lambda, sys.ctrl.slip,
                   p.omega, sys.ws.omega_dot, duty, (int)sys.ctrl.state,
                   p.a_long, sys.imu.a_long, p.pitch, sys.imu.pitch_rad,
                   sys.led);
        }

        if (braking) {
            float err = fabsf(sys.vs.v - p.v);
            if (err > r->max_vref_error) {
                r->max_vref_error = err;
            }
            if (p.lambda > r->peak_slip) {
                r->peak_slip = p.lambda;
            }
            slip_sum += p.lambda;
            slip_n++;
        }

        step(&p, duty, dt);
        t += dt;

        if (braking && p.v <= V_MEASURE_END) {
            break;
        }
    }

    r->abdicated      = sys.ctrl.abdicated;
    r->timed_out      = (p.v > V_MEASURE_END);
    r->distance_m     = p.x - x0;
    r->time_s         = t - t0;
    r->cycles         = sys.ctrl.cycles;
    r->cycle_hz       = abs_ctrl_cycle_hz(&sys.ctrl);
    r->mean_slip      = slip_n ? (float)(slip_sum / slip_n) : 0.0f;
    r->learned_bias   = sys.vs.bias;
}

static float peak_friction_coef(const surface_t *s, float *mu_peak)
{
    plant_cfg_t c = plant_cfg_default;
    float best_l = 0.0f, best_mu = -1.0f, l;

    c.c1 = s->c1;
    c.c2 = s->c2;
    c.c3 = s->c3;

    for (l = 0.0f; l <= 1.0f; l += 0.001f) {
        float mu = plant_mu(&c, l);
        if (mu > best_mu) {
            best_mu = mu;
            best_l  = l;
        }
    }
    *mu_peak = best_mu;
    return best_l;
}

static float ideal_distance(float mu_peak)
{
    /* Constant deceleration at mu_peak * g, from V0 to V_MEASURE_END. */
    float a = mu_peak * ABS_G_MPS2;
    return (speed0 * speed0 - V_MEASURE_END * V_MEASURE_END) / (2.0f * a);
}

int main(int argc, char **argv)
{
    int i;

    if (argc > 1 && strcmp(argv[1], "--csv") == 0) {
        sim_result r;
        run(&plant_surfaces[0], 1, 1, &r);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "--csv-locked") == 0) {
        sim_result r;
        run(&plant_surfaces[0], 0, 1, &r);
        return 0;
    }

    printf("Braking from %.1f m/s to %.1f m/s, one wheel, short-brake H-bridge.\n",
           speed0, V_MEASURE_END);
    printf("Dump threshold configured at %.0f%% slip.\n\n",
           ABS_SLIP_DUMP * 100.0f);

    printf("surface       mu_pk  at     ABS    locked   ideal   gain   "
           "cycles   rate  mean slip\n");
    printf("------------  -----  ---   -----   ------   -----  ------  "
           "------  -----  ---------\n");

    for (i = 0; i < PLANT_N_SURFACES; i++) {
        sim_result a, l;
        float mu_peak, l_peak, ideal, gain;

        l_peak = peak_friction_coef(&plant_surfaces[i], &mu_peak);
        ideal  = ideal_distance(mu_peak);

        run(&plant_surfaces[i], 1, 0, &a);
        run(&plant_surfaces[i], 0, 0, &l);

        gain = 100.0f * (l.distance_m - a.distance_m) / l.distance_m;

        printf("%-12s  %5.2f  %3.0f%%  %6.3f   %6.3f  %6.3f  %+5.1f%%  "
               "%5u  %5.1fHz  %5.1f%%\n",
               plant_surfaces[i].name, mu_peak, l_peak * 100.0f,
               a.distance_m, l.distance_m, ideal, gain,
               a.cycles, a.cycle_hz, a.mean_slip * 100.0f);
        if (a.abdicated) {
            printf("%-12s  (ABS abdicated: no duty both brakes and keeps "
                   "the wheel turning - fell back to base braking)\n", "");
        }
    }

    {
        sim_result a;
        run(&plant_surfaces[0], 1, 0, &a);
        printf("\nobserver on dry asphalt:  worst v_ref error %.3f m/s "
               "(%.1f%% of initial speed), learned accel bias %+.3f m/s^2\n",
               a.max_vref_error, 100.0f * a.max_vref_error / speed0,
               a.learned_bias);
    }

    return 0;
}
