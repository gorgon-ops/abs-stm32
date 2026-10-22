#ifndef ABS_CONFIG_H
#define ABS_CONFIG_H

/* ---- universal ---------------------------------------------------------- */

#define ABS_G_MPS2              9.80665f
#define ABS_PI                  3.14159265358979f

/* ---- RIG: drivetrain geometry ------------------------------------------- */

#define ABS_WHEEL_RADIUS_M      0.0325f

/* Encoder counts per wheel revolution, AFTER the timer's X4 quadrature
 * multiplication and AFTER any gear reduction between motor and wheel.
 *   counts_per_rev = ppr * 4 * gear_ratio
 * A 11-ppr hall encoder on a 9.6:1 gearbox gives 11 * 4 * 9.6 = 422.4. */
#define ABS_ENCODER_CPR         422.4f

/* ---- control loop ------------------------------------------------------- */

#define ABS_LOOP_HZ             1000.0f
#define ABS_LOOP_DT_S           (1.0f / ABS_LOOP_HZ)

/* ---- wheel speed filtering ---------------------------------------------- */

#define ABS_WS_TAU_SPEED_S      0.006f
#define ABS_WS_TAU_ACCEL_S      0.008f

#define ABS_WS_MIN_VALID_MPS    0.15f

/* ---- IMU fusion --------------------------------------------------------- */

#define ABS_IMU_PITCH_TAU_S     1.20f

#define ABS_IMU_ACCEL_TAU_S     0.005f

#define ABS_IMU_ACCEL_GATE_MPS2 0.60f

#define ABS_IMU_LEVEL_SAMPLES   512

/* ---- vehicle speed observer (2-state Kalman) ---------------------------- */

#define ABS_VS_Q_VEL            0.50f   /* (m/s)^2 per s  - unmodelled accel   */
#define ABS_VS_Q_BIAS           0.002f  /* (m/s^2)^2 per s - accel bias walk   */
#define ABS_VS_R_WHEEL          0.02f   /* (m/s)^2 - wheel speed noise, rolling*/
#define ABS_VS_INIT_VAR_V       1.0f
#define ABS_VS_INIT_VAR_B       0.25f

/* Hard ceiling */
#define ABS_VS_MAX_DECEL_MPS2   (1.30f * ABS_G_MPS2)

/* ---- ABS controller ----------------------------------------------------- */

/* Slip ratio lambda = (v_ref - v_wheel) / v_ref */
#define ABS_SLIP_DUMP           0.18f
#define ABS_SLIP_RECOVER        0.12f

/* Angular deceleration of the wheel */
#define ABS_WHEEL_DECEL_DUMP    (-2000.0f)

#define ABS_WHEEL_ACCEL_RECOVER (20.0f)
#define ABS_RATE_APPLY          8.0f
#define ABS_RATE_DUMP           60.0f
#define ABS_RATE_REAPPLY        6.0f
#define ABS_DUMP_FLOOR_FRAC     0.70f
#define ABS_LOCK_DECAY          0.80f
#define ABS_LOCK_MIN            0.02f
#define ABS_REAPPLY_KNEE_FRAC   0.95f
#define ABS_DUMP_MAX_S          0.030f
#define ABS_HOLD_MAX_S          0.010f
#define ABS_CUTOUT_MPS          0.40f

/* ---- LED ---------------------------------------------------------------- */

#define ABS_LED_FLASH_HZ        10.0f   
#define ABS_LED_SLOW_HZ         2.0f

#endif 