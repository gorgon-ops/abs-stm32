# Single-channel ABS on an STM32

Anti-lock braking for a one-wheel vehicle: hall encoder for wheel speed,
MPU-6500 for the vehicle speed reference the encoder cannot provide, an L9110S
H-bridge as the brake, and a lamp that tells you when the modulator is working.

**Status:** control law complete and validated in closed-loop simulation
against a quarter-vehicle plant. Board layer written and type-checked; hardware
bring-up is the next step.

---

## The actual problem

ABS regulates wheel slip:

```
        v_vehicle − v_wheel
  λ  =  ───────────────────
             v_vehicle
```

Tyre force peaks near 15–20% slip and falls away after it, so a locked wheel
makes less grip than a slipping one — and no steering force at all.

The awkward part is the numerator. The encoder measures `v_wheel`. Nothing
measures `v_vehicle`, and the sensor that looks like it could becomes wrong
exactly when it matters: a wheel that is locking reads slower than the vehicle,
which *is* the thing being detected. Trust it and the reference collapses onto
the wheel, slip computes as zero all the way down, and the ABS watches a
locking wheel and sees nothing wrong.

That is what the IMU is for. Not tilt compensation as an end in itself — it
exists to hold a velocity reference together through the seconds where the
wheel cannot be believed.

## Results

From 4.0 m/s to 0.5 m/s, closed loop through the real estimator, with a
quantised 16-bit encoder and an accelerometer carrying bias, noise and body
pitch. *ideal* is what a controller holding exactly peak-μ slip throughout
would achieve.

| surface | μ peak | at slip | ABS | locked | ideal | gain |
|---|---|---|---|---|---|---|
| dry asphalt | 1.17 | 17% | **0.795 m** | 0.919 m | 0.686 m | **+13.5%** |
| wet asphalt | 0.80 | 13% | **1.131 m** | 1.434 m | 1.002 m | **+21.1%** |
| snow | 0.19 | 6% | 5.817 m | 6.073 m | 4.226 m | +4.2% (abdicates) |
| ice | 0.05 | 6% | 16.075 m | 16.061 m | 16.061 m | −0.1% (abdicates) |

| metric | value |
|---|---|
| Control loop | 1 kHz |
| Modulation rate | 5.7 Hz dry, 6.0 Hz wet (production ABS: 4–20 Hz) |
| Worst velocity reference error | **0.071 m/s — 1.8% of initial speed** |
| Wheel acceleration noise floor | σ = 77 rad/s², dump threshold at −2000 |
| Host test coverage | 156 assertions |

Reproduce with `make`.

## Repository layout

Everything listed here exists.

```
firmware/common/app/        the control law. no HAL, host-tested, and the
                            same translation units the firmware links
  wheel_speed.c             counter unwrapping, ω and dω/dt
  imu_fusion.c              pitch, and gravity out of the longitudinal axis
  vehicle_speed.c           2-state Kalman: velocity + accelerometer bias
  abs_ctrl.c                the modulator state machine
  abs.c                     the whole channel behind one call
  abs_config.h              every threshold, with why it is that number

firmware/abs_node/          NUCLEO-F446RE board layer
  Core/Inc/board_config.h   pin map and the CubeMX settings to match
  Core/Src/mpu6500.c        I²C driver, DMA, so the loop never waits on it
  Core/Src/l9110s.c         the bridge, and the coast state it does not have
  Core/Src/abs_node.c       1 kHz loop, levelling, lamp, telemetry

sim/plant.c                 quarter-vehicle, Burckhardt tyre, short-brake,
                            and deliberately ugly sensor outputs
sim/abs_sim.c              ABS against a locked wheel across four surfaces

tests/test_estimator.c      62 assertions on the estimator chain
tests/test_ctrl.c           70 assertions on the modulator
tests/test_closed_loop.c    24 assertions on complete braking events
```

## Design decisions

**The wheel gets no vote on the reference while braking.** A braked wheel is
slower than the vehicle or equal to it, never faster, so a correction toward a
slower wheel can only drag the reference down — onto the very signal it exists
to judge. So during braking the observer coasts on the accelerometer, and only
an *upward* correction is accepted, since a wheel turning faster than the
estimate is proof the estimate drifted low. That correction moves velocity only
and never the bias state: a measurement applied in one direction only has
residuals that are not zero-mean, and a bias estimator fed those is poisoned for
every future stop.

**The accelerometer is disqualified as an attitude reference under braking.**
It finds "down" by assuming gravity is all it feels. At 1.2 g that assumption
is off by 50°, and the implied pitch is wrong by more than the entire range the
real pitch covers. The tell is the magnitude: gravity alone reads 1 g. So the
accelerometer is used for pitch only while total specific force is within
0.6 m/s² of 1 g, and the gyro carries the rest. A long filter time constant is
not a substitute — it only slows the error down, and a stop lasts half a second.

**Two dump triggers, either sufficient.** Slip above 18%, *or* wheel
deceleration below −2000 rad/s². The second needs no velocity reference at all
and fires earlier, because a wheel starts decelerating hard before it has
accumulated slip. Depending on slip alone would let a bad reference disable the
ABS silently.

**The dump has a floor and the refill has two stages.** Releasing to zero
throws away the whole brake to fix a wheel that needed slightly less of it, and
the ramp back is spent where the tyre makes no force. The controller releases
to 70% of the duty that caused the lock, refills quickly to just under it, then
creeps.

**Thresholds sit at the physics, not at the sweep optimum.** A parameter sweep
recommends dumping at 24% slip and produces a marginally shorter stop on dry
asphalt. That is past the friction peak; it wins only on this plant, on this
surface. The peak is a property of tyres, so the threshold is set there.

**The controller knows when to stop helping.** On a surface where no duty both
brakes and keeps the wheel turning, modulating makes the stop several times
longer than simply locking up — measured at 26.4 m against 6.1 m on snow. So it
detects that case and hands control back to base braking, and says so on the
lamp with its own blink rate. The alternative to a working ABS is not "no
ABS"; it is an ABS confidently making things worse.

The test for having given up is the duty actually being *commanded*, not the
controller's internal estimate of where the locking point is. Those agree while
the estimate decays and part company when a wheel locks before the apply ramp
has built any duty at all — a stiff brake, or an unplugged encoder reading a
permanently stationary wheel. Judged by the estimate, the controller would
abdicate having never released anything; judged by the output, it cannot.

## What went wrong on the way

Worth recording, because the failures were more instructive than the design:

- **The first complete system made stopping distances 26% worse.** Every unit
  test passed. The trust gate decided how much to believe the wheel *using
  slip*, and slip was computed from the reference the gate was protecting. At
  the moment true slip was 25%, the controller's estimate read 0.0%.
- **The observer learned −1.05 m/s² of accelerometer bias in the first 50 ms**,
  before the brake was ever touched. The wheel-speed filter started at zero and
  ramped to 4 m/s over ~30 ms; the Kalman filter saw velocity climbing while
  the accelerometer read nothing and explained it the only way it could.
- **Body pitch leaked 0.86 rad of error into the attitude estimate.** A 1.2 s
  complementary time constant was not remotely enough. Gating on specific-force
  magnitude took the observer's worst-case error from 95% of initial speed to
  1.8%.
- **An adaptation that fixed snow broke dry asphalt**, dropping it from +13.5%
  to −21%, because it fired on every dump rather than only on dumps that failed
  to recover the wheel.

Each of those is now a test.

## Building

```bash
make
```

Needs only a C compiler. The firmware is built by STM32CubeIDE from
`firmware/abs_node/`.

## Limitations

Simulation only so far: the plant's inertia and brake constant are assumed, not
measured on the rig. On low-μ surfaces the modulator abdicates rather than
regulate (see the table above); a commanded-torque brake would remove that limit.
