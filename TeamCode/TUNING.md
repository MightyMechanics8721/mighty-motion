# Tuning

Every constant below is `@Config`, so it can be changed from FtcDashboard without a rebuild.
**Dashboard edits do not survive an app restart** — once a value is right, write it into the
source.

Work top to bottom. Each step assumes the ones above it are done, because they genuinely depend
on each other: the pose gains are only meaningful once the feedforward is right, and the
feedforward is only meaningful once the localizer reports true inches.

---

## 1. Localizer — `Tune 2 Wheel Localizer`

Tunables: `TwoWheelOdometery.xOffset`, `yOffset`, `velocityIsFieldFrame`

**Pod offsets.** Push the robot a measured distance (a field tile is 23.5 in) and check `X [in]`
and `Y [in]` match. Then spin the robot exactly 360° by hand and check `Theta [deg]` returns to
where it started and that X and Y come back to roughly zero. If the position walks during a pure
rotation, the pod offsets are wrong.

**Velocity frame — do this once, it gates everything downstream.** Turn the robot to about 90°
and push it straight forward. Read:

```
resolved long. vel   should be positive and large
resolved lat.  vel   should be near zero
```

If those two are swapped, flip `TwoWheelOdometery.velocityIsFieldFrame` on the dashboard.

This matters because `driftedPose` is built from these velocities and feeds **every** autonomous
path. A wrong frame corrupts it at every heading except zero, and nothing else looks wrong.

---

## 2. Feedforward — `Tune Coast + Top Speed`, axis `FORWARD`

Tunables: `Drivetrain.FF_CONSTANTS.lf/lb/rb/rf` — each has `kS`, `kV`, `kA`

The drivetrain runs **open loop at the wheel**: all four motors are `RUN_WITHOUT_ENCODER` and
nothing reads wheel velocity back. `DrivetrainMotorController` is a plain map,
`power = kV*w + kA*a + kS*sign(w)`, with nothing correcting it. So these gains set the loop gain
of everything above them.

**kS** — the power needed to just barely move. Set `power` low, hold A, and raise `power` a
little at a time until the robot creeps. That value is `kS`.

**kV** — hold A at `power = 1.0` until `peak speed` stops climbing, then read:

```
top speed kV implies (in/s)        what the current kV predicts
kV matching the peak just measured what kV should be
```

If the two disagree, take the measured one.

> **Changing kV means retuning kP.** There is no velocity feedback, so kV and the pose gains
> multiply together. Halving kV doubles the effective pose gain. Do step 4 again afterwards.

**kA** can stay at its current value. Nothing commands a wheel acceleration — every call site
passes a zero acceleration vector — so it has no effect until someone adds motion profiling.

---

## 3. Coast distance — `Tune Coast + Top Speed`, each axis

Tunables: `Drivetrain.STOPPING_DISTANCE_PARAMETERS`
(`xLinear`/`xQuadratic`, `yLinear`/`yQuadratic`, `headingLinear`/`headingQuadratic`)

This is how far the robot carries after power is cut. `goToPose` and `followPath` both steer the
pose the robot *will* end up at, so these being wrong makes the robot stop short or overshoot.

For each of `FORWARD`, `STRAFE`, `TURN`:

1. Set `axis` and `power`.
2. Hold A until the speed plateaus.
3. Press B. Read `speed at cut` and `coast (in)` (or `coast (rad)` for TURN).
4. Repeat at 4 or so different powers to get a spread of speeds.

Fit `coast = linear * speed + quadratic * speed^2` to the pairs. A spreadsheet trendline
(polynomial, order 2, intercept 0) is enough. `linear` is command latency, `quadratic` is braking.

Sanity check: the `linear` term should come out roughly the same on all three axes, since it is
the same loop and the same latency. If forward and heading disagree by a factor of two, one fit
is wrong.

---

## 4. Pose controller — `Blue TeleOp` / `Red TeleOp` and a short auto

Tunables: `Drivetrain.POSE_CONSTANTS.xPIDConstants`, `yPIDConstants`, `headingPIDConstants`

These are **sqrt** controllers, not linear: the output is `kP * sqrt(|error|) * sign(error)`. The
gain therefore rises as the error shrinks, which is what makes it settle hard on the last inch.
Raising `kP` affects the whole approach, not just the far end.

Raise `kP` until the robot reaches the target briskly without oscillating around it. `kI` and
`kD` are zero and should stay zero unless you have a specific reason — `PID` supports them, but
nothing here has needed them.

Verify with `PoseControlSimTest` before going to the field; it reports settle distance and the
tight 0.25 in park case.

---

## 5. Path follower

Tunables: `Drivetrain.FOLLOWER_CONSTANTS` — `xPIDConstants`, `yPIDConstants`,
`headingPIDConstants`, `positionLookahead`, `headingLookahead`, `poseControlHandoffDistance`

`positionLookahead` (in) is how far ahead on the path the robot aims. Larger cuts corners and
runs smoother; smaller tracks the path tightly and can weave. `headingLookahead` is the same idea
for where the robot points.

`poseControlHandoffDistance` is how close to the last waypoint the follower gives up and hands
over to pose control. Keep it at or above `headingLookahead`: inside that radius the heading
lookahead circle reaches past the end of the path and the heading target falls back to the final
waypoint, which is a sane fallback but not what the tuning intends. They are equal today (30 in),
which is why the handoff looks seamless.

**`maxSpeed` is a per-path limiter, not a tuning constant.** Pass a smaller value to a
`followPath` call to run that path slower. It only has authority below the speed the feedforward
saturates at — measured in step 2. Above that it does nothing, so a limit of 40 on a robot that
tops out at 17 in/s is the same as no limit at all.

---

## 6. Shooter — `Tune Shooter`

Tunables: `Shooter.MOTOR_CONTROLLER_CONSTANTS.ffConstants` (`kS`, `kV`), `.pidConstants.kP`

Unlike the drivetrain this **does** close the loop on the flywheel encoder.

Set `targetVelocity` (RPM) and watch `Velocity (RPM)` against it. Raise `kV` until the steady
speed lands near target, then raise `kP` until it recovers quickly after a ball goes through.

> The telemetry in this OpMode is computed separately from the control call on purpose. Do not
> "simplify" it by calling `velocityPidController.calculate` again for display — that advances
> the controller's timer and integral, and the tuner starts lying to you.

---

## 7. Shooter range model — `ShooterModel`

Tunables: `ShooterModel.quadCoeff`, `linearCoeff`, `constant` (all rev/min)

`speed = quad * d^2 + linear * d + constant`, with `d` the distance to
`FieldConstants.shooterRangePoint`.

Park at a measured distance, adjust `Shooter.MOTOR_CONTROLLER_CONSTANTS` targets until shots land,
and record (distance, RPM). Collect 5 or 6 across the range you actually shoot from, then fit a
quadratic.

In TeleOp, `gamepad2` dpad up/down nudges `ShooterModel.constant` live. Note this is a **static**
field, so a nudge persists into the next OpMode run until the app restarts — check it before a
match if someone has been trimming.

---

## 8. Turret — `Tune Turret`, then `TurretStoppingDistance`

Tunables: `Turret.pidConstants.kP`, `Turret.staticGain`, `Turret.linearCoeff`,
`Turret.quadCoeff`, `Turret.THRESHOLD_PARAMETERS.angleThreshold`

Both OpModes bring the Drivetrain up even though they do not drive. That is deliberate: the
turret encoder is wired into the left front drive motor's port, and that motor's direction
decides the sign the encoder reports. Without the Drivetrain initialised the angle reads
backwards from what a match OpMode sees.

**`Tune Turret`** — left stick drives the turret; check `angle` moves the right way and that a
full sweep reads the angle you expect. If it is inverted, the encoder direction is wrong.

**`staticGain`** — smallest power that starts the turret moving.

**`kP`** — raise until it snaps to angle without hunting.

**`TurretStoppingDistance`** — set `power`, hold right bumper to spin, release, and read `Drifted`.
Repeat across several powers and fit `linearCoeff` / `quadCoeff` the same way as step 3. These
let `computeSpinPower` aim at where the turret *will* stop.

---

## 9. Field points — `FieldConstants`

Tunables: `turretAimX` / `turretAimYBlue`, `shooterRangeX` / `shooterRangeYBlue`,
`nearStartX` / `nearStartYBlue` / `nearStartHeadingBlueDeg`

Written blue-side and mirrored for red, so changing one moves both alliances together.

Note the aim point and the range point are deliberately different: the turret points at the goal
itself, while the range model was fitted against distance to a different reference. Move them
independently.

> **This codebase puts blue at negative Y.** The official FTC field coordinate system is the
> opposite — blue is positive Y. Every waypoint here is tuned to the inverted convention, so it is
> not something to fix, but anything arriving in official field coordinates (an AprilTag or
> Limelight pose) needs its Y negated first.

---

## Checking work without a robot

`./gradlew :TeamCode:testDebugUnitTest` runs the unit tests, including a closed-loop simulation of
the pose controller and path follower against the real control classes.

`TuningReportTest` prints what the current constants imply — top speed, whether `maxSpeed` can
bite, coast distance at top speed. Run it after changing `kV` to see the knock-on effects before
going to the field.

The simulation assumes `kV` is correct — it treats a commanded twist as achievable up to the
wheel ceiling. It validates control *logic*, not speed calibration. Its distances only mean
inches once step 2 is done.
