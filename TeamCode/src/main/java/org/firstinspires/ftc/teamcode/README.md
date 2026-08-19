# TeamCode layout

```
control/      PID, FeedForward, MotionProfile, LowPassFilter, ShooterModel,
              StoppingDistance and their constants. Pure maths, no hardware, all tested
drivetrain/   Drivetrain, TwoWheelOdometery, PoseController, GeometricController,
              DrivetrainMotorController, MecanumKinematicModel, Path, Drawing,
              and their tuners
hardware/     DcMotorAdvanced, ServoAdvanced, Encoder, Battery, DistanceSensor,
              GoBildaPinpointDriver
mechanisms/   Shooter, Turret, Intake, Indexer, Transfer, and their tuners
opmodes/auto/     the 12 competition routines and the Robot they share
opmodes/teleop/   the driver-control OpModes
storage/      StaticVariables, the only values carried from auto into teleop
util/         Utils (geometry and unit conversions), Timed (action timers)
```

`control/` never mentions our robot, which is why it is the part with unit tests.

## Auto to teleop handoff

`storage/StaticVariables` is the single home for values that outlive an OpMode. Autonomous saves:

```java
StaticVariables.saveRobotState(drivetrain.state);   // 3x1 x (in), y (in), heading (rad)
StaticVariables.saveTurretAngle(turret.getAngle()); // deg
```

TeleOp reads, with a fallback for when no Autonomous ran:

```java
SimpleMatrix startPose = StaticVariables.robotStateOr(Utils.makePoseVector(-51, -51, 45));
```

Autonomous saves twice: continuously from `updateRobotState()` inside the action tree, and once
more from `Robot.saveFinalState()` in a `finally` around `runBlocking`.

`Drivetrain.state` only refreshes inside a drivetrain action's `run()`. Once a routine stops
driving, the cached pose freezes, so `updateRobotState()` re-saves the same value for the rest of
the period. `saveFinalState()` calls `localize()` before reading, which is the only save that
reflects where the robot actually finished.

Only plain numbers go in here. A `HardwareMap` is rebuilt every OpMode, so a motor, servo or sensor
kept in a static would be a dead handle on the next run.

Do not add per-OpMode static holders. They were previously spread across the auton classes, and
two teleops read a field nothing wrote.

## Timing an action

`util/Timed` wraps any Action with a clock. The clock starts on the first `run()`, so a routine can
build its whole tree up front.

```java
Timed.deadline(drivetrain.goToPose(pose), 5)          // ends on arrival or at 5 s
Timed.deadline(action, 5, drivetrain::stopMotorsNow)  // and cuts power if the clock wins
Timed.forDuration(shooter.revShooter(v), 2)           // runs the full 2 s either way
Timed.once(action)                                    // one cycle, then done
Timed.wait(0.5)                                       // pause
```

Prefer these over another hand-rolled `ElapsedTime` inside an anonymous Action.

## Mechanism singletons

Every mechanism is a singleton. `initialize(hardwareMap)` builds a fresh instance and must be called
at the start of each OpMode; `getInstance()` throws if you forget.

`initialize()` also resets the runtime statics that class owns, so a value cannot leak into the
next run. Tuning knobs are left alone.

Do not switch these to lazy construction. `HardwareMap` is rebuilt every run, so an instance cached
from a previous run holds dead device handles.

Do not call another mechanism's `initialize()` from inside a constructor either — that swaps the
singleton out from under any OpMode that already captured it.

## Units

| Quantity | Unit | Notes |
| --- | --- | --- |
| Field position | in | |
| Drivetrain heading | rad | `Drivetrain.setInitialPose` is the exception and takes deg |
| Drivetrain velocity | in/s, rad/s | body frame |
| Turret angle | deg | wrapped to (-180, 180] by `Utils.angleWrapDegrees` |
| Turret velocity | deg/s | |
| Shooter velocity | rad/s | every entry point, no exceptions |
| `Shooter.calculateVelocity` | rev/min | the one rev/min value; convert at the call |
| Encoder position | ticks | |
| Encoder velocity | rad/s | |
| Time | s | |

Convert only at the boundary, using named helpers, never an inline expression:

```java
Utils.rpmToRadPerSec(rpm)     Utils.radPerSecToRpm(radPerSec)
Utils.angleWrap(radians)      Utils.angleWrapDegrees(degrees)
Math.toRadians(deg)           Math.toDegrees(rad)
```

Degrees appear only where a human types the number: dashboard fields, match constants, and
`setInitialPose`. Everything downstream is radians.

Field frame for position and heading, body frame for velocities: +x forwards, +y left, +heading
counter-clockwise. Wheel order is always lf, lb, rb, rf.

## Conventions

- Comments go above the method, not inside it.
- Statics hold numbers only, for values that outlive an OpMode or are tuned on the dashboard.

## Tests

```
./gradlew :TeamCode:testDebugUnitTest
```

A few seconds, no robot. Tests mirror the folders above under `src/test/java`.
