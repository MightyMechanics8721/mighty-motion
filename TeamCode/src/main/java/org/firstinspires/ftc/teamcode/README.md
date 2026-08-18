# TeamCode layout

```
control/      PID, FeedForward, MotionProfile, LowPassFilter and their constants
drivetrain/   Drivetrain, TwoWheelOdometery, PoseController, GeometricController,
              DrivetrainMotorController, MecanumKinematicModel, Path, Drawing
hardware/     DcMotorAdvanced, ServoAdvanced, Encoder, Battery, DistanceSensor,
              GoBildaPinpointDriver
mechanisms/   Shooter, Turret, Intake, Indexer, Transfer, HardwareConstants
opmodes/auto/     the 12 competition routines and the Robot they share
opmodes/teleop/   the driver-control OpModes
tuning/       every tuning and bench-test OpMode
util/         Utils (geometry), StaticVariables (auto to teleop handoff)
```

`control/` never mentions our robot, which is why it is the part with unit tests.

## Mechanism singletons

Every mechanism is a singleton. `initialize(hardwareMap)` builds a fresh instance and must be called
at the start of each OpMode; `getInstance()` throws if you forget.

Do not switch these to lazy construction. `HardwareMap` is rebuilt every run, so an instance cached
from a previous run holds dead device handles.

Do not call another mechanism's `initialize()` from inside a constructor either — that swaps the
singleton out from under any OpMode that already captured it.

## Conventions

- Inches and radians in the drivetrain. Degrees at the boundaries a human types into.
- The turret works in degrees throughout, wrapped to (-180, 180] by `Utils.angleWrapDegrees`.
- Shooter velocities are rad/s. Convert RPM at the call site with `* 2 * Math.PI / 60`.
  `autonomousVelocityInfinite` is the exception and takes RPM.
- Field frame for position and heading, body frame for velocities. +x forwards, +y left,
  +heading counter-clockwise.
- Wheel order is always lf, lb, rb, rf.
- Comments go above the method, not inside it.
- Statics hold numbers only, for values that outlive an OpMode or are tuned on the dashboard.

## Tests

```
./gradlew :TeamCode:testDebugUnitTest
```

A few seconds, no robot. Tests mirror the folders above under `src/test/java`.
