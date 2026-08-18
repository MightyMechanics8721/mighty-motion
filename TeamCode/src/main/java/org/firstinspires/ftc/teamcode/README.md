# TeamCode layout

Seven folders, and inside `opmodes/` a split between auto and teleop. Which folder a class belongs
in is decided by one question: **what does this thing know about?**

```
control/      maths that knows nothing about our robot
drivetrain/   the drive base and everything specific to it
hardware/     thin wrappers over physical devices
mechanisms/   the things bolted to the drive base
opmodes/
  auto/       the 12 autonomous routines
  teleop/     the driver-control OpModes
tuning/       what runs to find our numbers
util/         small helpers with no home of their own
```

## The folders

### `control/`
Reusable control maths. Nothing here mentions a motor, a wheel, or our robot's dimensions, which is
why it is the part covered by unit tests that run on a laptop.

`PID` · `PIDConstants` · `FeedForward` · `FFConstants` · `PoseConstants` · `MotionProfile` ·
`LowPassFilter` · `LowPassFilterParameters`

### `drivetrain/`
The drive base. `Drivetrain` is the front door; the rest are pieces it delegates to:
`TwoWheelOdometery`, `PoseController`, `GeometricController`, `DrivetrainMotorController`,
`MecanumKinematicModel`, `Path`, `Drawing`.

### `hardware/`
Thin wrappers over physical devices: `DcMotorAdvanced`, `ServoAdvanced`, `Encoder`, `Battery`,
`DistanceSensor`, `GoBildaPinpointDriver`.

### `mechanisms/`
`Shooter`, `Turret`, `Intake`, `Indexer`, `Transfer`, `HardwareConstants`.

Each is a singleton, built the correct way for FTC: `initialize(hardwareMap)` replaces the instance
at the start of every OpMode, and `getInstance()` throws if you forgot. Do not switch these to lazy
construction — the `HardwareMap` is rebuilt every run, so an instance cached from a previous run
holds dead device handles.

### `opmodes/`
`auto/` holds the 12 competition routines and the `Robot` helper they share. `teleop/` holds the
driver-control OpModes. The Blue/Red and Worlds/NoAuto variants are deliberately separate files —
their differences are real, not copy-paste drift.

### `tuning/`
Every tuning and bench-test OpMode, in one place.

### `util/`
`Utils` (angle wrapping, distances, pose vectors) and `StaticVariables` (carries the pose and turret
angle from Autonomous into TeleOp).

## Conventions

**Units.** Inches and radians inside the control code. Degrees only where a human types a number,
converted at that boundary. Fields carrying a unit say so: `// [in]`, `// [rad/s]`.

**Frames.** Field frame for position and heading; body frame for velocities and twists. `+x`
forwards, `+y` to the robot's left, `+heading` counter-clockwise.

**Wheel order.** Always `lf, lb, rb, rf`.

**Comments.** Above the method, describing what it does. Not inside the body.

**Statics.** Only for numbers, and only where a value must outlive an OpMode
(`StaticVariables`) or be editable on FTC Dashboard. Never a raw motor, servo, sensor or
`HardwareMap` outside the `initialize()` pattern above.

## Tests

```
./gradlew :TeamCode:testDebugUnitTest
```

Runs in a few seconds, no robot needed. Tests live under `src/test/java/...` in folders mirroring
these. Anything that does not touch hardware belongs under test.
