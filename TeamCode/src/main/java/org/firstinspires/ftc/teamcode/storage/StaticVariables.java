package org.firstinspires.ftc.teamcode.storage;

import org.ejml.simple.SimpleMatrix;

/**
 * Values carried from Autonomous into TeleOp.
 * <p>
 * Static fields survive between OpModes because the Robot Controller app process stays up. This is
 * the only place that relies on it, and it holds plain numbers only. Never put a motor, servo,
 * sensor or HardwareMap in a static field: those are rebuilt every run.
 */
public final class StaticVariables {

    /** Robot pose at the end of Autonomous: 3x1 x (in), y (in), heading (rad). Null until saved. */
    private static SimpleMatrix robotState;

    /** Turret angle at the end of Autonomous (deg). */
    private static double turretAngle;

    private StaticVariables() {
    }

    /** Stores the pose the robot finished Autonomous at. */
    public static void saveRobotState(SimpleMatrix state) {
        robotState = state.copy();
    }

    /** Stores the turret angle (deg) the robot finished Autonomous at. */
    public static void saveTurretAngle(double angleDeg) {
        turretAngle = angleDeg;
    }

    /** True once an Autonomous has stored a pose this power cycle. */
    public static boolean hasRobotState() {
        return robotState != null;
    }

    /** Stored pose, or fallback when no Autonomous ran. */
    public static SimpleMatrix robotStateOr(SimpleMatrix fallback) {
        return robotState == null ? fallback : robotState;
    }

    /** Stored turret angle (deg). Zero when no Autonomous ran. */
    public static double turretAngle() {
        return turretAngle;
    }

    /** Discards stored values. Use when the robot is repositioned by hand. */
    public static void clear() {
        robotState = null;
        turretAngle = 0;
    }
}
