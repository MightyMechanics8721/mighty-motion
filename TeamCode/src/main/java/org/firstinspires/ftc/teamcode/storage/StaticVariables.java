package org.firstinspires.ftc.teamcode.storage;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.field.Alliance;

/**
 * Values carried from Autonomous into TeleOp.
 * <p>
 * Static fields survive between OpModes because the Robot Controller app process stays up. This is
 * the only place that relies on it, and it holds plain numbers only. Never put a motor, servo,
 * sensor or HardwareMap in a static field: those are rebuilt every run.
 */
public final class StaticVariables {

    /**
     * Robot state at the end of Autonomous, as handed over by Drivetrain.state: 6x1 of x (in),
     * y (in), heading (rad), then body-frame vx, vy, omega. Only the first three rows are read
     * back. Null until saved.
     */
    private static SimpleMatrix robotState;

    /** Turret angle at the end of Autonomous (deg). */
    private static double turretAngle;

    /** Alliance the last Autonomous ran as. Null until an Autonomous stores one. */
    private static Alliance alliance;

    private StaticVariables() {
    }

    /** Stores the state the robot finished Autonomous at. Copies, so the caller may reuse it. */
    public static void saveRobotState(SimpleMatrix state) {
        robotState = state.copy();
    }

    /** Stores the turret angle (deg) the robot finished Autonomous at. */
    public static void saveTurretAngle(double angleDeg) {
        turretAngle = angleDeg;
    }

    /** Records the alliance the Autonomous ran as, so TeleOp can cross-check its own. */
    public static void saveAlliance(Alliance value) {
        alliance = value;
    }

    /** True once an Autonomous has stored an alliance this power cycle. */
    public static boolean hasAlliance() {
        return alliance != null;
    }

    /** Alliance the last Autonomous ran as, or null if none has. */
    public static Alliance alliance() {
        return alliance;
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
        alliance = null;
    }
}
