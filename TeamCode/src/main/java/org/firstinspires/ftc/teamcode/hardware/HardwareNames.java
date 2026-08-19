package org.firstinspires.ftc.teamcode.hardware;

/**
 * Device names as configured on the Robot Controller.
 * <p>
 * Two encoders are wired into ports that belong to another subsystem's motor, so renaming a device
 * here can break something that looks unrelated. Those cases are aliased below rather than repeated
 * as literals.
 */
public final class HardwareNames {

    public static final String LEFT_FRONT_MOTOR = "lfm";
    public static final String LEFT_BACK_MOTOR = "lbm";
    public static final String RIGHT_BACK_MOTOR = "rbm";
    public static final String RIGHT_FRONT_MOTOR = "rfm";

    public static final String ODOMETRY = "odo";

    public static final String SHOOTER_MOTOR_1 = "f1";
    public static final String SHOOTER_MOTOR_2 = "f2";
    public static final String SHOOTER_HARD_STOP = "hardStop";

    public static final String INTAKE_MOTOR = "intake";
    public static final String INDEXER_MOTOR = "indexer";

    public static final String BEAM_BREAK_BOTTOM = "bb1";
    public static final String BEAM_BREAK_MIDDLE = "bb2";
    public static final String BEAM_BREAK_TOP = "bb3";

    public static final String TURRET_LEFT_SERVO = "turretLeft";
    public static final String TURRET_RIGHT_SERVO = "turretRight";

    public static final String CURRENT_SENSOR = "cs";

    /** Turret encoder, wired into the left front drive motor's encoder port. */
    public static final String TURRET_ENCODER = LEFT_FRONT_MOTOR;

    /** Shooter flywheel encoder, wired into the indexer motor's encoder port. */
    public static final String SHOOTER_ENCODER = INDEXER_MOTOR;

    private HardwareNames() {
    }
}
