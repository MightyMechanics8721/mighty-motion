package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

/**
 * Settings shared by every {@link DcMotorAdvanced} on the robot.
 * <p>
 * These live here rather than on a subsystem because they describe the motors themselves. Holding
 * them on the drivetrain meant the shooter, intake and indexer all had to import the drivetrain to
 * find out what voltage to compensate for.
 */
@Config
public final class MotorConstants {

    /** Voltage the power values were tuned at; commands are scaled by this over actual. (V) */
    public static double maxVoltage = 12.5;

    /** Power changes smaller than this are not written through to the motor. */
    public static double acceptablePowerDifference = 0.0001;

    private MotorConstants() {
    }
}
