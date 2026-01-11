package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
public class Turret {

    // --- PID Constants (Dashboard Tunable) ---
    public static double Kp = 0.01;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    // --- Hardware constants ---
    private final double TICKS_PER_REV = 4000.0;
    private final double GEAR_RATIO = 140.0 / 30;

    // --- Hardware ---
    private final CRServo turretLeft;
    private final CRServo turretRight;
    private final Encoder turretEncoder; // <-- replaced DcMotorEx with Encoder

    // --- Utilities ---
    private final PID pid;
    private final FtcDashboard dashboard;

    // --- Constructor ---
    public Turret(HardwareMap hardwareMap) {
        turretLeft = hardwareMap.get(CRServo.class, "servodotLeft");
        turretRight = hardwareMap.get(CRServo.class, "servodotRight");
        turretEncoder = new Encoder(hardwareMap.get(
                com.qualcomm.robotcore.hardware.DcMotorEx.class,
                "lfm"
        )); // <-- use Encoder wrapper

        dashboard = FtcDashboard.getInstance();
        pid = new PID(Kp, Ki, Kd, PID.functionType.LINEAR);
    }

    // --- Hardware Functions ---

    /**
     * Returns the current turret angle in degrees
     */
    public double getAngle() {
        double ticks = turretEncoder.getCurrentPosition();
        return (ticks / TICKS_PER_REV) * 360.0 / GEAR_RATIO;
    }

    /**
     * Computes PID power to reach a desired angle
     */
    private double computeSpinPower(double desiredAngle) {
        return pid.calculate(desiredAngle, getAngle());
    }

    /**
     * Rotates the turret to a specific angle using PID as a Roadrunner Action
     */
    public Action setTurretAngle(double desiredAngle) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = computeSpinPower(desiredAngle);
                turretLeft.setPower(power);
                turretRight.setPower(power);

                telemetryPacket.put("Target Angle", desiredAngle);
                telemetryPacket.put("Current Angle", getAngle());
                telemetryPacket.put("Power", power);

                // Stop when within 1 degree
                return Math.abs(desiredAngle - getAngle()) < 1.0;
            }
        };
    }

    /**
     * Manual turret control using gamepad stick
     */
    public Action manualControl() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double stickPower = gamepad1.left_stick_y;
                turretLeft.setPower(stickPower);
                turretRight.setPower(stickPower);

                telemetryPacket.put("Manual Power", stickPower);
                return false; // continuous
            }
        };
    }

    // --- Auto-Aim Functions ---

    /**
     * Auto-aim at a field goal using robot pose
     */
    public Action autoAim(Pose2d robotPose, Vector2d goalPos) {
        double angleToGoal = computeRobotRelativeAngle(robotPose, goalPos);
        return setTurretAngle(angleToGoal);
    }

    /**
     * Compute dx from robot to goal
     */
    private double computeDx(Pose2d robotPose, Vector2d goalPos) {
        return goalPos.x - robotPose.position.x;
    }

    /**
     * Compute dy from robot to goal
     */
    private double computeDy(Pose2d robotPose, Vector2d goalPos) {
        return goalPos.y - robotPose.position.y;
    }

    /**
     * Compute field-relative angle to goal in degrees
     */
    private double computeFieldAngle(double dx, double dy) {
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    /**
     * Compute robot-relative angle to goal in degrees
     */
    private double computeRobotRelativeAngle(Pose2d robotPose, Vector2d goalPos) {
        double dx = computeDx(robotPose, goalPos);
        double dy = computeDy(robotPose, goalPos);
        double fieldAngle = computeFieldAngle(dx, dy);
        double robotHeading = Math.toDegrees(robotPose.heading.toDouble());
        double relativeAngle = fieldAngle - robotHeading;

        // Wrap 0–360
        return (relativeAngle % 360 + 360) % 360;
    }
}