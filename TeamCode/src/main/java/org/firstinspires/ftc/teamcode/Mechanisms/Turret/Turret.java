package org.firstinspires.ftc.teamcode.Mechanisms.Turret;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
public class Turret {
    // --- Tunable ---
    public static double staticGain = 0.4;
    public static PIDConstants pidConstants = new PIDConstants(0.02, 0.0, 0.0);
    public static double angleThreshold = 1.0;
    private static Turret instance;
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
    private Turret(HardwareMap hardwareMap) {
        turretLeft = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");
        turretEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lfm"), this.TICKS_PER_REV);
        // <-- use Encoder wrapper

        dashboard = FtcDashboard.getInstance();
        pid = new PID(pidConstants, PID.functionType.LINEAR);
        turretLeft.setDirection(CRServo.Direction.REVERSE);
        turretRight.setDirection(CRServo.Direction.REVERSE);
        turretEncoder.reset();

    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Turret(hardwareMap);
    }

    public static Turret getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Shooter not initialized!");
        }
        return instance;
    }

    // --- Hardware Functions ---

    /**
     * Returns the current turret angle in degrees
     */
    public double getAngle() {
        double ticks = turretEncoder.getCurrentPosition();
        return ((ticks / TICKS_PER_REV) * 360.0 / GEAR_RATIO) % 360;
    }

    /**
     * Computes PID power to reach a desired angle
     */
    private double computeSpinPower(double desiredAngle) {
        double pidOutput = pid.calculate(desiredAngle, getAngle());
        return staticGain * Math.signum(pidOutput) + pidOutput;
    }

    /**
     * Rotates the turret to a specific angle using PID as a Roadrunner Action
     */
    public Action setTurretAngle(double desiredAngle) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = computeSpinPower(clamp(desiredAngle, -90, 90));
                turretLeft.setPower(power);
                turretRight.setPower(power);

                telemetryPacket.put("Target Angle", desiredAngle);
                telemetryPacket.put("Current Angle", getAngle());
                telemetryPacket.put("Power", power);

                // Stop when within 1 degree
                if (Math.abs(desiredAngle - getAngle()) < 1.0) {
                    turretLeft.setPower(0);
                    turretRight.setPower(0);
                    return false;
                }
                return true;
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
    public Action autoAim(Vector2d goalPos) {
        //SimpleMatrix robotState = Drivetrain.state;
        //        Pose2d robotPose = new Pose2d(
        //                robotState.get(0, 0),
        //                robotState.get(1, 0),
        //                robotState.get(2, 0)
        //        );

        //double angleToGoal = computeRobotRelativeAngle(robotPose, goalPos);
        return setTurretAngle(90);
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