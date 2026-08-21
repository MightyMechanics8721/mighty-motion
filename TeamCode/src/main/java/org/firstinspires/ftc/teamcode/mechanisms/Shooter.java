package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

import org.firstinspires.ftc.teamcode.util.Timed;

import org.firstinspires.ftc.teamcode.util.Utils;

import static org.firstinspires.ftc.teamcode.util.Utils.calculateDistance;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.FFConstants;
import org.firstinspires.ftc.teamcode.control.FeedForward;
import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.control.PIDConstants;
import org.firstinspires.ftc.teamcode.control.ShooterModel;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.DcMotorAdvanced;
import org.firstinspires.ftc.teamcode.hardware.MotorConstants;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.ServoAdvanced;

//import static org.firstinspires.ftc.teamcode.drivetrain.Drivetrain.state;

@Config
public class Shooter {
    public static double SHOOTER_SCALE_FACTOR = 1;
    public static double SHOOTER_SECONDS_THRESHOLD = 2;

    /**
     * Configuration parameters for the motor controller (PID and FF constants).
     */
    public static MotorControllerConstants MOTOR_CONTROLLER_CONSTANTS
            = new MotorControllerConstants();
    public static double openPos = 0.5;
    public static double closePos = 0.65;
    private static Shooter instance;
    public final DcMotorAdvanced shooterMotor1;
    public final DcMotorAdvanced shooterMotor2;
    public final ServoAdvanced hardStop;
    public final PID velocityPidController;
    public final FeedForward velocityFeedForwardController;
    private final Encoder encoder;

    /**
     * Constructs a new Shooter mechanism and initializes its motor controller.
     *
     * @param hardwareMap the FTC HardwareMap used to retrieve motor hardwar
     */
    private Shooter(HardwareMap hardwareMap) {
        this.shooterMotor1 = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, HardwareNames.SHOOTER_MOTOR_1),
                MotorConstants.maxVoltage,
                MotorConstants.acceptablePowerDifference, true
        );

        this.shooterMotor2 = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, HardwareNames.SHOOTER_MOTOR_2),
                MotorConstants.maxVoltage,
                MotorConstants.acceptablePowerDifference, true
        );
        this.velocityPidController = new PID(
                MOTOR_CONTROLLER_CONSTANTS.pidConstants,
                PID.functionType.LINEAR
        );
        this.hardStop = new ServoAdvanced(hardwareMap.get(Servo.class, HardwareNames.SHOOTER_HARD_STOP));
        this.velocityFeedForwardController
                = new FeedForward(MOTOR_CONTROLLER_CONSTANTS.ffConstants);
        this.shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareNames.SHOOTER_ENCODER), 28);

    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Shooter(hardwareMap);
    }

    public static Shooter getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Shooter not initialized!");
        }
        return instance;
    }

    /**
     * @return shooter flywheel velocity (rad/s)
     */
    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    /** Ranges the goal once and holds the flywheel at the modelled speed. */
    public Action autoShoot(double x, double y) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                autoShootFunction(x, y);
                return false;
            }
        };
    }

    /**
     * autoShoot with a trim on the modelled flywheel speed.
     *
     * @param velocityMultiplier scales the modelled speed; 1.0 leaves it alone
     */
    public Action autoShoot(double x, double y, double velocityMultiplier) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                autoShootFunctionMultipliedPower(x, y, velocityMultiplier);
                return false;
            }
        };
    }

    /** Re-ranges the goal every iteration until the action is cancelled. */
    public Action autoShootMovingInfinite(double x, double y) {
        return new Action() {

            double time = -1;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (time < 0) {
                    timer.reset();
                }

                time = timer.seconds();

                autoShootMovingFunction(time, x, y);
                return true;
            }
        };
    }

    /** Tracks the goal for the full duration (s), then finishes. */
    public Action autoShootMovingGame(double x, double y, double cutoff) {
        return Timed.deadline(autoShootMovingInfinite(x, y), cutoff);
    }

    /**
     * @param distance distance to the goal (in)
     *
     * @return flywheel speed (rev/min)
     */
    public double calculateVelocity(double distance) {
        return ShooterModel.velocityForDistance(distance);
    }

    /**
     * Holds the flywheel at a speed until the action is cancelled.
     *
     * @param velocity target flywheel speed (rad/s)
     */
    public Action autonomousVelocityInfinite(double velocity) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = velocityPidController.calculate(velocity, getVelocity())
                        + velocityFeedForwardController.calculate(velocity, 0);
                shooterMotor1.setPower(power);
                shooterMotor2.setPower(power);
                return true;
            }
        };
    }

    /** Tracks the goal for the full duration (s). */
    public Action autoShootMovingTimed(double seconds, double x, double y) {
        return Timed.forDuration(autoShootMovingInfinite(x, y), seconds);
    }

    /**
     * Sets the desired shooter wheel velocity.
     *
     * @param velocity the target velocity to set for the shooter motors
     */
    public Action setShooterVelocityLoop(double velocity) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double ffPower = MOTOR_CONTROLLER_CONSTANTS.ffConstants.kS * Math.signum(velocity)
                        + MOTOR_CONTROLLER_CONSTANTS.ffConstants.kV * velocity;
                double pidPower = MOTOR_CONTROLLER_CONSTANTS.pidConstants.kP * (velocity
                        - getVelocity());
                double power = ffPower + pidPower;
                shooterMotor1.setPower(power);
                shooterMotor2.setPower(power);
                return false;
            }
        };
    }

    /**
     * Sets the desired shooter wheel velocity.
     *
     * @param velocity the target velocity to set for the shooter motors
     */
    public Action setShooterVelocityInfinite(double velocity) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = velocityPidController.calculate(velocity, getVelocity())
                        + velocityFeedForwardController.calculate(velocity, 0);
                shooterMotor1.setPower(power);
                shooterMotor2.setPower(power);
                return false;
            }
        };
    }

    /** Holds the flywheel at velocity (rad/s) for the full duration (s). */
    public Action setShooterVelocityTimed(double velocity, double seconds) {
        return Timed.forDuration(setShooterVelocityInfinite(velocity), seconds);
    }

    public Action hardStopClose() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hardStop.setPosition(closePos);
                return false;
            }
        };
    }

    public Action hardStopOpen() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hardStop.setPosition(openPos);
                return false;
            }
        };
    }

    /**
     * Ranges the goal from the motion-compensated pose and holds the flywheel at the speed the
     * shooter model asks for.
     *
     * @param x goal X (in)
     * @param y goal Y (in)
     */
    public void autoShootFunction(double x, double y) {
        autoShootFunctionMultipliedPower(x, y, 1.0);
    }

    /**
     * autoShootFunction with a trim on the modelled speed.
     *
     * @param x goal X (in)
     * @param y goal Y (in)
     * @param velocityMultiplier scales the modelled flywheel speed; 1.0 leaves it alone
     */
    public void autoShootFunctionMultipliedPower(
            double x, double y, double velocityMultiplier
    ) {
        Drivetrain drivetrain = Drivetrain.getInstance();
        double distance = calculateDistance(
                drivetrain.shootWhileMovingPose.get(0, 0),
                drivetrain.shootWhileMovingPose.get(1, 0), x,
                y
        );
        double velocity = Utils.rpmToRadPerSec(calculateVelocity(distance)) * velocityMultiplier;
        double power = velocityPidController.calculate(velocity, getVelocity())
                + velocityFeedForwardController.calculate(velocity, 0);
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    /**
     * Ranges the goal from the motion-compensated pose and holds the flywheel there.
     * <p>
     * Same pose as autoShootFunction, so how far the shot leads the robot's motion is set in one
     * place, THRESHOLD_PARAMETERS.compensationFactor, for autonomous and TeleOp alike.
     *
     * @param seconds time since the action started (s)
     * @param x goal X (in)
     * @param y goal Y (in)
     */
    public void autoShootMovingFunction(double seconds, double x, double y) {
        Drivetrain drivetrain = Drivetrain.getInstance();
        double distance = calculateDistance(
                drivetrain.shootWhileMovingPose.get(0, 0),
                drivetrain.shootWhileMovingPose.get(1, 0), x,
                y
        );
        double velocity = Utils.rpmToRadPerSec(calculateVelocity(distance));

        if (seconds <= SHOOTER_SECONDS_THRESHOLD) {
            velocity *= SHOOTER_SCALE_FACTOR;
        }

        //TODO TUNE THIS CONSTANT VALUE
        //        double velocity = Utils.rpmToRadPerSec(2750);
        double power = velocityPidController.calculate(velocity, getVelocity())
                + velocityFeedForwardController.calculate(velocity, 0);
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    /**
     * Holds PID and feedforward constants used by the motor controller. These constants can be
     * tuned via the FTC Dashboard.
     */
    public static class MotorControllerConstants {

        /**
         * Feedforward constants used for velocity control.
         */
        public FFConstants ffConstants = new FFConstants(0, 0.002, 0.23);

        /**
         * PID constants used for velocity control.
         */
        public PIDConstants pidConstants = new PIDConstants(0.2, 0, 0);
    }
}