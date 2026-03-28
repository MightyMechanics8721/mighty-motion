package org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain.state;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain.THRESHOLD_PARAMETERS;
import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.calculateDistance;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;
import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.FeedForward;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
public class Shooter {
    public static double SHOOTER_SCALE_FACTOR = 1;
    public static double SHOOTER_SECONDS_THRESHOLD = 2;

    /**
     * Configuration parameters for battery behavior.
     */
    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();
    /**
     * Configuration parameters for the motor controller (PID and FF constants).
     */
    public static MotorControllerConstants MOTOR_CONTROLLER_CONSTANTS
            = new MotorControllerConstants();
    /**
     * Constant values for Shooter Hardware
     */
    public static HardwareConstants SHOOTER_CONSTANTS = new HardwareConstants();

    /**
     * Configuration names for hardware mapping.
     */
    public static ConfigurationNames CONFIGURATION_NAMES = new ConfigurationNames();
    public static double openPos = 0.6;
    public static double closePos = 0.85;
    private static Shooter instance;
    public final DcMotorAdvanced shooterMotor1;
    public final DcMotorAdvanced shooterMotor2;
    public final ServoAdvanced hardStop;
    public final PID velocityPidController;
    public final FeedForward velocityFeedForwardController;
    private final Encoder encoder;
    private double autonVelocity = 2500;

    /**
     * Constructs a new Shooter mechanism and initializes its motor controller.
     *
     * @param hardwareMap the FTC HardwareMap used to retrieve motor hardwar
     */
    private Shooter(HardwareMap hardwareMap) {
        this.shooterMotor1 = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, "f1"),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference, true
        );


        this.shooterMotor2 = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, "f2"),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference, true
        );
        this.velocityPidController = new PID(
                MOTOR_CONTROLLER_CONSTANTS.pidConstants,
                PID.functionType.LINEAR
        );
        this.hardStop = new ServoAdvanced(hardwareMap.get(Servo.class, "hardStop"));
        this.velocityFeedForwardController
                = new FeedForward(MOTOR_CONTROLLER_CONSTANTS.ffConstants);
        this.shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "indexer"), 28);

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
     * Returns the current shooter flywheel velocity.
     *
     * @return the current velocity of the shooter flywheel (rad/sec)
     */
    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    /**
     * Automatically calculates shooter power based on distance from robot coordinates to goal
     * coordinate INSTANT action
     */
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
     * Automatically calculates shooter power based on distance from robot DRIFTED coordinates to
     * goal coordinate INSTANT action
     */
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

    /**
     * Automatically calculates shooter power based on distance from robot DRIFTED coordinates to
     * goal coordinate INSTANT action
     */
    public Action autoShootMovingGame(double x, double y, double cutoff) {
        return new Action() {

            double time = -1;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (time < 0) {
                    timer.reset();
                }

                //packet.put("autoShootMoving Timer", time);
                if (timer.seconds() < cutoff) {
                    autoShootMovingFunction(time, x, y);
                    //packet.put("autoShootMoving Done", true);
                    return true;
                }
                return false;
            }
        };
    }

    public double calculateVelocity(double distance) {
        return 8.8 * distance + 1288;
    }

    public Action autonomousVelocityInfinite(double desiredVelo) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power =
                        velocityPidController.calculate(desiredVelo * Math.PI / 30, getVelocity())
                                + velocityFeedForwardController.calculate(
                                desiredVelo * Math.PI / 30, 0);
                shooterMotor1.setPower(power);
                shooterMotor2.setPower(power);
                return true;
            }
        };
    }

    public Action autonomousSetVelocity(double velocity) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                autonVelocity = velocity;
                return false;
            }
        };

    }

    /**
     * Automatically calculates shooter power based on distance from robot DRIFTED coordinates to
     * goal coordinate Timed action
     */
    public Action autoShootMovingTimed(double seconds, double x, double y) {
        return new Action() {
            private double time = -1;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (time < 0) {
                    timer.reset();
                }
                time = timer.seconds();
                //packet.put("shooter time:", time);
                autoShootMovingFunction(seconds, x, y);
                if (time > seconds) {
                    //packet.put("timer", "complete");
                    return false;
                }
                return true;
            }
        };
    }


    /**
     * Sets the desired shooter wheel velocity.
     *
     * @param velocity the target velocity to set for the shooter motors
     */
    public Action setShooterVelocityLoop(double velocity) {
        Shooter shooter = this;
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

    /**
     * Sets the desired shooter wheel velocity.
     *
     * @param velocity the target velocity to set for the shooter motors
     */
    public Action setShooterVelocityInfinite(double velocity) {
        Shooter shooter = this;
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

    /**
     * Sets Shooter velocity until within threshold (currently 2.0 rad/s)
     *
     * @param velocity desired velocity rad/sec
     */
    public Action revShooter(double velocity) {
        Shooter shooter = this;
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (Math.abs(encoder.getVelocity() - velocity)
                        < SHOOTER_CONSTANTS.velocityTolerance) {
                    return false;
                }
                double power = velocityPidController.calculate(velocity, getVelocity())
                        + velocityFeedForwardController.calculate(velocity, 0);
                shooterMotor1.setPower(power);
                shooterMotor2.setPower(power);
                return true;
            }
        };
    }

    public Action setShooterVelocityTimed(double velocity, double seconds) {
        return new Action() {
            double time = -1;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (time < 0) {
                    timer.reset();
                }
                time = timer.seconds();
                double power = velocityPidController.calculate(velocity, getVelocity())
                        + velocityFeedForwardController.calculate(velocity, 0);
                shooterMotor1.setPower(power);
                shooterMotor2.setPower(power);

                return time <= seconds;
            }
        };
    }

    public Action hardStop() {
        return new Action() {
            boolean extend = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (timer.seconds() > 0.3) {
                    if (!extend) {
                        hardStopClose();
                        extend = true;
                    } else {
                        hardStopOpen();
                        extend = false;
                    }
                    timer.reset();
                }
                return true;
            }
        };
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
     * Automatically calculates shooter power based on distance from robot coordinates to goal
     * coordinate
     */
    public void autoShootFunction(double x, double y) {
        Drivetrain drivetrain = Drivetrain.getInstance();
        double distance = calculateDistance(
                drivetrain.state.get(0, 0),
                drivetrain.state.get(1, 0), x,
                y
        );
        double velocity = calculateVelocity(distance) * 2 * Math.PI / 60;
        double power = velocityPidController.calculate(velocity, getVelocity())
                + velocityFeedForwardController.calculate(velocity, 0);
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    /**
     * Automatically calculates shooter power based on distance from robot DRIFTED coordinates to
     * goal coordinate
     */
    public void autoShootMovingFunction(double seconds, double x, double y) {
        Drivetrain drivetrain = Drivetrain.getInstance();
        double distance = calculateDistance(
                drivetrain.preloadPose.get(0, 0),
                drivetrain.preloadPose.get(1, 0), x,
                y
        );
        double velocity = calculateVelocity(distance) * 2 * Math.PI / 60;

        if (seconds <= SHOOTER_SECONDS_THRESHOLD) {
            velocity *= SHOOTER_SCALE_FACTOR;
        }

        //TODO TUNE THIS CONSTANT VALUE
        //        double velocity = 2750 * 2 * Math.PI / 60;
        double power = velocityPidController.calculate(velocity, getVelocity())
                + velocityFeedForwardController.calculate(velocity, 0);
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    public boolean autoShootThresholdCheck() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        double distance = calculateDistance(
                drivetrain.driftedPose.get(0, 0),
                drivetrain.driftedPose.get(1, 0), -60,
                -60
        );
        double velocity = calculateVelocity(distance) * 2 * Math.PI / 60;
        return Math.abs(encoder.getVelocity() - velocity)
                >= SHOOTER_CONSTANTS.velocityTolerance;
    }

    /**
     * Holds configuration names for the shooter hardware. These correspond to names in the robot
     * configuration file.
     */
    public static class ConfigurationNames {

        /**
         * Name of the first shooter motor in the configuration.
         */
        public String shooterMotor1Name = "f1";

        /**
         * Name of the second shooter motor in the configuration.
         */
        public String shooterMotor2Name = "f2";

        /**
         * Name of the encoder associated with the shooter.
         */
        public String encoderName = "f1";
    }

    /**
     * Contains configuration parameters related to the battery. These parameters are used to
     * account for voltage variations.
     */
    public static class BatteryParameters {

        /**
         * Maximum expected voltage of the battery in volts.
         */
        public double maxVoltage = 12.5; // (V)
    }

    /**
     * Holds PID and feedforward constants used by the motor controller. These constants can be
     * tuned via the FTC Dashboard.
     */
    public static class MotorControllerConstants {

        /**
         * Feedforward constants used for velocity control.
         */
        public FFConstants ffConstants = new FFConstants(0.005, 0.0027, 0.12);

        /**
         * PID constants used for velocity control.
         */
        public PIDConstants pidConstants = new PIDConstants(0.11, 0, 0);
    }

    public static class HardwareConstants {
        double velocityTolerance = 2.0; // (rad/s)
    }
}