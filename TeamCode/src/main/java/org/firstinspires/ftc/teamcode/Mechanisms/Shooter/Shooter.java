package org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain.state;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain.THRESHOLD_PARAMETERS;
import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.calculateDistance;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;
import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.FeedForward;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
public class Shooter {
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
                hardwareMap.get(DcMotorEx.class, "f1"),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference
        );
        this.shooterMotor2 = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, "f2"),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference
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
        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rfm"), 28);

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
     * Returns the current shooter wheel velocity.
     *
     * @return the current velocity of the shooter wheels (units depend on encoder configuration)
     */
    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    public double calculateVelocity(double distance) {
        return 2695 + (-9.78 * distance) + 0.0811 * distance * distance
                + 0.000264 * distance * distance * distance;
    }

    public Action autoShoot() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //TODO: INIT DRIVETRAIN TO ACCESS STATE
                //                double distance = calculateDistance(state.get(0, 0), state.get
                //                (1, 0), -60, -60);
                //                double power = calculateVelocity(distance) * 2 * Math.PI / 60;
                //                shooterMotor1.setPower(power);
                //                shooterMotor2.setPower(power);
                //                telemetryPacket.put("Distance bot to goal (in) ", distance);
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
                // TODO: Returning false makes this run once but with the PID this will cause it
                //  to overshoot?
                return false;
            }
        };
    }

    public Action setShooterVelocityInstant(double velocity) {
        Shooter shooter = this;
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = velocityPidController.calculate(velocity, getVelocity())
                        + velocityFeedForwardController.calculate(velocity, 5);
                shooterMotor1.setPower(power);
                shooterMotor2.setPower(power);
                return Math.abs(shooterMotor1.getVelocity() - velocity)
                        >= SHOOTER_CONSTANTS.velocityTolerance;
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
                if (timer.seconds() > 0.3) { //timer just to prevent jittering, since its a toggle
                    if (!extend) {
                        hardStop.setPosition(1);
                    } else {
                        hardStop.setPosition(0);
                    }
                    timer.reset();
                }
                return true;
            }
        };
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
        public FFConstants ffConstants = new FFConstants(0, 0.0025, 0);

        /**
         * PID constants used for velocity control.
         */
        public PIDConstants pidConstants = new PIDConstants(0.0125, 0, 0);
    }

    public static class HardwareConstants {
        double velocityTolerance = 2.0; // (rad/s)
    }
}