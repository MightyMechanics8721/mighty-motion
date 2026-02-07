package org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

//import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain.state;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.calculateDistance;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
public class Shooter {
    private static Shooter instance;
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
    private final MotorController motorController;

    /**
     * Constructs a new Shooter mechanism and initializes its motor controller.
     *
     * @param hardwareMap the FTC HardwareMap used to retrieve motor hardwar
     */
    private Shooter(HardwareMap hardwareMap) {

        this.motorController = new MotorController(
                hardwareMap,
                new String[]{
                        CONFIGURATION_NAMES.shooterMotor1Name,
                        CONFIGURATION_NAMES.shooterMotor2Name
                },
                BATTERY_PARAMETERS.maxVoltage,
                CONFIGURATION_NAMES.encoderName,
                28.0
        );

        this.motorController.setVelocityPIDConstants(MOTOR_CONTROLLER_CONSTANTS.pidConstants);
        this.motorController.setVelocityFeedForwardConstants(MOTOR_CONTROLLER_CONSTANTS.ffConstants);
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
        return this.motorController.getVelocity();
    }

    public double calculateVelocity(double distance) {
        return 2695 + (-9.78 * distance) + 0.0811 * distance * distance
                + 0.000264 * distance * distance * distance;
    }

    public Action autoShoot() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ////                double distance = calculateDistance(state.get(0, 0), state
                //                .get(1, 0), -60, -60);
                ////                Shooter.this.motorController.setVelocity(
                ////                        calculateVelocity(distance) * 2 * Math.PI / 60);
                ////                telemetryPacket.put("Distance bot to goal (in) ", distance);
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
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shooter.motorController.setVelocity(velocity);

                return false;
            }
        };
    }

    public Action setShooterVelocityInstant(double velocity) {
        Shooter shooter = this;

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shooter.motorController.setVelocity(velocity);
                return Math.abs(shooter.motorController.getVelocity() - velocity)
                        >= shooter.SHOOTER_CONSTANTS.velocityTolerance;
            }
        };
    }

    public Action setShooterVelocityTimed(double velocity, double seconds) {
        return new Action() {
            double time = -1;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (time < 0) {
                    timer.reset();
                }
                time = timer.seconds();
                Shooter.this.motorController.setVelocity(velocity);
                return time <= seconds;
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
        public FFConstants ffConstants = new FFConstants(0, 0.00189, 0);

        /**
         * PID constants used for velocity control.
         */
        public PIDConstants pidConstants = new PIDConstants(0.009, 0, 0);
    }

    public static class HardwareConstants {
        double velocityTolerance = 2.0; // (rad/s)
    }
}