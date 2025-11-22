package org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
public class Shooter {

    /**
     * Configuration parameters for battery behavior.
     */
    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();

    /**
     * Configuration parameters for the motor controller (PID and FF constants).
     */
    public static MotorControllerConstants MOTOR_CONTROLLER_CONSTANTS = new MotorControllerConstants();

    /**
     * Configuration names for hardware mapping.
     */
    public static ConfigurationNames CONFIGURATION_NAMES = new ConfigurationNames();

    private Battery battery;
    private MotorController motorController;

    /**
     * Constructs a new Shooter mechanism and initializes its motor controller.
     *
     * @param hardwareMap the FTC HardwareMap used to retrieve motor hardware
     * @param battery     the Battery instance used to monitor voltage and apply compensation
     */
    public Shooter(HardwareMap hardwareMap, Battery battery) {

        this.motorController = new MotorController(hardwareMap,
                new String[]{this.CONFIGURATION_NAMES.shooterMotor1Name,
                        this.CONFIGURATION_NAMES.shooterMotor2Name},
                battery, this.BATTERY_PARAMETERS.maxVoltage,
                this.CONFIGURATION_NAMES.encoderName, 28.0);

        this.motorController.setVelocityPIDConstants(this.MOTOR_CONTROLLER_CONSTANTS.pidConstants);
        this.motorController.setVelocityFeedForwardConstants(this.MOTOR_CONTROLLER_CONSTANTS.ffConstants);
    }

    /**
     * Returns the current shooter wheel velocity.
     *
     * @return the current velocity of the shooter wheels (units depend on encoder configuration)
     */
    public double getVelocity() {
        return this.motorController.getVelocity();
    }

    /**
     * Sets the desired shooter wheel velocity.
     *
     * @param velocity the target velocity to set for the shooter motors
     */
    public void setVelocity(double velocity) {
        this.motorController.setVelocity(velocity);
    }

    /**
     * Holds configuration names for the shooter hardware.
     * These correspond to names in the robot configuration file.
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
     * Contains configuration parameters related to the battery.
     * These parameters are used to account for voltage variations.
     */
    public static class BatteryParameters {

        /**
         * Maximum expected voltage of the battery in volts.
         */
        public double maxVoltage = 12.5; // (V)
    }

    /**
     * Holds PID and feedforward constants used by the motor controller.
     * These constants can be tuned via the FTC Dashboard.
     */
    public static class MotorControllerConstants {

        /**
         * Feedforward constants used for velocity control.
         */
        public FFConstants ffConstants = new FFConstants(0, 0.00189, 0);

        /**
         * PID constants used for velocity control.
         */
        public PIDConstants pidConstants = new PIDConstants(0.008, 0, 0);
    }
}