package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;

import java.util.HashMap;
import java.util.Map;

/**
 * MotorController manages one or multiple DC motors,
 * providing control for power, velocity, and PID/Feedforward tuning.
 */
public class MotorController {

    // Map of motor names to motor objects
    private HashMap<String, DcMotorAdvanced> motors = new HashMap<>();

    // Optional battery reference for voltage compensation
    private Battery battery;

    // Maximum voltage used for scaling power
    private double maxVoltage;

    // Optional encoder for velocity feedback
    private Encoder encoder;

    // PID controller for velocity control
    private PID velocityPidController;

    // Stores PID constants for velocity loop
    private PIDConstants velocityPIDConstants;

    // Stores FeedForward constants for velocity loop
    private FFConstants velocityFeedForwardConstants;

    // FeedForward controller used to predict needed power
    private FeedForward velocityFeedForwardController;


    /**
     * Constructor: adds multiple motors from the hardware map
     *
     * @param hardwareMap FTC hardware map
     * @param motorNames  array of motor names
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames) {
        // Loop through every name and create a new motor
        for (String motorName : motorNames) {
            this.motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName)));
        }
    }

    /**
     * Constructor: adds a single motor from hardware map
     */
    public MotorController(HardwareMap hardwareMap, String motorName) {
        this(hardwareMap, new String[]{motorName});
    }

    /**
     * Constructor: multiple motors + encoder (for velocity control)
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorNames);
        // Create encoder for measuring rotational velocity
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * Constructor: single motor + encoder
     */
    public MotorController(HardwareMap hardwareMap, String motorName, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, new String[]{motorName}, encoderName, ticksPerRevolution);
    }

    /**
     * Constructor: multiple motors + battery (for voltage compensation)
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, Battery battery, double maxVoltage) {
        for (String motorName : motorNames) {
            motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName), battery, maxVoltage));
        }
    }

    /**
     * Constructor: single motor + battery + voltage cap
     */
    public MotorController(HardwareMap hardwareMap, String motorName, Battery battery, double maxVoltage) {
        this(hardwareMap, new String[]{motorName}, battery, maxVoltage);
    }

    /**
     * Constructor: multiple motors + encoder + battery
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, Battery battery, double maxVoltage, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorNames, battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * Constructor: single motor + encoder + battery
     */
    public MotorController(HardwareMap hardwareMap, String motorName, Battery battery, double maxVoltage, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorName, battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * Sets one power value to all motors
     */
    public void setPower(double power) {
        // Loops through each motor and applies the same power
        for (DcMotorAdvanced motor : motors.values()) {
            motor.setPower(power);
        }
    }

    /**
     * Sets the same power to specific motors only
     */
    public void setPower(String[] motorNames, double power) {
        for (String motorName : motorNames) {
            DcMotorAdvanced motor = motors.get(motorName);
            motor.setPower(power);
        }
    }

    /**
     * Sets individual power levels to each motor
     * Index order in array must match motor order in HashMap
     */
    public void setPower(double[] powers) {
        int index = 0;
        for (DcMotorAdvanced motor : motors.values()) {
            if (index < powers.length) {
                motor.setPower(powers[index]);
            }
            index++;
        }
    }

    /**
     * Returns velocity measured by encoder (radians/sec or ticks/sec)
     */
    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    /**
     * Sets one target velocity for all motors using PID + FeedForward
     */
    public void setVelocity(double targetVelocity) {
        // Base feedforward estimate (predictive power)
        double power = this.velocityFeedForwardController.calculate(targetVelocity, 0);

        // If encoder exists, adjust power with PID correction
        if (this.encoder != null) {
            double currentVelocity = this.getVelocity();
            power += this.velocityPidController.calculate(currentVelocity, targetVelocity);
        }

        // Apply power to all motors
        setPower(power);
    }

    /**
     * Sets multiple target velocities to corresponding motors
     */
    public void setVelocity(double[] targetVelocities) {
        int index = 0;
        for (DcMotorAdvanced motor : motors.values()) {
            // Predict power for each target velocity
            double power = velocityFeedForwardController.calculate(targetVelocities[index], 5);
            setPower(power); // Apply computed power
            index++;
        }
    }

    /**
     * Configures feedforward controller using FF constants
     */
    public void setVelocityFeedForwardConstants(FFConstants feedforwardConstants) {
        this.velocityFeedForwardConstants = feedforwardConstants;
        this.velocityFeedForwardController = new FeedForward(feedforwardConstants);
    }

    /**
     * Configures PID controller for velocity control using constants
     */
    public void setVelocityPIDConstants(PIDConstants pidConstants) {
        this.velocityPIDConstants = pidConstants;
        this.velocityPidController = new PID(pidConstants, PID.functionType.LINEAR);
    }
}