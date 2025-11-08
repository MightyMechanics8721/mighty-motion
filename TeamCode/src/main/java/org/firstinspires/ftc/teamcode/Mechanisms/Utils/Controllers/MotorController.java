package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * Manages one or more DC motors with optional encoder feedback, PID control, and voltage compensation.
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
     * Adds multiple motors from the hardware map.
     *
     * @param hardwareMap FTC hardware map
     * @param motorNames  array of motor names to initialize
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames) {
        for (String motorName : motorNames) {
            this.motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName)));
        }
    }

    /**
     * Adds one motor from the hardware map.
     *
     * @param hardwareMap FTC hardware map
     * @param motorName   name of the motor
     */
    public MotorController(HardwareMap hardwareMap, String motorName) {
        this(hardwareMap, new String[]{motorName});
    }

    /**
     * Adds multiple motors and an encoder (used for velocity feedback).
     *
     * @param hardwareMap        FTC hardware map
     * @param motorNames         array of motor names
     * @param encoderName        name of encoder motor
     * @param ticksPerRevolution encoder ticks per revolution
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorNames);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * Adds one motor and an encoder (used for velocity feedback).
     *
     * @param hardwareMap        FTC hardware map
     * @param motorName          name of motor
     * @param encoderName        name of encoder motor
     * @param ticksPerRevolution encoder ticks per revolution
     */
    public MotorController(HardwareMap hardwareMap, String motorName, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, new String[]{motorName}, encoderName, ticksPerRevolution);
    }

    /**
     * Adds multiple motors with voltage compensation.
     *
     * @param hardwareMap FTC hardware map
     * @param motorNames  array of motor names
     * @param battery     battery reference for voltage compensation
     * @param maxVoltage  max expected voltage for scaling power
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, Battery battery, double maxVoltage) {
        for (String motorName : motorNames) {
            motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName), battery, maxVoltage));
        }
    }

    /**
     * Adds one motor with voltage compensation.
     *
     * @param hardwareMap FTC hardware map
     * @param motorName   name of motor
     * @param battery     battery reference
     * @param maxVoltage  max expected voltage
     */
    public MotorController(HardwareMap hardwareMap, String motorName, Battery battery, double maxVoltage) {
        this(hardwareMap, new String[]{motorName}, battery, maxVoltage);
    }

    /**
     * Adds multiple motors, encoder, and battery compensation.
     *
     * @param hardwareMap        FTC hardware map
     * @param motorNames         array of motor names
     * @param battery            battery reference
     * @param maxVoltage         max expected voltage
     * @param encoderName        name of encoder motor
     * @param ticksPerRevolution encoder ticks per revolution
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, Battery battery, double maxVoltage, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorNames, battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * Adds one motor, encoder, and battery compensation.
     *
     * @param hardwareMap        FTC hardware map
     * @param motorName          name of motor
     * @param battery            battery reference
     * @param maxVoltage         max expected voltage
     * @param encoderName        name of encoder motor
     * @param ticksPerRevolution encoder ticks per revolution
     */
    public MotorController(HardwareMap hardwareMap, String motorName, Battery battery, double maxVoltage, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorName, battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * Sets the same power to all motors.
     *
     * @param power power value (-1.0 to +1.0)
     */
    public void setPower(double power) {
        for (DcMotorAdvanced motor : motors.values()) {
            motor.setPower(power);
        }
    }

    /**
     * Sets the same power to specific motors.
     *
     * @param motorNames array of motor names
     * @param power      power value (-1.0 to +1.0)
     */
    public void setPower(String[] motorNames, double power) {
        for (String motorName : motorNames) {
            DcMotorAdvanced motor = motors.get(motorName);
            motor.setPower(power);
        }
    }

    /**
     * Sets individual power values to each motor.
     * Order in array matches insertion order in motor map.
     *
     * @param powers array of power values
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
     * Gets measured velocity from encoder.
     *
     * @return velocity in ticks/sec or rad/sec
     */
    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    /**
     * Sets one target velocity for all motors using PID + FeedForward.
     *
     * @param targetVelocity desired velocity
     */
    public void setVelocity(double targetVelocity) {
        double power = this.velocityFeedForwardController.calculate(targetVelocity, 0);
        if (this.encoder != null) {
            double currentVelocity = this.getVelocity();
            power += this.velocityPidController.calculate(currentVelocity, targetVelocity);
        }
        setPower(power);
    }

    /**
     * Sets individual target velocities to motors.
     *
     * @param targetVelocities array of velocities
     */
    public void setVelocity(double[] targetVelocities) {
        int index = 0;
        for (DcMotorAdvanced motor : motors.values()) {
            double power = velocityFeedForwardController.calculate(targetVelocities[index], 5);
            setPower(power);
            index++;
        }
    }

    /**
     * Applies feedforward control constants for velocity control.
     *
     * @param feedforwardConstants feedforward constants
     */
    public void setVelocityFeedForwardConstants(FFConstants feedforwardConstants) {
        this.velocityFeedForwardConstants = feedforwardConstants;
        this.velocityFeedForwardController = new FeedForward(feedforwardConstants);
    }

    /**
     * Applies PID constants for velocity control.
     *
     * @param pidConstants PID constants
     */
    public void setVelocityPIDConstants(PIDConstants pidConstants) {
        this.velocityPIDConstants = pidConstants;
        this.velocityPidController = new PID(pidConstants, PID.functionType.LINEAR);
    }

//    public void setMotorDirection(String motorName, DcMotorSimple.Direction direction) {
//
//        // use the hash map of motors and then make the call below correctly
//
//        this.motors.get(motorName).setDirection(direction);
//
//    }
}