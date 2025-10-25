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

public class MotorController {
    private HashMap<String, DcMotorAdvanced> motors = new HashMap<>();
    private Battery battery;
    private double maxVoltage;
    private Encoder encoder;

    private PID velocityPidController;


    private PIDConstants velocityPIDConstants;

    private FFConstants velocityFeedForwardConstants;

    private FeedForward velocityFeedForwardController;

    /**
     * Simple case, putting an array of motors into a hardware map
     *
     * @param hardwareMap
     * @param motorNames
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames) {
        for (String motorName : motorNames) {
            this.motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName)));
        }

    }

    /**
     * Putting one motor into the hardware map
     *
     * @param hardwareMap
     * @param motorName
     */
    public MotorController(HardwareMap hardwareMap, String motorName) {
        this(hardwareMap, new String[]{motorName});
    }

    /**
     * Putting many motors into a hardware map with an encoder
     * and giving a ticksPerRevolution to spin at
     *
     * @param hardwareMap
     * @param motorNames
     * @param encoderName
     * @param ticksPerRevolution
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorNames);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * Putting one motor into a hardware map with an encoder
     * and giving a ticksPerRevolution to spin at
     *
     * @param hardwareMap
     * @param motorName
     * @param encoderName
     * @param ticksPerRevolution
     */
    public MotorController(HardwareMap hardwareMap, String motorName, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, new String[]{motorName}, encoderName, ticksPerRevolution);
    }

    /**
     * Putting many motors into a hardware map using a battery, so
     * a max voltage will be passed
     *
     * @param hardwareMap
     * @param motorNames
     * @param battery
     * @param maxVoltage
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, Battery battery, double maxVoltage) {
        for (String motorName : motorNames) {
            motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName), battery, maxVoltage));
        }
    }

    /**
     * Putting a motor into the hardware map with a battery
     * and a max voltage
     *
     * @param hardwareMap
     * @param motorName
     * @param battery
     * @param maxVoltage
     */
    public MotorController(HardwareMap hardwareMap, String motorName, Battery battery, double maxVoltage) {
        this(hardwareMap, new String[]{motorName}, battery, maxVoltage);
    }

    /**
     * Putting many motors into a hardware map with a battery
     * that also has an encoder and making it spin based on ticksPerRevolution
     *
     * @param hardwareMap
     * @param motorNames
     * @param battery
     * @param maxVoltage
     * @param encoderName
     * @param ticksPerRevolution
     */
    public MotorController(HardwareMap hardwareMap, String[] motorNames, Battery battery, double maxVoltage, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorNames, battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * Putting one motor into a hardware map with a
     * battery and spinning at ticksPerRevolution
     *
     * @param hardwareMap
     * @param motorName
     * @param battery
     * @param maxVoltage
     * @param encoderName
     * @param ticksPerRevolution
     */

    public MotorController(HardwareMap hardwareMap, String motorName, Battery battery, double maxVoltage, String encoderName, double ticksPerRevolution) {
        this(hardwareMap, motorName, battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName), ticksPerRevolution);
    }

    /**
     * setting power for motor
     *
     * @param power
     */
    public void setPower(double power) {

        for (DcMotorAdvanced motor : motors.values()) {
            motor.setPower(power);
        }

    }

    /**
     * Setting one power to many motors
     *
     * @param motorNames
     * @param power
     */
    public void setPower(String[] motorNames, double power) {
        for (String motorName : motorNames) {
            DcMotorAdvanced motor = motors.get(motorName);
            motor.setPower(power);
        }

    }

    /**
     * setting many powers to many motors
     *
     * @param powers
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
     * getting velocity from the encoder
     *
     * @return
     */
    public double getVelocity() {

        return this.encoder.getVelocity();

    }

    /**
     * Setting one target velocity for one or many motors
     *
     * @param targetVelocity
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
     * Setting many targetVelocities to many motor
     *
     * @param targetVelocities
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
     * Setting velocity feed forwards based on FF constants
     *
     * @param feedforwardConstants
     */
    public void setVelocityFeedForwardConstants(FFConstants feedforwardConstants) {
        this.velocityFeedForwardConstants = feedforwardConstants;
        this.velocityFeedForwardController = new FeedForward(feedforwardConstants);
    }

    /**
     * Setting velocity PID Constants based on PID constants
     *
     * @param pidConstants
     */
    public void setVelocityPIDConstants(PIDConstants pidConstants) {
        this.velocityPIDConstants = pidConstants;
        this.velocityPidController = new PID(pidConstants, PID.functionType.LINEAR);
    }

}

