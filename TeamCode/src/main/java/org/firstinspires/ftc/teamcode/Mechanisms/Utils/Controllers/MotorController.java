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

import java.util.HashMap;
import java.util.Map;

public class MotorController {
    private HashMap<String, DcMotorAdvanced> motors = new HashMap<>();
    private Battery battery;
    private double maxVoltage;
    private Encoder encoder;


    //CHOICE 3:
    public MotorController(HardwareMap hardwareMap, String[] motorNames) {
        for (String motorName : motorNames) {
            this.motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName)));
        }

    }

    //Choice 6:
    public MotorController(HardwareMap hardwareMap, String motorName) {
        this(hardwareMap, new String[]{motorName});
    }

    //Choice 2
    public MotorController(HardwareMap hardwareMap, String[] motorNames, String encoderName) {
        this(hardwareMap, motorNames);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName));
    }

    //Choice 5
    public MotorController(HardwareMap hardwareMap, String motorName, String encoderName) {
        this(hardwareMap, new String[]{motorName}, encoderName);
    }

    public MotorController(HardwareMap hardwareMap, String[] motorNames, Battery battery, double maxVoltage) {
        for (String motorName : motorNames) {
            motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName), battery, maxVoltage));
        }
    }

    public MotorController(HardwareMap hardwareMap, String motorName, Battery battery, double maxVoltage) {
        this(hardwareMap, new String[]{motorName}, battery, maxVoltage);
    }

    public MotorController(HardwareMap hardwareMap, String[] motorNames, Battery battery, double maxVoltage, String encoderName) {
        this(hardwareMap, motorNames, battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName));
    }

    public MotorController(HardwareMap hardwareMap, String motorName, Battery battery, double maxVoltage, String encoderName) {
        this(hardwareMap, motorName, battery, maxVoltage);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName));
    }

    // write a setpower function which accepts a single double called power and sets that power to ALL
    // motors in our hash map. ChatGPT how to loop through the values of a hash map!
    public void setPower(double power) {

        for (DcMotorAdvanced motor : motors.values()) {
            motor.setPower(power);
        }

    }

    // given a string array of motor names, loop through them and get the motors from the hash map and set motor power
    public void setPower(String[] motorNames, double power) {
        for (String motorName : motorNames) {
            DcMotorAdvanced motor = motors.get(motorName);
            motor.setPower(power);
        }

    }


//    public void setPower(double[] powers) {
//        for (DcMotorAdvanced motor : motors.values()) {
//            int position = 0;
//            motor.setPower(powers[position]);
//            position++;
//
//        }
//    }


//    public MotorController(HardwareMap hardwareMap, String[] motorNames, String encoderNames, Battery battery, double maxVoltage) {
//        for (String motorName : motorNames) {
//            this.motors.put(motorName, new DcMotorAdvanced(hardwareMap.get(DcMotorEx.class, motorName), battery, maxVoltage));
//        }
//    }
//


//    public MotorController(HardwareMap hardwareMap, Battery battery, double maxVoltage, String motorName) {


}


