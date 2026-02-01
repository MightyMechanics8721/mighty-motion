package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Battery {
    private static Battery instance;

    public static double updateFrequency = 2; // (Hz)

    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();

    private double voltage; // (V)

    private Battery(HardwareMap hardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
    }

    public static void initialize(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Battery(hardwareMap);
        }
    }

    public static Battery getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Battery not initialized!");
        }
        return instance;
    }

    public double getVoltage() {
        double timeLastUpdate = timer.seconds();
        if (timeLastUpdate >= 1.0 / updateFrequency) {
            this.voltage = this.voltageSensor.getVoltage();
            timer.reset();
        }
        return voltage;
    }
}
