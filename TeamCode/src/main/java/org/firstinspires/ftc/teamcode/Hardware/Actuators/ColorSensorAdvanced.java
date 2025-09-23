package org.firstinspires.ftc.teamcode.Hardware.Actuators;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class ColorSensorAdvanced {
    private final ColorSensor sensor;

    public ColorSensorAdvanced(HardwareMap hardwareMap, String deviceName) {
        sensor = hardwareMap.get(ColorSensor.class, deviceName);
    }

    // Example: get red value
    public int red() {
        return this.sensor.red();
    }

    public int green() {
        return this.sensor.green();
    }

    public int blue() {
        return this.sensor.blue();
    }

    public int alpha() {
        return this.sensor.alpha();
    }

    public void enableLed(boolean enable) {
        this.sensor.enableLed(enable);
    }

    public I2cAddr getI2cAddress() {
        return this.sensor.getI2cAddress();
    }

    public void setI2cAddress(I2cAddr newAddress) {
        this.sensor.setI2cAddress(newAddress);
    }
}
