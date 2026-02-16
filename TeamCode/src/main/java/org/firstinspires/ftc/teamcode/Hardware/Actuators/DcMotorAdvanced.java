package org.firstinspires.ftc.teamcode.Hardware.Actuators;


import static com.acmerobotics.roadrunner.Math.clamp;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;


public class DcMotorAdvanced {
    private final double powerThreshold;
    private final DcMotorEx motor;
    private final double maxVoltage;
    private double previousPower = 0;
    private boolean clipBeforeBatteryCompensation = false;

    public DcMotorAdvanced(
            DcMotorEx motor,
            double maxVoltage,
            double powerThreshold,
            boolean clipBeforeBatteryCompensation
    ) {
        this(motor, maxVoltage, powerThreshold);
        this.clipBeforeBatteryCompensation = clipBeforeBatteryCompensation;
    }

    public DcMotorAdvanced(DcMotorEx motor, double maxVoltage, double powerThreshold) {
        this.motor = motor;
        this.maxVoltage = maxVoltage;
        this.powerThreshold = powerThreshold;
    }

    public DcMotorAdvanced(DcMotorEx motor, double powerThreshold) {
        this(motor, Double.POSITIVE_INFINITY, powerThreshold);
    }

    public DcMotorAdvanced(DcMotorEx motor) {
        this(motor, Double.POSITIVE_INFINITY, 0.0);
    }

    public void setPower(double power) {
        TelemetryPacket packet = new TelemetryPacket();
        if (Math.abs(power - previousPower) > this.powerThreshold) {

            if (this.maxVoltage != Double.POSITIVE_INFINITY) {
                double batteryVoltage = Battery.getInstance().getVoltage();

                // NOTE: might want to clip the power between (-1 and 1).
                // Maybe for shooter - @kevin.
                if (this.clipBeforeBatteryCompensation) {
                    power = clamp(power, -1.0, 1.0);
                }

                power = maxVoltage / batteryVoltage * power;
                motor.setPower(power);

            } else {
                motor.setPower(power);
            }
        }
        previousPower = power;
    }

    public DcMotorSimple.Direction getDirection() {
        return motor.getDirection();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    /**
     * @return ticks/sec
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * ticks/sec
     */
    public void setVelocity(double angularRate) {
        motor.setVelocity(angularRate);
    }

    /**
     * @return desired unit /sec
     */
    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    /**
     * @return current consumed by motor (currentUnit)
     */
    public Object getCurrent(CurrentUnit currentUnit) {
        return motor.getCurrent(currentUnit);
    }
}
