package org.firstinspires.ftc.teamcode.hardware;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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

    /**
     * Sets motor power in [-1, 1]. Skips writes below powerThreshold, scales by
     * maxVoltage / batteryVoltage, clips to [-1, 1].
     */
    public void setPower(double power) {
        if (Math.abs(power - previousPower) <= this.powerThreshold) {
            return;
        }
        previousPower = power;

        double output = power;
        if (this.maxVoltage != Double.POSITIVE_INFINITY) {
            if (this.clipBeforeBatteryCompensation) {
                output = clamp(output, -1.0, 1.0);
            }
            double batteryVoltage = Battery.getInstance().getVoltage();
            if (batteryVoltage > 0) {
                output = maxVoltage / batteryVoltage * output;
            }
        }
        motor.setPower(clamp(output, -1.0, 1.0));
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
