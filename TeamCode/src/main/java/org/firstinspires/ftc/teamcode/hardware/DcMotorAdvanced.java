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
     * Sets motor power, skipping the hardware write when the request has not moved by more than
     * powerThreshold since the last one.
     * <p>
     * When a maxVoltage was supplied, the request is scaled by maxVoltage / batteryVoltage so a
     * given command produces the same torque as the pack drains. With clipBeforeBatteryCompensation
     * the request is limited to [-1, 1] before that scaling, letting compensation push it above
     * full power; without it the limit is applied after, so full power is the hard ceiling. Either
     * way the value that reaches the hardware is clipped to [-1, 1].
     *
     * @param power requested power in [-1, 1]
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
