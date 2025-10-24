package org.firstinspires.ftc.teamcode.Hardware.Actuators;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;

@Config
public class DcMotorAdvanced {
    public static double acceptablePowerDifference = 0.000001;
    public double previousPower = 0;
    private DcMotorEx motor;
    private double batteryVoltage;
    private double maxVoltage;
    private Battery battery;

    public DcMotorAdvanced(DcMotorEx motor, Battery battery, double maxVoltage) {
        this.motor = motor;
        this.battery = battery;
        this.batteryVoltage = battery.getVoltage();
        this.maxVoltage = maxVoltage;
    }

    public DcMotorAdvanced(DcMotorEx motor) {
        this.motor = motor;

    }

    public void setVoltage() {
        batteryVoltage = battery.getVoltage();
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setPower(double power) {
//        setVoltage();

        if (Math.abs(power - previousPower) > acceptablePowerDifference) {

            if (this.battery != null) {

                batteryVoltage = battery.getVoltage();
                motor.setPower(maxVoltage / batteryVoltage * power);

            } else if (this.battery == null) {

                motor.setPower(power);


            }
            previousPower = power;

        }
    }

    public DcMotorSimple.Direction getDirection() {
        return motor.getDirection();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    public void setVelocity(double angularRate, AngleUnit unit) {
        motor.setVelocity(angularRate, unit);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void setVelocity(double angularRate) {
        motor.setVelocity(angularRate);
    }

    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    public void setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
        motor.setPIDCoefficients(mode, pidCoefficients);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setPositionPIDFCoefficients(double p) {
        motor.setPositionPIDFCoefficients(p);
    }


    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode mode) {
        return motor.getPIDFCoefficients(mode);
    }

    public int getTargetPositionTolerance() {
        return motor.getTargetPositionTolerance();
    }

    public void setTargetPositionTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    public DcMotorController getController() {
        return motor.getController();
    }

    public int getPortNumber() {
        return motor.getPortNumber();
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public DcMotor.RunMode getMode() {
        return motor.getMode();
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public HardwareDevice.Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    public String getDeviceName() {
        return motor.getDeviceName();
    }

    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    public int getVersion() {
        return motor.getVersion();
    }

    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    public void close() {
        motor.close();
    }
}
