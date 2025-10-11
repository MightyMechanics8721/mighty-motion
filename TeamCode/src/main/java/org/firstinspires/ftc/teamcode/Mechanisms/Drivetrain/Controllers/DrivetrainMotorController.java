package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.FeedForward;

@Config
public class DrivetrainMotorController {
    public static double kA = 0.01;
    public static double kV = 0.0225;
    public static double kSlf = 0.067;
    public static double kSlb = 0.067;
    public static double kSrb = 0.067;
    public static double kSrf = 0.067;
    public static double vMax = 12.5;
    public VoltageSensor voltageSensor;
    public FeedForward ffLfm;
    public FeedForward ffLbm;
    public FeedForward ffRbm;
    public FeedForward ffRfm;
    public double uLf = 0;
    public double uLb = 0;
    public double uRb = 0;
    public double uRf = 0;
    public Battery battery;

    public DrivetrainMotorController(
            HardwareMap hardwareMap,
            Drivetrain.FFConstantsController ffConstants
    ) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.ffLfm = new FeedForward(ffConstants.lf);
        this.ffLbm = new FeedForward(ffConstants.lb);
        this.ffRbm = new FeedForward(ffConstants.rb);
        this.ffRfm = new FeedForward(ffConstants.rf);
        this.battery = new Battery(hardwareMap);
    }

    public SimpleMatrix calculate(SimpleMatrix wheelSpeeds, SimpleMatrix wheelAccelerations) {
        double voltage = battery.getVoltage();
        uLf = ffLfm.calculate(wheelSpeeds.get(0, 0), wheelAccelerations.get(0, 0));
        uLb = ffLbm.calculate(wheelSpeeds.get(1, 0), wheelAccelerations.get(1, 0));
        uRb = ffRbm.calculate(wheelSpeeds.get(2, 0), wheelAccelerations.get(2, 0));
        uRf = ffRfm.calculate(wheelSpeeds.get(3, 0), wheelAccelerations.get(3, 0));
        SimpleMatrix relativePower = new SimpleMatrix(
                new double[]{
                        uLf * vMax / voltage,
                        uLb * vMax / voltage,
                        uRb * vMax / voltage,
                        uRf * vMax / voltage
                }
        );
        return relativePower;
    }
}
