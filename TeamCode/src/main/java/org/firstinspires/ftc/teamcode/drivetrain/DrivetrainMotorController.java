package org.firstinspires.ftc.teamcode.drivetrain;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.control.FeedForward;

public class DrivetrainMotorController {

    private final FeedForward ffLfm;
    private final FeedForward ffLbm;
    private final FeedForward ffRbm;
    private final FeedForward ffRfm;

    public DrivetrainMotorController(
            Drivetrain.FFConstantsController ffConstants
    ) {
        this.ffLfm = new FeedForward(ffConstants.lf);
        this.ffLbm = new FeedForward(ffConstants.lb);
        this.ffRbm = new FeedForward(ffConstants.rb);
        this.ffRfm = new FeedForward(ffConstants.rf);
    }

    public SimpleMatrix calculate(SimpleMatrix wheelSpeeds, SimpleMatrix wheelAccelerations) {

        double uLf = ffLfm.calculate(wheelSpeeds.get(0, 0), wheelAccelerations.get(0, 0));
        double uLb = ffLbm.calculate(wheelSpeeds.get(1, 0), wheelAccelerations.get(1, 0));
        double uRb = ffRbm.calculate(wheelSpeeds.get(2, 0), wheelAccelerations.get(2, 0));
        double uRf = ffRfm.calculate(wheelSpeeds.get(3, 0), wheelAccelerations.get(3, 0));
        return new SimpleMatrix(
                new double[]{
                        uLf,
                        uLb,
                        uRb,
                        uRf
                }
        );
    }
}
