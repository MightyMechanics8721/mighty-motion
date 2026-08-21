package org.firstinspires.ftc.teamcode.drivetrain;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.sim.DrivetrainSim;

/** Prints what the tuned constants imply, so the numbers can be sanity checked against the robot. */
public class TuningReportTest {

    @Test
    public void reportDerivedLimits() {
        double r = Drivetrain.MECHANICAL_PARAMETERS.wheelRadius;
        double kV = Drivetrain.FF_CONSTANTS.lf.kV;
        double kS = Drivetrain.FF_CONSTANTS.lf.kS;
        double maxWheelRadPerSec = DrivetrainSim.MAX_WHEEL_SPEED;
        double maxInPerSec = maxWheelRadPerSec * r;

        System.out.println("---- what the feedforward implies ----");
        System.out.printf("kV %.4f power per rad/s, kS %.4f%n", kV, kS);
        System.out.printf("top wheel speed at full power : %.2f rad/s%n", maxWheelRadPerSec);
        System.out.printf("top ground speed              : %.1f in/s = %.2f ft/s%n",
                maxInPerSec, maxInPerSec / 12);
        System.out.printf("maxSpeed passed by every routine: 150 in/s = %.1f ft/s%n", 150 / 12.0);
        System.out.printf("so the cap is %s%n",
                150 > maxInPerSec ? "never reached - the motors saturate first" : "active");

        Drivetrain.StoppingDistanceParameters c = Drivetrain.STOPPING_DISTANCE_PARAMETERS;
        System.out.println("---- coast at that top speed ----");
        System.out.printf("forward : %.2f in%n",
                c.xLinear * maxInPerSec + c.xQuadratic * maxInPerSec * maxInPerSec);
        System.out.printf("sideways: %.2f in%n",
                c.yLinear * maxInPerSec + c.yQuadratic * maxInPerSec * maxInPerSec);
    }
}
