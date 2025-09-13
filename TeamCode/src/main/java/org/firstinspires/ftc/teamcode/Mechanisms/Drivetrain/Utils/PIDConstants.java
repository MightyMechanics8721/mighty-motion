package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils;

public class PIDConstants {
    public double kP;
    public double kI;
    public double kD;

    public PIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
