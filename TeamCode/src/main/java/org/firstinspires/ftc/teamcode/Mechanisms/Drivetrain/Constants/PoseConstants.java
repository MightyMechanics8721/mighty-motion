package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Constants;

public class PoseConstants {
    public PIDConstants xPID;
    public PIDConstants yPID;
    public PIDConstants thetaPID;

    public PoseConstants(PIDConstants x, PIDConstants y, PIDConstants theta) {
        xPID = x;
        yPID = y;
        thetaPID = theta;
    }
}
