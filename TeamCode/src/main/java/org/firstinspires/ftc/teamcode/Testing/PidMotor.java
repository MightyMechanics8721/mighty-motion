package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
public class PidMotor {
    // Variables
    public static double TICKS_PER_REV = 28;
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0;
    //Hardware
    protected DcMotorEx motorPid;
    //Mathmatics
    protected PID pid;
    HardwareMap hardwareMap;

    public PidMotor(HardwareMap hardwareMap, String motorName, double Kp, double Ki, double Kd, double TICKS_PER_REV) {
        this.hardwareMap = hardwareMap;
        pid = new PID(Kp, Ki, Kd, PID.functionType.LINEAR);
        motorPid = hardwareMap.get(DcMotorEx.class, motorName);
        PidMotor.TICKS_PER_REV = TICKS_PER_REV;
        PidMotor.Kp = Kp;
        PidMotor.Ki = Ki;
        PidMotor.Kd = Kd;
//        shooterRight = hardwareMap.get(DcMotorEx.class, "motorRight");

        motorPid.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shooterRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorPid.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Calculates the current Velocity of the Motor
     *
     * @return Unit: Rotations/Minute
     */
    public double calculateVelocity() {
        return motorPid.getVelocity() / TICKS_PER_REV * 60;
    }

    /**
     * Sets the Velocity of the Motor
     *
     * @return Motor power (-1.0, 1.0)
     */
    public double setVelocity(double desiredVelocity) {
        double currentVelocity = calculateVelocity();
        return pid.calculate(desiredVelocity, currentVelocity);
        //kV * desiredVelocity + output
    }

    /**
     * Gets current raw encoder Tick position (Not converted)
     *
     * @return
     */
    public int getEncoderTicks() {
        return motorPid.getCurrentPosition();
    }
}
