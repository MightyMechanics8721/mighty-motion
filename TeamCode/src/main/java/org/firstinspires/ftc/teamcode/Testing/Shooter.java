package org.firstinspires.ftc.teamcode.Testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter extends PidMotor {
    // Variables
    public static double TICKS_PER_REV = 28;
    public static double Kp = 0.00005;
    public static double Ki = 0;
    public static double Kd = 0;
    // Hardware
    protected DcMotorEx shooterLeft;
    protected DcMotorEx shooterRight;

    public Shooter(HardwareMap hardwareMap) {
        super(hardwareMap, "shooterLeft", Kp, Ki, Kd, TICKS_PER_REV);
        this.hardwareMap = hardwareMap;
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        motorPid = shooterLeft;

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);
        shooterRight.setDirection(DcMotorEx.Direction.FORWARD);
    }

    /**
     * Action: Spins the motor according to the desired Velocity.
     */
    public Action motorSpin(double desiredVelocity) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = setVelocity(desiredVelocity);
                shooterLeft.setPower(power);
                shooterRight.setPower(power);
                // NEGATIVE POWER BECAUSE REVERSED
                telemetryPacket.put("E Velocity (RPS)", motorPid.getVelocity() / TICKS_PER_REV);
                telemetryPacket.put("D Ticks", motorPid.getCurrentPosition());
                telemetryPacket.put("A Target Velocity (RPM)", desiredVelocity);
                telemetryPacket.put("B Current Velocity (RPM)", calculateVelocity());
                telemetryPacket.put("C Motor Power", power);

                // finish once close to target
                return false;
            }
        };
    }
}