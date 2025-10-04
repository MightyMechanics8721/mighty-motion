package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
public class PidMotor {
    public static double TICKS_PER_REV = 28;
    public static double Kp = 0.0075;
    public static double Ki = 0;
    public static double Kd = 0;
    public DcMotorEx motorLeft;
    public DcMotorEx motorRight;
    HardwareMap hardwareMap;
    private PID pid;

    public PidMotor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        pid = new PID(Kp, Ki, Kd, PID.functionType.LINEAR);
        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");

    }

    public double calculateVelocity() {
        return -motorLeft.getVelocity() / TICKS_PER_REV * 60;
    }

    public double setVelocity(double desiredVelocity) {
        double currentVelocity = calculateVelocity();
        double output = pid.calculate(desiredVelocity, currentVelocity);
        return Range.clip(output, -1.0, 1.0);
    }

    public int getEncoderTicks() {
        return motorLeft.getCurrentPosition();
    }

    /**
     * Calculates the Encoder's current position in degrees
     *
     * @return
     */


    public Action motorSpin(double desiredVelocity) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = setVelocity(desiredVelocity);
                motorLeft.setPower(-power);
                motorRight.setPower(-power);
                // NEGATIVE POWER BECAUSE REVERSED
                telemetryPacket.put("Velocity", motorLeft.getVelocity() / TICKS_PER_REV);
                telemetryPacket.put("Ticks", motorLeft.getCurrentPosition());
                telemetryPacket.put("A Target", desiredVelocity);
                telemetryPacket.put("B Current", calculateVelocity());
                telemetryPacket.put("C Power", power);

                // finish once close to target
                return Math.abs(desiredVelocity - calculateVelocity()) < 1;
            }
        };
    }
}
