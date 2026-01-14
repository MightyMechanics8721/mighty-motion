package org.firstinspires.ftc.teamcode.Testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
public class TurretServo {
    private static final double TICKS_PER_REV = 4000.0;
    // tuning
    public static double Kp = 1.0 / 180;
    public static double Ki = 0;
    public static double Kd = 0;
    private CRServo turretLeft;
    private CRServo turretRight;
    private DcMotorEx turretEncoder;
    private PID pid;

    public TurretServo(HardwareMap hardwareMap) {
        turretLeft = hardwareMap.get(CRServo.class, "servodot");
        turretRight = hardwareMap.get(CRServo.class, "servodotty");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "encoding");
        pid = new PID(new PIDConstants(Kp, Ki, Kd), PID.functionType.LINEAR);
    }

    /**
     * Calculates the PID power needed to spin the Servo.
     *
     * @param desiredHeading The angle (degrees) that we attempt to reach.
     * @return
     */
    public double spinPower(double desiredHeading) {
        double heading = getHeadingDegrees();
        double output = pid.calculate(desiredHeading, heading);
        return Range.clip(output, -1.0, 1.0);
    }

    /**
     * Spins the Turret
     *
     * @param desiredHeading The angle (degrees) that we attempt to reach.
     * @return
     */
    public Action turretSpin(double desiredHeading) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = spinPower(desiredHeading);
                turretLeft.setPower(-power);
                turretRight.setPower(-power);
// NEGATIVE POWER BECAUSE REVERSED
                telemetryPacket.put("A Target", desiredHeading);
                telemetryPacket.put("B Current", getHeadingDegrees());
                telemetryPacket.put("C Power", power);

                // finish once close to target
                return Math.abs(desiredHeading - getHeadingDegrees()) < 1;
            }
        };
    }

    /**
     * Gets the Encoder's current position in ticks
     *
     * @return
     */
    public int getEncoderTicks() {
        return turretEncoder.getCurrentPosition();
    }

    /**
     * Calculates the Encoder's current position in degrees
     *
     * @return
     */
    public double getHeadingDegrees() {
        double ticks = turretEncoder.getCurrentPosition();
        return (ticks / TICKS_PER_REV) * 360;
    }
}
