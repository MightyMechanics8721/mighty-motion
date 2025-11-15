package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

/**
 * TurretServo controls a pair of continuous rotation servos for turret rotation,
 * using both manual gamepad control and PID-based positioning.
 */
@Config
public class TurretServo {

    // --- PID Constants (Dashboard Tunable) ---
    public static double Kp = 0.01;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    // --- Constants ---
    private final double TICKS_PER_REV = 4000.0;
    private final double GEAR_RATIO = 140.0 / 30;
    // --- Hardware ---
    private final CRServo turretLeft;
    private final CRServo turretRight;
    private final DcMotorEx turretEncoder;

    // --- Utilities ---
    private final FtcDashboard dashboard;
    private final PID pid;

    // --- Constructor ---
    public TurretServo(HardwareMap hardwareMap) {
        turretLeft = hardwareMap.get(CRServo.class, "servodot");
        turretRight = hardwareMap.get(CRServo.class, "servodotty");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "encoding");

        dashboard = FtcDashboard.getInstance();
        pid = new PID(Kp, Ki, Kd, PID.functionType.LINEAR);
    }

    // --- PID Power Calculation ---

    /**
     * Calculates the power needed to spin the turret to the desired heading.
     *
     * @param desiredHeading Target heading in degrees.
     * @return Power value clipped between -1.0 and 1.0.
     */
    public double spinPower(double desiredHeading) {
        double currentMotorDegrees = (turretEncoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        double currentTurretDegrees = currentMotorDegrees / GEAR_RATIO;
        return pid.calculate(desiredHeading, currentTurretDegrees);
    }

    /**
     * Spins the turret to a specific heading using PID control.
     *
     * @param desiredHeading Target heading in degrees.
     */
    public Action turretSpin(double desiredHeading) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = spinPower(desiredHeading);

                turretLeft.setPower(power);
                turretRight.setPower(power);

                telemetryPacket.put("Target (Servo)", desiredHeading);
                telemetryPacket.put("Current (Servo)", getHeadingDegrees() % 360);
                telemetryPacket.put("Power", power);
                telemetryPacket.put("Current (Turret)", ((turretEncoder.getCurrentPosition() / TICKS_PER_REV) * 360.0 / GEAR_RATIO) % 360);

                // Stop once close enough to target
                return Math.abs(desiredHeading - getHeadingDegrees()) < 1.0;
            }
        };
    }

    /**
     * Direct manual control of the turret via gamepad stick.
     */
    public Action controllerTurret() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double stickPower = gamepad1.left_stick_y;
                turretLeft.setPower(stickPower);
                turretRight.setPower(stickPower);

                telemetryPacket.put("Manual Power", stickPower);
                return false; // Runs continuously
            }
        };
    }


    /**
     * @return Current turret encoder ticks.
     */
    public int getEncoderTicks() {
        return turretEncoder.getCurrentPosition();
    }

    /**
     * @return Current turret heading in degrees.
     */
    public double getHeadingDegrees() {
        double ticks = turretEncoder.getCurrentPosition();
        return (ticks / TICKS_PER_REV) * 360.0;
    }
}
