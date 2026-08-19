package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.util.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@Autonomous(name = "Tune Shooter", group = "Testing")
public class TuneShooter extends LinearOpMode {
    public static double targetVelocity = 2500; // (RPM)
    FtcDashboard dashboard;
    Shooter shooter;

    @Override
    public void runOpMode() {

        Battery.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        Battery.getInstance();
        Shooter.getInstance();
        shooter = Shooter.getInstance();
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Velocity (RPM)", 0.0);
        packet.put("Target Velocity (RPM)", targetVelocity);
        dashboard.sendTelemetryPacket(packet);
        double currentVelocity = 0;
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.05) { // ----- REVERSE -----
                shooter.setShooterVelocityLoop(Utils.rpmToRadPerSec(targetVelocity));
            } else {
                shooter.shooterMotor1.setPower(gamepad1.left_trigger * 10);
                shooter.shooterMotor2.setPower(gamepad1.left_trigger * 10);
            }
            shooter.setShooterVelocityLoop(Utils.rpmToRadPerSec(targetVelocity)).run(packet);

            currentVelocity = shooter.getVelocity();
            packet.put(
                    "shooterpowerPID",
                    shooter.velocityPidController.calculate(
                            Utils.rpmToRadPerSec(targetVelocity),
                            currentVelocity
                    )
            );
            packet.put(
                    "shooterpowerFF",
                    shooter.velocityFeedForwardController.calculate(
                            Utils.rpmToRadPerSec(targetVelocity), 5)
            );
            packet.put(
                    "shooterpower",
                    shooter.velocityPidController.calculate(
                            Utils.rpmToRadPerSec(targetVelocity),
                            currentVelocity
                    ) + shooter.velocityFeedForwardController.calculate(targetVelocity, 5)
            );
            packet.put("Velocity (RPM)", Utils.radPerSecToRpm(currentVelocity));
            packet.put("Target Velocity (RPM)", targetVelocity);
            packet.put("Target Velocity (Input)", Utils.rpmToRadPerSec(targetVelocity));
            dashboard.sendTelemetryPacket(packet);

        }

    }

}

