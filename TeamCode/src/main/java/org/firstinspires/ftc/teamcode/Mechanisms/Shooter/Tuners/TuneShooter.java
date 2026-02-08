package org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;

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
//            if (gamepad1.left_trigger > 0.05) { // ----- REVERSE -----
//                shooter.setShooterVelocityLoop(targetVelocity * 2 * Math.PI / 60.0);
//            } else {
//                shooter.shooterMotor1.setPower(gamepad1.left_trigger * 10);
//                shooter.shooterMotor2.setPower(gamepad1.left_trigger * 10);
//            }
            shooter.setShooterVelocityLoop(targetVelocity * 2 * Math.PI / 60.0).run(packet);


            currentVelocity = shooter.getVelocity();
            packet.put("shooterpowerPID", shooter.velocityPidController.calculate(targetVelocity * 2 * Math.PI / 60.0, currentVelocity));
            packet.put("shooterpowerFF", shooter.velocityFeedForwardController.calculate(targetVelocity * 2 * Math.PI / 60.0, 5));
            packet.put("shooterpower", shooter.velocityPidController.calculate(targetVelocity * 2 * Math.PI / 60.0, currentVelocity) + shooter.velocityFeedForwardController.calculate(targetVelocity, 5));
            packet.put("Velocity (RPM)", currentVelocity * 60.0 / (2 * Math.PI));
            packet.put("Target Velocity (RPM)", targetVelocity);
            packet.put("Target Velocity (Input)", targetVelocity * 2 * Math.PI / 60.0);
            dashboard.sendTelemetryPacket(packet);

        }


    }


}

