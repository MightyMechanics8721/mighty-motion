package org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;

@Config
@Autonomous(name = "Tune Shooter", group = "Testing")
public class TuneShooter extends LinearOpMode {

    public static double targetVelocity = 2500; // (RPM)
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        Battery.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        Shooter shooter = Shooter.getInstance();
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Velocity (RPM)", 0.0);
        packet.put("Target Velocity (RPM)", targetVelocity);
        dashboard.sendTelemetryPacket(packet);

        waitForStart();


        while (opModeIsActive()) {
            shooter.setShooterVelocityLoop(targetVelocity * 2 * Math.PI / 60.0);
            double velocity = shooter.getVelocity();
            packet.put("Velocity (RPM)", velocity * 60.0 / (2 * Math.PI));
            packet.put("Target Velocity (RPM)", targetVelocity);
            dashboard.sendTelemetryPacket(packet);

        }


    }


}

