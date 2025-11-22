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
@Autonomous(name = "Shooter Test", group = "Autonomous")
public class TunedShooter extends LinearOpMode {

    public static double targetVelocity = 2500; // (RPM)
    FtcDashboard dashboard;
    Battery battery;

    @Override
    public void runOpMode() {

        battery = new Battery(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, battery);
        dashboard = FtcDashboard.getInstance();
        ElapsedTime looptime = new ElapsedTime();
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Velocity (RPM)", 0.0);
        packet.put("Target Velocity (RPM)", targetVelocity);
        dashboard.sendTelemetryPacket(packet);

        waitForStart();


        while (opModeIsActive()) {

            shooter.setVelocity(targetVelocity * 2 * Math.PI / 60.0);
            double velocity = shooter.getVelocity();
            packet.put("Velocity (RPM)", velocity * 60.0 / (2 * Math.PI));
            packet.put("Target Velocity (RPM)", targetVelocity);
            dashboard.sendTelemetryPacket(packet);

        }


    }


}

