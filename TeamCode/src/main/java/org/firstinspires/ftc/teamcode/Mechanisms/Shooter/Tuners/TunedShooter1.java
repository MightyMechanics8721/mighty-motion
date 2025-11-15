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
public class TunedShooter1 extends LinearOpMode {

    public static double targetVelocity = 4000; // (RPM)
    FtcDashboard dashboard;
    Battery battery;

    @Override
    public void runOpMode() {

        battery = new Battery(hardwareMap);
        Shooter s1 = new Shooter(hardwareMap, battery);
        dashboard = FtcDashboard.getInstance();
        ElapsedTime looptime = new ElapsedTime();
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Velocity (RPM)", 0.0);
        dashboard.sendTelemetryPacket(packet);

        waitForStart();


        while (opModeIsActive()) {

            s1.setVelocity(targetVelocity * 2 * Math.PI / 60.0);
            double velocity = s1.getVelocity();
            packet.put("Velocity (RPM)", velocity * 60.0 / (2 * Math.PI));
            dashboard.sendTelemetryPacket(packet);

        }


    }


}

