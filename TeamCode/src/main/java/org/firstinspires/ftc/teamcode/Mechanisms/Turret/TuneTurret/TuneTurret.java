package org.firstinspires.ftc.teamcode.Mechanisms.Turret.TuneTurret;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;

@Config
@Autonomous(name = "Tune Turret", group = "Tuning")
public class TuneTurret extends LinearOpMode {
    public static double desiredAngle = 170;
    public static double time = 10;
    Turret turret;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        Turret.initialize(hardwareMap);
        turret = Turret.getInstance();
        dashboard = FtcDashboard.getInstance();
        packet.put("angle", turret.getAngle());
        dashboard.sendTelemetryPacket(packet);
        while (!opModeIsActive()) {
            packet.put("angle", turret.getAngle());
            dashboard.sendTelemetryPacket(packet);
        }
        waitForStart();
        while (opModeIsActive()) {
            turret.setTurretAngle(desiredAngle).run(packet);
            packet.put("angle", turret.getAngle());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
