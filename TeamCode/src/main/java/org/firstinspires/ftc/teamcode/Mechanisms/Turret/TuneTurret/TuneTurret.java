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
    Turret turret;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        Turret.initialize(hardwareMap);
        turret = Turret.getInstance();
        dashboard = FtcDashboard.getInstance();


        waitForStart();

        while (opModeIsActive()) {
            turret.manualControl(gamepad1.left_stick_y * 5).run(packet);
            packet.put("power", gamepad1.left_stick_y * 5);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
