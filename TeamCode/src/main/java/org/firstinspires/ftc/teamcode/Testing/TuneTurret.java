package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "TuneTurret", group = "1")
public class TuneTurret extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TurretServo turret = new TurretServo(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(turret.turretSpin(90)); // spin to 90°
        }
    }
}
