package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;
import org.firstinspires.ftc.teamcode.Testing.DistanceSensor;

@Config
@Autonomous(name = "Test Transfer", group = "Tuning")
public class TestIntakeSystem extends LinearOpMode {
    DistanceSensor distanceSensor;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        Intake.initialize(hardwareMap);
        Indexer.initialize(hardwareMap);
        DistanceSensor.initialize(hardwareMap);
        distanceSensor = DistanceSensor.getInstance();
        dashboard = FtcDashboard.getInstance();
        waitForStart();

        while (opModeIsActive()) {
            distanceSensor.ballDetection().run(packet);


            dashboard.sendTelemetryPacket(packet);
        }
    }
}
