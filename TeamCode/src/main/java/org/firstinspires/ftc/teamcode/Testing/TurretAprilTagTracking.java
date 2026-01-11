package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "TurretTrackingAprilTag", group = "1")
public class TurretAprilTagTracking extends LinearOpMode {

    private Limelight3A limelight;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(limelight, 0);
        TurretServo turret = new TurretServo(hardwareMap);
        TelemetryPacket packet = new TelemetryPacket();
        waitForStart();

        while (opModeIsActive()) {

            Actions.runBlocking(turret.limelightRelativeTurret()); // spin to 90°

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
