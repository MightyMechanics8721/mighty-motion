package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test AprilTagUtils", group="Testing")
public class TestAprilTagUtils extends LinearOpMode {

    private AprilTagUtils tagUtils;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the dashboard
        dashboard = FtcDashboard.getInstance();

        // Initialize your utility class
        tagUtils = new AprilTagUtils(hardwareMap, "limelight");

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            if (tagUtils.hasValidTarget()) {
                double tx = tagUtils.getHorizontalAngle();

                telemetry.addData("Status", "AprilTag detected");
                telemetry.addData("Horizontal Angle (deg)", tx);

                packet.put("Status", "AprilTag detected");
                packet.put("Horizontal Angle (deg)", tx);
            } else {
                telemetry.addData("Status", "No valid AprilTag detected");

                packet.put("Status", "No valid AprilTag detected");
            }

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
