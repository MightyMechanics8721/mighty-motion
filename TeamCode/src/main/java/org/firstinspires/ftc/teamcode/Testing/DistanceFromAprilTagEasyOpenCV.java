package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Distance From AprilTag (EasyOpenCV)", group="Testing")
public class DistanceFromAprilTagEasyOpenCV extends LinearOpMode {

    //private static final boolean USE_WEBCAM = true;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private FtcDashboard dashboard;

    // Camera parameters (must be tuned to your Logitech webcam)
    private static final double CAMERA_HEIGHT_METERS = 0.25; // 25 cm
    private static final double TAG_HEIGHT_METERS = 0.10;    // 10 cm

    @Override
    public void runOpMode() throws InterruptedException {
        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        // Open the webcam through VisionPortal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 0);

        waitForStart();
        TelemetryPacket packet = new TelemetryPacket();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);

                // FTC SDK already estimates pose of tag relative to camera
                double tx = Math.toDegrees(Math.atan2(tag.ftcPose.x, tag.ftcPose.y)); // sideways angle
                double ty = Math.toDegrees(Math.atan2(tag.ftcPose.z, tag.ftcPose.y)); // vertical angle

                // Distance straight from pose
                double distanceMeters = Math.sqrt(
                        tag.ftcPose.x * tag.ftcPose.x +
                        tag.ftcPose.y * tag.ftcPose.y +
                        tag.ftcPose.z * tag.ftcPose.z);

                packet.put("AprilTag ID", tag.id);
                packet.put("X (m)", tag.ftcPose.x);
                packet.put("Y (m)", tag.ftcPose.y);
                packet.put("Z (m)", tag.ftcPose.z);
                packet.put("Estimated Distance (m)", distanceMeters);

            } else {
                packet.put("Status", "No AprilTag detected");
            }

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
