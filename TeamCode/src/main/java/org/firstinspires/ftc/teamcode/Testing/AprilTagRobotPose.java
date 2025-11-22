package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name="AprilTag RobotPose (FTC SDK)", group="Testing")
public class AprilTagRobotPose extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        Position cameraPosition = new Position(
                DistanceUnit.INCH,
                0,    // forward offset
                0,    // left/right offset
                0,    // height
                0);   // timestamp

        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                0,      // yaw
                -90,    // pitch
                0,      // roll
                0);     // timestamp


        //
        // AprilTag processor
        //
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();


        //
        // VisionPortal with webcam
        //
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();


        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 15);


        waitForStart();


        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {

                AprilTagDetection detection = detections.get(0);

                // Robot Pose X,Y, and Z
                double myX = detection.robotPose.getPosition().x;
                double myY = detection.robotPose.getPosition().y;
                double myZ = detection.robotPose.getPosition().z;

                double myPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                double myRoll  = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                double myYaw   = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                packet.put("ID", detection.id);
                packet.put("Robot X (m)", myX);
                packet.put("Robot Y (m)", myY);
                packet.put("Robot Z (m)", myZ);
                packet.put("Pitch", myPitch);
                packet.put("Roll", myRoll);
                packet.put("Yaw", myYaw);

            } else {
                packet.put("Status", "No AprilTag detected");
            }

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
