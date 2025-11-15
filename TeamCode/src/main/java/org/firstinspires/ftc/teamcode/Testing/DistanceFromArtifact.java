package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Distance From Artifact", group="Testing")
public class DistanceFromArtifact extends LinearOpMode {

    private Limelight3A limelight;
    FtcDashboard dashboard;

    // Set this to the height difference between your camera and AprilTag (meters)
    private static final double CAMERA_HEIGHT_METERS = 0.25; // example: 25 cm above floor
    private static final double TAG_HEIGHT_METERS = 0.10;    // example: 10 cm


    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);


        // Switch to AprilTag detection pipeline
        limelight.pipelineSwitch(1);

        // Start polling for results
        limelight.start();
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(limelight, 0);
        waitForStart();
        TelemetryPacket packet = new TelemetryPacket();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                FiducialResult fiducial = result.getFiducialResults().get(0);

                // Horizontal and vertical angles to tag (degrees)
                double tx = fiducial.getTargetXDegrees();
                double ty = fiducial.getTargetYDegrees();

                // Convert vertical angle to radians
                double tyRad = Math.toRadians(ty);

                // Estimate distance using simple trigonometry
                double heightDiff = CAMERA_HEIGHT_METERS - TAG_HEIGHT_METERS;
                double distanceMeters = heightDiff / Math.tan(tyRad);

                packet.put("Estimated Distance (m)", distanceMeters);

                //"%.2f"

            } else {
                packet.put("Status", "No valid artifact detected");
            }

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}