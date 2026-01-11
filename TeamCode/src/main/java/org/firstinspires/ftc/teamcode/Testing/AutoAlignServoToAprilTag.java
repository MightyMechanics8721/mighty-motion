package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

@TeleOp(name="Auto Align Servo to AprilTag", group="Testing")
public class AutoAlignServoToAprilTag extends LinearOpMode {

    private Limelight3A limelight;
    private Servo cameraServo;
    private FtcDashboard dashboard;
    

    // Linear mapping constants (rounded)
    private static final double M = -155.05;
    private static final double B = 77.36;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");

        dashboard = FtcDashboard.getInstance();
        limelight.pipelineSwitch(0); // AprilTag detection pipeline
        limelight.start();

        telemetry.setMsTransmissionInterval(50);
        dashboard.startCameraStream(limelight, 0);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                FiducialResult fiducial = result.getFiducialResults().get(0);
                double tx = fiducial.getTargetXDegrees(); // Horizontal angle

                // Compute servo position from tx (limit to 0–1 range)
                double servoPos = (B - tx) / Math.abs(M);
                servoPos = Math.max(0.0, Math.min(1.0, servoPos)); // Clamp

                cameraServo.setPosition(servoPos);

                // Telemetry
                packet.put("AprilTag ID", fiducial.getFiducialId());
                packet.put("DistanceFromArtifact", fiducial.getFiducialId());
                packet.put("Limelight tx (deg)", tx);
                packet.put("Servo Position", servoPos);
                packet.put("Status", "Tracking tag");
            } else {
                packet.put("Status", "No valid AprilTag detected");
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
