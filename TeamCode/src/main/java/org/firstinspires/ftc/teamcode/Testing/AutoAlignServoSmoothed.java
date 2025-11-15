package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

@TeleOp(name="Auto Align Servo Smoothed", group="Testing")
public class AutoAlignServoSmoothed extends LinearOpMode {

    private Limelight3A limelight;
    private Servo cameraServo;
    private FtcDashboard dashboard;

    // Linear mapping constants
    private static final double M = -155.05;
    private static final double B = 77.36;

    // Servo limits
    private static final double SERVO_MIN = 0.4;
    private static final double SERVO_MAX = 0.6;

    // Deadzone threshold (degrees)
    private static final double TX_THRESHOLD = 1.0;

    // Servo smoothing constant
    private static final double SERVO_STEP = 0.005; // max increment per loop

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

        double currentServoPos = cameraServo.getPosition();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                FiducialResult fiducial = result.getFiducialResults().get(0);
                double tx = fiducial.getTargetXDegrees();

                double targetServoPos = currentServoPos;

                if (Math.abs(tx) > TX_THRESHOLD) {
                    // Compute target position and clamp to limits
                    targetServoPos = (B - tx) / Math.abs(M);
                    targetServoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, targetServoPos));
                }

                // Smoothly move servo towards target
                if (targetServoPos > currentServoPos + SERVO_STEP) {
                    currentServoPos += SERVO_STEP;
                } else if (targetServoPos < currentServoPos - SERVO_STEP) {
                    currentServoPos -= SERVO_STEP;
                } else {
                    currentServoPos = targetServoPos; // close enough
                }

                cameraServo.setPosition(currentServoPos);

                // Telemetry
                packet.put("AprilTag ID", fiducial.getFiducialId());
                packet.put("Limelight tx (deg)", tx);
                packet.put("Target Servo Pos", targetServoPos);
                packet.put("Current Servo Pos", currentServoPos);
                packet.put("Status", "Tracking tag");
            } else {
                packet.put("Status", "No valid AprilTag detected");
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
