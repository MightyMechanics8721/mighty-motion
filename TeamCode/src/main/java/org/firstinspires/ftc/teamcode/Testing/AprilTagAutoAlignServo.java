package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

@TeleOp(name="AprilTag Auto Align Servo", group="Testing")
public class AprilTagAutoAlignServo extends LinearOpMode {

    private Limelight3A limelight;
    private Servo cameraServo;
    private FtcDashboard dashboard;

    // Linear mapping constants (based on calibration)
    private static final double M = -155.05;
    private static final double B = 77.36;

    // Servo tolerance zone (degrees of tx allowed before moving)
    private static final double DEADBAND_DEGREES = 2.0;

    // Store last known servo position so we can "hold" it when aligned
    private double lastServoPos = 0.5;  // default / safe midpoint

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
                double tx = fiducial.getTargetXDegrees();

                // Only move servo if we are outside the acceptable "aligned" zone
                if (Math.abs(tx) > DEADBAND_DEGREES) {
                    double servoPos = (B - tx) / Math.abs(M);
                    servoPos = Math.max(0.0, Math.min(1.0, servoPos)); // Clamp 0-1
                    cameraServo.setPosition(servoPos);
                    lastServoPos = servoPos; // Save last position
                } else {
                    // Inside deadband → hold previous servo value
                    cameraServo.setPosition(lastServoPos);
                }

                // Telemetry
                packet.put("AprilTag ID", fiducial.getFiducialId());
                packet.put("Limelight tx (deg)", tx);
                packet.put("Servo Position", lastServoPos);
                packet.put("Aligned?", Math.abs(tx) <= DEADBAND_DEGREES);
            } else {
                packet.put("Status", "No valid AprilTag detected");
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
