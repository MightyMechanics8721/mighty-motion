package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ServoTestLive", group="Testing")
public class ServoTestLive extends LinearOpMode {

    public static double servoPos = 0.4;   // Editable in FTC Dashboard (0.0 - 1.0)

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "cameraServo");
        AprilTagUtils limelight = new AprilTagUtils(hardwareMap, "limelight");
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Open FTC Dashboard and edit 'servoPos'");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {

            // Make sure servo position stays in valid range
            servoPos = Math.max(0.0, Math.min(servoPos, 1.0));
            servo.setPosition(servoPos);

            // Read horizontal angle from Limelight
            double tx = limelight.getHorizontalAngle();

            // Send data to Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Servo Position", servoPos);
            packet.put("Limelight tx (deg)", tx);
            dashboard.sendTelemetryPacket(packet);

            // Also show on Driver Station telemetry
            telemetry.addData("Servo Position", servoPos);
            telemetry.addData("Limelight tx (deg)", tx);
            telemetry.update();
        }
    }
}
