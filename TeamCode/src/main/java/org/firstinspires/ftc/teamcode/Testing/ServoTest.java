package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ServoTest", group="Testing")
public class ServoTest extends LinearOpMode {

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        // Get the servo from the configuration
        Servo servo = hardwareMap.get(Servo.class, "cameraServo");
        AprilTagUtils limelight = new AprilTagUtils(hardwareMap, "limelight");
        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Servo Test Ready");
        telemetry.addLine("Press PLAY to begin");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {


            // Move servo to 0.4
            servo.setPosition(0.4);

            // Wait 1.5 seconds for servo to move
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 1.5) {
                idle();
            }

            TelemetryPacket packet = new TelemetryPacket();
            dashboard = FtcDashboard.getInstance();

            // Read horizontal angle from Limelight
            double tx = limelight.getHorizontalAngle();
            packet.put("Servo Position", 0.4);
            packet.put("Limelight tx (deg)", tx);
            telemetry.update();

            telemetry.addLine("Servo Test Complete");
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
