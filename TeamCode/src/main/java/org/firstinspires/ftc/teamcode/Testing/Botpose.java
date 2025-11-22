package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Test April Tag2", group = "Testing")
public class Botpose extends LinearOpMode {

    private Limelight3A limelight;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        dashboard = FtcDashboard.getInstance();

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // Get the FTC-style Pose3D
                Pose3D botpose = result.getBotpose_MT2();

                // Extract position EXACTLY like your AprilTag example
                double myX = botpose.getPosition().x;
                double myY = botpose.getPosition().y;
                double myZ = botpose.getPosition().z;

                // Send to dashboard
                packet.put("myX", myX);
                packet.put("myY", myY);
                packet.put("myZ", myZ);

                // For debugging:
                telemetry.addData("myX", myX);
                telemetry.addData("myY", myY);
                telemetry.addData("myZ", myZ);
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
