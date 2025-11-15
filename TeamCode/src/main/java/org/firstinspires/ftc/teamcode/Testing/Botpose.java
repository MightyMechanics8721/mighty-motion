package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Test April Tag", group = "Testing")
public class Botpose extends LinearOpMode {

    private Limelight3A limelight;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        dashboard = FtcDashboard.getInstance();   // <-- FIX

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();  // <-- FIX

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                Pose3D botpose = result.getBotpose_MT2();

                packet.put("tx", result.getTx());
                packet.put("ty", result.getTy());
                packet.put("Botpose", botpose.getPosition().x);
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
