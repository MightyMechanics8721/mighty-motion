package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import java.util.List;

@TeleOp(name="Limelight Left/Right", group="Linear Opmode")
public class LimelightLeftRight extends LinearOpMode {

    private Limelight3A limelight;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();


        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            int tagID = -1;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    tagID = fiducial.getFiducialId();
                }

                // Only process the tag you want (e.g., tag 1)
                if (tagID == 1) {
                    //double tx = result.getTx();
                    double tx = result.getTx();

                    if (tx < -1) {
                        packet.put("Position", "Left");
                    } else if (tx > 1) {
                        packet.put("Position", "Right");
                    } else {
                        packet.put("Position", "Center");
                    }

                    packet.put("Tag ID", tagID);
                    packet.put("tx", tx);
                } else {
                    packet.put("Position", "Other Tag Detected: " + tagID);
                }
            } else {
                packet.put("Position", "No Target");
            }

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);

        }
    }
}

