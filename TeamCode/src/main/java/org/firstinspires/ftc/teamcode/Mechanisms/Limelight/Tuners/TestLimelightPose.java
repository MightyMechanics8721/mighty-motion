package org.firstinspires.ftc.teamcode.Mechanisms.Limelight.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drawing;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
@TeleOp(name = "Tune LL", group = "123Competition")
public class TestLimelightPose extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        limelight.pipelineSwitch(1);
        waitForStart();
        /*
         * Starts polling for data.
         */
        limelight.start();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    packet.put("x", botpose.getPosition().x * 39.37);
                    packet.put("y", botpose.getPosition().y * 39.37);
                    packet.put("theta", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
                    SimpleMatrix botState = Utils.makePoseVector(
                            botpose.getPosition().x * 39.37,
                            botpose.getPosition().y * 39.37,
                            botpose.getOrientation().getYaw(AngleUnit.DEGREES)
                    );

                    Canvas canvas = packet.fieldOverlay();
                    Drawing.drawRobot(botState, canvas, "black");
                    dashboard.sendTelemetryPacket(packet);
                }
            }
        }
    }
}