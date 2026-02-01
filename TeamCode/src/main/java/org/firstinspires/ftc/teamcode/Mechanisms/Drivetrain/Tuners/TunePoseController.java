package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

@Config
@Autonomous(name = "Tune Pose Controller", group = "TeleOp")
public class TunePoseController extends LinearOpMode {
    public static double targetXPosition = 0;
    public static double targetYPosition = 0;
    public static double targetZPosition = 0;

    public static double positionThreshold = 2.0;
    public static double angleThreshold = Math.toRadians(2.5);

    public static boolean useStoppingDistance = true;


    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();

        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);

        Drivetrain drivetrain = Drivetrain.getInstance();


        TelemetryPacket packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        drivetrain.setInitialPose(0, 0, 0);
        dashboard.sendTelemetryPacket(packet);

        waitForStart();

        drivetrain.setInitialPose(0, 0, 0);

        while (opModeIsActive()) {
            SimpleMatrix desiredPose = new SimpleMatrix(
                    new double[][]{
                            new double[]{targetXPosition},
                            new double[]{targetYPosition},
                            new double[]{Math.toRadians(targetZPosition)}
                    }
            );

            packet = new TelemetryPacket();
            if (gamepad1.right_trigger > 0.1) {
                drivetrain.goToPose(
                        desiredPose,
                        positionThreshold,
                        angleThreshold,
                        useStoppingDistance
                ).run(packet);
            } else {
                drivetrain.stopMotors().run(packet);
            }

            dashboard.sendTelemetryPacket(packet);
        }
    }
}