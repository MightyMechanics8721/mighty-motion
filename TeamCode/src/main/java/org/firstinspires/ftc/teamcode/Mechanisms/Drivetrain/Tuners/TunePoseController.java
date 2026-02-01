package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

@Config
@Autonomous(name = "Tune Pose Controller", group = "Autonomous")
public class TunePoseController extends LinearOpMode {
    public static double targetXPosition = 0;
    public static double targetYPosition = 0;
    public static double targetZPosition = 0;


    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);

        Drivetrain drivetrain = Drivetrain.getInstance();

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            SimpleMatrix desiredPose = new SimpleMatrix(
                    new double[][]{
                            new double[]{targetXPosition},
                            new double[]{targetYPosition},
                            new double[]{Math.toRadians(targetZPosition)}
                    }
            );

            Actions.runBlocking(drivetrain.goToPose(desiredPose));
        }
    }
}