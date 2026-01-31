package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Geometry.Path;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;

@Config
@Autonomous(name = "Blue Near Pursuit", group = "aaa")
public class AutonBlueNearTesting extends LinearOpMode {


    @Override
    public void runOpMode() {
        // ----- TELEMETRY -----
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        // ---- HARDWARE -----
        Battery battery = new Battery(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, battery);
        Intake intake = new Intake(hardwareMap, battery);
        Indexer indexer = new Indexer(hardwareMap, battery);
        Shooter shooter = new Shooter(hardwareMap, battery);

        // ---- UTILS -----
        ElapsedTime looptime = new ElapsedTime();
        SleepAction sleepAction;
        // ---- INIT ----- FIX ODO,UPDATE
        drivetrain.setInitialPose(-51, -51, 45);
        //        drivetrain.localize();
        double[][] firstStep = {{-24, -24}, {6, -24}, {12, -36}, {12, -55}};
        double[][] shootToGate = {{12, -48}, {14, -56}};
        double[][] gateToShoot = {{12, -46}, {9, -36}, {-8, -18}};
        double[][] thirdRowStep = {{28, -36}, {36, -36}, {36, -55}};
        double[][] firstRowStep = {{-9, -36}, {-12, -36}, {-12, -55}};
        Path path = new Path(firstStep, -90, false, false);
        Path gate = new Path(shootToGate, -140, false, false);
        Path shoot = new Path(gateToShoot, -140, false, true);
        Path thirdRow = new Path(thirdRowStep, -90, false, false);
        Path firstRow = new Path(firstRowStep, -90, false, false);
        waitForStart();
        looptime.reset();
        drivetrain.setInitialPose(-51, -51, 45);
        Actions.runBlocking(
                new SequentialAction(
                        //                        Path path,
                        //        double maxSpeed,
                        //        double distanceThreshold,
                        //        double angleThreshold,
                        //        boolean useStoppingDistance,
                        //        TelemetryPacket packet
                        drivetrain.followPath(
                                path, 120, 1.5,
                                Math.toRadians(2.5), true
                        ),
                        drivetrain.goToPose(
                                Utils.makePoseVector(-6, -18, -90),
                                1.5, Math.toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                gate, 120, 1.5,
                                Math.toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                shoot, 120, 1.5,
                                Math.toRadians(2.5), true
                        )
                        //                        drivetrain.followPath(gate, 1.5, 0.05),
                        //                        drivetrain.followPath(shoot, 3, 0.05),
                        //                        drivetrain.followPath(gate, 1.5, 0.05),
                        //                        drivetrain.followPath(shoot, 3, 0.05),
                        //                        drivetrain.followPath(thirdRow, 1.5, 0.05),
                        //                        drivetrain.goToPose(Utils.makePoseVector(-6,
                        //                        -18, -90)),
                        //                        drivetrain.followPath(firstRow, 1.5, 0.05),
                        //                        drivetrain.goToPose(Utils.makePoseVector(-6,
                        //                        -18, -90))

                )
        );
    }
}
