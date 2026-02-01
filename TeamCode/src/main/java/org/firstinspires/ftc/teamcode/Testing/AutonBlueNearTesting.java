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
        Drivetrain.initialize(hardwareMap);
        Battery.initialize(hardwareMap);
        // ---- HARDWARE -----
        Battery battery = Battery.getInstance();
        Drivetrain drivetrain = Drivetrain.getInstance();
        Intake intake = new Intake(hardwareMap);
        Indexer indexer = new Indexer(hardwareMap, battery);
        Shooter shooter = new Shooter(hardwareMap);

        // ---- UTILS -----
        ElapsedTime looptime = new ElapsedTime();
        SleepAction sleepAction;
        // ---- INIT ----- FIX ODO,UPDATE
        TelemetryPacket packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        drivetrain.setInitialPose(-51, -51, 45);
        dashboard.sendTelemetryPacket(packet);
        //        drivetrain.localize();
        double[][] firstStep = {{-51, -51}, {-24, -24}, {6, -24}, {12, -36}, {12, -55}};
        double[][] secondRowToShoot = {{12, -55}, {12, -46}, {9, -36}, {-8, -18}};
        double[][] shootToGate = {{-8, -18}, {12, -48}, {14, -56}};
        double[][] gateToShoot = {{14, -56}, {12, -46}, {9, -36}, {-8, -14}};
        double[][] thirdRowStep = {{-8, -14}, {12, -20}, {36, -34}, {36, -36}, {36, -55}};
        double[][] firstRowStep = {{-8, -14}, {-12, -34}, {-12, -36}, {-12, -55}};
        Path path = new Path(firstStep, Math.toRadians(-90), false, false);
        Path secondToShoot = new Path(secondRowToShoot, Math.toRadians(-50), true, false);
        Path gate = new Path(shootToGate, Math.toRadians(-125), false, false);
        Path shoot = new Path(gateToShoot, Math.toRadians(-50), true, false);
        Path thirdRow = new Path(thirdRowStep, Math.toRadians(-90), false, false);
        Path firstRow = new Path(firstRowStep, Math.toRadians(-90), false, false);
        waitForStart();
        looptime.reset();
        drivetrain.setInitialPose(-51, -51, 45);
        Actions.runBlocking(
                new SequentialAction(
                        drivetrain.followPath(
                                path, 120, 1.5, Math
                                        .toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                secondToShoot, 120, 1.5,
                                Math.toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                gate, 120, 3, Math
                                        .toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                shoot, 120, 1.5, Math
                                        .toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                gate, 120, 3, Math
                                        .toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                shoot, 120, 1.5, Math
                                        .toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                thirdRow, 120, 3, Math
                                        .toRadians(2.5), true
                        ),
                        drivetrain.goToPose(
                                Utils.makePoseVector(
                                        -8, -18, -90),
                                3, Math.toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                shoot, 120, 1.5,
                                Math.toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                firstRow, 120, 3, Math
                                        .toRadians(2.5), true
                        ),
                        drivetrain.goToPose(
                                Utils.makePoseVector(-8, -14, -90),
                                3, Math.toRadians(2.5), true
                        ),
                        drivetrain.followPath(
                                shoot, 120, 1.5, Math
                                        .toRadians(2.5), true
                        ),
                        drivetrain.goToPose(
                                Utils.makePoseVector(-24, -6, -90), 3,
                                Math.toRadians(2.5), true
                        )


                )
        );
    }
}
