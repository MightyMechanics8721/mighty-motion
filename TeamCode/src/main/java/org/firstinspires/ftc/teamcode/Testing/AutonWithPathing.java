package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;

@Config
@Autonomous(name = "TEST BLUE AUTON", group = "Testing")
public class AutonWithPathing extends LinearOpMode {
    public static double xOffset = 0;
    public static double yOffset = 0;
    private double SHOOTER_VELOCITY_NORMAL = 2500;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        Battery.initialize(hardwareMap);
        Turret.initialize(hardwareMap);
        Indexer.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        Transfer.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        Robot.initialize(hardwareMap);
        Battery battery = Battery.getInstance();
        Turret turret = Turret.getInstance();
        Intake intake = Intake.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();
        Transfer transfer = Transfer.getInstance();
        Drivetrain drivetrain = Drivetrain.getInstance();
        Robot robot = Robot.getInstance();
        ElapsedTime looptime = new ElapsedTime();
        SleepAction sleepAction;
        // todo ---- INIT ----- FIX ODO,UPDATE
        TelemetryPacket packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        drivetrain.setInitialPose(-51, -51, 45);
        dashboard.sendTelemetryPacket(packet);
        double[][] firstStep = {{-51, -51}, {-12, -12}, {12, -12}, {12, -36}, {12, -59}};
        double[][] secondRowToShoot = {{12, -55}, {12, -46}, {9, -36}, {-8, -14}};
        double[][] shootToGate = {{-8, -14}, {13.5, -48}, {13.5, -59}};
        double[][] gateToShoot = {{13.5, -59}, {12, -46}, {9, -36}, {-8, -14}};
        double[][] thirdRowStep = {{-8, -14}, {12, -20}, {36, -20}, {36, -30}, {36, -36}, {36, -57}};
        double[][] firstRowStep = {{-8, -14}, {-12, -34}, {-12, -36}, {-12, -54}};
        Path path = new Path(firstStep, Math.toRadians(-90), false, false);
        Path secondToShoot = new Path(secondRowToShoot, Math.toRadians(-50), true, false);
        Path gate = new Path(shootToGate, Math.toRadians(-145), false, false);
        Path shoot = new Path(gateToShoot, Math.toRadians(-50), true, false);
        Path thirdRow = new Path(thirdRowStep, Math.toRadians(-90), false, false);
        Path firstRow = new Path(firstRowStep, Math.toRadians(-90), false, false);

        waitForStart();

        looptime.reset();
        drivetrain.setInitialPose(-51, -51, 45);
        //Alex Ko bless this code
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction( //init
                                shooter.revShooter(2000 * 2 * Math.PI / 60),
                                shooter.hardStopOpen()),
                        new ParallelAction( //main loop
                                shooter.autoShootMovingInfinite(),
                                turret.autoAimInfinite(new Vector2d(-68, -68)),

                                new SequentialAction(
                                        new ParallelAction(
                                                drivetrain.followPath(path, 150, 2,
                                                        Math.toRadians(2.5), true),
                                                new SequentialAction(
                                                        new SleepAction(0.4),
                                                        robot.moveShoot(), // SHOOT PRELOAD
                                                        transfer.ballDetectionTimed(2) // GATHER SECOND ROW
                                                )
                                        ),
                                        new SequentialAction( // SHOOT SECOND ROW
                                                shooter.hardStopOpen(),
                                                drivetrain.followPath(secondToShoot, 150, 2,
                                                        Math.toRadians(2.5), true),
                                                robot.moveShoot()
                                        ),
                                        new ParallelAction( // GATHER GATE
                                                shooter.hardStopClose(),
                                                drivetrain.followPath(gate, 150, 1, Math.toRadians(2.5), true),
                                                new SequentialAction(
                                                        new SleepAction(0.4),
                                                        transfer.ballDetectionTimed(2.5) // GATHER SECOND ROW
                                                )
                                        ),
                                        new SequentialAction( // SHOOT GATE COLLECT
                                                shooter.hardStopOpen(),
                                                drivetrain.followPath(shoot, 150, 2,
                                                        Math.toRadians(2.5), true),
                                                robot.moveShoot()
                                        ),
                                        new ParallelAction( // GATHER GATE 2
                                                shooter.hardStopClose(),
                                                drivetrain.followPath(gate, 150, 1, Math.toRadians(2.5), true),
                                                new SequentialAction(
                                                        new SleepAction(0.4),
                                                        transfer.ballDetectionTimed(2.5) // GATHER SECOND ROW
                                                )
                                        ),
                                        new SequentialAction( // SHOOT GATE COLLECT 2
                                                shooter.hardStopOpen(),
                                                drivetrain.followPath(shoot, 150, 2,
                                                        Math.toRadians(2.5), true),
                                                robot.moveShoot()
                                        ),
                                        new ParallelAction( //GO TO THIRD ROW
                                                drivetrain.followPath(
                                                        thirdRow, 150, 3, Math
                                                                .toRadians(2.5), true
                                                ),
                                                transfer.ballDetectionTimed(2) // GATHER THIRD ROW
                                        ),
                                        new SequentialAction( // SHOOT THIRD ROW
                                                shooter.hardStopOpen(),
                                                drivetrain.goToPose(Utils.makePoseVector(
                                                                -8, -14, -90),
                                                        3, Math.toRadians(2.5), true
                                                ), // GO TO SHOOTING POS
                                                robot.moveShoot() // SHOOT THIRD ROW
                                        ),
                                        new ParallelAction( //GO TO FIRST ROW
                                                drivetrain.followPath(
                                                        firstRow, 150, 3, Math
                                                                .toRadians(2.5), true
                                                ),
                                                transfer.ballDetectionTimed(2) // GATHER FIRST ROW
                                        ),
                                        new SequentialAction( // SHOOT FIRST ROW
                                                shooter.hardStopOpen(),
                                                drivetrain.goToPose(Utils.makePoseVector(
                                                                -8, -14, -90),
                                                        3, Math.toRadians(2.5), true
                                                ), // GO TO SHOOTING POS
                                                robot.moveShoot() // SHOOT FIRST ROW
                                        )

                                )
                        )
                )
        );
    }
}
