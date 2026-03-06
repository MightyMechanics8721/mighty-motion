package org.firstinspires.ftc.teamcode.Testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Geometry.Path;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;;

@Config
@Autonomous(name = "RED FAR Autonomous  3/5/2026", group = "TEST")
public class RedFar extends LinearOpMode {
    public static double xGate = 12;
    public static double yGate = -57.5;
    public static double thetaGate = -128.5;
    public static double FOLLOW_PATH_TIME = 2;
    public static double ROW_TRANSFER_TIME = 2;
    public static double staticTurretAngle;
    public static SimpleMatrix staticRobotState;
    private static double SHOOTER_VELOCITY_NORMAL = 2500;

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
        turret.reset();
        SleepAction sleepAction;
        // todo ---- INIT ----- FIX ODO,UPDATE
        TelemetryPacket packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        drivetrain.setInitialPose(60, 24, 180);
        dashboard.sendTelemetryPacket(packet);

        double[][] sHTohP = {{60, 24}, {58, 36}, {60, 48}, {60, 60}};
        double[][] hPtoShoot = {{60, 60}, {60, 48}, {60, 24}};
        double[][] sHToTR = {{60, 24}, {48, 20}, {36, 30}, {36, 36}, {36, 57}};
        double[][] TRTosH = {{36, 57}, {50, 30}, {60, 24}};
        Path shootToHumanPlayer = new Path(sHTohP, Math.toRadians(90), false, false);
        Path shootToHumanPlayerGATE = new Path(sHTohP, Math.toRadians(180), false, false);
        Path humanPlayerToShoot = new Path(hPtoShoot, Math.toRadians(90), true, false);
        Path shootToThirdRow = new Path(sHToTR, Math.toRadians(90), false, true);
        Path thirdRowToShoot = new Path(TRTosH, Math.toRadians(180), true, false);


        waitForStart();

        looptime.reset();
        Turret.staticTheta = 0;
        drivetrain.setInitialPose(60, 24, 180);
        turret.setInitialAngle(0);
        //Alex Ko bless this code
        Actions.runBlocking(
                new ParallelAction( //main loop
                        shooter.autoShootMovingInfinite(-57, 57),
                        turret.autoAimInfinite(new Vector2d(
                                -68,
                                68
                        )),
                        updateTurretAngle(),
                        updateRobotState(),
                        new SleepAction(1),
                        new SequentialAction(
                                robot.moveShootFAR(), // ----- SHOOT PRELOAD -----

                                //  ----- INTAKE HUMAN PLAYER -----
                                new ParallelAction(
                                        drivetrain.followPathTimed(
                                                shootToHumanPlayer,
                                                150,
                                                3,
                                                Math
                                                        .toRadians(
                                                                2.5),
                                                true, FOLLOW_PATH_TIME
                                        ),
                                        transfer.ballDetectionTimed(
                                                ROW_TRANSFER_TIME)),

                                // ----- SHOOT HUMAN PLAYER -----

                                drivetrain.followPathTimed(
                                        humanPlayerToShoot,
                                        150,
                                        0.0,
                                        Math
                                                .toRadians(
                                                        0.0),
                                        true, FOLLOW_PATH_TIME
                                ),
                                new SleepAction(0.1),
                                robot.moveShootFAR(),


                                // ----- INTAKE THIRD ROW -----

                                new ParallelAction(
                                        drivetrain.followPathTimed(
                                                shootToThirdRow,
                                                150,
                                                3,
                                                Math
                                                        .toRadians(
                                                                2.5),
                                                true, FOLLOW_PATH_TIME),
                                        transfer.ballDetectionTimed(ROW_TRANSFER_TIME)),

                                // ----- SHOOT THIRD ROW -----

                                drivetrain.followPathTimed(
                                        thirdRowToShoot,
                                        150,
                                        0.0,
                                        Math
                                                .toRadians(
                                                        0.0),
                                        true, FOLLOW_PATH_TIME
                                ),
                                new SleepAction(0.1),
                                robot.moveShootFAR(),

                                // ----- GO TO HUMAN PLAYER (COLLECT LINGERING BALLS FROM GATE) -----

                                new ParallelAction(
                                        drivetrain.followPathTimed(
                                                shootToHumanPlayerGATE,
                                                150,
                                                0.0,
                                                Math
                                                        .toRadians(
                                                                2.5),
                                                true, FOLLOW_PATH_TIME),
                                        transfer.ballDetectionTimed(10)),

                                drivetrain.followPathTimed(
                                        humanPlayerToShoot,
                                        150,
                                        0.0,
                                        Math
                                                .toRadians(
                                                        0.0),
                                        true, FOLLOW_PATH_TIME
                                ),
                                new SleepAction(0.1),
                                robot.moveShootFAR(),
                                new ParallelAction(
                                        drivetrain.followPathTimed(
                                                shootToHumanPlayerGATE,
                                                150,
                                                0.0,
                                                Math
                                                        .toRadians(
                                                                2.5),
                                                true, FOLLOW_PATH_TIME),
                                        transfer.ballDetectionTimed(10)),

                                drivetrain.followPathTimed(
                                        humanPlayerToShoot,
                                        150,
                                        0.0,
                                        Math
                                                .toRadians(
                                                        0.0),
                                        true, FOLLOW_PATH_TIME
                                ),
                                new SleepAction(0.1),
                                robot.moveShootFAR()
                        )

                )

        );
    }


    public Action updateTurretAngle() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                double turretAngle = Turret.getInstance().getAngle();


                if (opModeIsActive() && !isStopRequested()) {
                    staticTurretAngle = turretAngle;
                }
                return true;
            }
        };
    }

    public Action updateRobotState() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                SimpleMatrix robotState = Drivetrain.getInstance().state;
                if (opModeIsActive() && !isStopRequested()) {
                    staticRobotState = robotState;
                }
                return true;
            }
        };
    }
}