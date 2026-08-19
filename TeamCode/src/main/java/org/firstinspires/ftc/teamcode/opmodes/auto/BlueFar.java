package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.storage.StaticVariables;

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

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.Path;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

@Config
@Autonomous(name = "Blue FAR Autonomous  3/7", group = "TEST")
public class BlueFar extends LinearOpMode {
    public static double ROW_TRANSFER_TIME = 3;
    public static double velocity = 2700; // (rev/min)
    public double SHOOT_TIME = 3;

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
        drivetrain.setInitialPose(64, -24, -180);
        dashboard.sendTelemetryPacket(packet);

        double[][] preC = {{64, -24}, {60, -24}};
        double[][] shthP2 = {{60, -24}, {60, -36}, {60, -48}, {60, -62}};
        double[][] pullOut = {{60, -63}, {60, -58}};
        double[][] impreg = {{60, -53}, {60, -60}};
        //        double[][] shthP2c = {{46, -63}, {64, -64}};

        double[][] hPtoShoot = {{60, -60}, {60, -48}, {60, -24}};
        double[][] sHToTR = {{60, -24}, {36, -15}, {36, -62}};
        double[][] TRTosH = {{36, -62}, {50, -30}, {60, -24}};
        double[][] sHToLB = {{60, -24}, {45, -36}, {45, -48}, {55, -62}};
        double[][] LBTosH = {{55, -60}, {60, -24}};
        double[][] preCu = {{60, -24}, {60, -36}};

        Path um = new Path(preC, Math.toRadians(-180), false, true);
        Path shootToHumanPlayer = new Path(shthP2, Math.toRadians(-90), false, true);
        //        Path shootToHumanPlayerC = new Path(shthP2c, Math.toRadians(-90), false, false);
        Path shootToLingeringBalls = new Path(sHToLB, Math.toRadians(-140), false, true);
        Path shootToThirdRow = new Path(sHToTR, Math.toRadians(-90), false, true);

        Path thirdRowToShoot = new Path(TRTosH, Math.toRadians(-180), true, false);
        Path lingeringBallsToShoot = new Path(LBTosH, Math.toRadians(-180), true, false);
        Path humanPlayerToShoot = new Path(hPtoShoot, Math.toRadians(-90), true, false);
        Path cim = new Path(preCu, Math.toRadians(-180), false, false);
        //        Path pull0ut = new Path(pullOut, Math.toRadians(-70), false, true);
        Path impr3g = new Path(impreg, Math.toRadians(-90), false, true);
        waitForStart();

        looptime.reset();
        Turret.staticTheta = 0;
        drivetrain.setInitialPose(64, -24, -180);
        turret.setInitialAngle(0);
        //Alex Ko bless this code
        try {
            Actions.runBlocking(
                    new ParallelAction( //main loop
                                        shooter.autonomousVelocityInfinite(Utils.rpmToRadPerSec(velocity)),
                                        turret.autoAimInfinite(new Vector2d(
                                                -68,
                                                -69
                                        )),

                                        shooter.hardStopClose(),
                                        updateTurretAngle(),
                                        updateRobotState(),
                                        new SequentialAction(
                                                drivetrain.followPathTimed(
                                                        um,
                                                        150,
                                                        1,
                                                        0.5,
                                                        true,
                                                        0.5
                                                ),
                                                new SleepAction(1),
                                                robot.moveShootFAR(),
                                                // ----- SHOOT PRELOAD -----

                                                // ----- INTAKE THIRD ROW -----
                                                robot.gatherRow(
                                                        shootToThirdRow,
                                                        150,
                                                        1.5,
                                                        2.0,
                                                        1.5
                                                ),
                                                // ----- SHOOT THIRD ROW -----
                                                robot.shootFAR(
                                                        thirdRowToShoot,
                                                        150,
                                                        1.5,
                                                        2.0,
                                                        1.5
                                                ),

                                                //  ----- INTAKE HUMAN PLAYER -----
                                                //drivetrain.followPathTimed(pull0ut, 150, 2.0, 3.0,
                                                // true, 2),
                                                //
                                                //        robot.gatherRow(
                                                //                impr3g,
                                                //                150,
                                                //                0.5,
                                                //                1,
                                                //                1
                                                //        )
                                                //                                robot.gatherRow(
                                                //                                        shootToHumanPlayerC,
                                                //                                        150,
                                                //                                        1.5,
                                                //                                        2.0,
                                                //                                        2
                                                //                                ),
                                                robot.gatherRow(
                                                        shootToHumanPlayer,
                                                        150,
                                                        1.5,
                                                        2.0,
                                                        1
                                                ),
                                                new ParallelAction(
                                                        transfer.ballDetectionTimed(1),
                                                        // ----- SHOOT HUMAN PLAYER -----
                                                        robot.shootFARFast(
                                                                humanPlayerToShoot,
                                                                150,
                                                                1.5,
                                                                2.0,
                                                                1
                                                        )
                                                ),
                                                robot.gatherRow(
                                                        shootToHumanPlayer,
                                                        150,
                                                        1.5,
                                                        2.0,
                                                        1
                                                ),
                                                new ParallelAction(
                                                        transfer.ballDetectionTimed(1),
                                                        // ----- SHOOT HUMAN PLAYER -----
                                                        robot.shootFARFast(
                                                                humanPlayerToShoot,
                                                                150,
                                                                1.5,
                                                                2.0,
                                                                1
                                                        )
                                                ),
                                                robot.gatherRow(
                                                        shootToHumanPlayer,
                                                        150,
                                                        1.5,
                                                        2.0,
                                                        1
                                                ),
                                                new ParallelAction(
                                                        transfer.ballDetectionTimed(1),
                                                        // ----- SHOOT HUMAN PLAYER -----
                                                        robot.shootFARFast(
                                                                humanPlayerToShoot,
                                                                150,
                                                                1.5,
                                                                2.0,
                                                                1
                                                        )
                                                ),
                                                robot.gatherRow(
                                                        shootToHumanPlayer,
                                                        150,
                                                        1.5,
                                                        2.0,
                                                        1
                                                ),
                                                new ParallelAction(
                                                        transfer.ballDetectionTimed(1),
                                                        // ----- SHOOT HUMAN PLAYER -----
                                                        robot.shootFARFast(
                                                                humanPlayerToShoot,
                                                                150,
                                                                1.5,
                                                                2.0,
                                                                1
                                                        )
                                                ),
                                                robot.gatherRow(
                                                        shootToHumanPlayer,
                                                        150,
                                                        1.5,
                                                        2.0,
                                                        1
                                                ),
                                                new ParallelAction(
                                                        transfer.ballDetectionTimed(1),
                                                        // ----- SHOOT HUMAN PLAYER -----
                                                        robot.shootFARFast(
                                                                humanPlayerToShoot,
                                                                150,
                                                                1.5,
                                                                2.0,
                                                                1
                                                        )
                                                ),
                                                robot.gatherRow(
                                                        shootToHumanPlayer,
                                                        150,
                                                        1.5,
                                                        2.0,
                                                        1
                                                ),
                                                new ParallelAction(
                                                        transfer.ballDetectionTimed(1),
                                                        // ----- SHOOT HUMAN PLAYER -----
                                                        robot.shootFARFast(
                                                                humanPlayerToShoot,
                                                                150,
                                                                1.5,
                                                                2.0,
                                                                1
                                                        )
                                                ),

                                                // ----- GO TO HUMAN PLAYER (COLLECT LINGERING BALLS
                                                // FROM GATE) -----

                                                drivetrain.followPathTimed(
                                                        cim,
                                                        150,
                                                        0,
                                                        0,
                                                        true,
                                                        1.5
                                                )

                                        )
                    )

            );
        } finally {
            Robot.saveFinalState();
        }
    }

    public Action updateTurretAngle() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                double turretAngle = Turret.getInstance().getAngle();

                if (opModeIsActive() && !isStopRequested()) {
                    StaticVariables.saveTurretAngle(turretAngle);
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
                    StaticVariables.saveRobotState(robotState);
                }
                return true;
            }
        };
    }
}