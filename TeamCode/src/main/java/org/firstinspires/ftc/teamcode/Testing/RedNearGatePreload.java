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
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;;

@Config
@Autonomous(name = "Red Near BACKUP PRELOAD ", group = "TESTING")
public class RedNearGatePreload extends LinearOpMode {
    public static double xGateBackup = 17;
    public static double yGateBackup = 55.5;
    public static double thetaGateBackup = 135;
    public static double xGate = 11.5;
    public static double yGate = 57.5;
    public static double thetaGate = 120;
    public static double FOLLOW_PATH_TIME = 1.5;
    public static double GATE_TRANSFER_TIME = 2.5;
    public static double ROW_TRANSFER_TIME = 1.75;
    public static double EXTRA_TRANSFER_TIME = 0.5;
    public static double ALL_PATH_TIME = 4;
    //        public static double preloadTime = 0.45;
    public static double staticTurretAngle;
    public static SimpleMatrix staticRobotState;
    private double SHOOTER_VELOCITY_NORMAL = 2500;

    public Action updateTurretAngle() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                double turretAngle = Turret.getInstance().getAngle();


                if (opModeIsActive() && !isStopRequested()) {
                    RedNearGate.staticTurretAngle = turretAngle;
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
                    RedNearGate.staticRobotState = robotState;
                }
                return true;
            }
        };
    }

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
        drivetrain.setInitialPose(-51, 51, -45);
        dashboard.sendTelemetryPacket(packet);
        double[][] firstStep = {{-51, 51}, {-12, 12}, {12, 12}};
        double[][] firstHalfStep = {{12, 12}, {12, 36}, {12, 59}};

        double[][] secondRowToShoot = {{12, 55}, {12, 46}, {9, 36}, {-8, 14}};
        double[][] shootToGate = {
                {-8, 14},
                {13.5, 30},
                {13.5, 48},
                {xGate, yGate}
        };
        double[][] gateToShoot = {{13.5, 59}, {12, 46}, {9, 36}, {-8, 14}};
//        double[][] gateToShootFinal = {{13.5, 59}, {12, 46}, {9, 36}, {-8, 14}, {-36, 12}};
        double[][] thirdRowStep = {
                {-8, 14},
                {12, 20},
                {36, 20},
                {36, 30},
                {36, 36},
                {36, 60}
        };
        double[][] thirdRowToShoot = {{36, 60}, {-8, 14}};
        double[][] firstRowStep = {{-8, 14}, {-12, 34}, {-12, 36}, {-12, 54}};
        Path preload = new Path(firstStep, Math.toRadians(10), false, false);
        Path firstHalf = new Path(firstHalfStep, Math.toRadians(90), false, false);
        Path secondToShoot = new Path(secondRowToShoot, Math.toRadians(50), true, false);
        Path gate = new Path(shootToGate, Math.toRadians(thetaGate), false, false);
        Path shoot = new Path(gateToShoot, Math.toRadians(50), true, false);
//        Path shootFinal = new Path(gateToShootFinal, Math.toRadians(50), true, false);
        Path thirdRow = new Path(thirdRowStep, Math.toRadians(90), false, false);
        Path thirdToShoot = new Path(thirdRowToShoot, Math.toRadians(45), true, false);
        Path firstRow = new Path(firstRowStep, Math.toRadians(90), false, false);

        waitForStart();

        looptime.reset();
        Turret.staticTheta = 0;
        drivetrain.setInitialPose(-51, 51, -45);
        turret.setInitialAngle(180);
        //Alex Ko bless this code
        Actions.runBlocking(
                new SequentialAction(
                        shooter.hardStopOpen(),
                        new ParallelAction( //main loop
                                shooter.autoShootMovingInfinite(-57, 57),
                                turret.autoAimInfinite(new Vector2d(
                                        -68,
                                        68
                                )),
                                StaticVariables.updateTurretAngle(opModeIsActive(), isStopRequested()),
                                StaticVariables.updateRobotState(opModeIsActive(), isStopRequested()),
                                new SequentialAction(
                                        new SequentialAction(
                                                drivetrain.followPathTimed(
                                                        preload, 150, 2,
                                                        Math.toRadians(2.5), true, ALL_PATH_TIME
                                                ),
                                                shooter.hardStopOpen(),
                                                new SleepAction(0.1),
                                                robot.moveShoot()
                                        ),


                                        new ParallelAction( //GO TO SECOND ROW
                                                shooter.hardStopClose(),
                                                drivetrain.followPathTimed(
                                                        firstHalf,
                                                        150,
                                                        1.5,
                                                        Math
                                                                .toRadians(
                                                                        2.5),
                                                        true, ALL_PATH_TIME
                                                ),
                                                transfer.ballDetectionTimed(
                                                        ROW_TRANSFER_TIME)
                                                // GATHER SECOND ROW
                                        ),
                                        new SequentialAction( // SHOOT SECOND ROW
                                                drivetrain.followPathTimed(
                                                        secondToShoot,
                                                        150,
                                                        2,
                                                        Math.toRadians(2.5),
                                                        true, ALL_PATH_TIME
                                                ),
                                                shooter.hardStopOpen(),
                                                new SleepAction(0.1),
                                                robot.moveShoot()
                                        ),
                                        new ParallelAction( // GO TO GATE
                                                shooter.hardStopClose(),
                                                new SequentialAction(
                                                        drivetrain
                                                                .followPathTimed(
                                                                        gate,
                                                                        150,
                                                                        0.0,
                                                                        Math.toRadians
                                                                                (0.0),
                                                                        true,
                                                                        FOLLOW_PATH_TIME
                                                                ),
                                                        drivetrain.goToPoseTimed(
                                                                Utils
                                                                        .makePoseVector(
                                                                                xGateBackup,
                                                                                yGateBackup,
                                                                                thetaGateBackup
                                                                        ),
                                                                0.5,
                                                                Math.toRadians(
                                                                        2.5),
                                                                true, ALL_PATH_TIME
                                                        )
                                                ),
                                                new SequentialAction(
                                                        new SleepAction(1),
                                                        transfer.ballDetectionTimed(
                                                                GATE_TRANSFER_TIME)
                                                        // COLLECT GATE
                                                )
                                        ),
                                        new ParallelAction(
                                                transfer.ballDetectionTimed(
                                                        EXTRA_TRANSFER_TIME),
                                                new SequentialAction(
                                                        // SHOOT GATE COLLECT
                                                        drivetrain.followPathTimed(
                                                                shoot,
                                                                150,
                                                                2,
                                                                Math.toRadians(2.5),
                                                                true, ALL_PATH_TIME
                                                        ),
                                                        shooter.hardStopOpen(),
                                                        new SleepAction(0.1),
                                                        robot.moveShoot()
                                                )
                                        ),
                                        new ParallelAction( //GO TO THIRD ROW
                                                drivetrain.followPathTimed(
                                                        thirdRow,
                                                        150,
                                                        1.5,
                                                        Math
                                                                .toRadians(
                                                                        2.5),
                                                        true, ALL_PATH_TIME
                                                ),
                                                transfer.ballDetectionTimed(
                                                        ROW_TRANSFER_TIME)
                                                // GATHER THIRD ROW
                                        ),
                                        new SequentialAction( // SHOOT THIRD ROW
                                                drivetrain.followPathTimed(
                                                        thirdToShoot,
                                                        150,
                                                        2,
                                                        Math.toRadians(2.5),
                                                        true, ALL_PATH_TIME
                                                ),
                                                shooter.hardStopOpen(),
                                                new SleepAction(0.1),
                                                robot.moveShoot()
                                                // SHOOT THIRD ROW
                                        ),
                                        new ParallelAction( // GO TO GATE 2
                                                shooter.hardStopClose(),
                                                new SequentialAction(
                                                        drivetrain
                                                                .followPathTimed(
                                                                        gate,
                                                                        150,
                                                                        0.0,
                                                                        Math.toRadians
                                                                                (0.0),
                                                                        true,
                                                                        FOLLOW_PATH_TIME
                                                                ),
                                                        drivetrain.goToPoseTimed(
                                                                Utils
                                                                        .makePoseVector(
                                                                                xGateBackup,
                                                                                yGateBackup,
                                                                                thetaGateBackup
                                                                        ),
                                                                0.5,
                                                                Math.toRadians(
                                                                        2.5),
                                                                true, ALL_PATH_TIME
                                                        )
                                                ),
                                                new SequentialAction(
                                                        new SleepAction(1),
                                                        transfer.ballDetectionTimed(
                                                                GATE_TRANSFER_TIME)
                                                        // COLLECT GATE 2
                                                )
                                        ),
                                        new ParallelAction(
                                                transfer.ballDetectionTimed(
                                                        EXTRA_TRANSFER_TIME),
                                                new SequentialAction(
                                                        // SHOOT GATE 2 COLLECT
                                                        drivetrain.followPathTimed(
                                                                shoot,
                                                                150,
                                                                2,
                                                                Math.toRadians(2.5),
                                                                true, ALL_PATH_TIME
                                                        ),
                                                        shooter.hardStopOpen(),
                                                        new SleepAction(0.1),
                                                        robot.moveShoot()
                                                )
                                        ),
                                        new ParallelAction( //GO TO FIRST ROW
                                                drivetrain.followPathTimed(
                                                        firstRow,
                                                        150,
                                                        2,
                                                        Math
                                                                .toRadians(
                                                                        2.5),
                                                        true, ALL_PATH_TIME
                                                ),
                                                transfer.ballDetectionTimed(
                                                        ROW_TRANSFER_TIME)
                                                // GATHER FIRST ROW
                                        ),
                                        new SequentialAction( // SHOOT FIRST ROW
                                                drivetrain.goToPoseTimed(
                                                        Utils.makePoseVector(
                                                                -8,
                                                                14,
                                                                90
                                                        ),
                                                        2,
                                                        Math.toRadians(2.5),
                                                        true, ALL_PATH_TIME
                                                ),
                                                shooter.hardStopOpen(),
                                                new SleepAction(0.1),
                                                robot.moveShoot()
                                                // SHOOT FIRST ROW
                                        ),
                                        new ParallelAction( // GO TO GATE 3
                                                shooter.hardStopClose(),
                                                new SequentialAction(
                                                        drivetrain
                                                                .followPathTimed(
                                                                        gate,
                                                                        150,
                                                                        0.0,
                                                                        Math.toRadians
                                                                                (0.0),
                                                                        true,
                                                                        FOLLOW_PATH_TIME
                                                                ),
                                                        drivetrain.goToPoseTimed(
                                                                Utils
                                                                        .makePoseVector(
                                                                                xGateBackup,
                                                                                yGateBackup,
                                                                                thetaGateBackup
                                                                        ),
                                                                0.5,
                                                                Math.toRadians(
                                                                        2.5),
                                                                true, ALL_PATH_TIME
                                                        )
                                                ),
                                                new SequentialAction(
                                                        new SleepAction(1),
                                                        transfer.ballDetectionTimed(
                                                                GATE_TRANSFER_TIME)
                                                        // COLLECT GATE 3
                                                )
                                        ),
                                        new ParallelAction(
                                                transfer.ballDetectionTimed(
                                                        EXTRA_TRANSFER_TIME),
                                                new SequentialAction(
                                                        // SHOOT GATE COLLECT 3
                                                        drivetrain.followPathTimed(
                                                                shoot,
                                                                150,
                                                                1.5,
                                                                Math.toRadians(2),
                                                                true, ALL_PATH_TIME
                                                        ),
                                                        shooter.hardStopOpen(),
                                                        new SleepAction(0.1),
                                                        robot.moveShoot(),
                                                        new SleepAction(0.1),
                                                        drivetrain.goToPoseTimed(Utils.makePoseVector
                                                                        (-36, 12, 0.5),
                                                                2, Math.toRadians(2), true, ALL_PATH_TIME)
                                                )
                                        )

                                )
                        )
                )
        );
    }
}
