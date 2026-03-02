//package org.firstinspires.ftc.teamcode.Testing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Geometry.Path;
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
//import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
//import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
//import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
//import org.firstinspires.ftc.teamcode.Mechanisms.Transfer.Transfer;
//import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;
//
//@Config
//@Autonomous(name = "Gate Red", group = "Competition")
//public class RedNearAutonomousGate extends LinearOpMode {
//    public static double xGate = 12;
//    public static double yGate = 57.5;
//    public static double thetaGate = 128.5;
//
//    public static double xGateBack = 12;
//    public static double yGateBack = 57.5;
//    public static double thetaGateBack = 128.5;
//
//    public static double FOLLOW_PATH_TIME = 2;
//    private double SHOOTER_VELOCITY_NORMAL = 2500;
//
//    @Override
//    public void runOpMode() {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        Battery.initialize(hardwareMap);
//        Turret.initialize(hardwareMap);
//        Indexer.initialize(hardwareMap);
//        Intake.initialize(hardwareMap);
//        Shooter.initialize(hardwareMap);
//        Transfer.initialize(hardwareMap);
//        Drivetrain.initialize(hardwareMap);
//        Robot.initialize(hardwareMap);
//        Battery battery = Battery.getInstance();
//        Turret turret = Turret.getInstance();
//        Intake intake = Intake.getInstance();
//        Indexer indexer = Indexer.getInstance();
//        Shooter shooter = Shooter.getInstance();
//        Transfer transfer = Transfer.getInstance();
//        Drivetrain drivetrain = Drivetrain.getInstance();
//        Robot robot = Robot.getInstance();
//        ElapsedTime looptime = new ElapsedTime();
//        turret.reset();
//        SleepAction sleepAction;
//        // todo ---- INIT ----- FIX ODO,UPDATE
//        TelemetryPacket packet = new TelemetryPacket();
//        drivetrain.setTelemetry(packet);
//        drivetrain.setInitialPose(-51, 51, -45);
//        dashboard.sendTelemetryPacket(packet);
//        double[][] firstStep = {{-51, 51}, {-12, 12}, {12, 12}, {12, 36}, {12, 59}};
//        double[][] secondRowToShoot = {{12, 55}, {12, 46}, {9, 36}, {-8, 14}};
//        double[][] shootToGate = {
//                {-8, 14},
//                {13.5, 30},
//                {13.5, 48},
//                {xGate, yGate}
//        };
//        double[][] gateToShoot = {{13.5, 59}, {12, 46}, {9, 36}, {-8, 14}};
//        double[][] gateToShootFinal = {{13.5, 59}, {12, 46}, {9, 36}, {-8, 14}, {-36, 12}};
//        double[][] thirdRowStep = {
//                {-8, 14},
//                {12, 20},
//                {36, 20},
//                {36, 30},
//                {36, 36},
//                {36, 57}
//        };
//        double[][] firstRowStep = {{-8, 14}, {-12, 34}, {-12, 36}, {-12, 54}};
//        Path path = new Path(firstStep, Math.toRadians(90), false, false);
//        Path secondToShoot = new Path(secondRowToShoot, Math.toRadians(50), true, false);
//        Path gate = new Path(shootToGate, Math.toRadians(thetaGate), false, false);
//        Path shoot = new Path(gateToShoot, Math.toRadians(50), true, false);
//        Path shootFinal = new Path(gateToShootFinal, Math.toRadians(50), true, false);
//        Path thirdRow = new Path(thirdRowStep, Math.toRadians(90), false, false);
//        Path firstRow = new Path(firstRowStep, Math.toRadians(90), false, false);
//
//        waitForStart();
//
//        looptime.reset();
//        Turret.staticTheta = 0;
//        drivetrain.setInitialPose(-51, 51, -45);
//        turret.setInitialAngle(180);
//        //Alex Ko bless this code
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction( //init
//                                            shooter.revShooter(2000 * 2 * Math.PI / 60),
//                                            shooter.hardStopOpen()
//                        ),
//                        new ParallelAction( //main loop
//                                            shooter.autoShootMovingInfinite(-57, 57),
//                                            turret.autoAimInfinite(new Vector2d(
//                                                    -68,
//                                                    68
//                                            )),
//                                            turret.saveAngleAndCount(this),
//                                            drivetrain.updateStaticState(opModeIsActive()),
//                                            new SequentialAction(
//                                                    new ParallelAction(
//                                                            drivetrain.followPath(
//                                                                    path, 100, 2,
//                                                                    Math.toRadians(2.5), true
//                                                            ),
//                                                            new SequentialAction(
//                                                                    turret.cutoffTurret(),
//                                                                    new ParallelAction(
//                                                                            turret
//                                                                            .setTurretAngleTimed(
//                                                                                    180,
//                                                                                    1
//                                                                            ),
//                                                                            new SequentialAction(
//                                                                                    new
//                                                                                    SleepAction(
//                                                                                            0.3),
//                                                                                    robot
//                                                                                    .moveShoot()
//                                                                            )
//                                                                    ),
//                                                                    // SHOOT PRELOAD
//                                                                    transfer.ballDetectionTimed
//                                                                    (2),
//                                                                    // GATHER SECOND ROW
//                                                                    turret.resumeTurret()
//                                                            )
//                                                    ),
//                                                    new SequentialAction( // SHOOT SECOND ROW
//                                                                          shooter.hardStopOpen(),
//                                                                          drivetrain.followPath(
//                                                                                  secondToShoot,
//                                                                                  150,
//                                                                                  2,
//                                                                                  Math
//                                                                                  .toRadians(2.5),
//                                                                                  true
//                                                                          ),
//                                                                          robot.moveShoot()
//                                                    ),
//                                                    new ParallelAction( // GATHER GATE
//                                                                        shooter.hardStopClose(),
//                                                                        drivetrain
//                                                                        .followPathTimed(
//                                                                                gate,
//                                                                                150,
//                                                                                0.0,
//                                                                                Math.toRadians
//                                                                                (0.0),
//                                                                                true,
//                                                                                FOLLOW_PATH_TIME
//                                                                        ),
//                                                                        new SequentialAction(
//
//                                                                                new SleepAction
//                                                                                (1),
//                                                                                transfer
//                                                                                .ballDetectionTimed(
//                                                                                        2)
//                                                                                // GATHER
//                                                                                SECOND ROW
//                                                                        )
//                                                    ),
//                                                    new SequentialAction( // SHOOT GATE COLLECT
//                                                                          drivetrain.followPath(
//                                                                                  shoot,
//                                                                                  150,
//                                                                                  2,
//                                                                                  Math
//                                                                                  .toRadians(2.5),
//                                                                                  true
//                                                                          ),
//                                                                          shooter.hardStopOpen(),
//                                                                          robot.moveShoot()
//                                                    ),
//                                                    new ParallelAction( //GO TO THIRD ROW
//                                                                        drivetrain.followPath(
//                                                                                thirdRow,
//                                                                                150,
//                                                                                3,
//                                                                                Math
//                                                                                        .toRadians(
//                                                                                                2.5),
//                                                                                true
//                                                                        ),
//                                                                        transfer
//                                                                        .ballDetectionTimed(
//                                                                                2)
//                                                                        // GATHER THIRD ROW
//                                                    ),
//                                                    new SequentialAction( // SHOOT THIRD ROW
//                                                                          shooter.hardStopOpen(),
//                                                                          drivetrain.goToPose(
//                                                                                  Utils
//                                                                                  .makePoseVector(
//                                                                                          -8,
//                                                                                          14,
//                                                                                          90
//                                                                                  ),
//                                                                                  3,
//                                                                                  Math
//                                                                                  .toRadians(2.5),
//                                                                                  true
//                                                                          ),
//                                                                          new SleepAction(0.25),
//                                                                          // GO TO SHOOTING POS
//                                                                          robot.moveShoot()
//                                                                          // SHOOT THIRD ROW
//                                                    ),
//                                                    new ParallelAction( // GATHER GATE 2
//                                                                        shooter.hardStopClose(),
//                                                                        drivetrain
//                                                                        .followPathTimed(
//                                                                                gate,
//                                                                                150,
//                                                                                0.0,
//                                                                                Math.toRadians
//                                                                                (0.0),
//                                                                                true,
//                                                                                FOLLOW_PATH_TIME
//                                                                        ),
//                                                                        new SequentialAction(
//                                                                                new SleepAction
//                                                                                (1),
//                                                                                transfer
//                                                                                .ballDetectionTimed(
//                                                                                        2.5)
//                                                                                // GATHER
//                                                                                SECOND ROW
//                                                                        )
//                                                    ),
//                                                    new SequentialAction( // SHOOT GATE 2 COLLECT
//                                                                          drivetrain.followPath(
//                                                                                  shoot,
//                                                                                  150,
//                                                                                  2,
//                                                                                  Math
//                                                                                  .toRadians(2.5),
//                                                                                  true
//                                                                          ),
//                                                                          shooter.hardStopOpen(),
//                                                                          robot.moveShoot()
//                                                    ),
//                                                    new ParallelAction( //GO TO FIRST ROW
//                                                                        drivetrain.followPath(
//                                                                                firstRow,
//                                                                                150,
//                                                                                3,
//                                                                                Math
//                                                                                        .toRadians(
//                                                                                                2.5),
//                                                                                true
//                                                                        ),
//                                                                        transfer
//                                                                        .ballDetectionTimed(
//                                                                                2)
//                                                                        // GATHER FIRST ROW
//                                                    ),
//                                                    new SequentialAction( // SHOOT FIRST ROW
//                                                                          shooter.hardStopOpen(),
//                                                                          drivetrain.goToPose(
//                                                                                  Utils
//                                                                                  .makePoseVector(
//                                                                                          -8,
//                                                                                          14,
//                                                                                          90
//                                                                                  ),
//                                                                                  3,
//                                                                                  Math
//                                                                                  .toRadians(2.5),
//                                                                                  true
//                                                                          ),
//                                                                          // GO TO SHOOTING POS
//                                                                          robot.moveShoot()
//                                                                          // SHOOT FIRST ROW
//                                                    ),
//                                                    new ParallelAction( // GATHER GATE 3
//                                                                        shooter.hardStopClose(),
//                                                                        drivetrain
//                                                                        .followPathTimed(
//                                                                                gate,
//                                                                                150,
//                                                                                0.0,
//                                                                                Math.toRadians
//                                                                                (0.0),
//                                                                                true,
//                                                                                FOLLOW_PATH_TIME
//                                                                        ),
//                                                                        new SequentialAction(
//                                                                                new SleepAction
//                                                                                (1),
//                                                                                transfer
//                                                                                .ballDetectionTimed(
//                                                                                        2.5)
//                                                                                // GATHER
//                                                                                SECOND ROW
//                                                                        )
//                                                    ),
//                                                    new SequentialAction( // SHOOT GATE COLLECT 3
//                                                                          drivetrain.followPath(
//                                                                                  shootFinal,
//                                                                                  150,
//                                                                                  2,
//                                                                                  Math
//                                                                                  .toRadians(2.5),
//                                                                                  true
//                                                                          ),
//                                                                          shooter.hardStopOpen(),
//                                                                          robot.moveShoot()
//                                                    )
//                                            )
//                        )
//                )
//        );
//    }
//}
