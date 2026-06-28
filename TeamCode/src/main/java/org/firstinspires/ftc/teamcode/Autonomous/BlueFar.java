package org.firstinspires.ftc.teamcode.Autonomous;

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
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;
import org.firstinspires.ftc.teamcode.aTeleop.StaticVariables;;

@Config
@Autonomous(name = "Blue FAR Autonomous  3/7", group = "TEST")
public class BlueFar extends LinearOpMode {
    public static double ROW_TRANSFER_TIME = 2;
    public static double staticTurretAngle;
    public static SimpleMatrix staticRobotState;
    public static double velocity = 2500;
    public double SHOOT_TIME = 0.4;
    public double ALL_TIME = 4;

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
        drivetrain.setInitialPose(64, -24, 180);
        dashboard.sendTelemetryPacket(packet);

        double[][] sHTohP = {{60, -24}, {50, -36}, {45, -48}, {50, -63}, {63, -63}};
        double[][] shthP2 = {{60, -24}, {45, -36}, {45, -48}, {46, -63}};
        double[][] shthP2c = {{46, -63}, {64, -64}};
        double[][] hPtoShoot = {{60, -60}, {60, -48}, {60, -24}};
        double[][] sHToTR = {{60, -24}, {36, -15}, {36, -62}};
        double[][] TRTosH = {{36, -62}, {50, -30}, {60, -24}};
        double[][] sHToLB = {{60, -24}, {45, -36}, {45, -48}, {55, -60}};
        double[][] LBTosH = {{55, -60}, {60, -24}};

        Path shootToHumanPlayer = new Path(shthP2, Math.toRadians(0), false, true);
        Path shootToHumanPlayerC = new Path(shthP2c, Math.toRadians(-90), false, false);
        Path shootToLingeringBalls = new Path(sHToLB, Math.toRadians(-180), false, true);
        Path shootToThirdRow = new Path(sHToTR, Math.toRadians(-90), false, true);

        Path thirdRowToShoot = new Path(TRTosH, Math.toRadians(-180), true, false);
        Path lingeringBallsToShoot = new Path(LBTosH, Math.toRadians(-180), true, false);
        Path humanPlayerToShoot = new Path(hPtoShoot, Math.toRadians(-90), true, false);


        waitForStart();

        looptime.reset();
        Turret.staticTheta = 0;
        drivetrain.setInitialPose(64, -24, 180);
        turret.setInitialAngle(0);
        //Alex Ko bless this code
        Actions.runBlocking(
                new ParallelAction( //main loop
                                    shooter.autonomousVelocityInfinite(velocity),
                                    turret.autoAimInfinite(new Vector2d(
                                            -68,
                                            -70
                                    )),
                                    shooter.hardStopClose(),
                                    updateTurretAngle(),
                                    updateRobotState(),
                                    new SequentialAction(
                                            new SleepAction(1.5),
                                            robot.moveShootFAR(),
                                            // ----- SHOOT PRELOAD -----


                                            // ----- INTAKE THIRD ROW -----
                                            robot.gatherRow(
                                                    shootToThirdRow,
                                                    150,
                                                    1.5,
                                                    2.0,
                                                    ROW_TRANSFER_TIME
                                            ),

                                            // ----- SHOOT THIRD ROW -----
                                            robot.shootFAR(
                                                    thirdRowToShoot,
                                                    150,
                                                    1.5,
                                                    2.0,
                                                    4
                                            ),

                                            //  ----- INTAKE HUMAN PLAYER -----
                                            drivetrain.followPathTimed(
                                                    shootToHumanPlayer,
                                                    150,
                                                    1.5,
                                                    2.0,
                                                    true,
                                                    2
                                            ),

                                            robot.gatherRow(
                                                    shootToHumanPlayerC,
                                                    150,
                                                    1.5,
                                                    2.0,
                                                    3
                                            ),
                                            // ----- SHOOT HUMAN PLAYER -----
                                            robot.shootFAR(
                                                    humanPlayerToShoot,
                                                    150,
                                                    1.5,
                                                    2.0,
                                                    4
                                            ),
                                            // ----- GO TO HUMAN PLAYER (COLLECT LINGERING BALLS
                                            // FROM GATE) -----
                                            robot.gatherLingeringBalls(
                                                    shootToLingeringBalls,
                                                    lingeringBallsToShoot,
                                                    150,
                                                    15,
                                                    15
                                            )
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
                    BlueNearWorlds.staticTurretAngle = turretAngle;
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
                    BlueNearWorlds.staticRobotState = robotState;
                }
                return true;
            }
        };
    }
}