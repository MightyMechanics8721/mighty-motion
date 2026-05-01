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
@Autonomous(name = "Blue FAR  Worlds", group = "TEST")
public class BlueFarSimple extends LinearOpMode {

    public static double staticTurretAngle;
    public static SimpleMatrix staticRobotState;

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

        double[][] sHTohP = {{60, -24}, {45, -36}, {45, -48}, {45, -60}};
        double[][] shTohPContinued = {{45, -60}, {45, -63}, {63, -63}};
        double[][] hPtoShoot = {{60, -60}, {60, -48}, {60, -24}};
        double[][] sHToTR = {{60, -24}, {48, -20}, {36, -30}, {36, -36}, {36, -57}};
        double[][] TRTosH = {{36, -57}, {50, -30}, {60, -24}};
        double[][] sHToLB = {{60, -24}, {45, -36}, {45, -48}, {55, -60}};
        double[][] LBTosH = {{55, -60}, {60, -24}};
        Path shootToHumanPlayer = new Path(sHTohP, Math.toRadians(-180), false, false);
        Path shootToHumanPlayerContinued = new Path(
                shTohPContinued,
                Math.toRadians(-180),
                false,
                false
        );
        Path shootToLingeringBalls = new Path(sHToLB, Math.toRadians(-180), false, false);
        Path shootToThirdRow = new Path(sHToTR, Math.toRadians(-90), false, true);

        Path thirdRowToShoot = new Path(TRTosH, Math.toRadians(-180), true, false);
        Path lingeringBallsToShoot = new Path(LBTosH, Math.toRadians(-180), true, false);
        Path humanPlayerToShoot = new Path(hPtoShoot, Math.toRadians(-90), true, false);


        waitForStart();

        looptime.reset();
        Turret.staticTheta = 0;
        drivetrain.setInitialPose(64, -24, -180);
        turret.setInitialAngle(0);
        //Alex Ko bless this code
        Actions.runBlocking(
                new ParallelAction( //main loop
                                    shooter.autonomousVelocityInfinite(2100),
                                    turret.autoAimTimed(
                                            new Vector2d(
                                                    -68,
                                                    -68
                                            ), 20
                                    ),
                                    updateTurretAngle(),
                                    updateRobotState(),
                                    new SequentialAction(
                                            new SleepAction(1),
                                            robot.shootFAR(),
                                            drivetrain.goToPoseTimed(
                                                    Utils.makePoseVector(
                                                            64, -48
                                                            , -180
                                                    ), 1, Math.toRadians(1), true, 5
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