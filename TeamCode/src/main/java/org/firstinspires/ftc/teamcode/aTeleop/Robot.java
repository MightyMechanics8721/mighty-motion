package org.firstinspires.ftc.teamcode.aTeleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class Robot {
    private static Robot instance;
    Battery battery;
    Turret turret;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    Transfer transfer;
    Drivetrain drivetrain;
    FtcDashboard dashboard;

    private Robot(HardwareMap hardwareMap) {
        Battery.initialize(hardwareMap);
        Turret.initialize(hardwareMap);
        Indexer.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        Transfer.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        battery = Battery.getInstance();
        turret = Turret.getInstance();
        intake = Intake.getInstance();
        indexer = Indexer.getInstance();
        shooter = Shooter.getInstance();
        transfer = Transfer.getInstance();
        drivetrain = Drivetrain.getInstance();
        dashboard = FtcDashboard.getInstance();
    }

    public static Robot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Robot not initialized!");
        }
        return instance;
    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Robot(hardwareMap);
    }

    /**
     * ----- Wrap in parallel action with path -----
     */
    public SequentialAction moveShoot(double shootTime) {
        return new SequentialAction(
                transfer.setIntakeIndexerPower(1, 1),
                new SleepAction(shootTime),
                new ParallelAction(
                        transfer.setIntakeIndexerPower(0, 0),
                        shooter.hardStopClose()
                )
        );
    }

    /**
     * ----- Wrap in parallel action with path -----
     */
    public SequentialAction moveShoot() {
        return new SequentialAction(
                transfer.setIntakeIndexerPower(1, 1),
                new SleepAction(0.5),
                new ParallelAction(
                        transfer.setIntakeIndexerPower(0, 0),
                        shooter.hardStopClose()
                )
        );
    }

    public SequentialAction shoot(Path path, double maxSpeed, double distanceThreshold, double angleThreshold, double pathTime, double shootTime) {
        return new SequentialAction(
                shooter.hardStopClose(),
                drivetrain.followPathTimed(
                        path,
                        maxSpeed,
                        distanceThreshold,
                        Math.toRadians(angleThreshold),
                        true, pathTime
                ),
                new SleepAction(0.05),
                shooter.hardStopOpen(),
                new SleepAction(0.1),
                moveShoot(shootTime)
        );
    }

    public SequentialAction shootFAR(Path path, double maxSpeed, double distanceThreshold, double angleThreshold, double pathTime, double shootTime) {
        return new SequentialAction(
                shooter.hardStopClose(),
                drivetrain.followPathTimed(
                        path,
                        maxSpeed,
                        distanceThreshold,
                        Math.toRadians(angleThreshold),
                        true, pathTime
                ),
                new SleepAction(0.05),
                shooter.hardStopOpen(),
                new SleepAction(0.1),
                moveShootFAR()
        );
    }


    public ParallelAction shootIntake(Path path, double maxSpeed, double distanceThreshold, double angleThreshold, double pathTime, double extraTime, double shootTime) {
        return new ParallelAction(
                transfer.ballDetectionTimed(extraTime),
                new SequentialAction(
                        shooter.hardStopClose(),
                        drivetrain.followPathTimed(
                                path,
                                maxSpeed,
                                distanceThreshold,
                                Math.toRadians(angleThreshold),
                                true, pathTime
                        ),
                        shooter.hardStopOpen(),
                        new SleepAction(0.1),
                        moveShoot(shootTime)
                )
        );
    }

    /**
     * @param path              PATH
     * @param maxSpeed          MAXSPEED IN/S
     * @param distanceThreshold IN (radius)
     * @param angleThreshold    DEGREE
     * @param pathTime          TIMER
     * @return
     */
    public ParallelAction gatherRow(Path path, double maxSpeed, double distanceThreshold, double angleThreshold, double pathTime) {
        return new ParallelAction(
                shooter.hardStopClose(),
                drivetrain.followPathTimed(
                        path,
                        maxSpeed,
                        distanceThreshold,
                        Math
                                .toRadians(
                                        angleThreshold),
                        true, pathTime
                ),
                transfer.ballDetectionTimed(
                        pathTime)
        );
    }

    public ParallelAction gatherGate(Path path, double maxSpeed, double distanceThreshold,
                                     double angleThreshold, double xGateBackup,
                                     double yGateBackup, double thetaGateBackup, double pathTime, double backUpTime) {
        return new ParallelAction(
                shooter.hardStopClose(),
                new SequentialAction(
                        drivetrain
                                .followPathTimed(
                                        path,
                                        maxSpeed,
                                        0.0,
                                        Math.toRadians
                                                (0.0),
                                        true,
                                        pathTime),
                        new ParallelAction(
                                drivetrain.goToPoseTimed(
                                        Utils
                                                .makePoseVector(
                                                        xGateBackup,
                                                        yGateBackup,
                                                        thetaGateBackup),
                                        distanceThreshold,
                                        Math.toRadians(
                                                angleThreshold),
                                        true, backUpTime
                                ),
                                transfer.ballDetectionTimed(
                                        backUpTime))
                )
        );
    }

    /**
     * ----- Wrap in parallel action with path -----
     */
    public SequentialAction moveShootFAR() {
        return new SequentialAction(
                transfer.setIntakeIndexerPower(0.3, 0.3),
                new SleepAction(1),
                new ParallelAction(
                        transfer.setIntakeIndexerPower(0, 0),
                        shooter.hardStopClose()
                )
        );
    }

    public SequentialAction gatherLingeringBalls(Path toBalls, Path toShoot, double maxSpeed, double pathTime, double waitTime) {
        return new SequentialAction(
                drivetrain.followPathTimed(
                        toBalls,
                        150,
                        1.5,
                        Math
                                .toRadians(
                                        2.0),
                        true, pathTime),
                transfer.ballDetectionTimed(waitTime),
                drivetrain.followPathTimed(
                        toShoot,
                        150,
                        1.5,
                        Math
                                .toRadians(
                                        2.0),
                        true, pathTime
                ),
                new SleepAction(0.3),
                moveShootFAR()
        );

    }

    public Action shootAtPose(
            SimpleMatrix desiredPose,
            double distanceThreshold,
            double angleThreshold,
            double velocity
    ) {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (drivetrain.inStoppingZone(desiredPose, distanceThreshold, angleThreshold)) {
                    Actions.runBlocking(new ParallelAction(
                            indexer.setIndexerPower(1),
                            intake.setIntakePower(-1),
                            shooter.setShooterVelocityTimed(
                                    2350 * 2 * Math.PI / 60,
                                    0.5
                            )
                    ));
                    return true;
                }
                return false;
            }
        };

    }
}