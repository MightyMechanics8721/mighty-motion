package org.firstinspires.ftc.teamcode.opmodes.auto;

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

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.Path;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.storage.StaticVariables;
import org.firstinspires.ftc.teamcode.util.Utils;

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

    /**
     * Saves the end-of-auton pose and turret angle. Safe to call while the OpMode is being torn
     * down; a hardware read that fails leaves the last good value in place.
     */
    public static void saveFinalState() {
        try {
            StaticVariables.saveRobotState(Drivetrain.getInstance().state);
        } catch (RuntimeException ignored) {
        }
        try {
            StaticVariables.saveTurretAngle(Turret.getInstance().getAngle());
        } catch (RuntimeException ignored) {
        }
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

    public SequentialAction shoot(
            Path path,
            double maxSpeed,
            double distanceThreshold,
            double angleThreshold,
            double pathTime,
            double shootTime
    ) {
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

    public SequentialAction shootFAR(
            Path path,
            double maxSpeed,
            double distanceThreshold,
            double angleThreshold,
            double pathTime
    ) {
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
    public SequentialAction shootFARFast(
            Path path,
            double maxSpeed,
            double distanceThreshold,
            double angleThreshold,
            double pathTime
    ) {
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
                moveShootFARFast()
        );
    }

    public SequentialAction shootFAR(

    ) {
        return new SequentialAction(
                shooter.hardStopOpen(),
                new SleepAction(0.1),
                moveShootFAR()
        );
    }

    public ParallelAction shootIntake(
            Path path,
            double maxSpeed,
            double distanceThreshold,
            double angleThreshold,
            double pathTime,
            double extraTime,
            double shootTime
    ) {
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
     * @param path PATH
     * @param maxSpeed MAXSPEED IN/S
     * @param distanceThreshold IN (radius)
     * @param angleThreshold DEGREE
     * @param pathTime TIMER
     *
     * @return
     */
    public ParallelAction gatherRow(
            Path path,
            double maxSpeed,
            double distanceThreshold,
            double angleThreshold,
            double pathTime
    ) {
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

    public ParallelAction gatherGate(
            Path path, double maxSpeed, double distanceThreshold,
            double angleThreshold, double xGateBackup,
            double yGateBackup, double thetaGateBackup, double pathTime, double backUpTime
    ) {
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
                                        pathTime
                                ),
                        new ParallelAction(
                                drivetrain.goToPoseTimed(
                                        Utils
                                                .makePoseVector(
                                                        xGateBackup,
                                                        yGateBackup,
                                                        thetaGateBackup
                                                ),
                                        distanceThreshold,
                                        Math.toRadians(
                                                angleThreshold),
                                        true, backUpTime
                                ),
                                transfer.ballDetectionTimed(
                                        backUpTime)
                        )
                )
        );
    }

    public ParallelAction gatherGateNoBackup(
            Path path, double maxSpeed, double pathTime,
            double ballTime
    ) {
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
                                        pathTime
                                ),
                        transfer.ballDetectionTimed(
                                ballTime)
                )
        );
    }

    /**
     * ----- Wrap in parallel action with path -----
     */
    public SequentialAction moveShootFAR() {
        return new SequentialAction(
                shooter.hardStopOpen(),
                transfer.setIntakeIndexerPower(0.5, 0.4),

                new SleepAction(2),
                new ParallelAction(
                        transfer.setIntakeIndexerPower(0, 0),
                        shooter.hardStopClose()

                )
        );
    }

    public SequentialAction moveShootFARFast() {
        return new SequentialAction(
                shooter.hardStopOpen(),
                transfer.setIntakeIndexerPower(0.5, 0.4),

                new SleepAction(0.5),
                new ParallelAction(
                        transfer.setIntakeIndexerPower(0, 0),
                        shooter.hardStopClose()

                )
        );
    }

    public SequentialAction gatherLingeringBalls(
            Path toBalls,
            Path toShoot,
            double maxSpeed,
            double pathTime,
            double waitTime
    ) {
        return new SequentialAction(
                drivetrain.followPathTimed(
                        toBalls,
                        150,
                        1.5,
                        Math
                                .toRadians(
                                        2.0),
                        true, pathTime
                ),
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
                                    Utils.rpmToRadPerSec(2350),
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