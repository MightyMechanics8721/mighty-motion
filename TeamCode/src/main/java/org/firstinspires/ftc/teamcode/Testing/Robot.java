package org.firstinspires.ftc.teamcode.Testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
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
    public SequentialAction moveShoot() {
        return new SequentialAction(
                transfer.setIntakeIndexerPower(1, 1),
                new SleepAction(0.6),
                new ParallelAction(
                        transfer.setIntakeIndexerPower(0, 0),
                        shooter.hardStopClose()
                )
        );
    }

    /**
     * ----- Wrap in parallel action with path -----
     */
    public SequentialAction moveShootFAR() {
        return new SequentialAction(
                transfer.setIntakeIndexerPower(0.3, 0.3),
                new SleepAction(0.6),
                new ParallelAction(
                        transfer.setIntakeIndexerPower(0, 0),
                        shooter.hardStopClose()
                )
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