package org.firstinspires.ftc.teamcode.Testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public void initialize(HardwareMap hardwareMap) {
        instance = new Robot(hardwareMap);
    }

    public Robot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Robot not initialized!");
        }
        return instance;
    }

    /**
     * ----- Wrap in parallel action with path -----
     *
     * @return
     */
    public Action MoveShoot() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                new SequentialAction(
                        shooter.autoShootMovingTimed(3),
                        shooter.hardStopOpen(),
                        transfer.setIntakeIndexerPower(1, 1),
                        new SleepAction(0.5),
                        transfer.setIntakeIndexerPower(0, 0));
                return false;
            }
        };
    }
}