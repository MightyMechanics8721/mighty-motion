package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;

@Config
@Autonomous(name = "IndexAndShoot", group = "4")
public class TestAutoIndexAndShoot extends LinearOpMode {
    HardwareMap hardwareMap;
    Battery battery;
    DigitalChannel beamBreak;
    DigitalChannel beamBreak3;
    DigitalChannel beamBreak2;
    DigitalChannel beamBreak4;

    @Override
    public void runOpMode() {

        Indexer indexer = new Indexer(hardwareMap, battery);
        Shooter shooter = new Shooter(hardwareMap, battery);
        Intake intake = new Intake(hardwareMap, battery);
        Turret turret = new Turret(hardwareMap);

        beamBreak = hardwareMap.get(DigitalChannel.class, "beam1");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        beamBreak.setState(true);
        beamBreak3 = hardwareMap.get(DigitalChannel.class, "beam3");
        beamBreak3.setMode(DigitalChannel.Mode.OUTPUT);

        beamBreak2 = hardwareMap.get(DigitalChannel.class, "beam2");
        beamBreak2.setMode(DigitalChannel.Mode.INPUT);
        beamBreak2.setState(true);
        beamBreak4 = hardwareMap.get(DigitalChannel.class, "beam4");
        beamBreak4.setMode(DigitalChannel.Mode.OUTPUT);


        //init beambreaks

        waitForStart();
        while (opModeIsActive()) {

            intake.setIntakePower(0.1);

            boolean b1 = beamBreak.getState();
            boolean b2 = beamBreak2.getState();

            if (!b2) {
                indexer.setIndexerPower(0);
            } else if (!b1) {
                indexer.setIndexerPower(0.3);
            }

            if (!b1 && !b2) {
                indexer.setIndexerPower(0.3);
                shooter.setShooterVelocityLoop(2500);
            }
        }

    }
}
