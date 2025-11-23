package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    @Override
    public void runOpMode() {

        Indexer indexer = new Indexer(hardwareMap, battery);
        Shooter shooter = new Shooter(hardwareMap, battery);
        Intake intake = new Intake(hardwareMap, battery);
        Turret turret = new Turret(hardwareMap);

        //init beambreaks

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
