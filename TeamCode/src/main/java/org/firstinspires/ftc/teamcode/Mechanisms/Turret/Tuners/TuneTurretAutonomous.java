package org.firstinspires.ftc.teamcode.Mechanisms.Turret.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;

@Config
@Autonomous(name = "TuneTurret", group = "1")
public class TuneTurretAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(turret.turretSpin(90)); // spin to 90°
        }
    }
}
