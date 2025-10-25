package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "Tune", name = "TestOuttake")
public class TestOuttake extends LinearOpMode {
    public static double desiredVelocity = 3000;


    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(shooter.motorSpin(desiredVelocity));

        }
    }
}
