package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "A", name = "MotorPidTest")
public class MotorPidTest extends LinearOpMode {
    public static double desiredVelocity = 2700;

    @Override
    public void runOpMode() throws InterruptedException {
        PidMotor pidMotor = new PidMotor(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(pidMotor.motorSpin(desiredVelocity));
        }
    }
}
