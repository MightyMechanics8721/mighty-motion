package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "TuneSenseColor", group = "1")
public class TuneSenseColor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SenseColor colorSensor = new SenseColor(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(colorSensor.ColorDetect()); // spin to 90°
        }
    }
}

