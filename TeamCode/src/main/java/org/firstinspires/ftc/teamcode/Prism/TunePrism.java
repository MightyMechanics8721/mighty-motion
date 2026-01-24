package org.firstinspires.ftc.teamcode.Prism;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;

@Autonomous(name = "Test Prism")
public class TunePrism extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Prism prism = new Prism(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            Actions.runBlocking(prism.ballCheck(true));
        }
    }
}
