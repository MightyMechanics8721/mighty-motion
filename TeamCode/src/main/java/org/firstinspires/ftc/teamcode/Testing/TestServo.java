package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "TestServo", group = "2")
public class TestServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo portZero = hardwareMap.get(Servo.class, "servodot");
        Servo portOne = hardwareMap.get(Servo.class, "servodotty");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                portOne.setPosition(1);
                portZero.setPosition(1);
            } else if (gamepad1.circle) {
                portOne.setPosition(0);
                portZero.setPosition(0);
            }


        }
    }


}


