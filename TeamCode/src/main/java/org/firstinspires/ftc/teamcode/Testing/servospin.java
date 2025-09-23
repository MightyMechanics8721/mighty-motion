package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "Test", group = "1")
public class servospin extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo continousServoRight = hardwareMap.get(CRServo.class, "servodot");
        CRServo continousServoRightTwo = hardwareMap.get(CRServo.class, "servodotty");
        DcMotorEx spinEncoder = hardwareMap.get(DcMotorEx.class, "encoding");
        double pos = spinEncoder.getCurrentPosition();
        double encoderLastPosition = pos;
        waitForStart();

        while (opModeIsActive()) {

            pos = spinEncoder.getCurrentPosition();
            if (Math.abs(pos - encoderLastPosition) > 1) {
                if (pos > encoderLastPosition) {
                    continousServoRight.setPower(0.5);
                    continousServoRightTwo.setPower(0.5);
                    encoderLastPosition = pos;
                } else if (pos < encoderLastPosition) {
                    continousServoRight.setPower(-0.5);
                    continousServoRightTwo.setPower(-0.5);
                    encoderLastPosition = pos;
                }
            } else {
                continousServoRight.setPower(0);
                continousServoRightTwo.setPower(0);
                encoderLastPosition = pos;

            }

        }
    }


}


