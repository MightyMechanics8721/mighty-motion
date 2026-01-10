package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;

@TeleOp(name = "Turret Manual Test3")
public class TurretManualTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo turretLeft = hardwareMap.get(CRServo.class, "servodotLeft");
        CRServo turretRight = hardwareMap.get(CRServo.class, "servodotRight");
        Encoder turretEncoder = new Encoder(hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "lfm")); // <-- use Encoder wrapper

        waitForStart();

        while (opModeIsActive()) {

            if (Math.abs(gamepad1.right_stick_x) > 0.01) {
                turretLeft.setPower(gamepad1.right_stick_x);
                turretRight.setPower(gamepad1.right_stick_x);
            } else {
                turretLeft.setPower(0.00);
                turretRight.setPower(0.00);
            }

            telemetry.addData("Encoder position", turretEncoder.getCurrentPosition() );
        }
    }
}
