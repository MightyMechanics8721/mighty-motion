package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

public class TuneAccel extends LinearOpMode {
    @Override
    public void runOpMode() {
        double stopPos = 0;
        boolean isStopped = false;
        Battery battery = new Battery(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, battery);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                drivetrain.motorLeftFront.setPower(1);
                drivetrain.motorRightFront.setPower(1);
                drivetrain.motorRightBack.setPower(1);
                drivetrain.motorLeftBack.setPower(1);
            } else if (gamepad1.b) {
                drivetrain.motorLeftFront.setPower(0);
                drivetrain.motorRightFront.setPower(0);
                drivetrain.motorRightBack.setPower(0);
                drivetrain.motorLeftBack.setPower(0);
                isStopped = true;
                stopPos = drivetrain.state.get(0, 0);
            } else if (isStopped) {

            }
        }
    }
}
