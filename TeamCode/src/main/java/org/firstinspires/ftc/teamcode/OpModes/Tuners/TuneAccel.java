package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

public class TuneAccel extends LinearOpMode {
    @Override
    public void runOpMode() {
        double stopPos = 0;
        double presentPos = 0;
        double lastPos;
        boolean isStopped = false;
        Battery battery = new Battery(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, battery);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && !isStopped) { //Using isStopped as a total cut to controls
                drivetrain.motorLeftFront.setPower(1);
                drivetrain.motorRightFront.setPower(1);
                drivetrain.motorRightBack.setPower(1);
                drivetrain.motorLeftBack.setPower(1);
            } else if (gamepad1.b && !isStopped) {
                drivetrain.motorLeftFront.setPower(0);
                drivetrain.motorRightFront.setPower(0);
                drivetrain.motorRightBack.setPower(0);
                drivetrain.motorLeftBack.setPower(0);
                isStopped = true;
                timer.reset();
                stopPos = drivetrain.state.get(0, 0);
                presentPos = stopPos;
            } else if (isStopped) {
                lastPos = presentPos;
                presentPos = drivetrain.state.get(0, 0);
                telemetry.addData("Stopping distance: ", stopPos - presentPos);
                telemetry.addData("Stopping time: ", timer.seconds());
                if (Math.abs((lastPos - presentPos)) < 0.01) {
                    isStopped = false;
                }
            }
        }
    }
}
