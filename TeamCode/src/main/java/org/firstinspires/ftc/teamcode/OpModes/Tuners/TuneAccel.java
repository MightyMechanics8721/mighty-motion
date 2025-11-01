package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

@Config
@Autonomous(group = "a", name = "tuneaccel")
public class TuneAccel extends LinearOpMode {
    public static double power = 1.0;
    public double maxVelocity;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        double stopPos = 10;
        double presentPos = 100;
        double lastPos;
        boolean isStopped = false;
        Battery battery = new Battery(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, battery);
        ElapsedTime timer = new ElapsedTime();

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Current velocity (in/s)", 0);
        telemetry.addData("Current velocity (Max - Current)", maxVelocity);
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            drivetrain.localize();
            //max velocity
            if (maxVelocity < drivetrain.state.get(4, 0))
                maxVelocity = drivetrain.state.get(4, 0);
            if (gamepad1.triangle && !isStopped) { //Using isStopped as a total cut to controls
                drivetrain.motorLeftFront.setPower(power);
                drivetrain.motorRightFront.setPower(power);
                drivetrain.motorRightBack.setPower(power);
                drivetrain.motorLeftBack.setPower(power);
            } else if (gamepad1.square && !isStopped) {
                drivetrain.motorLeftFront.setPower(0);
                drivetrain.motorRightFront.setPower(0);
                drivetrain.motorRightBack.setPower(0);
                drivetrain.motorLeftBack.setPower(0);
                isStopped = true;
                timer.reset();
                stopPos = drivetrain.state.get(0, 0);
//                presentPos = stopPos;
            } else if (isStopped) {

                presentPos = drivetrain.state.get(0, 0);

                telemetry.addData("Stopping distance", presentPos - stopPos);
                telemetry.addData("stopPos", stopPos);
                telemetry.addData("PresentPos", presentPos);
                telemetry.addData("Stopping time", timer.seconds());
//                if (Math.abs((lastPos - presentPos)) < 0.01) {
//                    isStopped = false;
//                }
            }
            presentPos = drivetrain.state.get(0, 0);
//            telemetry.addData("Stopping distance 2", presentPos - stopPos);
//            telemetry.addData("Current velocity (Max - Current)", maxVelocity);
            telemetry.addData("Current velocity (in/s)", drivetrain.state.get(4, 0));
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.update();
        }
    }
}
