package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

@Config
@TeleOp(group = "a", name = "tuneaccel")
public class TuneAccel extends LinearOpMode {
    public static double power = 1.0;
    public double maxVelocity = 0;
    boolean isReset = true;
    FtcDashboard dashboard;
    double stopPos = 0;
    double presentPos = 0;
    double lastPos;
    boolean isStopped = false;

    @Override
    public void runOpMode() {
        Battery battery = new Battery(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, battery);
        ElapsedTime timer = new ElapsedTime();

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.localize();

            // Allow toggling reset mode
            if (gamepad1.circle) {
                isReset = !isReset;
                sleep(100); // prevent rapid toggling
            }

            if (isReset) {
                stopPos = 0;
                presentPos = 0;
                lastPos = 0;
                isStopped = false;
                maxVelocity = 0;
            } else {
                // Measure max velocity
                if (maxVelocity < drivetrain.state.get(4, 0))
                    maxVelocity = drivetrain.state.get(4, 0);

                // Drive controls
                if (gamepad1.triangle && !isStopped) {
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
                } else if (isStopped) {
                    presentPos = drivetrain.state.get(0, 0);

                    telemetry.addData("Stopping distance", presentPos - stopPos);
                    telemetry.addData("Stopping time (s)", timer.seconds());
                }
            }

            telemetry.addData("Current velocity (in/s)", drivetrain.state.get(4, 0));
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addData("isReset", isReset);
            telemetry.update();
        }
    }
}
