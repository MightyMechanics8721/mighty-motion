package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
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
    double AAAx = 0;
    double AAAy = 0;
    double dy = 0;
    double dx = 0;
    double deltaHeading = 0;


    double absoluteHeading = 0;

    @Override
    public void runOpMode() {
        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        Drivetrain drivetrain = Drivetrain.getInstance();
        ElapsedTime timer = new ElapsedTime();

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("Current velocity (in/s)", drivetrain.state.get(5, 0));
        telemetry.addData("Max Velocity", maxVelocity);


        waitForStart();

        double relXMovement = 0.0;

        double previousHeading = 0.0;
        double prevXEncoder = 0.0;

        double relYMovement = 0.0;
        double prevYEncoder = 0.0;

        while (opModeIsActive()) {
            drivetrain.localize();
            telemetry.addData(
                    "BRUH",
                    drivetrain.twoWheelOdo.odo.getHeading(UnnormalizedAngleUnit.RADIANS)
            );
            deltaHeading = drivetrain.state.get(2, 0) - previousHeading;

            if (deltaHeading > 180) {
                deltaHeading -= 360;
            } else if (deltaHeading < -180) {
                deltaHeading += 360;
            }
            absoluteHeading += deltaHeading;

            telemetry.addData("deltaheading", deltaHeading * 180 / Math.PI);
            telemetry.addData("previousheading", previousHeading * 180 / Math.PI);

            previousHeading = drivetrain.state.get(2, 0); // moght break for x.y

            //            dEncoder = drivetrain.twoWheelOdo.odo.getEncoderX() - prevEncoder;
            dx =
                    (drivetrain.twoWheelOdo.odo.getEncoderX() - prevXEncoder) / 2000.0 * 2 *
                            Math.PI
                            * 0.63 //
                            // convert
                            // to in
                            + drivetrain.twoWheelOdo.odo.getXOffset(DistanceUnit.INCH) * (
                            deltaHeading);

            relXMovement += dx;


            //            previousHeading = drivetrain.state.get(2, 0);
            prevXEncoder = drivetrain.twoWheelOdo.odo.getEncoderX();

            dy =
                    (drivetrain.twoWheelOdo.odo.getEncoderY() - prevYEncoder) / 2000.0 * 2 *
                            Math.PI
                            * 0.63 //
                            // convert
                            // to in
                            + drivetrain.twoWheelOdo.odo.getYOffset(DistanceUnit.INCH) * (
                            deltaHeading);

            relYMovement += dy;

            telemetry.addData("offet", drivetrain.twoWheelOdo.odo.getYOffset(DistanceUnit.INCH));
            telemetry.addData("encoder Y (ticks)", drivetrain.twoWheelOdo.odo.getEncoderY());
            telemetry.addData(
                    "encoder Y (in)",
                    drivetrain.twoWheelOdo.odo.getEncoderY() / 2000.0 * 2 * Math.PI
                            * 0.63
            );

            //
            //            previousHeading = absoluteHeading; // moght break for x.y
            prevYEncoder = drivetrain.twoWheelOdo.odo.getEncoderY();

            // Allow toggling reset mode
            if (gamepad1.circle) {
                isReset = !isReset;
                sleep(600); // prevent rapid toggling
            }

            if (isReset) {
                stopPos = 0;
                presentPos = 0;
                lastPos = 0;
                isStopped = false;
                maxVelocity = 0;

                if (gamepad1.cross) {
                    AAAx = gamepad1.left_stick_y;
                    drivetrain.motorLeftFront.setPower(AAAx);
                    drivetrain.motorLeftBack.setPower(AAAx);
                    drivetrain.motorRightFront.setPower(AAAx);
                    drivetrain.motorRightBack.setPower(AAAx);
                } else {
                    AAAx = gamepad1.left_stick_x;
                    drivetrain.motorLeftFront.setPower(-AAAx);
                    drivetrain.motorLeftBack.setPower(AAAx);
                    drivetrain.motorRightFront.setPower(-AAAx);
                    drivetrain.motorRightBack.setPower(AAAx);
                }
            } else {

                // Drive controls
                if (gamepad1.triangle && !isStopped) {
                    drivetrain.motorLeftFront.setPower(-power);
                    drivetrain.motorRightFront.setPower(power);
                    drivetrain.motorRightBack.setPower(power);
                    drivetrain.motorLeftBack.setPower(-power);


                } else if (gamepad1.square && !isStopped) {
                    stopPos = drivetrain.twoWheelOdo.odo.getHeading(UnnormalizedAngleUnit
                                                                            .RADIANS);
                    drivetrain.motorLeftFront.setPower(0);
                    drivetrain.motorRightFront.setPower(0);
                    drivetrain.motorRightBack.setPower(0);
                    drivetrain.motorLeftBack.setPower(0);
                    isStopped = true;
                    timer.reset();

                } else if (isStopped) {
                    presentPos
                            = drivetrain.twoWheelOdo.odo.getHeading(UnnormalizedAngleUnit
                                                                            .RADIANS);

                    telemetry.addData("Stopping distance", presentPos - stopPos);
                    telemetry.addData("Stopping time (s)", timer.seconds());
                }
            }

            if (maxVelocity < drivetrain.state.get(5, 0)) {
                maxVelocity = drivetrain.state.get(5, 0);
            }

            telemetry.addData("Current velocity (in/s)", drivetrain.state.get(5, 0));
            telemetry.addData("Absolute Heading", absoluteHeading);
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addData("isReset", isReset);
            telemetry.update();
        }
    }
}
