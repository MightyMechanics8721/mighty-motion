package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    double turn = 0;

    @Override
    public void runOpMode() {
        Battery battery = new Battery(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, battery);
        ElapsedTime timer = new ElapsedTime();

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("Current velocity (in/s)", drivetrain.state.get(4, 0));
        telemetry.addData("Max Velocity", maxVelocity);


        waitForStart();

        double relXMovement = 0.0;

        double previousHeading = 0.0;
        double prevXEncoder = 0.0;

        while (opModeIsActive()) {
            drivetrain.localize();
            //            dEncoder = drivetrain.twoWheelOdo.odo.getEncoderX() - prevEncoder;
            double dx =
                    (drivetrain.twoWheelOdo.odo.getEncoderX() - prevXEncoder) / 2000.0 * 2 * Math.PI
                            * 0.63 //
                            // convert
                            // to in
                            + drivetrain.twoWheelOdo.odo.getXOffset(DistanceUnit.INCH) * (
                            drivetrain.state.get(
                                    2
                                    , 0
                            )
                                    - previousHeading);

            relXMovement += dx;

            previousHeading = drivetrain.state.get(2, 0);
            prevXEncoder = drivetrain.twoWheelOdo.odo.getEncoderX();

            // Allow toggling reset mode
            if (gamepad1.circle) {
                isReset = !isReset;
                sleep(1000); // prevent rapid toggling
            }

            if (isReset) {
                stopPos = 0;
                presentPos = 0;
                lastPos = 0;
                isStopped = false;
                maxVelocity = 0;
                if (gamepad1.cross) {
                    turn = gamepad1.left_stick_x;
                    drivetrain.motorLeftFront.setPower(turn);
                    drivetrain.motorLeftBack.setPower(turn);
                    drivetrain.motorRightFront.setPower(turn);
                    drivetrain.motorRightBack.setPower(turn);
                    telemetry.addData("Turn", turn);
                } else {
                    turn = gamepad1.left_stick_y;
                    drivetrain.motorLeftFront.setPower(turn);
                    drivetrain.motorLeftBack.setPower(turn);
                    drivetrain.motorRightFront.setPower(turn);
                    drivetrain.motorRightBack.setPower(turn);
                    telemetry.addData("Turn", turn);
                }
            } else {

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
                    stopPos = relXMovement;
                } else if (isStopped) {
                    presentPos = relXMovement;

                    telemetry.addData("Stopping distance", presentPos - stopPos);
                    telemetry.addData("Stopping time (s)", timer.seconds());
                }
            }

            if (maxVelocity < drivetrain.state.get(3, 0)) {
                maxVelocity = drivetrain.state.get(3, 0);
            }

            telemetry.addData("Current velocity (in/s)", drivetrain.state.get(3, 0));
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addData("isReset", isReset);
            telemetry.update();
        }
    }
}
