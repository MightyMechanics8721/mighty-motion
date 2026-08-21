package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Measures top speed and coast distance, on one axis at a time.
 * <p>
 * Drive at a fixed power until the speed stops climbing, cut power, and read how far the robot
 * carried. Each run gives one (speed at cut, coast distance) pair; take a few at different powers
 * and fit them to the coefficients in {@link Drivetrain.StoppingDistanceParameters}. Only the TURN
 * axis was ever measurable here before, which is why the forward and strafe coefficients have no
 * provenance.
 * <p>
 * Peak speed on FORWARD is also what settles FF_CONSTANTS.kV. kV says full power should reach
 * (1 - kS) / kV rad/s at the wheel, so (1 - kS) / kV * wheelRadius in/s on the ground. If the
 * measured top speed disagrees, kV is fitted in the wrong units and every speed the controllers
 * talk about is scaled by that error.
 * <p>
 * Controls: A drives at power, B cuts power and measures the coast, X clears the run.
 */
@Config
@TeleOp(group = "a", name = "Tune Coast + Top Speed")
public class TuneAccel extends LinearOpMode {

    public enum Axis {
        FORWARD,
        STRAFE,
        TURN
    }

    public static Axis axis = Axis.FORWARD;
    /** Drive power for the run, [-1, 1]. Take readings at several values. */
    public static double power = 1.0;

    @Override
    public void runOpMode() {
        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        Drivetrain drivetrain = Drivetrain.getInstance();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        ElapsedTime coastTimer = new ElapsedTime();
        boolean coasting = false;
        double peakSpeed = 0;
        double speedAtCut = 0;
        double coastStartX = 0;
        double coastStartY = 0;
        double coastStartHeading = 0;
        double coastDistance = 0;
        double coastTime = 0;

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.localize();
            double speed = speedOn(axis, drivetrain);

            if (gamepad1.x) {
                coasting = false;
                peakSpeed = 0;
                speedAtCut = 0;
                coastDistance = 0;
                coastTime = 0;
                setAxisPower(drivetrain, axis, 0);
            } else if (gamepad1.a) {
                coasting = false;
                peakSpeed = Math.max(peakSpeed, Math.abs(speed));
                setAxisPower(drivetrain, axis, power);
            } else if (gamepad1.b) {
                if (!coasting) {
                    coasting = true;
                    speedAtCut = speed;
                    coastStartX = drivetrain.state.get(0, 0);
                    coastStartY = drivetrain.state.get(1, 0);
                    coastStartHeading = drivetrain.state.get(2, 0);
                    coastTimer.reset();
                }
                setAxisPower(drivetrain, axis, 0);
                coastDistance = axis == Axis.TURN
                        ? Utils.angleWrap(drivetrain.state.get(2, 0) - coastStartHeading)
                        : Utils.calculateDistance(
                                coastStartX, coastStartY,
                                drivetrain.state.get(0, 0), drivetrain.state.get(1, 0));
                coastTime = coastTimer.seconds();
            } else {
                setAxisPower(drivetrain, axis, 0);
            }

            telemetry.addData("axis", axis);
            telemetry.addData("power", power);
            telemetry.addData("A drive / B cut+measure / X clear", coasting ? "COASTING" : "");
            telemetry.addData(unit(axis, "speed now"), speed);
            telemetry.addData(unit(axis, "peak speed"), peakSpeed);
            telemetry.addLine("---- last coast ----");
            telemetry.addData(unit(axis, "speed at cut"), speedAtCut);
            telemetry.addData(axis == Axis.TURN ? "coast (rad)" : "coast (in)", coastDistance);
            telemetry.addData("coast time (s)", coastTime);
            if (axis == Axis.FORWARD) {
                telemetry.addLine("---- what kV predicts ----");
                double kS = Drivetrain.FF_CONSTANTS.lf.kS;
                double radius = Drivetrain.MECHANICAL_PARAMETERS.wheelRadius;
                telemetry.addData("top speed kV implies (in/s)",
                        (1.0 - kS) / Drivetrain.FF_CONSTANTS.lf.kV * radius);
                telemetry.addData("kV matching the peak just measured",
                        peakSpeed > 0 ? (1.0 - kS) / (peakSpeed / radius) : 0);
            }
            telemetry.update();
        }
    }

    /** Body frame speed on the axis under test. */
    private static double speedOn(Axis axis, Drivetrain drivetrain) {
        switch (axis) {
            case STRAFE:
                return drivetrain.state.get(4, 0);
            case TURN:
                return drivetrain.state.get(5, 0);
            case FORWARD:
            default:
                return drivetrain.state.get(3, 0);
        }
    }

    private static String unit(Axis axis, String label) {
        return label + (axis == Axis.TURN ? " (rad/s)" : " (in/s)");
    }

    /**
     * Powers the wheels to move along one axis only.
     * <p>
     * Signs follow {@link MecanumKinematicModel}: forward drives all four the same way, strafe
     * opposes the diagonals, turn opposes the sides.
     */
    private static void setAxisPower(Drivetrain drivetrain, Axis axis, double power) {
        double leftFront;
        double leftBack;
        double rightBack;
        double rightFront;
        switch (axis) {
            case STRAFE:
                leftFront = -power;
                leftBack = power;
                rightBack = -power;
                rightFront = power;
                break;
            case TURN:
                leftFront = -power;
                leftBack = -power;
                rightBack = power;
                rightFront = power;
                break;
            case FORWARD:
            default:
                leftFront = power;
                leftBack = power;
                rightBack = power;
                rightFront = power;
                break;
        }
        drivetrain.motorLeftFront.setPower(leftFront);
        drivetrain.motorLeftBack.setPower(leftBack);
        drivetrain.motorRightBack.setPower(rightBack);
        drivetrain.motorRightFront.setPower(rightFront);
    }
}
