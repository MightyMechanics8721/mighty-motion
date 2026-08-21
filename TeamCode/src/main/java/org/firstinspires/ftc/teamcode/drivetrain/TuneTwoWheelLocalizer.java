package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.hardware.Battery;

@Config
@Autonomous(name = "Tune 2 Wheel Localizer", group = "Autonomous")
public class TuneTwoWheelLocalizer extends LinearOpMode {
    Drivetrain drivetrain;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        drivetrain = Drivetrain.getInstance();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        ElapsedTime looptime = new ElapsedTime();

        telemetry.addLine("Push the robot straight forward at a heading of about 90 deg.");
        telemetry.addLine("long. vel should be positive and lat. vel near zero.");
        telemetry.addLine("If they come out swapped, flip TwoWheelOdometery.velocityIsFieldFrame.");
        telemetry.update();
        waitForStart();

        looptime.reset();

        while (opModeIsActive()) {
            // localize() draws into the packet, so it needs a fresh one each iteration.
            TelemetryPacket packet = new TelemetryPacket();
            drivetrain.setTelemetry(packet);

            drivetrain.localize();
            SimpleMatrix reported = drivetrain.twoWheelOdo.reportedVelocities();

            telemetry.addLine("Looptime [ms]: " + looptime.milliseconds());
            telemetry.addLine("X [in]: " + drivetrain.state.get(0, 0));
            telemetry.addLine("Y [in]: " + drivetrain.state.get(1, 0));
            telemetry.addLine("Theta [deg]: " + Math.toDegrees(drivetrain.state.get(2, 0)));
            telemetry.addLine("");
            telemetry.addLine("velocityIsFieldFrame: " + TwoWheelOdometery.velocityIsFieldFrame);
            telemetry.addLine("reported velX [in/s]: " + reported.get(0, 0));
            telemetry.addLine("reported velY [in/s]: " + reported.get(1, 0));
            telemetry.addLine("resolved long. vel [in/s]: " + drivetrain.state.get(3, 0));
            telemetry.addLine("resolved lat. vel [in/s]: " + drivetrain.state.get(4, 0));
            telemetry.addLine("angularVelocity [rad/s]: " + drivetrain.state.get(5, 0));
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);

            looptime.reset();
        }
    }
}