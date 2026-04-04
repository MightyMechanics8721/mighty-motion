package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

@Config
@Autonomous(name = "Tune 2 Wheel Localizer", group = "Autonomous")
public class TuneTwoWheelLocalizer extends LinearOpMode {
    Drivetrain drivetrain;
    // Use FTCDashboard
    FtcDashboard dashboard;
    TelemetryPacket packet;

    Battery battery;

    @Override
    public void runOpMode() {
        // Set dashboard
        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        drivetrain = Drivetrain.getInstance();
        packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
        telemetry = dashboard.getTelemetry();

        ElapsedTime looptime = new ElapsedTime();


        telemetry.addLine("Looptime [ms]: " + 0);
        telemetry.addLine("X [in]: " + 0);
        telemetry.addLine("Y [in]: " + 0);
        telemetry.addLine("Theta [deg]: " + 0);
        telemetry.addLine("XVelocity [in/s]: " + 0);
        telemetry.addLine("YVelocity [in/s]: " + 0);
        telemetry.addLine("angularVelocity [rad/s]: " + 0);
        telemetry.update();
        waitForStart();

        looptime.reset();

        while (opModeIsActive()) {
            drivetrain.localize();
            telemetry.addLine("Looptime [ms]: " + looptime.milliseconds());
            telemetry.addLine("X [in]: " + (drivetrain.state.get(0, 0)));
            telemetry.addLine("Y [in]: " + (drivetrain.state.get(1, 0)));
            telemetry.addLine("Theta [deg]: " + Math.toDegrees(drivetrain.state.get(2, 0)));
            telemetry.addLine("XVelocity [in/s]: " + drivetrain.state.get(3, 0));
            telemetry.addLine("YVelocity [in/s]: " + drivetrain.state.get(4, 0));
            telemetry.addLine("angularVelocity [rad/s]: " + drivetrain.state.get(5, 0));
            telemetry.update();


            looptime.reset();
        }
    }
}