package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Filters.LowPassFilterParameters;

@Config
@TeleOp(name = "MotorControlExample")
public class FilteringTest extends LinearOpMode {

    // Tunable from FTC Dashboard
    //    public static double smoothingFactor = 0.5;
    public static LowPassFilterParameters filterParams = new LowPassFilterParameters(0.5);

    // Declare motor + dashboard
    DcMotorEx motor;
    FtcDashboard dashboard;

    // Low-pass filter + params
    LowPassFilter velocityFilter;
    //    LowPassFilterParameters filterParams;

    @Override
    public void runOpMode() {
        // Map motor from config (must match name in Robot Configuration)
        motor = hardwareMap.get(DcMotorEx.class, "motorName");

        // Init dashboard
        dashboard = FtcDashboard.getInstance();

        // Init parameters + filter
        velocityFilter = new LowPassFilter(filterParams);

        waitForStart();

        while (opModeIsActive()) {
            // Start motor on A button
            //            if (gamepad1.a) {
            //                motor.setPower(0.5);
            //            }

            // Stop motor on B button
            //            if (gamepad1.b) {
            //                motor.setPower(0.0);
            //            }

            // Raw velocity from encoder
            double rawVelocity = motor.getVelocity();

            // Update filter (uses smoothingFactor inside params)
            double filteredVelocity = velocityFilter.update(rawVelocity);

            // Keep filter params synced with Dashboard slider
            //            filterParams.smoothingFactor = smoothingFactor;

            // Send both raw + filtered velocities to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Raw Velocity", rawVelocity);
            packet.put("Filtered Velocity", filteredVelocity);
            packet.put("Smoothing Factor", filterParams.smoothingFactor);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
