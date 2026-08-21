package org.firstinspires.ftc.teamcode.opmodes.tuning;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.teamcode.hardware.Battery;

@Config
@TeleOp(name = "DrivetrainTest", group = "Testing")
public class TestDrivetrain extends OpMode {

    public static double slowMultiplier = 0.25;
    FtcDashboard dashboard;
    ElapsedTime timer = new ElapsedTime();
    Battery battery;
    Drivetrain drivetrain;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private Map<String, Action> runningActions = new HashMap<>();

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        battery = Battery.getInstance();
        drivetrain = Drivetrain.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        // A packet accumulates every canvas op written to it, so it cannot outlive one iteration.
        TelemetryPacket packet = new TelemetryPacket();
        drivetrain.manualControl(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        ).run(packet);

        dash.sendTelemetryPacket(packet);
    }
}
