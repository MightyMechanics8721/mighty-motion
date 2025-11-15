package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

import java.util.HashMap;
import java.util.Map;


@Config
@TeleOp
public class TeleopWithActions extends OpMode {
    Drivetrain drivetrain = null;
    FtcDashboard dashboard;
    Battery battery;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private Map<String, Action> runningActions = new HashMap<>();

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        battery = new Battery(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, battery);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        runningActions.put(
                "manualDrive",
                drivetrain.manualControl(
                        -gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                )
        );
        HashMap<String, Action> newActions = new HashMap<>();
        for (Map.Entry<String, Action> entry : runningActions.entrySet()) {
            entry.getValue().preview(packet.fieldOverlay());
            if (entry.getValue().run(packet)) {
                newActions.put(entry.getKey(), entry.getValue());
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
