package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "Full Robot Test")
public class TeleOpTest extends LinearOpMode {

    public static double targetVelocity = 2500; // (RPM)
    public static double SHOOTER_VELOCITY = 2500;

    Battery battery;
    FtcDashboard dashboard;

    // Hardware
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;

    private Map<String, Action> runningActions = new HashMap<>();

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        battery = new Battery(hardwareMap);
        intake = new Intake(hardwareMap, battery);
        indexer = new Indexer(hardwareMap, battery);
        shooter = new Shooter(hardwareMap, battery);

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            // ----- INTAKE -----
            if (gamepad1.right_trigger > 0.2) {
                intake.setIntakePower(1.0);
            } else if (gamepad1.left_trigger > 0.2) {
                intake.setIntakePower(-1.0);
            } else {
                intake.setIntakePower(0.0);
            }

            // ----- INDEXER -----
            if (gamepad1.b) {
                indexer.setIndexerPower(1.0);
            } else {
                indexer.setIndexerPower(0.0);
            }

            // ----- SHOOTER -----
            if (gamepad1.right_bumper) {
                shooter.setShooterVelocity(SHOOTER_VELOCITY * 2 * Math.PI / 60.0);
            } else {
                shooter.setShooterVelocity(0);
            }

            // ----- RUN ACTIONS -----
            HashMap<String, Action> newActions = new HashMap<>();
            for (Map.Entry<String, Action> entry : runningActions.entrySet()) {
                entry.getValue().preview(packet.fieldOverlay());
                if (entry.getValue().run(packet)) {
                    newActions.put(entry.getKey(), entry.getValue());
                }
            }
            runningActions = newActions;

            dashboard.sendTelemetryPacket(packet);

            // Dashboard telemetry
            dashboard.getTelemetry().addData("Shooter Vel", shooter.getVelocity());
            dashboard.getTelemetry().addData("Battery Voltage", battery.getVoltage());
            dashboard.getTelemetry().update();
        }
    }
}
