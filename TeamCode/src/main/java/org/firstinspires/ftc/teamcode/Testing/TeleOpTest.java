package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;
import org.firstinspires.ftc.teamcode.Prism.Prism;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;

@Config
@TeleOp(name = "Full Robot Test")
public class TeleOpTest extends LinearOpMode {
    public static double targetVelocity = 2500; // (RPM)
    public static double SHOOTER_VELOCITY_NORMAL = 2500;
    public static double SHOOTER_VELOCITY_CLOSE = 2250;
    public static double SHOOTER_VELOCITY_FAR = 3500;

    Battery battery;
    FtcDashboard dashboard;
    Prism prism;
    // LED code
    private final Runnable backgroundProcessingRunnable = () ->
    {
        while (opModeIsActive() || opModeInInit()) {
            prism.runCheck();
            sleep(1);
        }
    };
    // Hardware
    private Turret turret;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
    private Drivetrain drivetrain;
    private Map<String, Action> runningActions = new HashMap<>();
    // LEDs
    private ExecutorService backgroundExecutor = null;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        battery = new Battery(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap, battery);
        indexer = new Indexer(hardwareMap, battery);
        shooter = new Shooter(hardwareMap, battery);
        drivetrain = new Drivetrain(hardwareMap, battery);
        prism = new Prism(hardwareMap);
        this.initBackgroundThreads();

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            // ----- DRIVETRAIN -----
            runningActions.put(
                    "manualDrive", drivetrain.manualControl(
                            -gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    )
            );
            // ----- INTAKE && INDEXER -----
            if (gamepad1.right_trigger > 0.1) {
                runningActions.put("intake", intake.setIntakePower(-1));
                runningActions.put("indexer", indexer.setIndexerPower(1));
            } else if (gamepad1.left_trigger > 0.1) {
                runningActions.put("intake", intake.setIntakePower(1));
            } else if (gamepad1.left_bumper) {
                runningActions.put("indexer", indexer.setIndexerPower(-1));
            } else {
                runningActions.put("intake", intake.setIntakePower(0.0));
                runningActions.put("indexer", indexer.setIndexerPower(0.0));
            }
            // ----- TURRET -----
            //            if (gamepad2.left_trigger > 0.15) {
            //                runningActions.put("turret", turret.autoAim(new Vector2d(-60, -60)));
            //            }
            if (gamepad2.dpad_up) {
                runningActions.put("turret", turret.setTurretAngle(0));
            } else if (gamepad2.dpad_left) {
                runningActions.put("turret", turret.setTurretAngle(90));
            } else if (gamepad2.dpad_right) {
                runningActions.put("turret", turret.setTurretAngle(-90));
            }

            // ----- SHOOTER -----
            if (gamepad2.right_trigger > 0.05) {
                runningActions.put("shooter", shooter.autoShoot());
            } else if (gamepad2.left_trigger > 0.05) { // ----- REVERSE -----
                runningActions.put(
                        "shooter", shooter.setShooterVelocityLoop(-SHOOTER_VELOCITY_NORMAL
                                                                          / 2 * 2 * Math.PI / 60)
                );
            } else if (gamepad2.square) { // ------ NORMAL ------
                runningActions.put(
                        "shooter", shooter.setShooterVelocityLoop(
                                SHOOTER_VELOCITY_NORMAL * 2 * Math.PI / 60)
                );
            } else if (gamepad2.cross) { // ------ CLOSE ------
                runningActions.put(
                        "shooter",
                        shooter.setShooterVelocityLoop(SHOOTER_VELOCITY_CLOSE * 2 * Math.PI / 60)
                );

            } else if (gamepad2.triangle) { // ------ FAR ------
                runningActions.put(
                        "shooter",
                        shooter.setShooterVelocityLoop(SHOOTER_VELOCITY_FAR * 2 * Math.PI / 60)
                );
            } else {
                runningActions.put("shooter", shooter.setShooterVelocityLoop(0));
            }

            // ----- INDEXER -----
            //
            //            if (gamepad1.cross) {
            //                runningActions.put("indexer", indexer.setIndexerPower(1.0));
            //            } else {
            //                runningActions.put("indexer", indexer.setIndexerPower(0.0));
            //            }

            //            if (gamepad2.right_trigger > 0.05) {
            //                runningActions.put("shooter", shooter.setShooterVelocity
            //                (SHOOTER_VELOCITY * ((double) gamepad1.right_trigger)));
            //                // right_trigger -- float -- 0-255
            //            }

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

    public void initBackgroundThreads() {
        this.backgroundExecutor = ThreadPool.newSingleThreadExecutor("Background Thread");
        this.backgroundExecutor.submit(backgroundProcessingRunnable);
    }

    public void stopBackgroundThreads() {
        if (backgroundExecutor != null) {
            backgroundExecutor.shutdownNow();
            backgroundExecutor = null;
        }
    }
}
