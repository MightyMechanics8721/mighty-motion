package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;
import org.firstinspires.ftc.teamcode.Testing.DistanceSensor;

import java.util.HashMap;
import java.util.Map;

@Config
@TeleOp(name = "TeleOp", group = "Competition")
public class DecodeTeleOp extends LinearOpMode {
    public static double targetVelocity = 2500; // (RPM)
    public static double SHOOTER_VELOCITY_NORMAL = 2500;
    public static double SHOOTER_VELOCITY_CLOSE = 2250;
    public static double SHOOTER_VELOCITY_FAR = 3500;
    Battery battery;
    Turret turret;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    //    DistanceSensor distanceSensor;
    Transfer transfer;
    Drivetrain drivetrain;

    FtcDashboard dashboard;

    private Map<String, Action> runningActions = new HashMap<>();

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        Battery.initialize(hardwareMap);
        Turret.initialize(hardwareMap);
        Indexer.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        //        DistanceSensor.initialize(hardwareMap);
        Transfer.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);

        // Hardware
        turret = Turret.getInstance();
        intake = Intake.getInstance();
        indexer = Indexer.getInstance();
        shooter = Shooter.getInstance();
        //        distanceSensor = DistanceSensor.getInstance();
        transfer = Transfer.getInstance();
        drivetrain = Drivetrain.getInstance();

        battery = Battery.getInstance();

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

            //             ----- INTAKE && INDEXER -----
            if (gamepad1.right_trigger > 0.1) {
                runningActions.put(
                        "transfer", transfer.runIntakeAndIndexer(1, 1));

            } else if (gamepad1.left_trigger > 0.1) {
                runningActions.put(
                        "transfer", transfer.runIntakeAndIndexer(-1, 0));
            } else if (gamepad1.left_bumper) {
                runningActions.put(
                        "transfer", transfer.runIntakeAndIndexer(0, -1));
            } else if (gamepad1.right_bumper) {
                runningActions.put(
                        "transfer", transfer.ballDetection());
            } else {
                runningActions.put(
                        "transfer", transfer.runIntakeAndIndexer(0, 0));
            }
            // ----- TURRET -----
            if (gamepad2.left_trigger > 0.05) {
                runningActions.put("turret", turret.autoAim(new Vector2d(-60, -60)));
            } else if (gamepad2.dpad_up) {
                runningActions.put("turret", turret.setTurretAngle(0));
            } else if (gamepad2.dpad_left) {
                runningActions.put("turret", turret.setTurretAngle(90));
            } else if (gamepad2.dpad_right) {
                runningActions.put("turret", turret.setTurretAngle(-90));
            } else {
                runningActions.put("turret", turret.setTurretAngle(0));
            }

            if (gamepad1.dpad_up) {
                runningActions.put("stopper", shooter.hardStopOpen());
            }
            if (gamepad1.dpad_down) {
                runningActions.put("stopper", shooter.hardStopClose());
            }

            // ----- SHOOTER -----
            if (gamepad2.right_trigger > 0.05) {
                runningActions.put("shooter", shooter.autoShoot());
                runningActions.put("stopper", shooter.hardStopOpen());
            } else if (gamepad2.left_bumper) { // ----- REVERSE -----
                runningActions.put(
                        "shooter", shooter.setShooterVelocityLoop(-SHOOTER_VELOCITY_NORMAL
                                                                          / 2 * 2 * Math.PI / 60)
                );
            } else if (gamepad2.square) { // ------ NORMAL ------
                runningActions.put(
                        "shooter", shooter.setShooterVelocityLoop(
                                SHOOTER_VELOCITY_NORMAL * 2 * Math.PI / 60)
                );
                runningActions.put("stopper", shooter.hardStopOpen());
            } else if (gamepad2.cross) { // ------ CLOSE ------
                runningActions.put(
                        "shooter",
                        shooter.setShooterVelocityLoop(SHOOTER_VELOCITY_CLOSE * 2 * Math.PI / 60)
                );
                runningActions.put("stopper", shooter.hardStopOpen());
            } else if (gamepad2.triangle) { // ------ FAR ------
                runningActions.put(
                        "shooter",
                        shooter.setShooterVelocityLoop(SHOOTER_VELOCITY_FAR * 2 * Math.PI / 60)
                );
                runningActions.put("stopper", shooter.hardStopOpen());
            } else {
                runningActions.put("shooter", shooter.setShooterVelocityLoop(0));
                runningActions.put("stopper", shooter.hardStopClose());
            }

            // ----- DISTANCE SENSOR -----


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
}
