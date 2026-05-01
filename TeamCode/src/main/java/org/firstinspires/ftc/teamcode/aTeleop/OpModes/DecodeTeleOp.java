package org.firstinspires.ftc.teamcode.aTeleop.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Filters.LowPassFilterParameters;

import java.util.HashMap;
import java.util.Map;

@Config
@TeleOp(name = "TESTING TeleOp", group = "Competition")
public class DecodeTeleOp extends LinearOpMode {
    public static double targetVelocity = 2500; // (RPM)
    public static double SHOOTER_VELOCITY_IDLE = 1500;
    public static double SHOOTER_VELOCITY_NORMAL = 1600;
    public static double SHOOTER_VELOCITY_CLOSE = 1300;
    public static double SHOOTER_VELOCITY_FAR = 2100;
    public static double autoShootMultiplier = 1.0;
    public static double getAutoShootDistanceSubtraction = 12.0;
    boolean rumbleStop;
    Battery battery;
    Turret turret;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    //    DistanceSensor distanceSensor;
    Transfer transfer;
    Drivetrain drivetrain;

    FtcDashboard dashboard;
    ElapsedTime timer;
    LowPassFilter filter;
    double lastTime;

    private Map<String, Action> runningActions = new HashMap<>();

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        double autoAimBias = 0.0;
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
        filter = new LowPassFilter(new LowPassFilterParameters(0.99));

        battery = Battery.getInstance();
        timer = new ElapsedTime();
        lastTime = 0;
        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            //If we have another stopper action already, this won't fire
            runningActions.put("stopper", shooter.hardStopClose());
            // ----- DRIVETRAIN -----
            runningActions.put(
                    "manualDrive", drivetrain.manualControl(
                            -gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    )
            );

            //             ----- INTAKE && INDEXER -----
            if (gamepad1.right_trigger > 0.1 || gamepad2.right_bumper) {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(1, 1));

            } else if (gamepad1.left_trigger > 0.1) {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(-1, 0));
            } else if (gamepad1.left_bumper) {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(0, -1));
            } else if (gamepad1.right_bumper) {
                runningActions.put(
                        "transfer", transfer.ballDetection());
            } else {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(0, 0));
            }
            // ----- TURRET -----
            if (gamepad2.left_trigger > 0.05) {
                runningActions.put("turret", turret.autoAim(new Vector2d(-60, -60), autoAimBias));
            }

            if (gamepad2.dpad_up) {
                SHOOTER_VELOCITY_FAR += 5;
            } else if (gamepad2.dpad_down) {
                SHOOTER_VELOCITY_FAR -= 5;
            }

            if (gamepad2.dpad_left) {
                autoAimBias += 0.5;
            } else if (gamepad2.dpad_right) {
                autoAimBias -= 0.5;
            }

            if (gamepad1.dpad_up) {
                runningActions.put("stopper", shooter.hardStopOpen());
            }
            if (gamepad1.dpad_down) {
                runningActions.put("stopper", shooter.hardStopClose());
            }

            // ----- SHOOTER -----
            if (gamepad2.right_trigger > 0.05) {
                runningActions.put("shooter",
                                   shooter.autoShoot(
                                           -70 + getAutoShootDistanceSubtraction,
                                           -70 + getAutoShootDistanceSubtraction,
                                           autoShootMultiplier
                                   )
                );
                runningActions.put("stopper", shooter.hardStopOpen());
            } else if (gamepad2.left_bumper) { // ----- REVERSE -----
                runningActions.put("stopper", shooter.hardStopOpen());
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
                runningActions.put(
                        "shooter",
                        shooter.setShooterVelocityLoop(SHOOTER_VELOCITY_IDLE * 2 * Math.PI / 60)
                );
            }

            if (transfer.ballCount == 3) {
                if (!rumbleStop) {
                    gamepad1.rumble(1000);
                    gamepad1.rumble(1000);
                    rumbleStop = true;
                }
            } else {
                rumbleStop = false;
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
            double time = timer.seconds();
            // Dashboard telemetry
            dashboard.getTelemetry()
                     .addData("Shooter Vel", (shooter.getVelocity() * 60) / (2 * Math.PI));
            dashboard.getTelemetry().addData(
                    "Intake Current",
                    intake.getIntakeMotor().getCurrent(CurrentUnit.MILLIAMPS).toString()
            );
            dashboard.getTelemetry().addData("Looptime", time - lastTime);
            lastTime = time;
            dashboard.getTelemetry().addData("Battery Voltage", battery.getVoltage());
            dashboard.getTelemetry().update();
        }

    }
}
