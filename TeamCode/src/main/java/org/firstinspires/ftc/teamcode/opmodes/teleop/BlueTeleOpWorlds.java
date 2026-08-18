package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.util.Utils.calculateDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.opmodes.auto.BlueNearWorlds;

@Config
@TeleOp(name = "Blue TeleOp Worlds", group = "1123Competition")
public class BlueTeleOpWorlds extends LinearOpMode {
    public static double targetVelocity = 2500; // (RPM)
    public static double SHOOTER_VELOCITY_IDLE = 1500;
    public static double SHOOTER_VELOCITY_NORMAL = 1600;
    public static double SHOOTER_VELOCITY_CLOSE = 1300;
    public static double SHOOTER_VELOCITY_FAR = 3100;
    public static double robotLength = 14.25; //in
    public static double robotWidth = 16.75; //in
    public static double dist = 80;
    public static double autoShootMultiplier = 1.0;
    boolean rumbleStop;
    Battery battery;
    Turret turret;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    Transfer transfer;
    Drivetrain drivetrain;
    FtcDashboard dashboard;
    private double mult = 1;
    private double myCacheAngle = 0.0;
    private double myCacheCount = 0.0;

    private Map<String, Action> runningActions = new HashMap<>();

    @Override
    public void runOpMode() {
        double autoAimBias = 0.0;

        myCacheAngle = Turret.staticTheta;
        myCacheCount = Turret.staticThetaUpdateCounter;

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        Battery.initialize(hardwareMap);
        Turret.initialize(hardwareMap);
        Indexer.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        Transfer.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        // Hardware
        turret = Turret.getInstance();
        intake = Intake.getInstance();
        indexer = Indexer.getInstance();
        shooter = Shooter.getInstance();
        transfer = Transfer.getInstance();
        drivetrain = Drivetrain.getInstance();
        battery = Battery.getInstance();

        drivetrain.setTelemetry(packet);

        drivetrain.setInitialPose(
                BlueNearWorlds.staticRobotState.get(0, 0),
                BlueNearWorlds.staticRobotState.get(1, 0),
                Math.toDegrees(BlueNearWorlds.staticRobotState.get(2, 0))
        );
        turret.setInitialAngle(BlueNearWorlds.staticTurretAngle);
        packet.put("x", BlueNearWorlds.staticRobotState.get(0, 0));
        packet.put("y", BlueNearWorlds.staticRobotState.get(1, 0));
        packet.put("theta", Math.toDegrees(BlueNearWorlds.staticRobotState.get(2, 0)));
        packet.put("turret", BlueNearWorlds.staticTurretAngle);
        packet.put("x_real", drivetrain.state.get(0, 0));
        packet.put("y_real", drivetrain.state.get(1, 0));
        packet.put("theta_real", Math.toDegrees(drivetrain.state.get(2, 0)));
        packet.put("turret_real", turret.getAngle());
        dashboard.sendTelemetryPacket(packet);
        waitForStart();
        drivetrain.setInitialPose(
                BlueNearWorlds.staticRobotState.get(0, 0),
                BlueNearWorlds.staticRobotState.get(1, 0),
                Math.toDegrees(BlueNearWorlds.staticRobotState.get(2, 0))
        );
        turret.setInitialAngle(BlueNearWorlds.staticTurretAngle);

        while (opModeIsActive()) {
            packet.put("x", BlueNearWorlds.staticRobotState.get(0, 0));
            packet.put("y", BlueNearWorlds.staticRobotState.get(1, 0));
            packet.put("theta", Math.toDegrees(BlueNearWorlds.staticRobotState.get(2, 0)));
            packet.put("turret", BlueNearWorlds.staticTurretAngle);
            packet.put("x_real", drivetrain.state.get(0, 0));
            packet.put("y_real", drivetrain.state.get(1, 0));
            packet.put("theta_real", Math.toDegrees(drivetrain.state.get(2, 0)));
            packet.put("turret_real", turret.getAngle());
            packet.put("main loop turret angle (deg)", turret.getAngle());
            dashboard.sendTelemetryPacket(packet);
            //            TelemetryPacket packet = new TelemetryPacket();
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
            if (calculateDistance(
                    drivetrain.state.get(0, 0),
                    drivetrain.state.get(1, 0), -60,
                    -60
            ) > dist) {
                mult = 0.5;
            } else {
                mult = 1;
            }
            if (gamepad1.right_trigger > 0.1 || gamepad2.right_bumper) {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(mult, mult));
            } else if (gamepad1.left_trigger > 0.1 && gamepad1.left_bumper) {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(-mult, -mult));
            } else if (gamepad1.left_trigger > 0.1) {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(-mult, 0));
            } else if (gamepad1.left_bumper) {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(0, -mult));
            } else if (gamepad1.right_bumper) {
                runningActions.put(
                        "transfer", transfer.ballDetection());
            } else {
                runningActions.put(
                        "transfer", transfer.setIntakeIndexerPower(0, 0));
            }
            // ----- TURRET -----
            if (gamepad2.dpad_up) {
                SHOOTER_VELOCITY_FAR += 5;
                Shooter.constant += 5;
            } else if (gamepad2.dpad_down) {
                SHOOTER_VELOCITY_FAR -= 5;
                Shooter.constant -= 5;
            }

            if (gamepad2.dpad_left) {
                autoAimBias += 1;
            } else if (gamepad2.dpad_right) {
                autoAimBias -= 1;
            }

            if (gamepad2.left_trigger > 0.05) {
                runningActions.put("turret", turret.autoAim(new Vector2d(-70, -70), autoAimBias));
            } else {
                runningActions.put("turret", turret.setTurretAngle(Turret.bias));
            }

            if (gamepad1.dpad_up) {
                runningActions.put("stopper", shooter.hardStopOpen());
            }
            if (gamepad1.dpad_down) {
                runningActions.put("stopper", shooter.hardStopClose());
            }

            // ----- SHOOTER -----
            if (gamepad2.right_trigger > 0.05) {
                runningActions.put("shooter", shooter.autoShoot(-60, -60, autoShootMultiplier));
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

            if (gamepad1.dpad_left) {
                drivetrain.setInitialPose(72 - robotLength / 2, -72 + robotWidth / 2, 180);
            }
            if (gamepad1.dpad_right) {
                drivetrain.setInitialPose(72 - robotLength / 2, 72 - robotWidth / 2, 180);
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

            // Dashboard telemetry
            dashboard.getTelemetry().addData("Shooter Vel", shooter.getVelocity());
            dashboard.getTelemetry().addData("Battery Voltage", battery.getVoltage());
            dashboard.getTelemetry().update();
        }

    }
}
