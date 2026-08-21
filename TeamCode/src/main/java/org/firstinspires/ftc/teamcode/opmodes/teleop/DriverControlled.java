package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.util.Utils.calculateDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.HashMap;
import java.util.Map;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.control.ShooterModel;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Battery;
import org.firstinspires.ftc.teamcode.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.storage.StaticVariables;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * The driver-controlled routine, shared by both alliances.
 * <p>
 * The two sides of the field differ only in the sign of Y, so the alliance is the single thing a
 * subclass supplies; every goal and starting coordinate comes from {@link FieldConstants} mirrored
 * through it. Whether an Autonomous ran is not a separate OpMode either: the pose and turret angle
 * are taken from {@link StaticVariables} when one stored them, and fall back to the near starting
 * pose when none did.
 *
 * @see BlueTeleOp
 * @see RedTeleOp
 */
@Config
public abstract class DriverControlled extends LinearOpMode {

    public static double SHOOTER_VELOCITY_IDLE = 1500;
    public static double SHOOTER_VELOCITY_NORMAL = 1600;
    public static double SHOOTER_VELOCITY_CLOSE = 1300;
    public static double SHOOTER_VELOCITY_FAR = 3100;

    public static double robotLength = 14.25; // (in)
    public static double robotWidth = 16.75; // (in)

    /** Beyond this range from the goal the intake and indexer are run at half power. (in) */
    public static double dist = 80;
    public static double autoShootMultiplier = 1.0;

    private final Map<String, Action> runningActions = new HashMap<>();

    private Battery battery;
    private Turret turret;
    private Shooter shooter;
    private Transfer transfer;
    private Drivetrain drivetrain;
    private FtcDashboard dashboard;

    private boolean rumbleStop;
    private double mult = 1;

    /** The side of the field this OpMode plays from. */
    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        Alliance alliance = alliance();
        Vector2d turretGoal = FieldConstants.turretAimPoint(alliance);
        Vector2d shooterGoal = FieldConstants.shooterRangePoint(alliance);

        double autoAimBias = 0.0;

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Battery.initialize(hardwareMap);
        Turret.initialize(hardwareMap);
        Indexer.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        Transfer.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);

        turret = Turret.getInstance();
        shooter = Shooter.getInstance();
        transfer = Transfer.getInstance();
        drivetrain = Drivetrain.getInstance();
        battery = Battery.getInstance();

        SimpleMatrix startPose =
                StaticVariables.robotStateOr(FieldConstants.nearStartPose(alliance));
        double startTurretAngle = StaticVariables.turretAngle();

        TelemetryPacket initPacket = new TelemetryPacket();
        drivetrain.setTelemetry(initPacket);
        applyStartingState(startPose, startTurretAngle);

        // Shout if the Autonomous that just ran was the other alliance: the turret would spend the
        // match tracking the opponent's goal, and nothing else would look wrong.
        telemetry.addData("ALLIANCE", alliance);
        if (StaticVariables.hasAlliance() && StaticVariables.alliance() != alliance) {
            telemetry.addLine("*** WARNING: Autonomous ran as " + StaticVariables.alliance()
                    + ", this is " + alliance + " ***");
            telemetry.addLine("*** Wrong TeleOp, or the pose carried over is not yours ***");
        }
        telemetry.addData("Pose source",
                StaticVariables.hasRobotState() ? "carried over from Autonomous"
                        : "no Autonomous ran, using near start");
        telemetry.addData("Start x (in)", startPose.get(0, 0));
        telemetry.addData("Start y (in)", startPose.get(1, 0));
        telemetry.addData("Start heading (deg)", Math.toDegrees(startPose.get(2, 0)));
        telemetry.addData("Start turret (deg)", startTurretAngle);
        telemetry.addData("Turret goal", turretGoal.x + ", " + turretGoal.y);
        telemetry.update();

        initPacket.put("start x (in)", startPose.get(0, 0));
        initPacket.put("start y (in)", startPose.get(1, 0));
        initPacket.put("start heading (deg)", Math.toDegrees(startPose.get(2, 0)));
        initPacket.put("start turret (deg)", startTurretAngle);
        dashboard.sendTelemetryPacket(initPacket);

        waitForStart();

        // The odometry can drift during the init pause, so seed it again on START.
        applyStartingState(startPose, startTurretAngle);

        while (opModeIsActive()) {
            // A TelemetryPacket accumulates every canvas op written to it and never clears, so it
            // cannot outlive one iteration.
            TelemetryPacket packet = new TelemetryPacket();
            drivetrain.setTelemetry(packet);

            packet.put("x pos (in)", drivetrain.state.get(0, 0));
            packet.put("y pos (in)", drivetrain.state.get(1, 0));
            packet.put("heading (deg)", Math.toDegrees(drivetrain.state.get(2, 0)));
            packet.put("turret angle (deg)", turret.getAngle());

            // Overwritten below by any branch that wants the stopper open.
            runningActions.put("stopper", shooter.hardStopClose());

            // ----- DRIVETRAIN -----
            runningActions.put(
                    "manualDrive", drivetrain.manualControl(
                            -gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    )
            );

            // ----- INTAKE && INDEXER -----
            mult = calculateDistance(
                    drivetrain.state.get(0, 0),
                    drivetrain.state.get(1, 0),
                    shooterGoal.x,
                    shooterGoal.y
            ) > dist ? 0.5 : 1;

            if (gamepad1.right_trigger > 0.1 || gamepad2.right_bumper) {
                runningActions.put("transfer", transfer.setIntakeIndexerPower(mult, mult));
            } else if (gamepad1.left_trigger > 0.1 && gamepad1.left_bumper) {
                runningActions.put("transfer", transfer.setIntakeIndexerPower(-mult, -mult));
            } else if (gamepad1.left_trigger > 0.1) {
                runningActions.put("transfer", transfer.setIntakeIndexerPower(-mult, 0));
            } else if (gamepad1.left_bumper) {
                runningActions.put("transfer", transfer.setIntakeIndexerPower(0, -mult));
            } else if (gamepad1.right_bumper) {
                runningActions.put("transfer", transfer.ballDetection());
            } else {
                runningActions.put("transfer", transfer.setIntakeIndexerPower(0, 0));
            }

            // ----- TURRET -----
            if (gamepad2.dpad_up) {
                SHOOTER_VELOCITY_FAR += 5;
                ShooterModel.constant += 5;
            } else if (gamepad2.dpad_down) {
                SHOOTER_VELOCITY_FAR -= 5;
                ShooterModel.constant -= 5;
            }

            if (gamepad2.dpad_left) {
                autoAimBias += 1;
            } else if (gamepad2.dpad_right) {
                autoAimBias -= 1;
            }

            if (gamepad2.left_trigger > 0.05) {
                runningActions.put("turret", turret.autoAim(turretGoal, autoAimBias));
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
                runningActions.put("shooter",
                        shooter.autoShoot(shooterGoal.x, shooterGoal.y, autoShootMultiplier));
                runningActions.put("stopper", shooter.hardStopOpen());
            } else if (gamepad2.left_bumper) { // ----- REVERSE -----
                runningActions.put("stopper", shooter.hardStopOpen());
                runningActions.put("shooter", shooterAt(-SHOOTER_VELOCITY_NORMAL / 2));
            } else if (gamepad2.square) { // ------ NORMAL ------
                runningActions.put("shooter", shooterAt(SHOOTER_VELOCITY_NORMAL));
                runningActions.put("stopper", shooter.hardStopOpen());
            } else if (gamepad2.cross) { // ------ CLOSE ------
                runningActions.put("shooter", shooterAt(SHOOTER_VELOCITY_CLOSE));
                runningActions.put("stopper", shooter.hardStopOpen());
            } else if (gamepad2.triangle) { // ------ FAR ------
                runningActions.put("shooter", shooterAt(SHOOTER_VELOCITY_FAR));
                runningActions.put("stopper", shooter.hardStopOpen());
            } else {
                runningActions.put("shooter", shooterAt(SHOOTER_VELOCITY_IDLE));
            }

            // Re-seed the pose against a known corner of the field.
            if (gamepad1.dpad_left) {
                drivetrain.setInitialPose(
                        72 - robotLength / 2,
                        alliance.mirrorY(-72 + robotWidth / 2),
                        180
                );
            }
            if (gamepad1.dpad_right) {
                drivetrain.setInitialPose(
                        72 - robotLength / 2,
                        alliance.mirrorY(72 - robotWidth / 2),
                        180
                );
            }

            if (transfer.ballCount == 3) {
                if (!rumbleStop) {
                    gamepad1.rumble(1000);
                    rumbleStop = true;
                }
            } else {
                rumbleStop = false;
            }

            // ----- RUN ACTIONS -----
            runActions(packet);

            dashboard.sendTelemetryPacket(packet);

            dashboard.getTelemetry().addData("Alliance", alliance);
            dashboard.getTelemetry().addData("Shooter Vel", shooter.getVelocity());
            dashboard.getTelemetry().addData("Battery Voltage", battery.getVoltage());
            dashboard.getTelemetry().update();
        }
    }

    private Action shooterAt(double rpm) {
        return shooter.setShooterVelocityLoop(Utils.rpmToRadPerSec(rpm));
    }

    private void applyStartingState(SimpleMatrix startPose, double turretAngleDeg) {
        drivetrain.setInitialPose(
                startPose.get(0, 0),
                startPose.get(1, 0),
                Math.toDegrees(startPose.get(2, 0))
        );
        turret.setInitialAngle(turretAngleDeg);
    }

    /** Runs each queued action once, keeping the ones that report themselves unfinished. */
    private void runActions(TelemetryPacket packet) {
        Map<String, Action> survivors = new HashMap<>();
        for (Map.Entry<String, Action> entry : runningActions.entrySet()) {
            entry.getValue().preview(packet.fieldOverlay());
            if (entry.getValue().run(packet)) {
                survivors.put(entry.getKey(), entry.getValue());
            }
        }
        runningActions.clear();
        runningActions.putAll(survivors);
    }
}
