package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

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
import org.firstinspires.ftc.teamcode.util.Timed;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Every far-side autonomous, in one OpMode.
 * <p>
 * The far routines are not one plan mirrored, the way the near ones are: each works the field
 * differently and only shares waypoints. They are gathered here so the alliance, the routine and
 * the start delay are all one menu choice, and so the shared routes live in {@link FarPaths}
 * rather than in four near-copies of the same file.
 * <p>
 * A routine can now be run on the alliance it was never run on. The routes mirror, but the timings
 * were tuned on one side only, so treat the other side as untested.
 *
 * @see AutoMenu
 * @see FarStrategy
 */
@Config
@Autonomous(name = "Far Auto", group = "1Comp")
public class FarAuto extends LinearOpMode {

    public static Alliance defaultAlliance = Alliance.BLUE;
    public static FarStrategy defaultStrategy = FarStrategy.HUMAN_PLAYER_CYCLE;

    public static double MAX_SPEED = 150;
    public static double velocity = 2700; // (rev/min)
    /** Trips to the human player in the HUMAN_PLAYER_CYCLE routine. */
    public static int humanPlayerCycles = 6;

    /** Start pose, written blue-side. */
    public static double startXBlue = 64;
    public static double startYBlue = -24;
    public static double startHeadingBlueDeg = -180;

    /** Where SIMPLE_SHOOT and MOVE_ONLY finish, written blue-side. */
    public static double simpleParkYBlue = -28;
    public static double moveOnlyParkYBlue = -48;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        // The init menu has to be readable whether the driver is on the Driver Station
        // or on FtcDashboard, and dashboard telemetry is not a mirror of the DS by
        // default.
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Battery.initialize(hardwareMap);
        Turret.initialize(hardwareMap);
        Indexer.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        Transfer.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        Robot.initialize(hardwareMap);

        Turret turret = Turret.getInstance();
        Shooter shooter = Shooter.getInstance();
        Drivetrain drivetrain = Drivetrain.getInstance();
        Robot robot = Robot.getInstance();
        turret.reset();

        AutoChoices<FarStrategy> choices = AutoMenu.select(
                this, new AutoChoices<>(defaultAlliance, defaultStrategy, 0.0));
        if (isStopRequested()) {
            return;
        }

        Alliance alliance = choices.alliance;
        FarStrategy strategy = choices.strategy;
        StaticVariables.saveAlliance(alliance);

        // Keep a record of what was actually selected. When a routine misbehaves at an event the
        // first question is always which one ran, and telemetry is gone by the time anyone asks.
        RobotLog.ii("FarAuto", "Auto choices: %s", choices);
        telemetry.addData("Running", choices);
        telemetry.update();

        FarPaths paths = new FarPaths(alliance);

        TelemetryPacket packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        drivetrain.setInitialPose(
                startXBlue,
                alliance.mirrorY(startYBlue),
                alliance.mirrorHeadingDeg(startHeadingBlueDeg)
        );
        turret.setInitialAngle(0);
        dashboard.sendTelemetryPacket(packet);

        // Sit out the period, but only after the pose is seeded: TeleOp reads it back from
        // StaticVariables and would otherwise start from wherever the odometry happened to reset.
        if (strategy == FarStrategy.DO_NOTHING) {
            try {
                while (opModeIsActive()) {
                    idle();
                }
            } finally {
                Robot.saveFinalState();
            }
            return;
        }

        try {
            List<Action> background = new ArrayList<>();
            background.add(saveStateContinuously());
            switch (strategy) {
                case MOVE_ONLY:
                    // Drives off the line and nothing else; the flywheel never spins.
                    break;
                case SIMPLE_SHOOT:
                    // Ranges the goal for its one shot rather than holding a fixed speed.
                    background.add(turret.autoAimInfinite(
                            FieldConstants.turretAimPoint(alliance)));
                    background.add(shooter.autoShoot(
                            FieldConstants.turretAimPoint(alliance).x,
                            FieldConstants.turretAimPoint(alliance).y
                    ));
                    break;
                default:
                    // The cycling routines hold one speed all period and re-aim continuously.
                    background.add(shooter.autonomousVelocityInfinite(
                            Utils.rpmToRadPerSec(velocity)));
                    background.add(turret.autoAimInfinite(
                            FieldConstants.turretAimPoint(alliance)));
                    background.add(shooter.hardStopClose());
                    break;
            }
            background.add(new SequentialAction(
                    Timed.pause(choices.startDelaySeconds),
                    routine(robot, drivetrain, paths, alliance, strategy)
            ));

            Actions.runBlocking(new ParallelAction(background.toArray(new Action[0])));
        } finally {
            Robot.saveFinalState();
        }
    }

    private Action routine(
            Robot robot,
            Drivetrain drivetrain,
            FarPaths paths,
            Alliance alliance,
            FarStrategy strategy
    ) {
        Transfer transfer = Transfer.getInstance();
        Shooter shooter = Shooter.getInstance();
        Turret turret = Turret.getInstance();
        List<Action> steps = new ArrayList<>();

        if (strategy == FarStrategy.MOVE_ONLY) {
            steps.add(drivetrain.goToPoseTimed(
                    Utils.makePoseVector(
                            startXBlue,
                            alliance.mirrorY(moveOnlyParkYBlue),
                            alliance.mirrorHeadingDeg(startHeadingBlueDeg)
                    ),
                    1, Math.toRadians(1), true, 5
            ));
            return new SequentialAction(steps.toArray(new Action[0]));
        }

        if (strategy == FarStrategy.SIMPLE_SHOOT) {
            steps.add(new SleepAction(3));
            steps.add(robot.shootFAR());
            steps.add(drivetrain.goToPoseTimed(
                    Utils.makePoseVector(
                            startXBlue,
                            alliance.mirrorY(simpleParkYBlue),
                            alliance.mirrorHeadingDeg(startHeadingBlueDeg)
                    ),
                    1, Math.toRadians(1), true, 5
            ));
            return new SequentialAction(steps.toArray(new Action[0]));
        }

        // ----- opening: nudge off the wall and fire the preload -----
        steps.add(drivetrain.followPathTimed(
                paths.openingNudge, MAX_SPEED, 1, 0.5, true, strategy.openingPathTime));
        steps.add(new SleepAction(1));
        steps.add(robot.moveShootFAR());

        // ----- third row -----
        steps.add(robot.gatherRow(
                paths.shootToThirdRow, MAX_SPEED, 1.5, 2.0, strategy.rowTransferTime));
        steps.add(robot.shootFAR(
                paths.thirdRowToShoot, MAX_SPEED, 1.5, 2.0, strategy.shootPathTime));

        if (strategy == FarStrategy.HUMAN_PLAYER_CYCLE) {
            for (int i = 0; i < humanPlayerCycles; i++) {
                steps.add(robot.gatherRow(paths.shootToHumanPlayer, MAX_SPEED, 1.5, 2.0, 1));
                steps.add(new ParallelAction(
                        transfer.ballDetectionTimed(1),
                        robot.shootFARFast(paths.humanPlayerToShoot, MAX_SPEED, 1.5, 2.0, 1)
                ));
            }
        } else { // THIRD_ROW_LINGER
            steps.add(drivetrain.followPathTimed(
                    paths.shootToHumanPlayer, MAX_SPEED, 1.5, 2.0, true, 2));
            if (paths.humanPlayerCorner != null) {
                steps.add(robot.gatherRow(paths.humanPlayerCorner, MAX_SPEED, 1.5, 2.0, 2));
            }
            steps.add(robot.shootFAR(paths.humanPlayerToShoot, MAX_SPEED, 1.5, 2.0, 2));
            steps.add(robot.gatherLingeringBalls(
                    paths.shootToLingering, paths.lingeringToShoot, MAX_SPEED, 15, 10));
        }

        steps.add(drivetrain.followPathTimed(paths.finishNudge, MAX_SPEED, 0, 0, true, 1.5));
        return new SequentialAction(steps.toArray(new Action[0]));
    }

    /** Keeps the pose and turret angle in StaticVariables fresh for TeleOp. */
    private Action saveStateContinuously() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (opModeIsActive() && !isStopRequested()) {
                    StaticVariables.saveRobotState(Drivetrain.getInstance().state);
                    StaticVariables.saveTurretAngle(Turret.getInstance().getAngle());
                }
                return true;
            }
        };
    }
}
