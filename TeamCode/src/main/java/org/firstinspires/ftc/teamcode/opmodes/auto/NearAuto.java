package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.Path;
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
 * Every near-side autonomous, in one OpMode.
 * <p>
 * Alliance, routine and start delay are picked on the init menu rather than by choosing between
 * six near-identical OpModes. The routes come from {@link NearPaths}, written blue-side and
 * mirrored, so the two alliances cannot drift apart.
 *
 * @see AutoMenu
 * @see AutoStrategy
 */
@Config
@Autonomous(name = "Near Auto", group = "1Comp")
public class NearAuto extends LinearOpMode {

    public static Alliance defaultAlliance = Alliance.BLUE;
    public static AutoStrategy defaultStrategy = AutoStrategy.GATE_CYCLE;

    public static double MAX_SPEED = 150;
    public static double ALL_PATH_TIME = 5;
    public static double GATE_TRANSFER_TIME = 1.75;
    public static double EXTRA_TRANSFER_TIME = 1;

    public static double xGate = 12;
    public static double xGateBackup = 17;
    public static double yGateBackupBlue = -56.5;
    public static double thetaGateBackupBlue = -120;

    /** Where the routine parks, written blue-side. */
    public static double parkXBlue = 0;
    public static double parkYBlue = -24;
    public static double parkHeadingBlueDeg = -90;

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

        AutoChoices<AutoStrategy> choices = AutoMenu.select(
                this, new AutoChoices<>(defaultAlliance, defaultStrategy, 0.0));
        if (isStopRequested()) {
            return;
        }

        Alliance alliance = choices.alliance;
        AutoStrategy strategy = choices.strategy;
        StaticVariables.saveAlliance(alliance);

        // Keep a record of what was actually selected. When a routine misbehaves at an event the
        // first question is always which one ran, and telemetry is gone by the time anyone asks.
        RobotLog.ii("NearAuto", "Auto choices: %s", choices);
        telemetry.addData("Running", choices);
        telemetry.update();

        NearPaths paths = new NearPaths(
                alliance, xGate, strategy.gateYBlue, strategy.gateHeadingBlueDeg);
        SimpleMatrix startPose = FieldConstants.nearStartPose(alliance);

        TelemetryPacket packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        drivetrain.setInitialPose(
                startPose.get(0, 0),
                startPose.get(1, 0),
                Math.toDegrees(startPose.get(2, 0))
        );
        turret.setInitialAngle(alliance.mirrorHeadingDeg(-180));
        dashboard.sendTelemetryPacket(packet);

        // Sit out the period, but only after the pose is seeded: TeleOp reads it back from
        // StaticVariables and would otherwise start from wherever the odometry happened to reset.
        if (strategy == AutoStrategy.DO_NOTHING) {
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
            Actions.runBlocking(new SequentialAction(
                    shooter.hardStopClose(),
                    new ParallelAction(
                            shooter.autoShootMovingInfinite(
                                    FieldConstants.shooterRangePoint(alliance).x,
                                    FieldConstants.shooterRangePoint(alliance).y
                            ),
                            turret.autoAimInfinite(FieldConstants.turretAimPoint(alliance)),
                            saveStateContinuously(),
                            new SequentialAction(
                                    Timed.pause(choices.startDelaySeconds),
                                    routine(robot, turret, drivetrain, paths, alliance, strategy)
                            )
                    )
            ));
        } finally {
            Robot.saveFinalState();
        }
    }

    /** The routine proper, once the preload volley has been fired. */
    private Action routine(
            Robot robot,
            Turret turret,
            Drivetrain drivetrain,
            NearPaths paths,
            Alliance alliance,
            AutoStrategy strategy
    ) {
        List<Action> steps = new ArrayList<>();

        // ----- PRELOAD ROW -----
        steps.add(turret.cutoffTurret());
        steps.add(new ParallelAction(
                turret.setTurretAngleTimed(
                        alliance.mirrorHeadingDeg(-180), strategy.preloadTurretTime),
                shoot(robot, paths.preload, strategy)
        ));
        steps.add(turret.resumeTurret());

        // ----- SECOND ROW -----
        steps.add(gatherRow(robot, paths.firstHalf, strategy));
        steps.add(shoot(robot, paths.secondToShoot, strategy));

        if (strategy == AutoStrategy.GATE_CYCLE) {
            // Four gate cycles back to back, then the first row.
            for (int i = 0; i < 4; i++) {
                steps.add(robot.gatherGateNoBackup(
                        paths.gate, MAX_SPEED, strategy.followPathTime, GATE_TRANSFER_TIME));
                steps.add(shootIntake(robot, paths.shoot, strategy));
            }
            steps.add(gatherRow(robot, paths.firstRow, strategy));
            steps.add(shoot(robot, paths.firstToShoot, strategy));
        } else {
            // Alternate a row with a gate collection, backing off the gate each time.
            steps.add(gatherGate(robot, paths.gate, alliance, strategy));
            steps.add(shootIntake(robot, paths.shoot, strategy));

            steps.add(gatherRow(robot, paths.thirdRow, strategy));
            steps.add(shoot(robot, paths.thirdToShoot, strategy));
            steps.add(gatherGate(robot, paths.gate, alliance, strategy));
            steps.add(shootIntake(robot, paths.shoot, strategy));

            steps.add(gatherRow(robot, paths.firstRow, strategy));
            steps.add(shoot(robot, paths.firstToShoot, strategy));
            steps.add(gatherGate(robot, paths.gate, alliance, strategy));
            steps.add(shootIntake(robot, paths.shoot, strategy));
        }

        // ----- PARK -----
        steps.add(new ParallelAction(
                new SequentialAction(turret.cutoffTurret(), turret.setTurretAngleTimed(0, 1)),
                drivetrain.goToPoseTimed(
                        Utils.makePoseVector(
                                parkXBlue,
                                alliance.mirrorY(parkYBlue),
                                alliance.mirrorHeadingDeg(parkHeadingBlueDeg)
                        ),
                        0.25,
                        Math.toRadians(0.25),
                        true,
                        ALL_PATH_TIME
                )
        ));

        return new SequentialAction(steps.toArray(new Action[0]));
    }

    private Action shoot(Robot robot, Path path, AutoStrategy strategy) {
        return robot.shoot(path, MAX_SPEED, 2, 2.5, ALL_PATH_TIME, strategy.shootTime);
    }

    private Action shootIntake(Robot robot, Path path, AutoStrategy strategy) {
        return robot.shootIntake(path, MAX_SPEED, 2, 2.5, ALL_PATH_TIME, EXTRA_TRANSFER_TIME,
                strategy.shootTime);
    }

    private Action gatherRow(Robot robot, Path path, AutoStrategy strategy) {
        return robot.gatherRow(path, MAX_SPEED, 1.5, 2.5, strategy.rowTransferTime);
    }

    private Action gatherGate(
            Robot robot, Path path, Alliance alliance, AutoStrategy strategy
    ) {
        return robot.gatherGate(
                path,
                MAX_SPEED,
                0.5,
                2.5,
                xGateBackup,
                alliance.mirrorY(yGateBackupBlue),
                alliance.mirrorHeadingDeg(thetaGateBackupBlue),
                strategy.followPathTime,
                GATE_TRANSFER_TIME
        );
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
