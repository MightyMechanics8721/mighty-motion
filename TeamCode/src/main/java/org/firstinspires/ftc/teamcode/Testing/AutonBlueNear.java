package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;

@Config
@Autonomous(name = "Blue Near", group = "test")
public class AutonBlueNear extends LinearOpMode {
    public static double SHOOTER_VELOCITY = 2350;

    /**
     * SHOOOTER DIRECTION: FORWARD INDEXER DIRECTION: INTAKE DIRECTION:
     */


    @Override
    public void runOpMode() {
        // ----- TELEMETRY -----
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        // ---- HARDWARE -----
        Battery battery = new Battery(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, battery);
        Intake intake = new Intake(hardwareMap, battery);
        Indexer indexer = new Indexer(hardwareMap, battery);
        Shooter shooter = new Shooter(hardwareMap, battery);

        // ---- UTILS -----
        ElapsedTime looptime = new ElapsedTime();
        SleepAction sleepAction;
        // ---- INIT ----- FIX ODO,UPDATE
        drivetrain.setInitialPose(-51, -51, -135);
        //        drivetrain.localize();

        waitForStart();
        looptime.reset();
        drivetrain.setInitialPose(-51, -51, -135);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                drivetrain.goToPose(Utils.makePoseVector(-12, -12, -135)),
                                shooter.setShooterVelocityInstant(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60
                                )
                        ),
                        new ParallelAction(
                                indexer.setIndexerPower(1),
                                intake.setIntakePower(-1),
                                shooter.setShooterVelocityTimed(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60,
                                        0.5
                                )
                        ),
                        shooter.setShooterVelocityInstant(
                                -SHOOTER_VELOCITY / 2 * 2 * Math.PI / 60
                        ),

                        // ----- GATHER FIRST ROW -----
                        drivetrain.goToPose(Utils.makePoseVector(-12, -12, -90)),
                        drivetrain.goToPose(Utils.makePoseVector(-12, -36, -90)),

                        new ParallelAction(
                                new SequentialAction(
                                        drivetrain.goToPose(Utils.makePoseVector(-12, -40.5, -90)),
                                        drivetrain.goToPose(Utils.makePoseVector(-12, -53, -90))
                                ),

                                new InstantAction(() -> indexer.motorController.setPower(1)),
                                indexer.setIndexerPower(1),
                                intake.setIntakePower(-1)
                        ),

                        // ----- Move to shooting pos -----
                        new ParallelAction(
                                drivetrain.goToPose(
                                        Utils.makePoseVector(-12, -12, -135)
                                        , 0.75, 0.05
                                ),
                                indexer.setIndexerPower(-0.25),
                                shooter.setShooterVelocityTimed(
                                        -SHOOTER_VELOCITY / 3.5 * 2 * Math.PI / 60, 2.5
                                )
                        ),

                        new ParallelAction(
                                indexer.setIndexerPower(-0.5),
                                shooter.setShooterVelocityInstant(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60
                                )
                        ),
                        new ParallelAction(
                                indexer.setIndexerPower(1),
                                intake.setIntakePower(-1),
                                shooter.setShooterVelocityTimed(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60,
                                        0.5
                                )
                        ),


                        shooter.setShooterVelocityInstant(
                                -SHOOTER_VELOCITY / 2 * 2 * Math.PI / 60
                        ),

                        // ----- GATHER SECOND BALL

                        drivetrain.goToPose(Utils.makePoseVector(12, -24, -90)),
                        drivetrain.goToPose(Utils.makePoseVector(12, -36, -90)),
                        new ParallelAction(
                                new SequentialAction(
                                        drivetrain.goToPose(Utils.makePoseVector(12, -40.5, -90)),
                                        drivetrain.goToPose(Utils.makePoseVector(12, -58, -90))
                                ),

                                new InstantAction(() -> indexer.motorController.setPower(1)),
                                indexer.setIndexerPower(1),
                                intake.setIntakePower(-1)
                        ),
                        drivetrain.goToPose(Utils.makePoseVector(12, -48, -90)),
                        // ----- Move to Shooting Pos -----
                        new ParallelAction(
                                drivetrain.goToPose(
                                        Utils.makePoseVector(-12, -12, -135)
                                        , 0.75, 0.05
                                ),
                                indexer.setIndexerPower(-0.25),
                                shooter.setShooterVelocityTimed(
                                        -SHOOTER_VELOCITY / 3.5 * 2 * Math.PI / 60, 2
                                )
                        ),
                        new ParallelAction(
                                indexer.setIndexerPower(-0.5),
                                shooter.setShooterVelocityInstant(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60
                                )
                        ),
                        new ParallelAction(
                                indexer.setIndexerPower(1),
                                intake.setIntakePower(-1),
                                shooter.setShooterVelocityTimed(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60,
                                        0.75
                                )
                        ),

                        shooter.setShooterVelocityInstant(
                                -SHOOTER_VELOCITY / 2 * 2 * Math.PI / 60
                        ),

                        // ----- GATHER THIRD ROW -----

                        drivetrain.goToPose(Utils.makePoseVector(36, -24, -90)),
                        drivetrain.goToPose(Utils.makePoseVector(36, -36, -90)),
                        new ParallelAction(
                                new SequentialAction(
                                        drivetrain.goToPose(Utils.makePoseVector(36, -40.5, -90)),
                                        drivetrain.goToPose(Utils.makePoseVector(36, -58, -90))
                                ),

                                new InstantAction(() -> indexer.motorController.setPower(1)),
                                indexer.setIndexerPower(1),
                                intake.setIntakePower(-1)
                        ),

                        // ----- Move to Shooting Pos -----
                        new ParallelAction(
                                drivetrain.goToPose(
                                        Utils.makePoseVector(-12, -12, -135)
                                        , 0.5, 0.05
                                ),
                                indexer.setIndexerPower(-0.25),
                                shooter.setShooterVelocityTimed(
                                        -SHOOTER_VELOCITY / 3.5 * 2 * Math.PI / 60, 2
                                )
                        ),
                        new ParallelAction(
                                indexer.setIndexerPower(-0.5),
                                shooter.setShooterVelocityInstant(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60
                                )
                        ),
                        new ParallelAction(
                                indexer.setIndexerPower(1),
                                intake.setIntakePower(-1),
                                shooter.setShooterVelocityTimed(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60,
                                        0.5
                                )
                        ),
                        drivetrain.goToPose(
                                Utils.makePoseVector(-54, -12, -90)
                                , 0.5, 0.05
                        )

                )
        );
    }
}
