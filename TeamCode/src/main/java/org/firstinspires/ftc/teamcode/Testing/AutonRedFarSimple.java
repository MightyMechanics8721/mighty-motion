package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@Autonomous(name = "Red Far SIMPLE", group = "test")
public class AutonRedFarSimple extends LinearOpMode {
    public static double SHOOTER_VELOCITY = 3100;

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
        drivetrain.setInitialPose(64, 16, 180);
        //        drivetrain.localize();

        waitForStart();
        looptime.reset();
        drivetrain.setInitialPose(64, 16, 180);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                drivetrain.goToPose(Utils.makePoseVector(49.5, 13.5, 157.5)),
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
                        drivetrain.goToPose(
                                Utils.makePoseVector(60, 36, 180)
                                , 0.5, 0.05
                        )

                )
        );
    }
}
