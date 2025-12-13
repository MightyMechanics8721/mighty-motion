package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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
    public static double SHOOTER_VELOCITY = 2500;

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
                        new SleepAction(1),
                        new ParallelAction(
                                indexer.setIndexerPower(1),
                                intake.setIntakePower(1),
                                shooter.setShooterVelocityTimed(
                                        SHOOTER_VELOCITY * 2 * Math.PI / 60,
                                        3
                                )
                        ),
                        new SleepAction(1),
                        indexer.setIndexerPower(0),
                        intake.setIntakePower(0),
                        shooter.setShooterVelocityInstant(0)


                )
        );


        // MAKE NEW ACTION --- STOP POWER TO INDEXER SHOOTER
        //        public Action stopShooting(){
        //            return new SeAction(
        //
        //            );
        //        }
    }
}
