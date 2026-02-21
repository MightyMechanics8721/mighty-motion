//package org.firstinspires.ftc.teamcode.Mechanisms.Robot;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.ejml.simple.SimpleMatrix;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
//import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
//import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
//import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;
//
//public class Robot {
//    private static Robot instance;
//    private Battery battery;
//    private Indexer indexer;
//    private Intake intake;
//    private Shooter shooter;
//    private Drivetrain drivetrain;
//
//    private Robot() {
//        intake = Intake.getInstance();
//        indexer = Indexer.getInstance();
//        shooter = Shooter.getInstance();
//        drivetrain = Drivetrain.getInstance();
//        battery = Battery.getInstance();
//    }
//
//    public static void initialize() {
//        instance = new Robot();
//    }
//
//    public static Robot getInstance() {
//        if (instance == null) {
//            throw new IllegalStateException("Robot not initialized!");
//        }
//        return instance;
//    }
//
//    public Action shootAtPose(
//            SimpleMatrix desiredPose,
//            double distanceThreshold,
//            double angleThreshold,
//            double velocity
//    ) {
//        return new Action() {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (drivetrain.inStoppingZone(desiredPose, distanceThreshold, angleThreshold)) {
//                    Actions.runBlocking(new ParallelAction(
//                            indexer.setIndexerPower(1),
//                            intake.setIntakePower(-1),
//                            shooter.setShooterVelocityTimed(
//                                    2350 * 2 * Math.PI / 60,
//                                    0.5
//                            )
//                    ));
//                    return true;
//                }
//                return false;
//            }
//        };
//
//    }
//}
