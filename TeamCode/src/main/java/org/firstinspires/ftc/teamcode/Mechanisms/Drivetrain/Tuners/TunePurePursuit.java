//package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.ejml.simple.SimpleMatrix;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Geometry.Path;
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
//
//
//@Config
//@Autonomous(name = "Tune Pure Pursuit", group = "Autonomous")
//public class TunePurePursuit extends LinearOpMode {
//    public static double maxSpeed = 10;
//    public static PathSelection pathSelection = PathSelection.STRAIGHT_FORWARD;
//
//    @Override
//    public void runOpMode() {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//
//        Battery.initialize(hardwareMap);
//        Drivetrain.initialize(hardwareMap);
//
//        Battery battery = Battery.getInstance();
//        Drivetrain drivetrain = Drivetrain.getInstance();
//
//        Path straightForwardPath = new Path(
//                new double[][]{{0, 0}, {36, 0}, {64, 0}},
//                Math.toRadians(0), false,
//                false
//        );
//        Path straightForwardOffsetPath = new Path(
//                new double[][]{{0, 0}, {36, 0}, {64, 0}},
//                Math.toRadians(135),
//                false,
//                true
//        );
//        Path straightReversePath = new Path(
//                new double[][]{{64, 0}, {36, 0}, {0, 0}},
//                Math.toRadians(0),
//                true,
//                false
//        );
//        Path straight_90_Path = new Path(
//                new double[][]{{0, 0}, {36, 0}, {64, 0}},
//                Math.toRadians(90),
//                false,
//                false
//        );
//        Path reverse_90_Path = new Path(
//                new double[][]{{0, 0}, {36, 0}, {64, 0}},
//                Math.toRadians(90),
//                true,
//                false
//        );
//
//
//        Path complexPath = new Path(
//                new double[][]{{0, 0}, {24, 0}, {24, -24}, {64, -24}, {64, 0}},
//                Math.toRadians(90),
//                false,
//                false
//        );
//
//        Path path = straightForwardPath;
//
//        TelemetryPacket packet = new TelemetryPacket();
//        drivetrain.setTelemetry(packet);
//        drivetrain.setInitialPose(-0, 0, 0);
//        dashboard.sendTelemetryPacket(packet);
//
//        waitForStart();
//
//        drivetrain.setInitialPose(-0, 0, 0);
//
//        while (opModeIsActive()) {
//            switch (pathSelection) {
//                case STRAIGHT_FORWARD:
//                    path = straightForwardPath;
//                case STRAIGHT_REVERSE:
//                    path = straightReversePath;
//                case STRAIGHT_OFFSET:
//                    path = straightForwardOffsetPath;
//                case COMPLEX:
//                    path = complexPath;
//                case TURN_90_FORWARD:
//                    path = straight_90_Path;
//                case TURN_90_REVERSE:
//                    path = reverse_90_Path;
//            }
//
//            packet = new TelemetryPacket();
//
//            if (gamepad1.right_trigger > 0.1) {
//                drivetrain.followPath(
//                        new Path(
//                                new double[][]{{0, 0}, {-60, 0}, {-60, 60}},
//                                Math.toRadians(-90), true,
//                                false
//                        ), maxSpeed, 2,
//                        0.05, true
//                ).run(packet);
//            } else if (gamepad1.left_trigger > 0.1) {
//                drivetrain.goToPose(
//                        Utils.makePoseVector(
//                                0, 0, 0), Drivetrain.THRESHOLD_PARAMETERS.distanceThreshold,
//                        Drivetrain.THRESHOLD_PARAMETERS.angleThreshold, true
//                ).run(packet);
//            } else {
//                drivetrain.stopMotors().run(packet);
//            }
//
//            dashboard.sendTelemetryPacket(packet);
//        }
//    }
//
//    public enum PathSelection {
//        STRAIGHT_FORWARD,
//        STRAIGHT_OFFSET,
//        STRAIGHT_REVERSE,
//        TURN_90_FORWARD,
//        TURN_90_REVERSE,
//        COMPLEX,
//    }
//}