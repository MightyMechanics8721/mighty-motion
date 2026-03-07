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
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
//
//@Config
//@Autonomous(name = "Tune Static", group = "Autonomous")
//public class TuneStaticGain extends LinearOpMode {
//    FtcDashboard dashboard;
//    Drivetrain drivetrain;
//    TelemetryPacket packet;
//
//    @Override
//    public void runOpMode() {
//        Drivetrain.initialize(hardwareMap);
//        drivetrain = Drivetrain.getInstance();
//        packet = new TelemetryPacket();
//        drivetrain.setTelemetry(packet);
//        dashboard = FtcDashboard.getInstance();
//        dashboard.sendTelemetryPacket(packet);
//        telemetry = dashboard.getTelemetry();
//
//
//        // TODO: fix this problem
//        //        drivetrain.motorController.ffLfm.setGains(0, 0, drivetrain.motorController
//        //        .ffLfm.kS);
//        //        drivetrain.motorController.ffLbm.setGains(0, 0, drivetrain.motorController
//        //        .ffLbm.kS);
//        //        drivetrain.motorController.ffRbm.setGains(0, 0, drivetrain.motorController
//        //        .ffRbm.kS);
//        //        drivetrain.motorController.ffRfm.setGains(0, 0, drivetrain.motorController
//        //        .ffRfm.kS);
//        ElapsedTime looptime = new ElapsedTime();
//        SimpleMatrix speeds = new SimpleMatrix(
//                new double[][]{
//                        {1},
//                        {1},
//                        {1},
//                        {1}
//
//                }
//        );
//        SimpleMatrix accelerations = new SimpleMatrix(
//                new double[][]{
//                        {0},
//                        {0},
//                        {0},
//                        {0}
//                }
//        );
//        waitForStart();
//
//        looptime.reset();
//
//        while (opModeIsActive()) {
//            drivetrain.localize();
//            drivetrain.setWheelSpeedAcceleration(speeds, accelerations);
//            looptime.reset();
//        }
//    }
//}
