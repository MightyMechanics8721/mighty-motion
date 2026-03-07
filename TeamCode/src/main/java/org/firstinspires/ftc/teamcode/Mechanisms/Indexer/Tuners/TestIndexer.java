//package org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Tuners;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
//import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
//
//@Config
//@Autonomous(name = "Test Indexer", group = "Testing")
//public class TestIndexer extends LinearOpMode {
//
//    FtcDashboard dashboard;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Battery.initialize(hardwareMap);
//        Indexer.initialize(hardwareMap);
//        Indexer indexer = Indexer.getInstance();
//
//        double power;
//
//        TelemetryPacket packet = new TelemetryPacket();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            power = gamepad1.right_trigger;
//            indexer.setIndexerPower(power).run(packet);
//        }
//    }
//}
