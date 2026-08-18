//package org.firstinspires.ftc.teamcode.mechanisms;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.hardware.Battery;
//import org.firstinspires.ftc.teamcode.mechanisms.Indexer;
//import org.firstinspires.ftc.teamcode.mechanisms.Intake;
//
//@Config
//@Autonomous(name = "Test Transfer", group = "Tuning")
//public class TestIntakeSystem extends LinearOpMode {
//    DistanceSensor distanceSensor;
//    FtcDashboard dashboard;
//    Battery battery;
//
//    @Override
//    public void runOpMode() {
//        TelemetryPacket packet = new TelemetryPacket();
//        Intake.initialize(hardwareMap);
//        Indexer.initialize(hardwareMap);
//        Battery.initialize(hardwareMap);
//        DistanceSensor.initialize(hardwareMap);
//        distanceSensor = DistanceSensor.getInstance();
//        battery = Battery.getInstance();
//        dashboard = FtcDashboard.getInstance();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            distanceSensor.ballDetection().run(packet);
//            dashboard.sendTelemetryPacket(packet);
//        }
//    }
//}
