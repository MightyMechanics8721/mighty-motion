//package org.firstinspires.ftc.teamcode.tuning;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.hardware.Battery;
//import org.firstinspires.ftc.teamcode.mechanisms.Indexer;
//import org.firstinspires.ftc.teamcode.mechanisms.Intake;
//
//@Config
//@Autonomous(name = "TestIntake", group = "5")
//public class TestIntake extends LinearOpMode {
//
//    FtcDashboard dashboard;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Battery.initialize(hardwareMap);
//        Intake.initialize(hardwareMap);
//        Intake intake = Intake.getInstance();
//
//        double power;
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            power = (double) gamepad1.left_stick_x;
//            Actions.runBlocking(intake.setIntakePower(power));
//        }
//    }
//}
