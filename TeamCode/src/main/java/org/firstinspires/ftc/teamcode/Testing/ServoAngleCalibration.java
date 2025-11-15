//package org.firstinspires.ftc.teamcode.Testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name="ServoAngleCalibration", group="Testing")
//public class ServoAngleCalibration extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
//        AprilTagUtils limelight = new AprilTagUtils(hardwareMap, "limelight");
//
//        waitForStart();
//
//        double[] testPositions = {0.5, 0.4, 0.6};
//
//        for (double pos : testPositions) {
//            cameraServo.setPosition(pos);
//            sleep(1000); // wait for servo to settle
//
//            double tx = limelight.getHorizontalAngle();
//            telemetry.addData("Servo Position", pos);
//            telemetry.addData("Limelight tx (deg)", tx);
//            telemetry.update();
//
//            sleep(3000); // time to record it manually
//        }
//    }
//}

package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ServoAngleCalibration", group="Testing")
public class ServoAngleCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        AprilTagUtils limelight = new AprilTagUtils(hardwareMap, "limelight");
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        // Move servo to 0.4
        cameraServo.setPosition(0.5);

        // Wait 1 second for the servo to settle
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.0) {
            idle();
        }

        // Read horizontal angle and show telemetry
        double tx = limelight.getHorizontalAngle();
        telemetry.addData("Servo Position", 0.5);
        telemetry.addData("Limelight tx (deg)", tx);
        telemetry.update();

        // Keep OpMode alive so you can see telemetry
        while (opModeIsActive()) {
            idle();
        }
    }
}
