package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Autonomous(name = "Ethan Test", group = "Autonomous")
public class OPMode extends LinearOpMode {
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        //setup variables
        MotorController motorController = new MotorController(hardwareMap, new String[]{"f1", "f2"});
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            //motorController.setPower(0.5);
//            motorController.setPower(new String[]{"f1"}, 0.10);
//            motorController.setPower(new String[]{"f2"}, -0.10);
            motorController.setPower(new double[]{0.5, 1.0});
            telemetry.addData("test", 5);
            telemetry.update();
        
        }
    }
}
