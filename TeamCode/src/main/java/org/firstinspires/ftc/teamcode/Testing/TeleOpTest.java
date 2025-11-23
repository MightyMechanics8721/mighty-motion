package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;

@TeleOp(name = "Full Robot Test")
public class TeleOpTest extends LinearOpMode {

    public static double targetVelocity = 2500; // (RPM)
    public static double SHOOTER_VELOCITY = 2500;
    Battery battery;
    FtcDashboard dashboard;
    private Intake intake;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private Indexer indexer;
    private Shooter shooter;

    public void runOpMode() {


        dashboard = FtcDashboard.getInstance();
        battery = new Battery(hardwareMap);
        intake = new Intake(hardwareMap, battery);
        indexer = new Indexer(hardwareMap, battery);
        shooter = new Shooter(hardwareMap, battery);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.2)  //forward intake
                intake.setPower(1.0);

            else if (gamepad1.left_trigger > 0.2)  //reverse intake
                intake.setPower(-1.0);
            else  //stop intake
                intake.setPower(0.0);


            if (gamepad1.b) {
                indexer.setPower(1.0);       //starts indexer
            } else {
                indexer.setPower(0.0);       //stop indexer
            }

            if (gamepad1.right_bumper) {
                shooter.setVelocity(SHOOTER_VELOCITY * 2 * Math.PI / 60.0);  //spin up shooter to target velocity
            } else
                shooter.setVelocity(0);


        }
        telemetry.addData("Shooter Target Vel", gamepad1.right_bumper ? SHOOTER_VELOCITY : 0);
        telemetry.addData("Battery (V)", battery.getVoltage());
        dashboard.getTelemetry().addData("Shooter Vel", shooter.getVelocity());
        dashboard.getTelemetry().addData("Battery Voltage", battery.getVoltage());
    }
}
