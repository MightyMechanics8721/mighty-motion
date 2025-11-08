package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
@Autonomous(name = "Motor Controller Test ", group = "Autonomous")
public class TestMotorController extends LinearOpMode {
    public static double targetVelocity = 1000.0; // (RPM)
    public static double targetVelocity2 = 5000.0;

    public static FFConstants feedforwardConstants = new FFConstants(0, 5000, 0);
    public static PIDConstants pidConstants = new PIDConstants(0, 0, 0);
    protected double encoderResolution = 145.1;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        //setup variables
        MotorController motorController = new MotorController(hardwareMap, new String[]{"f1", "f2"}, "f1", 145.1);
        motorController.setVelocityFeedForwardConstants(feedforwardConstants);
        motorController.setVelocityPIDConstants(pidConstants);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            motorController.setVelocity(new double[]{
                    targetVelocity * 0.10472,   // first motor (f1)
                    targetVelocity2 * 0.10472   // second motor (f2)
            });


            // Create a telemetry packet for FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();

            double velocity = motorController.getVelocity();

            packet.put("Velocity in Radians/sec", velocity);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();


        }
    }
}
