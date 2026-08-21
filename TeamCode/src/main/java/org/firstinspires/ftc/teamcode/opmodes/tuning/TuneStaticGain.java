package org.firstinspires.ftc.teamcode.opmodes.tuning;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.hardware.Battery;

@Config
@Autonomous(name = "Tune Static", group = "Autonomous")
public class TuneStaticGain extends LinearOpMode {
    FtcDashboard dashboard;
    Drivetrain drivetrain;
    TelemetryPacket packet;

    @Override
    public void runOpMode() {
        // Battery first: DcMotorAdvanced.setPower reads it for voltage compensation.
        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        drivetrain = Drivetrain.getInstance();
        packet = new TelemetryPacket();
        drivetrain.setTelemetry(packet);
        dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
        telemetry = dashboard.getTelemetry();

        ElapsedTime looptime = new ElapsedTime();
        SimpleMatrix speeds = new SimpleMatrix(
                new double[][]{
                        {1},
                        {1},
                        {1},
                        {1}

                }
        );
        SimpleMatrix accelerations = new SimpleMatrix(
                new double[][]{
                        {0},
                        {0},
                        {0},
                        {0}
                }
        );
        waitForStart();

        looptime.reset();

        while (opModeIsActive()) {
            drivetrain.localize();
            drivetrain.setWheelSpeedAcceleration(speeds, accelerations);
            looptime.reset();
        }
    }
}
