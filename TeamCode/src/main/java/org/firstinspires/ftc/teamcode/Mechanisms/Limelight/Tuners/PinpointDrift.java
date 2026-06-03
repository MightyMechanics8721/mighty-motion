package org.firstinspires.ftc.teamcode.Mechanisms.Limelight.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drawing;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.PID;

@Config
@TeleOp(name = "PP Drift", group = "123Competition")
public class PinpointDrift extends LinearOpMode {

    private Limelight3A limelight;
    private Drivetrain drivetrain;
    private Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        Battery.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        drivetrain = Drivetrain.getInstance();
        shooter = Shooter.getInstance();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        ElapsedTime timer = new ElapsedTime();
        double[] xPos = new double[100];
        double[] yPos = new double[100];
        double[] tPos = new double[100];
        int loops = 0;
        int seconds = 0;
        double xSum = 0;
        double ySum = 0;
        double tSum = 0;
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            drivetrain.localize();
            Actions.runBlocking(shooter.setShooterVelocityLoop(1500));
            packet.put("Heading", Math.toDegrees(drivetrain.state.get(2)));
            if (seconds < 100) {
                if (timer.seconds() < 1) {
                    xSum += drivetrain.state.get(0);
                    ySum += drivetrain.state.get(1);
                    tSum += drivetrain.state.get(2);
                    loops++;
                } else {
                    xPos[seconds] = xSum / loops;
                    yPos[seconds] = ySum / loops;
                    tPos[seconds] = tSum / loops;
                    xSum = 0;
                    ySum = 0;
                    tSum = 0;
                    loops = 0;
                    timer.reset();
                    seconds++;
                }
            } else {
                packet.put("xDrift 0", xPos[0]);
                packet.put("xDrift 99", xPos[99]);
                packet.put("yDrift 0", yPos[0]);
                packet.put("yDrift 99", yPos[99]);
                packet.put("tDrift 0", Math.toDegrees(tPos[0]));
                packet.put("tDrift 99", Math.toDegrees(tPos[99]));
            }
            dashboard.sendTelemetryPacket(packet);
        }
    }
}