package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Geometry.Path;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Planners.MotionProfile;
//import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.TelemetryTracking;

/**
 * THIS IS AN AUTONOMOUS OPMODE WE WILL USE TO TEST YOUR DRIVETRAIN'S MOTOR DIRECTIONS. MAKE SURE
 * YOU ADD THE @CONFIG AT THE TOP OF ALL YOUR TUNING/TESTING OPMODES. FTC DASHBOARD IS BETTER THAN
 * PRINTING ONTO THE PHONE VIA FTC SDK TELEMETRY. DASHBOARD TELEMETRY IS BETTER!
 */
@Config
@Autonomous(name = "Tune Geo", group = "Autonomous")
public class TuneGeo extends LinearOpMode {
    public static double desiredX = 0;
    public static double desiredY = 0;
    public static double desiredTheta = 0;
    // Create drivetrain object
    Drivetrain drivetrain = null;
    // Use FTCDashboard
    FtcDashboard dashboard;
    //    TelemetryTracking tracking;
    Battery battery;

    @Override
    public void runOpMode() {
        // Set dashboard
        battery = new Battery(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, battery);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        ElapsedTime looptime = new ElapsedTime();

        telemetry.addData("x", 0);
        telemetry.addData("y", 0);
        telemetry.addData("theta", 0);
        telemetry.addData("desiredX", 0);
        telemetry.addData("desiredY", 0);
        telemetry.addData("desiredTheta", 0);
        telemetry.update();
        waitForStart();
        looptime.reset();
        double[][] coords = {{0, 0}, {24, 0}, {24, -24}, {72, -24}, {72, 0}};
        Path path = new Path(coords, 90, false, false);
        Actions.runBlocking(drivetrain.followPath(path));
    }
}