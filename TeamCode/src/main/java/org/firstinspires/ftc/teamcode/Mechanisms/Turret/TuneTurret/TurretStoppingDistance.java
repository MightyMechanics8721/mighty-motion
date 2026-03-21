package org.firstinspires.ftc.teamcode.Mechanisms.Turret.TuneTurret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Turret.Turret;

@Config
@Autonomous(name = "TurretStoppingDistance")
public class TurretStoppingDistance extends LinearOpMode {
    public static double power = 0;

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        Turret.initialize(hardwareMap);
        Turret turret = Turret.getInstance();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        packet.put("velocity", turret.getVelocity());

        waitForStart();

        while (opModeIsActive()) {
            boolean run = gamepad1.right_bumper;
            boolean reset = gamepad1.left_bumper;

            turret.setTurretPower(power, run, reset, packet);
            packet.put("velocity", turret.getVelocity());
            packet.put("power", power);
            packet.put("run?", run);
            packet.put("reset", reset);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
