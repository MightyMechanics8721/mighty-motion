package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Battery;

@Config
@Autonomous(name = "TurretStoppingDistance")
public class TurretStoppingDistance extends LinearOpMode {
    public static double power = 0;

    @Override
    public void runOpMode() {
        // The turret encoder shares the left front drive motor's port, and that motor's direction
        // decides the sign the encoder reports. Bring the Drivetrain up so the angle here matches
        // what the match OpModes see.
        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        Turret.initialize(hardwareMap);
        Turret turret = Turret.getInstance();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        turret.setInitialAngle(0);
        waitForStart();
        int count = 0;
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            boolean run = gamepad1.right_bumper;
            boolean reset = gamepad1.left_bumper;

            turret.setTurretPower(power, run, reset, packet);
            packet.put("velocity", turret.getVelocity());
            packet.put("power", power);
            packet.put("run?", run);
            packet.put("reset", reset);
            packet.put("loops", count);
            count++;
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
