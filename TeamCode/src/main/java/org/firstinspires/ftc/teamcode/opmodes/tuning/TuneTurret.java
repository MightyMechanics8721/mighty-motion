package org.firstinspires.ftc.teamcode.opmodes.tuning;

import org.firstinspires.ftc.teamcode.mechanisms.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Battery;

@Config
@Autonomous(name = "Tune Turret", group = "Tuning")
public class TuneTurret extends LinearOpMode {
    Turret turret;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        // The turret encoder shares the left front drive motor's port, and that motor's direction
        // decides the sign the encoder reports. Bring the Drivetrain up so the angle here matches
        // what the match OpModes see.
        Battery.initialize(hardwareMap);
        Drivetrain.initialize(hardwareMap);
        Turret.initialize(hardwareMap);
        turret = Turret.getInstance();
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            turret.manualControl(gamepad1.left_stick_y).run(packet);
            packet.put("power", gamepad1.left_stick_y);
            packet.put("angle", turret.getAngle());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
