package org.firstinspires.ftc.teamcode.opmodes.tuning;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

import org.firstinspires.ftc.teamcode.util.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.mechanisms.Shooter.MOTOR_CONTROLLER_CONSTANTS;

import org.firstinspires.ftc.teamcode.hardware.Battery;

@Config
@Autonomous(name = "Tune Shooter", group = "Testing")
public class TuneShooter extends LinearOpMode {
    public static double targetVelocity = 2500; // (RPM)
    FtcDashboard dashboard;
    Shooter shooter;

    @Override
    public void runOpMode() {

        Battery.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        shooter = Shooter.getInstance();
        dashboard = FtcDashboard.getInstance();
        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            double target = Utils.rpmToRadPerSec(targetVelocity);

            // One control update per iteration. PID.calculate advances the controller's timer and
            // integral, so calling it again for telemetry would change what the shooter is doing.
            shooter.setShooterVelocityLoop(target).run(packet);

            double currentVelocity = shooter.getVelocity();
            double ffPower = MOTOR_CONTROLLER_CONSTANTS.ffConstants.kS * Math.signum(target)
                    + MOTOR_CONTROLLER_CONSTANTS.ffConstants.kV * target;
            double pidPower = MOTOR_CONTROLLER_CONSTANTS.pidConstants.kP
                    * (target - currentVelocity);

            packet.put("shooterpowerFF", ffPower);
            packet.put("shooterpowerPID", pidPower);
            packet.put("shooterpower", ffPower + pidPower);
            packet.put("Velocity (RPM)", Utils.radPerSecToRpm(currentVelocity));
            packet.put("Target Velocity (RPM)", targetVelocity);
            packet.put("Target Velocity (rad/s)", target);
            dashboard.sendTelemetryPacket(packet);
        }
    }

}

