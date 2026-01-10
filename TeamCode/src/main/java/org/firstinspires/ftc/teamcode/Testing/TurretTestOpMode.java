package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;

@TeleOp(name = "Turret Test Blue Goal1")
@Disabled
public class TurretTestOpMode extends LinearOpMode {

    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialize Turret ---
        turret = new Turret(hardwareMap);

        waitForStart();

        // --- Blue goal coordinates in FTC field coordinate system ---
        // +X points away from alliance wall, +Y points left of +X
        double blueGoalX = 72.0;  // far side of the field
        double blueGoalY = 0.0;   // centered
        Vector2d blueGoal = new Vector2d(blueGoalX, blueGoalY);

        // --- Example starting robot pose ---
        // Replace with your real robot pose if using localization
        Pose2d robotPose = new Pose2d(0, 0, 0);

        TelemetryPacket packet = new TelemetryPacket();

        while (opModeIsActive()) {

            // --- Manual control ---
            turret.manualControl().run(packet);

            // --- Auto-aim to blue goal: press gamepad1.a ---
            if (gamepad1.a) {
                Action aimAction = turret.autoAim(robotPose, blueGoal);

                while (!aimAction.run(packet) && opModeIsActive()) {
                    // continuously update telemetry
                    packet.put("Current Angle", turret.getAngle());
                    packet.put("Target Angle", turret.getAngle()); // target angle is inside setTurretAngle
                }
            }

            // Always display current angle (even outside auto-aim)
            packet.put("Current Angle", turret.getAngle());

            // Send telemetry packet to dashboard
            // If you are connected to FtcDashboard, call:
            // FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
