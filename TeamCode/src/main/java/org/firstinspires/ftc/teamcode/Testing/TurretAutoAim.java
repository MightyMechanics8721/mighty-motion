package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * TurretAutoAim computes the angle the turret needs to aim at a goal
 * and returns an Action that rotates the turret there.
 */

public class TurretAutoAim {

    // The turret controller handles PID and servo movement
    private final TurretServo turret;

    // Store the turret object to command it later
    public TurretAutoAim(TurretServo turret) {
        this.turret = turret;
    }

    /**
     * Computes the robot-relative angle to a goal and returns an Action
     * to rotate the turret to that angle.
     *
     * @param robotPose Current robot position and heading
     * @param goalPos   Goal position on the field
     * @return Action that moves the turret to aim at the goal
     */
    public Action autoAim(Pose2d robotPose, Vector2d goalPos) {
        // Find vector from robot to goal
        double dx = goalPos.x - robotPose.position.x;
        double dy = goalPos.y - robotPose.position.y;

        // Compute angle to goal relative to the field
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        // Convert to angle relative to the robot
        double robotRelativeAngle =
                fieldAngle - Math.toDegrees(robotPose.heading.toDouble());

        // Make sure angle is between 0 and 360
        robotRelativeAngle = (robotRelativeAngle % 360 + 360) % 360;

        // Return an Action that uses the turret PID to rotate
        return turret.turretSpin(robotRelativeAngle);
    }


}
