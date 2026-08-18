package org.firstinspires.ftc.teamcode.drivetrain;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.control.PID.functionType;
import org.firstinspires.ftc.teamcode.control.PIDConstants;
import org.firstinspires.ftc.teamcode.control.PoseConstants;
import org.firstinspires.ftc.teamcode.util.Utils;

public class PoseController {
    // PID Controllers for X, Y, and Theta (heading).
    private final PID xPID;
    private final PID yPID;
    private final PID tPID;

    /**
     * Constructor for the Pose Controller.
     * <p>
     * Initializes PID controllers for x, y, and heading with specific gains.
     * <p>
     * TODO: Refactor to accept a parameter structure from Drivetrain.
     */
    public PoseController(
            PoseConstants poseConstants
    ) {
        this.xPID = new PID(poseConstants.xPID, functionType.SQRT);
        this.yPID = new PID(poseConstants.yPID, functionType.SQRT);
        this.tPID = new PID(poseConstants.thetaPID, functionType.SQRT);
    }

    public PoseController(
            PIDConstants xPIDConstants,
            PIDConstants yPIDConstants,
            PIDConstants headingPIDConstants
    ) {
        this.xPID = new PID(xPIDConstants, functionType.SQRT);
        this.yPID = new PID(yPIDConstants, functionType.SQRT);
        this.tPID = new PID(headingPIDConstants, functionType.SQRT);
    }

    /**
     * Calculates a velocity vector to move from a current pose to a desired pose. Applies PID
     * control in the robot's frame and returns motor power commands.
     *
     * @param pose (SimpleMatrix) The current robot pose [x; y; heading] as a column matrix.
     * @param desiredPose (SimpleMatrix) The target robot pose [x; y; heading] as a column matrix.
     *
     * @return (SimpleMatrix) A 3x1 matrix representing the drive power to apply to each wheel.
     */
    public SimpleMatrix calculate(SimpleMatrix pose, SimpleMatrix desiredPose) {

        // Compute error in the global field frame.
        SimpleMatrix errorVectorInFieldFrame = new SimpleMatrix(
                new double[][]{
                        new double[]{desiredPose.get(0, 0) - pose.get(0, 0)},
                        new double[]{desiredPose.get(1, 0) - pose.get(1, 0)},
                        new double[]{Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0))}
                }
        );

        // Convert error to the robot's frame of reference.
        SimpleMatrix errorVectorInRobotFrame = Utils.rotateGlobalToBody(
                errorVectorInFieldFrame,
                pose.get(2, 0)
        );

        // Use PID controllers to calculate control effect in robot frame.
        double vX = xPID.calculate(
                errorVectorInRobotFrame.get(0, 0),
                0
        );                                         // inches/sec?
        double vY = yPID.calculate(
                errorVectorInRobotFrame.get(1, 0),
                0
        );                                         // inches/sec?
        double omega = tPID.calculate(
                Utils.angleWrap(desiredPose.get(2, 0) - pose.get(2, 0)),
                0
        );   // radians/sec?

        // Create a velocity vector in robot frame.
        return new SimpleMatrix(
                new double[][]{
                        new double[]{vX},
                        new double[]{vY},
                        new double[]{omega}
                }
        );
    }
}

