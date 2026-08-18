package org.firstinspires.ftc.teamcode.util;

import org.ejml.simple.SimpleMatrix;

public class Utils {

    /**
     * Creates a 3×3 homogeneous rotation matrix for a rotation by the specified angle about the
     * origin.
     *
     * @param angle The rotation angle in radians
     *
     * @return 3 × 3 SimpleMatrix to be used in converting coordinate frames
     */
    public static SimpleMatrix angleToRotationMatrix(double angle) {
        SimpleMatrix rotation = new SimpleMatrix(
                new double[][]{
                        new double[]{Math.cos(angle), -1 * Math.sin(angle), 0},
                        new double[]{Math.sin(angle), Math.cos(angle), 0},
                        new double[]{0, 0, 1},
                        }
        );
        return rotation;
    }

    /**
     * Transforms a 3×1 vector from the robot’s body frame into the global (world) frame by applying
     * a rotation by the specified heading angle.
     *
     * @param vectorInBodyFrame The matrix of local coordinates [x, y, θ]
     * @param angle the θ value for the relative rotation
     *
     * @return the matrix of global coordinates
     */
    public static SimpleMatrix rotateBodyToGlobal(SimpleMatrix vectorInBodyFrame, double angle) {
        return angleToRotationMatrix(angle).mult(vectorInBodyFrame);
    }

    /**
     * Transforms a 3×1 vector from the global (world) frame frame into the robot’s body frame by
     * applying a rotation by the specified heading angle.
     *
     * @param vectorInBodyFrame The matrix of local coordinates [x, y, θ]
     * @param angle the θ value for the relative rotation
     *
     * @return the matrix of global coordinates
     */
    public static SimpleMatrix rotateGlobalToBody(SimpleMatrix vectorInBodyFrame, double angle) {
        return angleToRotationMatrix(angle).invert().mult(vectorInBodyFrame);
    }

    /**
     * Converts an angle in radians to its equal value between -π and π
     *
     * @param radians Unsimplified radian value of the angle
     *
     * @return The equivalent radian value between -π and π
     */
    public static double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    /**
     * Returns the simple geometric distance between two coordinates.
     *
     * @param xPosition1 First X coordinate
     * @param yPosition1 First Y coordinate
     * @param xPosition2 Second X coordinate
     * @param yPosition2 Second Y coordinate
     *
     * @return calculated distance between (xPosition1, yPosition1) and (xPosition2, yPosition2)
     */
    public static double calculateDistance(
            double xPosition1,
            double yPosition1,
            double xPosition2,
            double yPosition2
    ) {
        return Math.sqrt(
                Math.pow((xPosition2 - xPosition1), 2) + Math.pow((yPosition2 - yPosition1), 2));
    }

    /**
     * Returns a 3 x 1 SimpleMatrix of x, y, θ
     *
     * @param xPosition X Position
     * @param yPosition Y Position
     * @param thetaPosition Angle in degrees
     *
     * @return
     */
    public static SimpleMatrix makePoseVector(
            double xPosition,
            double yPosition,
            double thetaPosition
    ) {
        return new SimpleMatrix(
                new double[][]{
                        new double[]{xPosition},
                        new double[]{yPosition},
                        new double[]{Math.toRadians(thetaPosition)}
                }
        );
    }
}
