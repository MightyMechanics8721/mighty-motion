package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils;

import org.ejml.simple.SimpleMatrix;
public class Utils {

    /***
     * Returns a transform matrix to convert between global and local coordinate frames
     * @param th Angle to be used in the transform
     * @return 3 x 3 SimpleMatrix to be used in converting coordinate frames
     */
    public static SimpleMatrix R(double th) {
        SimpleMatrix rotation = new SimpleMatrix(
                new double[][]{
                        new double[]{Math.cos(th), -1*Math.sin(th), 0},
                        new double[]{Math.sin(th), Math.cos(th), 0},
                        new double[]{0, 0, 1},
                }
        );
        return rotation;
    }

    /***
     * Rotate from local robot coords to global coords
     * @param vectorBody The matrix of local coordinates
     * @param th the θ value for the relative rotation
     * @return the matrix of global coordinates
     */
    public static SimpleMatrix rotateBodyToGlobal(SimpleMatrix vectorBody, double th) {
        return R(th).mult(vectorBody);
    }

    /***
     * Rotate from global robot coords to local coords
     * @param vectorGlobal The matrix of global coordinates
     * @param th the θ value for the relative rotation
     * @return the matrix of global coordinates
     */
    public static SimpleMatrix rotateGlobalToBody(SimpleMatrix vectorGlobal, double th) {
        return R(th).invert().mult(vectorGlobal);
    }

    /***
     * Converts an angle in radians to a value between -π and π
     * @param radians
     * @return radian value between -π and π
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

    /***
     * returns the simple geometric distance between two coordinates
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    public static double calculateDistance(double x1, double y1,double x2,double y2){
        return Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2));
    }

    /***
     * Returns a 3 x 1 SimpleMatrix of x, y, θ
     * @param posX
     * @param posY
     * @param posTheta
     * @return
     */
    public static SimpleMatrix makePoseVector(double posX, double posY, double posTheta){
        return new SimpleMatrix(
            new double[][]{
                new double[]{posX},
                new double[]{posY},
                new double[]{Math.toRadians(posTheta)}
            }
        );
    }
}
