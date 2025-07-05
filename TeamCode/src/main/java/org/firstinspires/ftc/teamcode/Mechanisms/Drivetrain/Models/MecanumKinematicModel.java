package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Models;

import org.ejml.simple.SimpleMatrix;
public class MecanumKinematicModel {
    // The wheel radius
    public static double r = 2.16535; //in
    // Distance from center of the bot to the front/back
    public static double l = 5.7; //in
    // Distance from the center to the left/right
    public static double w = 5.31496; //in

    // 4 x 3 SimpleMatrix used to represent the relative positions of each wheel
    // Used in inverseKinematics to get the target speed for each of 4 wheels and their x, y, θ
    static SimpleMatrix H_inv = new SimpleMatrix(
            new double[][]{
                    new double[]{1d, -1d, -(l + w)},
                    new double[]{1d, 1d, -(l + w)},
                    new double[]{1d, -1d, (l + w)},
                    new double[]{1d, 1d, (l + w)}
            }
    );

    /***
     * Returns target robot wheel speeds
     * @param twist chassis twist
     * @return wheel speeds as a 4 x 3 SimpleMatrix representing each wheel's respective x, y, θ
     */
    public static SimpleMatrix inverseKinematics(SimpleMatrix twist) {
        SimpleMatrix wheelSpeeds = H_inv.scale(1 / r).mult(twist);
        return wheelSpeeds;
    }
}
