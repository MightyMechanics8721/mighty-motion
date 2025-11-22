package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Geometry.Path;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Models.MecanumKinematicModel;

@Config
public class GeometricController {
    public static double lookAheadXY = 5;
    public static double lookAheadTheta = 10;
    public boolean useStaticHeading = false;
    public PoseController poseControl
            = new PoseController(
            Drivetrain.POSE_CONSTANTS.xPIDConstants,
            Drivetrain.POSE_CONSTANTS.yPIDConstants,
            Drivetrain.POSE_CONSTANTS.headingPIDConstants
    );
    int lastIndexXY = 0;
    int lastIndexTheta = 0;
    int lastLookaheadXY = 0;
    int lastLookaheadTheta = 0;

    public GeometricController() {
    }


    static double[] calcCircleLineIntersection(
            double xPos,
            double yPos,
            int i,
            double radius,
            double[][] wayPoints
    ) {
        double x0 = wayPoints[i][0];
        double y0 = wayPoints[i][1];
        double x1 = wayPoints[i + 1][0];
        double y1 = wayPoints[i + 1][1];

        double dx = x1 - x0;
        double dy = y1 - y0;

        double fx = x0 - xPos;
        double fy = y0 - yPos;

        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - radius * radius;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            // no intersection
            return new double[]{-99999, -99999};
        }

        double sqrtDisc = Math.sqrt(discriminant);
        double t1 = (-b + sqrtDisc) / (2 * a);
        double t2 = (-b - sqrtDisc) / (2 * a);

        double t;

        // pick any valid t in [0,1]
        boolean t1Valid = t1 >= 0 && t1 <= 1;
        boolean t2Valid = t2 >= 0 && t2 <= 1;

        if (t1Valid && t2Valid) {
            t = Math.max(t1, t2);
        } else if (t1Valid) {
            t = t1;
        } else if (t2Valid) {
            t = t2;
        } else {
            // both intersections are outside the segment
            return new double[]{-99999, -99999};
        }

        double pX = x0 + dx * t;
        double pY = y0 + dy * t;

        return new double[]{pX, pY};
    }

    // To be more consistent with PoseController, pass in SimpleMatrix Pose here,
    // then first two lines grab the x and y position from the pose.
    public SimpleMatrix calculate(SimpleMatrix pose, Path path) {
        double x = pose.get(0, 0);
        double y = pose.get(1, 0);
        // Assuming you read my Path class feedback, might want to change to path.getWaypoints() :-)
        double[][] xyPoints = path.getWaypoints();

        // rename to xyPoints to be consistent with thetaPoints!
        LinkedHashSet<double[]> furthestIntersectionPointXY = new LinkedHashSet<>();
        LinkedHashSet<double[]> furthestIntersectionPointTheta = new LinkedHashSet<>();
        for (int i = lastIndexXY; i < xyPoints.length - 1; i++) {
            double[] intersection = calcCircleLineIntersection(x, y, i, lookAheadXY, xyPoints);
            if (!Arrays.equals(intersection, new double[]{-99999, -99999})) {
                furthestIntersectionPointXY.add(intersection);
                lastLookaheadXY = i;
            }
        }
        lastIndexXY = lastLookaheadXY;
        // You may want to have a 'lastLookaheadXY' AND a 'lastLookaheadTheta' as they may not be
        // the same
        // Make sure to reset them both in the reset function
        for (int i = lastIndexTheta; i < xyPoints.length - 1; i++) {
            double[] intersection = calcCircleLineIntersection(x, y, i, lookAheadTheta, xyPoints);
            if (!Arrays.equals(intersection, new double[]{-99999, -99999})) {
                furthestIntersectionPointTheta.add(intersection);
                lastLookaheadTheta = i;
            }
        }
        lastIndexTheta = lastLookaheadTheta;
        if (furthestIntersectionPointXY.isEmpty() || furthestIntersectionPointTheta.isEmpty()) {
            SimpleMatrix desiredPose = new SimpleMatrix(
                    new double[]{
                            path.getFinalPoint()[0],
                            path.getFinalPoint()[1],
                            path.finalHeading
                    }
            );
            return desiredPose;
        }


        ArrayList<double[]> thetaArray = new ArrayList<>(furthestIntersectionPointTheta);
        ArrayList<double[]> posArray = new ArrayList<>(furthestIntersectionPointXY);

        // Rename this to furthestIntersectionPointTheta or something more descriptive.
        // Also do the same for furthestIntersectionPointXY!!!
        double[] furthestPointTheta = thetaArray.get(thetaArray.size() - 1);

        double desiredTheta;

        if (path.useStaticHeading) {
            desiredTheta = path.finalHeading;
        } else {
            // You get furthestIntersectionPointTheta above for a reason. Use it to make this
            // more readable!
            desiredTheta = Math.atan2(
                    (furthestPointTheta[1] - y),
                    (furthestPointTheta[0] - x)
            );
            if (path.reverse) {
                if (Math.signum(desiredTheta) == -1) {
                    desiredTheta += Math.PI;
                } else if (Math.signum(desiredTheta) == 0) {
                    desiredTheta -= Math.PI;
                }
            }

        }

        // You are calling to thetaArray in the position spots!
        // See above: get the xy furthest point (furthestIntersectionPointXY)
        // and use that!
        SimpleMatrix desiredPose = new SimpleMatrix(
                new double[]{
                        posArray.get(posArray.size() - 1)[0],
                        posArray.get(posArray.size() - 1)[1],
                        desiredTheta
                }
        );
        return desiredPose;
    }

    public void resetLookAhead() {
        lastLookaheadXY = 0;
        lastLookaheadTheta = 0;
        lastIndexTheta = 0;
        lastIndexXY = 0;
    }
}