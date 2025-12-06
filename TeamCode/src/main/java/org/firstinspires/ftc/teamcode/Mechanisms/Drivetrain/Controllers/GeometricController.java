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


    private static double[] closestPointOnSegment(
            double px, double py,
            double x0, double y0,
            double x1, double y1
    ) {
        double dx = x1 - x0;
        double dy = y1 - y0;
        double segLenSq = dx * dx + dy * dy;

        double t;
        if (segLenSq == 0) {
            t = 0.0; // segment is a point
        } else {
            t = ((px - x0) * dx + (py - y0) * dy) / segLenSq;
            t = Math.max(0.0, Math.min(1.0, t)); // clamp to [0, 1]
        }

        double projX = x0 + t * dx;
        double projY = y0 + t * dy;
        return new double[]{projX, projY};
    }

    private static double[] findClosestPointOnPath(
            double px, double py,
            double[][] wayPoints
    ) {
        double bestX = wayPoints[0][0];
        double bestY = wayPoints[0][1];
        double bestDistSq = Double.POSITIVE_INFINITY;

        for (int i = 0; i < wayPoints.length - 1; i++) {
            double[] cand = closestPointOnSegment(
                    px, py,
                    wayPoints[i][0], wayPoints[i][1],
                    wayPoints[i + 1][0], wayPoints[i + 1][1]
            );

            double dx = cand[0] - px;
            double dy = cand[1] - py;
            double distSq = dx * dx + dy * dy;

            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestX = cand[0];
                bestY = cand[1];
            }
        }

        return new double[]{bestX, bestY};
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

    public SimpleMatrix calculate(SimpleMatrix pose, Path path) {
        double x = pose.get(0, 0);
        double y = pose.get(1, 0);
        double[][] xyPoints = path.getWaypoints();

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

        for (int i = lastIndexTheta; i < xyPoints.length - 1; i++) {
            double[] intersection = calcCircleLineIntersection(x, y, i, lookAheadTheta, xyPoints);
            if (!Arrays.equals(intersection, new double[]{-99999, -99999})) {
                furthestIntersectionPointTheta.add(intersection);
                lastLookaheadTheta = i;
            }
        }
        lastIndexTheta = lastLookaheadTheta;
        // If we didn't find any lookahead intersections, fall back to nearest point
        double[] fallbackPoint = null;
        if (furthestIntersectionPointXY.isEmpty() || furthestIntersectionPointTheta.isEmpty()) {
            fallbackPoint = findClosestPointOnPath(x, y, xyPoints);
        }

        ArrayList<double[]> thetaArray = new ArrayList<>(furthestIntersectionPointTheta);
        ArrayList<double[]> posArray = new ArrayList<>(furthestIntersectionPointXY);

        double[] positionPoint;
        double[] thetaPoint;

        // position (XY)
        if (posArray.isEmpty()) {
            if (fallbackPoint == null) {
                fallbackPoint = findClosestPointOnPath(x, y, xyPoints);
            }
            positionPoint = fallbackPoint;
        } else {
            positionPoint = posArray.get(posArray.size() - 1);
        }

        // heading point (for theta lookahead)
        if (thetaArray.isEmpty()) {
            if (fallbackPoint == null) {
                fallbackPoint = findClosestPointOnPath(x, y, xyPoints);
            }
            thetaPoint = fallbackPoint;
        } else {
            thetaPoint = thetaArray.get(thetaArray.size() - 1);
        }

        double desiredTheta;
        if (path.useStaticHeading) {
            desiredTheta = path.finalHeading;
        } else {
            desiredTheta = Math.atan2(
                    (thetaPoint[1] - y),
                    (thetaPoint[0] - x)
            );
            if (path.reverse) {
                if (Math.signum(desiredTheta) == -1) {
                    desiredTheta += Math.PI;
                } else if (Math.signum(desiredTheta) == 0) {
                    desiredTheta -= Math.PI;
                }
            }
        }

        SimpleMatrix desiredPose = new SimpleMatrix(
                new double[]{
                        positionPoint[0],
                        positionPoint[1],
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