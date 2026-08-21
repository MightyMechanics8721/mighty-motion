package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.util.Utils;

import org.ejml.simple.SimpleMatrix;

public class GeometricController {
    public int lastIndexXY = 0;
    int lastIndexTheta = 0;
    int lastLookaheadXY = 0;
    int lastLookaheadTheta = 0;
    private double lookAheadXY;
    private double lookAheadTheta;
    private TelemetryPacket packet = new TelemetryPacket();

    public GeometricController(double positionLookahead, double headingLookahead) {
        this.lookAheadXY = positionLookahead;
        this.lookAheadTheta = headingLookahead;
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


    /**
     * Point where the lookahead circle leaves segment i, or null when it does not cross it.
     */
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
            return null;
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
            return null;
        }

        double pX = x0 + dx * t;
        double pY = y0 + dy * t;

        return new double[]{pX, pY};
    }

    public void setTelemetry(TelemetryPacket packet) {
        this.packet = packet;
    }

    /**
     * Closest point to (px, py) anywhere on the path.
     */
    private static double[] findClosestPointOnPath(double px, double py, double[][] wayPoints) {
        double[] best = {wayPoints[0][0], wayPoints[0][1]};
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
                best = cand;
            }
        }
        return best;
    }

    public SimpleMatrix calculate(SimpleMatrix pose, Path path) {
        double x = pose.get(0, 0);
        double y = pose.get(1, 0);
        double[][] xyPoints = path.getWaypoints();

        ArrayList<double[]> posArray = new ArrayList<>();
        ArrayList<double[]> thetaArray = new ArrayList<>();
        for (int i = lastIndexXY; i < xyPoints.length - 1; i++) {
            double[] intersection = calcCircleLineIntersection(x, y, i, lookAheadXY, xyPoints);
            if (intersection != null) {
                posArray.add(intersection);
                lastLookaheadXY = i;
            }
        }
        lastIndexXY = lastLookaheadXY;

        for (int i = lastIndexTheta; i < xyPoints.length - 1; i++) {
            double[] intersection = calcCircleLineIntersection(x, y, i, lookAheadTheta, xyPoints);
            if (intersection != null) {
                thetaArray.add(intersection);
                lastLookaheadTheta = i;
            }
        }
        lastIndexTheta = lastLookaheadTheta;

        double[] positionPoint = posArray.isEmpty()
                ? findClosestPointOnPath(x, y, xyPoints)
                : posArray.get(posArray.size() - 1);
        double[] thetaPoint = thetaArray.isEmpty()
                ? path.getFinalPoint()
                : thetaArray.get(thetaArray.size() - 1);

        double desiredTheta;
        if (path.useStaticHeading) {
            desiredTheta = path.finalHeading;
        } else {
            double dxAim = thetaPoint[0] - x;
            double dyAim = thetaPoint[1] - y;
            desiredTheta = (dxAim == 0 && dyAim == 0)
                    ? pose.get(2, 0)
                    : Math.atan2(dyAim, dxAim);
            if (path.reverse) {
                desiredTheta = Utils.angleWrap(desiredTheta + Math.PI);
            }
        }

        SimpleMatrix desiredPose = new SimpleMatrix(
                new double[]{
                        positionPoint[0],
                        positionPoint[1],
                        desiredTheta
                }
        );

        Canvas canvas = this.packet.fieldOverlay();
        Drawing.drawPoint(positionPoint, canvas, "purple");
        Drawing.drawPoint(thetaPoint, canvas, "orange");

        return desiredPose;
    }

    public void resetLookAhead() {
        lastLookaheadXY = 0;
        lastLookaheadTheta = 0;
        lastIndexTheta = 0;
        lastIndexXY = 0;
    }
}