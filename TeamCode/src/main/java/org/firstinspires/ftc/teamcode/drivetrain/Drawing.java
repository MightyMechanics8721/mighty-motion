package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.ejml.simple.SimpleMatrix;

public final class Drawing {
    private Drawing() {
    }

    public static void drawRobot(SimpleMatrix state, Canvas canvas, String color) {
        final double ROBOT_RADIUS = 7.5;

        double xPosition = state.get(0, 0); // x
        double yPosition = state.get(1, 0); // y
        double heading = state.get(2, 0); // heading

        canvas.setStrokeWidth(1);
        canvas.setStroke(color);
        canvas.strokeCircle(xPosition, yPosition, ROBOT_RADIUS);

        canvas.strokeLine(
                xPosition, yPosition, Math.cos(heading) * ROBOT_RADIUS + xPosition,
                Math.sin(heading) * ROBOT_RADIUS + yPosition
        );
    }

    public static void drawPath(double[][] points, Canvas canvas, String color) {
        canvas.setStrokeWidth(1);
        canvas.setStroke(color);
        for (int i = 0; i < points.length - 1; i++) {
            canvas.strokeLine(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1]);
        }
    }

    public static void drawCircle(
            double[] point, Canvas canvas, double radius, String color,
            boolean fill
    ) {
        canvas.setStrokeWidth(1);
        canvas.setStroke(color);
        if (fill) {
            canvas.setFill(color);
            canvas.fillCircle(point[0], point[1], radius);
        } else {
            canvas.strokeCircle(point[0], point[1], radius);
        }
    }

    public static void drawLine(
            double[] startPoint, double[] endPoint, Canvas canvas, String color
    ) {
        canvas.setStrokeWidth(1);
        canvas.setStroke(color);
        canvas.strokeLine(
                startPoint[0], startPoint[1], endPoint[0], endPoint[1]
        );
    }

    public static void drawPoint(double[] point, Canvas canvas, String color) {
        drawCircle(point, canvas, 1.5, color, true);
    }

}
