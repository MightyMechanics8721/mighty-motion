package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Controllers.GeometricController;

public final class Drawing {
    private Drawing() {
    }


    public static void drawRobot(Canvas canvas, SimpleMatrix state) {
        final double ROBOT_RADIUS = 7.5;

        double x_position = state.get(0, 0); // x
        double y_position = state.get(1, 0); // y
        double heading = state.get(2, 0); // heading

        canvas.setStrokeWidth(1);
        canvas.strokeCircle(x_position, y_position, ROBOT_RADIUS);

        canvas.strokeLine(
                x_position, y_position, Math.cos(heading) * ROBOT_RADIUS + x_position,
                Math.sin(heading) * ROBOT_RADIUS + y_position
        );
    }

    public static void drawPoint(Canvas canvas, SimpleMatrix state, double[][] points) {
        final double ROBOT_RADIUS = 7.5;

        double x_position = state.get(0, 0); // x
        double y_position = state.get(1, 0); // y
        double heading = state.get(2, 0); // heading

        canvas.setStrokeWidth(1);
        canvas.strokeCircle(x_position, y_position, ROBOT_RADIUS);

        canvas.strokeLine(
                x_position, y_position, Math.cos(heading) * ROBOT_RADIUS + x_position,
                Math.sin(heading) * ROBOT_RADIUS + y_position
        );
        final double POINT_RADIUS = 1;
        canvas.setStrokeWidth(1);
        for (int i = 0; i < points.length - 1; i++) {
            canvas.strokeLine(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1]);
        }

        // Draw the pos point RED
        // Draw the theta point BLUE
        canvas.setStrokeWidth(2);
        canvas.setStroke("red");
        canvas.strokeCircle(GeometricController.geoPosPointX, GeometricController.geoPosPointY, 2);

        canvas.setStrokeWidth(2);
        canvas.setStroke("blue");
        canvas.strokeCircle(GeometricController.geoThetaX, GeometricController.geoThetaY, 2);

        canvas.setStrokeWidth(2);
        canvas.setStroke("green");
        canvas.strokeCircle(
                Drivetrain.stoppingDistancePose.get(0, 0),
                Drivetrain.stoppingDistancePose.get(1, 0), 2
        );
    }

    public static void drawTarget(Canvas canvas, SimpleMatrix state) {

        double x_position = state.get(0, 0); // x
        double y_position = state.get(1, 0); // y
        double heading = state.get(2, 0); // heading

        canvas.setStrokeWidth(2);
        canvas.setFill("red");
        canvas.strokeCircle(x_position, y_position, 2);
    }
}
