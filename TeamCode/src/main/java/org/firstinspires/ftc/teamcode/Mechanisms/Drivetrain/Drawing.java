package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Planners.ProfiledPathGenerator.generatePath;

import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.roadrunner.Vector2d;

import org.ejml.simple.SimpleMatrix;

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
        double[][] splinePoints = generatePath(new SimpleMatrix(points));
        for (int i = 0; i < splinePoints[0].length; i++) {
            canvas.strokeCircle(splinePoints[0][i], splinePoints[1][i], POINT_RADIUS);
        }
    }
}
