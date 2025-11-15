package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.roadrunner.Vector2d;

import org.ejml.simple.SimpleMatrix;

public final class Drawing {
    private Drawing() {
    }


    public static void drawRobot(Canvas canvas, SimpleMatrix state) {
        final double ROBOT_RADIUS = 9;

        double x_position = state.get(0, 0); // x
        double y_position = state.get(1, 0); // y
        double heading_position = state.get(2, 0); // heading

        canvas.setStrokeWidth(1);
        canvas.strokeCircle(x_position, y_position, heading_position);

        //        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        //        Vector2d p1 = t.position.plus(halfv);
        //        Vector2d p2 = p1.plus(halfv);


        //        canvas.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
