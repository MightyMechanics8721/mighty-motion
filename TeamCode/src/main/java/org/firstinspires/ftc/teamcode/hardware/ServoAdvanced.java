package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

/** Servo wrapper that skips writes smaller than a tolerance. */
public class ServoAdvanced {
    private final Servo servo;
    private double servoTolerance = 0.001;
    private Double lastPos = null;

    public ServoAdvanced(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double newPos) {
        if (lastPos == null || Math.abs(newPos - lastPos) > servoTolerance) {
            servo.setPosition(newPos);
            lastPos = newPos;
        }
    }

    /**
     * @return the last position commanded, or the servo's own reading before anything was
     * commanded
     */
    public double getPosition() {
        return lastPos == null ? servo.getPosition() : lastPos;
    }

    public double getTolerance() {
        return servoTolerance;
    }

    public void setTolerance(double servoTolerance) {
        this.servoTolerance = servoTolerance;
    }
}

