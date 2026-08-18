package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {
    private final double TICKS_PER_RADIAN; // (ticks/rad)
    private DcMotorEx encoder;
    private int storedPos = 0; // (ticks) reading at the last reset

    /**
     * @param ticksPerRevolution encoder ticks per shaft revolution (ticks/rev)
     */
    public Encoder(DcMotorEx encoder, double ticksPerRevolution) {
        this.encoder = encoder;
        this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.TICKS_PER_RADIAN = ticksPerRevolution / (2 * Math.PI);
    }

    //    public Encoder(DcMotorEx encoder, double ticksPerRevolution, boolean resetEncoder) {
    //        this.encoder = encoder;
    //        this.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //        storedPos = this.encoder.getCurrentPosition();
    //        this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //        this.TICKS_PER_RADIAN = ticksPerRevolution / (2 * Math.PI);
    //    }

    /** Zeroes the reading in software. */
    public void reset() {
        storedPos = encoder.getCurrentPosition();
    }

    /**
     * @return shaft position since the last reset (ticks)
     */
    public int getCurrentPosition() {
        return encoder.getCurrentPosition() - storedPos;
    }

    /**
     * @return shaft velocity (rad/s)
     */
    public double getVelocity() {
        return this.encoder.getVelocity() / this.TICKS_PER_RADIAN;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        encoder.setDirection(direction);
    }

}
