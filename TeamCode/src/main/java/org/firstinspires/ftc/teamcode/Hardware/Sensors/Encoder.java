package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {
    private final double TICKS_PER_RADIAN;
    private DcMotorEx encoder;
    private int storedPos = 0;

    public Encoder(DcMotorEx encoder, double ticksPerRevolution) {
        this.encoder = encoder;
        this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.TICKS_PER_RADIAN = ticksPerRevolution / (2 * Math.PI);
    }


    public void reset() {
        storedPos = encoder.getCurrentPosition();
    }

    public int getCurrentPosition() {
        return encoder.getCurrentPosition() - storedPos;
    }

    public double getVelocity() {
        return this.encoder.getVelocity() / this.TICKS_PER_RADIAN;


    }

    public void setDirection(DcMotorSimple.Direction direction) {
        encoder.setDirection(direction);
    }

}
