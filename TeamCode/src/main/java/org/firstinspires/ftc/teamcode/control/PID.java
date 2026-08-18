package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    /** Integral clamp. */
    public static double maxIntegralSum = 1000;
    public double eIntegralSum;
    public double eDerivative;
    public double ePrev;
    public functionType type;
    public ElapsedTime timer = new ElapsedTime();
    private final PIDConstants pidConstants;
    private boolean firstRun = true;

    public PID(PIDConstants pidConstants, functionType type) {
        this.pidConstants = pidConstants;
        this.type = type;
        reset();
    }

    /** Clears integral and derivative state. */
    public void reset() {
        eIntegralSum = 0;
        eDerivative = 0;
        ePrev = 0;
        firstRun = true;
        timer.reset();
    }

    /**
     * One PID update. The I and D terms are skipped on the first call and whenever dt is zero.
     */
    public double calculate(double target, double currentState) {
        double error = target - currentState;
        double dt = timer.seconds();
        timer.reset();

        if (firstRun || dt <= 0) {
            eDerivative = 0;
            firstRun = false;
        } else {
            accumulateIntegral(error, dt);
            eDerivative = (error - ePrev) / dt;
        }
        ePrev = error;
        return proportional(error) + (pidConstants.kI * eIntegralSum)
                + (pidConstants.kD * eDerivative);
    }

    /**
     * One PID update, taking D from the measured speed instead of differentiating the error.
     */
    public double calculate(double target, double currentState, double currentSpeed) {
        double error = target - currentState;
        double dt = timer.seconds();
        timer.reset();

        if (firstRun || dt <= 0) {
            firstRun = false;
        } else {
            accumulateIntegral(error, dt);
        }
        ePrev = error;
        eDerivative = -currentSpeed;
        return proportional(error) + (pidConstants.kI * eIntegralSum)
                + (pidConstants.kD * -currentSpeed);
    }

    /** Adds error * dt to the integral, clamped to maxIntegralSum. */
    private void accumulateIntegral(double error, double dt) {
        eIntegralSum += error * dt;
        eIntegralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, eIntegralSum));
    }

    /** kP * error, or kP * sqrt(|error|) * sign(error) in SQRT mode. */
    private double proportional(double error) {
        if (type == functionType.LINEAR) {
            return pidConstants.kP * error;
        }
        return pidConstants.kP * Math.sqrt(Math.abs(error)) * Math.signum(error);
    }

    public enum functionType {
        LINEAR,
        SQRT
    }
}
