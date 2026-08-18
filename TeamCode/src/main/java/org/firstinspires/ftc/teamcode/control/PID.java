package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    /**
     * Largest |integral| term allowed, to stop windup while a mechanism is stalled or saturated.
     */
    public static double maxIntegralSum = 1000;
    public double eIntegralSum;
    public double eDerivative;
    public double ePrev;
    public functionType type;
    public ElapsedTime timer = new ElapsedTime();
    private final PIDConstants pidConstants;
    /**
     * True until the first calculate() call, which has no previous error and no meaningful dt.
     */
    private boolean firstRun = true;

    public PID(PIDConstants pidConstants, functionType type) {
        this.pidConstants = pidConstants;
        this.type = type;
        reset();
    }

    /**
     * Clears accumulated integral and derivative state. Call before re-using a controller for a new
     * motion, otherwise error from the previous one leaks into it.
     */
    public void reset() {
        eIntegralSum = 0;
        eDerivative = 0;
        ePrev = 0;
        firstRun = true;
        timer.reset();
    }

    /**
     * Runs one PID update and returns the control effort.
     * <p>
     * The integral term accumulates {@code error * dt} and is clamped to +/- maxIntegralSum. The
     * derivative term is {@code (error - previousError) / dt}. On the first call, and whenever dt
     * is not positive, the I and D terms are skipped and only the proportional term applies.
     *
     * @param target the setpoint
     * @param currentState the measured value
     *
     * @return the control effort, in whatever units the gains were tuned for
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
     * Runs one PID update using a measured speed for the derivative term instead of differentiating
     * the error.
     * <p>
     * Taking D from the measurement rather than the error avoids the spike a normal derivative
     * gives when the setpoint jumps, which matters for a mechanism that is commanded to a new
     * target in one step.
     *
     * @param target the setpoint
     * @param currentState the measured value
     * @param currentSpeed the measured rate of change of the state
     *
     * @return the control effort, in whatever units the gains were tuned for
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

    /**
     * Adds this cycle's contribution to the running integral of error over time, clamped so a
     * stalled mechanism cannot wind it up without bound.
     */
    private void accumulateIntegral(double error, double dt) {
        eIntegralSum += error * dt;
        eIntegralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, eIntegralSum));
    }

    /**
     * The proportional term. LINEAR gives {@code kP * error}; SQRT gives
     * {@code kP * sqrt(|error|) * sign(error)}, which has proportionally more authority at small
     * errors and softens the response to large ones.
     */
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
