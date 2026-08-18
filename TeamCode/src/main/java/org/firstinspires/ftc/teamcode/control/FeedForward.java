package org.firstinspires.ftc.teamcode.control;

public class FeedForward {
    private FFConstants feedforwardConstants;

    /***
     * @param feedforwardConstants kV, kA, kS, Voltage constant, acceleration constant, static constant (double)
     */
    public FeedForward(FFConstants feedforwardConstants) {
        this.feedforwardConstants = feedforwardConstants;
    }

    /***
     *
     * @param desiredVelocity (double) The target velocity of our bot
     * @param desiredAcceleration (double) The target acceleration of our bot
     * @return (double) The voltage calculation to send to the motors
     */
    public double calculate(double desiredVelocity, double desiredAcceleration) {
        return this.feedforwardConstants.kV * desiredVelocity + this.feedforwardConstants.kA * desiredAcceleration + this.feedforwardConstants.kS * Math.signum(desiredVelocity);
    }

    public void setGains(FFConstants feedforwardConstants) {
        this.feedforwardConstants = feedforwardConstants;
    }

    public FFConstants getFeedforwardConstants() {
        return this.feedforwardConstants;
    }
}

