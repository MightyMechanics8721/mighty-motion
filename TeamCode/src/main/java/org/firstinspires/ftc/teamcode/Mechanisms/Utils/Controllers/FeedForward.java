package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;

public class FeedForward {
    private FFConstants feedforwardConstants;


    /***
     *
     * @param kV (double) Voltage constant
     * @param kA (double) Acceleration constant
     * @param kS (double) Static constant
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

