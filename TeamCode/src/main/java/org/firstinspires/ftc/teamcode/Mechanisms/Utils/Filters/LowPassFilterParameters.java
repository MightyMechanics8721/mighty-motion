package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Filters;

/**
 * A simple class to store parameters for the Low Pass Filter.
 * Right now, it only holds the smoothing factor, but you can
 * add more parameters later if needed.
 */
public class LowPassFilterParameters {
    public double smoothingFactor;  // smoothing factor (0 < value < 1)

    /**
     * Constructor
     *
     * @param smoothingFactor initial smoothing factor
     */
    public LowPassFilterParameters(double smoothingFactor) {
        this.smoothingFactor = smoothingFactor;
    }
}
