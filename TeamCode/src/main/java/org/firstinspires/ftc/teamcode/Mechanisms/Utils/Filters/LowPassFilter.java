package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Filters;

/**
 * A reusable Low-Pass Filter for smoothing noisy signals. Example: motor velocity, IMU data,
 * distance sensors, etc.
 */
public class LowPassFilter {
    private LowPassFilterParameters params;   // holds smoothing factor
    private double prevFilteredValue;         // last output

    /**
     * Constructor
     *
     * @param params LowPassFilterParameters object with smoothing factor
     */
    public LowPassFilter(LowPassFilterParameters params) {
        this.params = params;
        this.prevFilteredValue = 0.0;
    }

    /**
     * Update filter with a new input value.
     *
     * @param rawValue the latest measurement
     *
     * @return filtered output
     */
    public double update(double rawValue) {
        this.prevFilteredValue = params.smoothingFactor * rawValue
                + (1 - params.smoothingFactor) * prevFilteredValue;
        return this.prevFilteredValue;
    }

    /**
     * Reset filter to a specific value.
     */
    public void reset(double value) {
        this.prevFilteredValue = value;
    }

    /**
     * Dynamically change parameters.
     */
    public void setParameters(LowPassFilterParameters params) {
        this.params = params;
    }
}
