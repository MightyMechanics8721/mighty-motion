package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Filters;

/**
 * Simple 1D Kalman Filter.
 *
 * Predict step:  uses odometry delta as the control input. Correct step:  uses an absolute sensor
 * (Limelight AprilTag) as the measurement.
 *
 * We run three of these in parallel — one each for x, y, and heading.
 */
public class KalmanFilter1D {

    private double x;   // state estimate
    private double p;   // estimate covariance (uncertainty)
    private double q;   // process noise — how much we distrust odometry per cycle
    private double r;   // measurement noise — how much we distrust the camera

    /**
     * @param initialState starting value (e.g. 0 or starting field coordinate)
     * @param initialCovariance starting uncertainty — set high to let first measurement dominate
     * @param processNoise Q — odometry noise per cycle. Higher = trust odo less.
     * @param measurementNoise R — camera noise per measurement. Higher = trust camera less.
     */
    public KalmanFilter1D(
            double initialState, double initialCovariance,
            double processNoise, double measurementNoise
    ) {
        this.x = initialState;
        this.p = initialCovariance;
        this.q = processNoise;
        this.r = measurementNoise;
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    /**
     * Predict step — call every loop with the odometry delta.
     *
     * @param delta change in state from Pinpoint since last cycle (mm or rad)
     */
    public void predict(double delta) {
        x += delta;
        p += q;
    }

    /**
     * Update / correct step — call only when you have a valid Limelight reading.
     *
     * @param measurement absolute position from Limelight (converted to same units as state)
     */
    public void correct(double measurement) {
        double k = p / (p + r);       // Kalman gain
        x = x + k * (measurement - x);
        p = (1.0 - k) * p;
    }

    /**
     * Correct step for angles — handles wraparound at ±π. Use this for the heading filter.
     */
    public void correctAngle(double measuredAngleRad) {
        double error = normalizeAngle(measuredAngleRad - x);
        double k = p / (p + r);
        x = normalizeAngle(x + k * error);
        p = (1.0 - k) * p;
    }

    public double getEstimate() {
        return x;
    }

    public double getCovariance() {
        return p;
    }

    public double getKalmanGain() {
        return p / (p + r);
    }

    public void setProcessNoise(double q) {
        this.q = q;
    }

    public void setMeasurementNoise(double r) {
        this.r = r;
    }

    public void setState(double x) {
        this.x = x;
    }
}