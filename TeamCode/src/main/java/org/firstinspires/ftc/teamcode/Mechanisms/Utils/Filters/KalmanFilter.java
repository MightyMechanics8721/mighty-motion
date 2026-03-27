package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Filters;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * Kalman-filtered localizer that fuses: - GoBILDA Pinpoint odometry  (predict step — fast, drifts
 * over time) - Limelight 3A MegaTag2 AprilTags (correct step — absolute, intermittent)
 *
 * Architecture (per the CTRL ALT FTC / zero-drift approach): Three independent 1D Kalman filters
 * for x, y, heading. Predict every loop with Pinpoint deltas. Correct only when Limelight has a
 * valid, trustworthy pose.
 *
 * Units: internal state is in MILLIMETERS and RADIANS (matching Pinpoint). Limelight reports meters
 * → converted on ingress. Public getters return inches and degrees for convenience.
 *
 * Usage: PinpointLimelightKalman localizer = new PinpointLimelightKalman(hardwareMap);
 * localizer.setStartPose(startX_mm, startY_mm, startHeading_rad);
 *
 * // in your loop: localizer.update(); double x = localizer.getXInches(); double y =
 * localizer.getYInches(); double heading = localizer.getHeadingDegrees();
 */
public class KalmanFilter {
    // Process noise Q — how much odometry drifts per loop cycle.
    // Higher = trust odometry less = camera corrections hit harder.
    // Start here and tune empirically.
    private static final double Q_POSITION = 2.0;   // in² per cycle
    private static final double Q_HEADING = 0.002;  // rad² per cycle
    // Measurement noise R — how noisy the Limelight pose is.
    // Lower = trust camera more. Increase if you see jitter on corrections.
    private static final double R_POSITION = 50.0;   // in²
    private static final double R_HEADING = 0.05;   // rad²
    // Initial covariance — set high so the first good reading dominates.
    private static final double INITIAL_P = 100.0;
    private static final double MAX_CORRECTION_MM = 500.0;  // reject if jump is unreasonably large

    private static final String LIMELIGHT_NAME = "limelight";
    private static final int LL_PIPELINE = 1; // AprilTag pipeline index
    private final Limelight3A limelight;
    // ---- Kalman filters (x in in, y in in, heading in rad) ----
    private final KalmanFilter1D kfX;
    private final KalmanFilter1D kfY;
    private final KalmanFilter1D kfHeading;
    Drivetrain drivetrain;
    // ---- Previous Pinpoint readings (for computing deltas) ----
    private double prevPinpointX;
    private double prevPinpointY;
    private double prevPinpointHeading;
    private boolean firstLoop = true;

    // ================================================================
    //  Construction
    // ================================================================

    public KalmanFilter(HardwareMap hwMap) {

        // --- Limelight ---
        limelight = hwMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(LL_PIPELINE);
        limelight.start();

        // --- Kalman filters ---
        kfX = new KalmanFilter1D(0, INITIAL_P, Q_POSITION, R_POSITION);
        kfY = new KalmanFilter1D(0, INITIAL_P, Q_POSITION, R_POSITION);
        kfHeading = new KalmanFilter1D(0, INITIAL_P, Q_HEADING, R_HEADING);
        Drivetrain.initialize(hwMap);
        drivetrain = Drivetrain.getInstance();
    }

    // ================================================================
    //  Main loop
    // ================================================================

    public void update() {
        // --- 1. Read Pinpoint ---

        double ppX = drivetrain.state.get(0, 0);
        double ppY = drivetrain.state.get(1, 0);
        double ppHeading = drivetrain.state.get(2, 0);

        if (firstLoop) {
            prevPinpointX = ppX;
            prevPinpointY = ppY;
            prevPinpointHeading = ppHeading;
            firstLoop = false;
            return;
        }

        // --- 2. Compute odometry deltas ---
        double dx = ppX - prevPinpointX;
        double dy = ppY - prevPinpointY;
        double dHeading = ppHeading - prevPinpointHeading;

        prevPinpointX = ppX;
        prevPinpointY = ppY;
        prevPinpointHeading = ppHeading;

        // --- 3. PREDICT with Pinpoint deltas ---
        kfX.predict(dx);
        kfY.predict(dy);
        kfHeading.predict(dHeading);

        // --- 4. Feed Pinpoint heading to Limelight for MegaTag2 ---
        double headingDeg = Math.toDegrees(drivetrain.state.get(2, 0));
        limelight.updateRobotOrientation(headingDeg);

        // --- 5. CORRECT with Limelight (when available) ---
        LLResult result = limelight.getLatestResult();
        if (isLimelightValid(result)) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                // Limelight returns meters → convert to IN
                double llX = botpose.getPosition().x * 39.37;
                double llY = botpose.getPosition().y * 39.37;
                double llYawRad = Math.toRadians(
                        botpose.getOrientation().getYaw(AngleUnit.DEGREES));

                // Sanity check: reject corrections that are unreasonably far from estimate
                if (Math.abs(llX - kfX.getEstimate()) < MAX_CORRECTION_MM
                        && Math.abs(llY - kfY.getEstimate()) < MAX_CORRECTION_MM) {

                    kfX.correct(llX);
                    kfY.correct(llY);
                    kfHeading.correctAngle(llYawRad);
                }
            }
        }
    }

    // ================================================================
    //  Limelight validation
    // ================================================================

    private boolean isLimelightValid(LLResult result) {
        if (result == null) return false;
        if (!result.isValid()) return false;

        // getBotpose_MT2() returns null if no tags are in view
        Pose3D pose = result.getBotpose_MT2();
        if (pose == null) return false;

        // Reject if the pose is clearly at the origin (no real data)
        double x = pose.getPosition().x;
        double y = pose.getPosition().y;
        if (x == 0.0 && y == 0.0) return false;

        return true;
    }

    // ================================================================
    //  Public getters
    // ================================================================

    /**
     * Filtered X in millimeters (field frame).
     */
    public double getXMm() {
        return kfX.getEstimate();
    }

    /**
     * Filtered Y in millimeters (field frame).
     */
    public double getYMm() {
        return kfY.getEstimate();
    }

    /**
     * Filtered heading in radians.
     */
    public double getHeadingRad() {
        return kfHeading.getEstimate();
    }

    /**
     * Filtered X in inches.
     */
    public double getXInches() {
        return kfX.getEstimate() / 25.4;
    }

    /**
     * Filtered Y in inches.
     */
    public double getYInches() {
        return kfY.getEstimate() / 25.4;
    }

    /**
     * Filtered heading in degrees.
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(kfHeading.getEstimate());
    }

    /**
     * Get a Pose2D (mm, mm, degrees) suitable for Pinpoint setPosition or telemetry.
     */
    public Pose2D getPose2D() {
        return new Pose2D(
                DistanceUnit.MM, kfX.getEstimate(), kfY.getEstimate(),
                AngleUnit.RADIANS, kfHeading.getEstimate()
        );
    }

    /**
     * Kalman gain for X — useful for telemetry / debugging.
     */
    public double getKalmanGainX() {
        return kfX.getKalmanGain();
    }

    /**
     * Kalman gain for Y.
     */
    public double getKalmanGainY() {
        return kfY.getKalmanGain();
    }

    // ================================================================
    //  Configuration helpers
    // ================================================================

    /**
     * Set the starting pose in field coordinates. Call once before the main loop (e.g. at the start
     * of autonomous).
     *
     * @param xMm starting X in mm
     * @param yMm starting Y in mm
     * @param headingRad starting heading in radians
     */
    public void setStartPose(double xMm, double yMm, double headingRad) {
        kfX.setState(xMm);
        kfY.setState(yMm);
        kfHeading.setState(headingRad);

        drivetrain.setInitialPose(
                xMm, yMm, headingRad
        );

        prevPinpointX = xMm;
        prevPinpointY = yMm;
        prevPinpointHeading = headingRad;
        firstLoop = false;
    }

    /**
     * Live-tune Q and R (e.g. from gamepad during testing).
     */
    public void setProcessNoise(double qPos, double qHeading) {
        kfX.setProcessNoise(qPos);
        kfY.setProcessNoise(qPos);
        kfHeading.setProcessNoise(qHeading);
    }

    public void setMeasurementNoise(double rPos, double rHeading) {
        kfX.setMeasurementNoise(rPos);
        kfY.setMeasurementNoise(rPos);
        kfHeading.setMeasurementNoise(rHeading);
    }

    /**
     * Access the Limelight directly.
     */
    public Limelight3A getLimelight() {
        return limelight;
    }
}