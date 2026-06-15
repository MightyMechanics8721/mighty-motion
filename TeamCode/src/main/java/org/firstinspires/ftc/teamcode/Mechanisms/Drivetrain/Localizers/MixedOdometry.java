package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Localizers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

/**
 * MixedOdometry
 *
 * A Kalman filter localizer that fuses Pinpoint dead-reckoning with Limelight 3A AprilTag pose
 * corrections.
 *
 * State vector: [x, y, theta, xDrift, yDrift, thetaDrift]^T - x, y, theta      : true robot pose
 * (inches, radians) - xDrift, yDrift, thetaDrift : estimated systematic bias in Pinpoint readings
 *
 * Usage: MixedOdometry localizer = new MixedOdometry(drivetrain, limelight);
 * localizer.reset(startX, startY, startHeadingRad);
 *
 * // In your loop: localizer.update(); double[] pose = localizer.getPose(); // [x, y, theta]
 */
@Config
public class MixedOdometry {

    // -------------------------------------------------------------------------
    // Tunable initial pose (editable from FTC Dashboard)
    // -------------------------------------------------------------------------
    public static double initial_x_est = 0.0;
    public static double initial_y_est = 0.0;
    public static double initial_heading_est = 0.0;

    // -------------------------------------------------------------------------
    // Tunable initial covariance
    // -------------------------------------------------------------------------
    public static double initial_x_var = 0.1;
    public static double initial_y_var = 0.1;
    public static double initial_heading_var = 0.1;

    public static double initial_x_drift_var = 0.0001;
    public static double initial_y_drift_var = 0.0001;
    public static double initial_heading_drift_var = 0.0001;

    // -------------------------------------------------------------------------
    // Tunable process noise
    // -------------------------------------------------------------------------
    public static double x_process_var = 1E-6;
    public static double y_process_var = 1E-6;
    public static double heading_process_var = 1E-6;

    public static double x_drift_process_var_per_sec = 0.0002636;
    public static double y_drift_process_var_per_sec = 0.00007597;
    public static double heading_drift_process_var_per_sec = 0.00246;

    // -------------------------------------------------------------------------
    // Hardware references
    // -------------------------------------------------------------------------
    private final Drivetrain drivetrain;
    private final Limelight3A limelight;
    // Fixed system matrices (built once in constructor)
    private final SimpleMatrix stateTransitionMatrix; // A  (6x6 identity)
    private final SimpleMatrix controlMatrix;         // B  (6x3)
    private final SimpleMatrix pinpointOutputMatrix;  // Cpp (3x6)
    private final SimpleMatrix limelightOutputMatrix; // Cll (3x6)
    private final SimpleMatrix pinpointMeasCov;       // Rpp (3x3)
    private final ElapsedTime timer = new ElapsedTime();
    // -------------------------------------------------------------------------
    // Filter state
    // -------------------------------------------------------------------------
    private SimpleMatrix poseWithDrift;   // 6x1: [x, y, θ, xd, yd, θd]
    private SimpleMatrix stateCovariance; // 6x6
    private SimpleMatrix prevPinpointMeas; // last known Pinpoint reading

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * @param drivetrain Your Drivetrain instance (must expose a localize() method and a public
     * `state` SimpleMatrix where rows 0-2 are [x, y, theta]).
     * @param limelight Limelight3A hardware device.
     */
    public MixedOdometry(Drivetrain drivetrain, Limelight3A limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        // A: identity — state carries forward unchanged between updates
        stateTransitionMatrix = SimpleMatrix.identity(6);

        // B: Pinpoint delta [dx, dy, dθ] maps onto the first 3 state elements
        controlMatrix = new SimpleMatrix(
                6, 3, true, new double[]{
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0
        }
        );

        // Cpp: Pinpoint observes pose + drift (each axis coupled with its drift)
        pinpointOutputMatrix = new SimpleMatrix(
                3, 6, true, new double[]{
                1, 0, 0, 1, 0, 0,
                0, 1, 0, 0, 1, 0,
                0, 0, 1, 0, 0, 1
        }
        );

        // Cll: Limelight observes true pose only (no drift term)
        limelightOutputMatrix = new SimpleMatrix(
                3, 6, true, new double[]{
                1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0
        }
        );

        // Rpp: Pinpoint measurement noise (diagonal, tuned via Dashboard vars)
        pinpointMeasCov = SimpleMatrix.diag(
                x_process_var,
                y_process_var,
                heading_process_var
        );

        // Seed state from Dashboard-tunable initial pose
        initState(initial_x_est, initial_y_est, initial_heading_est);
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    private static double angleWrapRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    /**
     * Resets the filter to a known starting pose. Call this at the start of auto or whenever you
     * have a trusted pose fix.
     *
     * @param x Starting x position (inches)
     * @param y Starting y position (inches)
     * @param heading Starting heading (radians)
     */
    public void reset(double x, double y, double heading) {
        initState(x, y, heading);
        timer.reset();
    }

    /**
     * Runs one predict→update cycle.  Call this once per loop iteration. Internally pulls a fresh
     * Pinpoint reading from the drivetrain and optionally fuses a Limelight fix if one is
     * available.
     */
    public void update() {
        double loopTime = timer.seconds();
        timer.reset();

        // ---- Read Pinpoint ------------------------------------------------
        drivetrain.localize();
        SimpleMatrix pinpointMeas = drivetrain.state.extractMatrix(0, 3, 0, 1);
        SimpleMatrix changeInPinpoint = pinpointMeas.minus(prevPinpointMeas);
        prevPinpointMeas = pinpointMeas.copy();

        // ---- Process noise (drift grows with time) -------------------------
        SimpleMatrix processNoiseCov = SimpleMatrix.diag(
                x_process_var,
                y_process_var,
                heading_process_var,
                x_drift_process_var_per_sec * loopTime,
                y_drift_process_var_per_sec * loopTime,
                heading_drift_process_var_per_sec * loopTime
        );

        // ==================================================================
        // Prediction step
        // ==================================================================
        SimpleMatrix predictedState =
                stateTransitionMatrix.mult(poseWithDrift)
                                     .plus(controlMatrix.mult(changeInPinpoint));

        SimpleMatrix predictedCovariance =
                stateTransitionMatrix
                        .mult(stateCovariance)
                        .mult(stateTransitionMatrix.transpose())
                        .plus(processNoiseCov);

        // ==================================================================
        // Update step 1: Pinpoint
        // ==================================================================
        SimpleMatrix pinpointS =
                pinpointOutputMatrix
                        .mult(predictedCovariance)
                        .mult(pinpointOutputMatrix.transpose())
                        .plus(pinpointMeasCov);

        SimpleMatrix pinpointKalmanGain =
                predictedCovariance
                        .mult(pinpointOutputMatrix.transpose())
                        .mult(pinpointS.invert());

        SimpleMatrix pinpointInnovation =
                pinpointMeas.minus(pinpointOutputMatrix.mult(predictedState));
        pinpointInnovation.set(2, 0, angleWrapRadians(pinpointInnovation.get(2, 0)));

        SimpleMatrix updatedState =
                predictedState.plus(pinpointKalmanGain.mult(pinpointInnovation));

        // Information-form covariance update: P = inv(inv(Ppred) + C^T inv(R) C)
        SimpleMatrix updatedCovariance =
                predictedCovariance.invert()
                                   .plus(
                                           pinpointOutputMatrix.transpose()
                                                               .mult(pinpointMeasCov.invert())
                                                               .mult(pinpointOutputMatrix)
                                   )
                                   .invert();

        // ==================================================================
        // Update step 2: Limelight (only when a valid fix is available)
        // ==================================================================
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid() && llResult.getBotpose() != null) {
            // Convert metres → inches for x and y; yaw already in radians
            SimpleMatrix limelightMeas = new SimpleMatrix(
                    3, 1, true, new double[]{
                    llResult.getBotpose().getPosition().x * 39.37,
                    llResult.getBotpose().getPosition().y * 39.37,
                    llResult.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)
            }
            );

            double[] std = llResult.getStddevMt1();
            SimpleMatrix limelightMeasCov = SimpleMatrix.diag(
                    Math.pow(std[0] * 39.37, 2),
                    Math.pow(std[1] * 39.37, 2),
                    Math.pow(Math.toRadians(std[5]), 2)
            );

            SimpleMatrix limelightS =
                    limelightOutputMatrix
                            .mult(updatedCovariance)
                            .mult(limelightOutputMatrix.transpose())
                            .plus(limelightMeasCov);

            SimpleMatrix limelightKalmanGain =
                    updatedCovariance
                            .mult(limelightOutputMatrix.transpose())
                            .mult(limelightS.invert());

            SimpleMatrix limelightInnovation =
                    limelightMeas.minus(limelightOutputMatrix.mult(updatedState));
            limelightInnovation.set(2, 0, angleWrapRadians(limelightInnovation.get(2, 0)));

            updatedState =
                    updatedState.plus(limelightKalmanGain.mult(limelightInnovation));

            updatedCovariance =
                    updatedCovariance.invert()
                                     .plus(
                                             limelightOutputMatrix.transpose()
                                                                  .mult(limelightMeasCov.invert())
                                                                  .mult(limelightOutputMatrix)
                                     )
                                     .invert();
        }

        poseWithDrift = updatedState;
        stateCovariance = updatedCovariance;
    }

    /**
     * Returns the current best-estimate pose.
     *
     * @return double[3] = { x (inches), y (inches), heading (radians) }
     */
    public double[] getPose() {
        return new double[]{
                poseWithDrift.get(0, 0),
                poseWithDrift.get(1, 0),
                poseWithDrift.get(2, 0)
        };
    }

    /**
     * @return Estimated x position in inches.
     */
    public double getX() {
        return poseWithDrift.get(0, 0);
    }

    /**
     * @return Estimated y position in inches.
     */
    public double getY() {
        return poseWithDrift.get(1, 0);
    }

    /**
     * @return Estimated heading in radians.
     */
    public double getHeading() {
        return poseWithDrift.get(2, 0);
    }

    /**
     * @return Estimated heading in degrees (convenience).
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(poseWithDrift.get(2, 0));
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * Sends localizer state to FTC Dashboard. Call this after update() if you want live telemetry.
     *
     * @param packet An existing TelemetryPacket to write into.
     */
    public void sendTelemetry(TelemetryPacket packet) {
        packet.put("x pos (in) [KF]", getX());
        packet.put("y pos (in) [KF]", getY());
        packet.put("heading (deg) [KF]", getHeadingDegrees());

        packet.put("x drift (in) [KF]", poseWithDrift.get(3, 0));
        packet.put("y drift (in) [KF]", poseWithDrift.get(4, 0));
        packet.put("heading drift (deg) [KF]", Math.toDegrees(poseWithDrift.get(5, 0)));

        packet.put("x variance", stateCovariance.get(0, 0));
        packet.put("y variance", stateCovariance.get(1, 1));
        packet.put("heading variance", stateCovariance.get(2, 2));

        packet.put("x drift variance", stateCovariance.get(3, 3));
        packet.put("y drift variance", stateCovariance.get(4, 4));
        packet.put("heading drift variance", stateCovariance.get(5, 5));
    }

    private void initState(double x, double y, double heading) {
        poseWithDrift = new SimpleMatrix(
                6, 1, true, new double[]{
                x, y, heading,
                0.0, 0.0, 0.0
        }
        );

        stateCovariance = SimpleMatrix.diag(
                initial_x_var,
                initial_y_var,
                initial_heading_var,
                initial_x_drift_var,
                initial_y_drift_var,
                initial_heading_drift_var
        );

        prevPinpointMeas = new SimpleMatrix(
                3, 1, true, new double[]{
                x, y, heading
        }
        );
    }
}