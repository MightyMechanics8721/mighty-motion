package org.firstinspires.ftc.teamcode.Mechanisms.Limelight.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;

@Config
@TeleOp(name = "Kalman Filter Test OpMode")
public class kalman extends LinearOpMode {

    // -----------------------------
    // Tuneable init pose
    // -----------------------------
    public static double initial_x_est = 0.0;
    public static double initial_y_est = 0.0;
    public static double initial_heading_est = 0.0;

    // -----------------------------
    // Initial covariance
    // -----------------------------
    public static double initial_x_var = 0.1;
    public static double initial_y_var = 0.1;
    public static double initial_heading_var = 0.1;

    public static double initial_x_drift_var = 0.0001;
    public static double initial_y_drift_var = 0.0001;
    public static double initial_heading_drift_var = 0.0001;

    // -----------------------------
    // Process noise
    // -----------------------------
    public static double x_process_var = 1E-6;
    public static double y_process_var = 1E-6;
    public static double heading_process_var = 1E-6;

    public static double x_drift_process_var_per_sec = 0.0002636;
    public static double y_drift_process_var_per_sec = 0.00007597;
    public static double heading_drift_process_var_per_sec = 0.00246;

    private final ElapsedTime timer = new ElapsedTime();
    Limelight3A ll;
    Drivetrain drivetrain;
    Shooter shooter;
    private FtcDashboard dashboard;

    // Replace these with your real objects
    // private Drivetrain drivetrain;
    // private Limelight limelight;

    /**
     * Wraps angle to [-pi, pi].
     */
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
     * Draw robot on FTC Dashboard. This is a simple circle + heading line.
     */
    private static void drawRobot(TelemetryPacket packet, SimpleMatrix pose, String color) {
        double x = pose.get(0, 0);
        double y = pose.get(1, 0);
        double heading = pose.get(2, 0);

        double robotRadius = 9.0;
        double headingLineLength = 12.0;

        double x2 = x + headingLineLength * Math.cos(heading);
        double y2 = y + headingLineLength * Math.sin(heading);

        packet.fieldOverlay()
              .setStroke(color)
              .strokeCircle(x, y, robotRadius)
              .strokeLine(x, y, x2, y2);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        Drivetrain.initialize(hardwareMap);
        drivetrain = Drivetrain.getInstance();
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        // -----------------------------
        // Initialize robot classes here
        // -----------------------------
        // drivetrain = new Drivetrain(hardwareMap);
        // limelight = hardwareMap.get(Limelight.class, "limelight");

        /*
         * State:
         * [x, y, theta, xDrift, yDrift, thetaDrift]^T
         */
        SimpleMatrix poseWithDrift = new SimpleMatrix(
                6, 1, true, new double[]{
                initial_x_est,
                initial_y_est,
                initial_heading_est,
                0.0,
                0.0,
                0.0
        }
        );

        SimpleMatrix stateCovariance = SimpleMatrix.diag(
                initial_x_var,
                initial_y_var,
                initial_heading_var,
                initial_x_drift_var,
                initial_y_drift_var,
                initial_heading_drift_var
        );

        /*
         * Pinpoint measurement covariance.
         * 3x3 because Pinpoint measures [x, y, theta].
         */
        SimpleMatrix pinpointMeasCov = SimpleMatrix.diag(
                x_process_var,
                y_process_var,
                heading_process_var
        );

        /*
         * A matrix.
         * State transition matrix.
         */
        final SimpleMatrix stateTransitionMatrix = SimpleMatrix.identity(6);

        /*
         * B matrix.
         * Control matrix.
         * This applies Pinpoint delta to x, y, theta only.
         */
        final SimpleMatrix controlMatrix = new SimpleMatrix(
                6, 3, true, new double[]{
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0
        }
        );

        /*
         * Cpp matrix.
         * Pinpoint observes pose + drift.
         */
        final SimpleMatrix pinpointOutputMatrix = new SimpleMatrix(
                3, 6, true, new double[]{
                1, 0, 0, 1, 0, 0,
                0, 1, 0, 0, 1, 0,
                0, 0, 1, 0, 0, 1
        }
        );

        /*
         * Cll matrix.
         * Limelight observes the true pose, not the drift.
         */
        final SimpleMatrix limelightOutputMatrix = new SimpleMatrix(
                3, 6, true, new double[]{
                1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0
        }
        );

        /*
         * Initial previous Pinpoint measurement.
         * Replace this with your actual initial drivetrain pose if needed.
         */
        SimpleMatrix prevPinpointMeas = new SimpleMatrix(
                3, 1, true, new double[]{
                initial_x_est,
                initial_y_est,
                initial_heading_est
        }
        );

        telemetry.addLine("Kalman filter initialized");
        telemetry.update();
        ll.pipelineSwitch(1);
        Battery.initialize(hardwareMap);
        Shooter.initialize(hardwareMap);
        shooter = Shooter.getInstance();
        waitForStart();
        ll.start();
        timer.reset();

        while (opModeIsActive()) {
            Actions.runBlocking(shooter.setShooterVelocityLoop(1500));
            double loopTime = timer.seconds();
            timer.reset();
            drivetrain.localize();
            SimpleMatrix pinpointMeas = drivetrain.state.extractMatrix(0, 3, 0, 1);
            SimpleMatrix changeInPinpoint = pinpointMeas.minus(prevPinpointMeas);
            prevPinpointMeas = pinpointMeas.copy();

            /*
             * Process noise covariance.
             */
            SimpleMatrix processNoiseCov = SimpleMatrix.diag(
                    x_process_var,
                    y_process_var,
                    heading_process_var,
                    x_drift_process_var_per_sec * loopTime,
                    y_drift_process_var_per_sec * loopTime,
                    heading_drift_process_var_per_sec * loopTime
            );

            // ============================================================
            // Prediction step
            // ============================================================

            SimpleMatrix predictedState = stateTransitionMatrix.mult(poseWithDrift)
                                                               .plus(controlMatrix.mult(
                                                                       changeInPinpoint));

            SimpleMatrix predictedCovariance =
                    stateTransitionMatrix
                            .mult(stateCovariance)
                            .mult(stateTransitionMatrix.transpose())
                            .plus(processNoiseCov);

            // ============================================================
            // Update step 1: Pinpoint
            // ============================================================

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

            // Optional but useful: wrap heading innovation
            pinpointInnovation.set(2, 0, angleWrapRadians(pinpointInnovation.get(2, 0)));

            SimpleMatrix updatedState =
                    predictedState.plus(pinpointKalmanGain.mult(pinpointInnovation));

            /*
             * Information-form covariance update from your pseudocode:
             *
             * P = inv(inv(Ppred) + C^T inv(R) C)
             */
            SimpleMatrix updatedCovariance =
                    predictedCovariance.invert()
                                       .plus(
                                               pinpointOutputMatrix.transpose()
                                                                   .mult(pinpointMeasCov.invert())
                                                                   .mult(pinpointOutputMatrix)
                                       )
                                       .invert();

            // ============================================================
            // Update step 2: Limelight
            // ============================================================

            boolean hasValidLimelight = false;

            SimpleMatrix limelightMeas = null;
            SimpleMatrix limelightMeasCov = null;

            /*
             * Replace this whole placeholder block with your actual Limelight code.
             *
             * Pseudocode:
             *
             * LLResult limelightResult = limelight.getLatestResult();
             *
             * if (limelightResult != null && limelightResult.isValid()) {
             *     Pose3D botPose = limelightResult.getBotpose();
             *
             *     if (botPose != null) {
             *         limelightMeas = new SimpleMatrix(3, 1, true, new double[]{
             *             botPose.getPosition().x,
             *             botPose.getPosition().y,
             *             botPose.getOrientation().getYaw(AngleUnit.RADIANS)
             *         });
             *
             *         double[] std = limelightResult.getStddevMt1();
             *
             *         limelightMeasCov = SimpleMatrix.diag(
             *             std[0] * std[0],
             *             std[1] * std[1],
             *             std[5] * std[5]
             *         );
             *
             *         hasValidLimelight = true;
             *     }
             * }
             */

            LLResult llm = ll.getLatestResult();

            if (llm != null && llm.isValid()) {
                limelightMeas = new SimpleMatrix(
                        new double[][]{
                                new double[]{llm.getBotpose().getPosition().x * 39.37},
                                new double[]{llm.getBotpose().getPosition().y * 39.37},
                                new double[]{
                                        llm.getBotpose()
                                           .getOrientation()
                                                .getYaw(AngleUnit.RADIANS)
                                },
                                }
                );
                limelightMeasCov = new SimpleMatrix(
                        new double[][]{
                                new double[]{Math.pow(llm.getStddevMt1()[0] * 39.37, 2), 0, 0},
                                new double[]{0, Math.pow(llm.getStddevMt1()[1] * 39.37, 2), 0},
                                new double[]{
                                        0, 0, Math.pow(
                                        Math.toRadians(llm.getStddevMt1()[5]),
                                        2
                                )
                                },
                                }
                );
                hasValidLimelight = true;
            }

            if (hasValidLimelight) {

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

            /*
             * Save running estimate.
             */
            poseWithDrift = updatedState;
            stateCovariance = updatedCovariance;

            // ============================================================
            // Telemetry
            // ============================================================

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("x pos (in) [KF]", poseWithDrift.get(0, 0));
            packet.put("y pos (in) [KF]", poseWithDrift.get(1, 0));
            packet.put("heading (deg) [KF]", Math.toDegrees(poseWithDrift.get(2, 0)));

            packet.put("x drift (in) [KF]", poseWithDrift.get(3, 0));
            packet.put("y drift (in) [KF]", poseWithDrift.get(4, 0));
            packet.put("heading drift (deg) [KF]", Math.toDegrees(poseWithDrift.get(5, 0)));

            packet.put("x variance", stateCovariance.get(0, 0));
            packet.put("y variance", stateCovariance.get(1, 1));
            packet.put("heading variance", stateCovariance.get(2, 2));

            packet.put("x drift variance", stateCovariance.get(3, 3));
            packet.put("y drift variance", stateCovariance.get(4, 4));
            packet.put("heading drift variance", stateCovariance.get(5, 5));

            packet.put("has valid limelight", hasValidLimelight);

            /*
             * Drawing:
             *
             * Replace drawRobot(...) with whatever drawing method you use.
             * For FTC Dashboard, you can draw on packet.fieldOverlay().
             */

            drawRobot(packet, pinpointMeas, "blue");

            if (hasValidLimelight && limelightMeas != null) {
                drawRobot(packet, limelightMeas, "green");
            }

            drawRobot(packet, poseWithDrift.extractMatrix(0, 3, 0, 1), "red");

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("x KF", poseWithDrift.get(0, 0));
            telemetry.addData("y KF", poseWithDrift.get(1, 0));
            telemetry.addData("heading KF deg", Math.toDegrees(poseWithDrift.get(2, 0)));
            telemetry.update();
        }
    }
}