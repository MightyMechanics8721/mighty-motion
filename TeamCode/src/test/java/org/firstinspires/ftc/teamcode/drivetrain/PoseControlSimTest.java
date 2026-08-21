package org.firstinspires.ftc.teamcode.drivetrain;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.sim.DrivetrainSim;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Drives the real pose controller against {@link DrivetrainSim} to see how goToPose actually
 * settles.
 */
public class PoseControlSimTest {

    private static final double DT = 0.02; // (s) 50 Hz, near a real loop rate
    private static final double DISTANCE_THRESHOLD = 2.0; // (in)
    private static final double ANGLE_THRESHOLD = Math.toRadians(2.5);

    private static SimpleMatrix pose(double x, double y, double headingDeg) {
        return Utils.makePoseVector(x, y, headingDeg);
    }

    /** The same predicate Drivetrain.inStoppingZone applies. */
    private static boolean inStoppingZone(
            SimpleMatrix pose, SimpleMatrix target, double distanceThreshold, double angleThreshold
    ) {
        double distance = Utils.calculateDistance(
                pose.get(0, 0), pose.get(1, 0), target.get(0, 0), target.get(1, 0));
        double heading = Utils.angleWrap(target.get(2, 0) - pose.get(2, 0));
        return Math.abs(distance) <= Math.abs(distanceThreshold)
                && Math.abs(heading) <= Math.abs(angleThreshold);
    }

    private static PoseController poseController() {
        return new PoseController(
                Drivetrain.POSE_CONSTANTS.xPIDConstants,
                Drivetrain.POSE_CONSTANTS.yPIDConstants,
                Drivetrain.POSE_CONSTANTS.headingPIDConstants
        );
    }

    /**
     * Runs the goToPose control law until it declares itself finished, then lets the robot coast.
     *
     * @param stopOnDriftedPose false reproduces today's behaviour, where the stop test reads the
     * real pose while the controller steers the drifted one; true requires both to be inside
     *
     * @return distance from the target once the robot has actually come to rest (in)
     */
    private static double settleDistance(SimpleMatrix target, boolean stopOnDriftedPose) {
        DrivetrainSim sim = new DrivetrainSim(0, 0, 0);
        PoseController controller = poseController();

        boolean finished = false;
        for (int i = 0; i < 1000 && !finished; i++) {
            boolean realInside =
                    inStoppingZone(sim.pose(), target, DISTANCE_THRESHOLD, ANGLE_THRESHOLD);
            boolean driftInside =
                    inStoppingZone(sim.driftedPose(), target, DISTANCE_THRESHOLD, ANGLE_THRESHOLD);
            finished = stopOnDriftedPose ? (realInside && driftInside) : realInside;
            if (finished) {
                break;
            }
            // goToPose steers the drifted pose whenever useStoppingDistance is set.
            sim.step(controller.calculate(sim.driftedPose(), target), DT);
        }
        assertTrue("controller never reached the target", finished);

        for (int i = 0; i < 1000 && sim.speed() > 0.05; i++) {
            sim.coast(DT);
        }
        return Utils.calculateDistance(sim.x(), sim.y(), target.get(0, 0), target.get(1, 0));
    }

    @Test
    public void goToPoseDrivesToTheTarget() {
        DrivetrainSim sim = new DrivetrainSim(0, 0, 0);
        SimpleMatrix target = pose(24, 12, 45);
        PoseController controller = poseController();

        for (int i = 0; i < 1000; i++) {
            if (inStoppingZone(sim.pose(), target, DISTANCE_THRESHOLD, ANGLE_THRESHOLD)) {
                return;
            }
            sim.step(controller.calculate(sim.pose(), target), DT);
        }
        throw new AssertionError(String.format(
                "never converged; ended %.1f in away at heading %.1f deg",
                Utils.calculateDistance(sim.x(), sim.y(), 24, 12),
                Math.toDegrees(sim.headingRad())));
    }

    /**
     * The invariant that matters: once goToPose says it is done and the robot has stopped, it is
     * inside the threshold it was given.
     * <p>
     * Steering the drifted pose while testing the real one looks like it should let the robot
     * coast back out. It does not, because the sqrt controller has already slowed the approach to
     * a crawl by the time the real pose crosses the line. Both stop tests settle the same.
     */
    @Test
    public void goToPoseSettlesInsideItsThreshold() {
        double onRealPose = settleDistance(pose(48, 0, 0), false);
        double onDriftedPose = settleDistance(pose(48, 0, 0), true);
        System.out.printf("settle: stop-on-real %.2f in, stop-on-drift %.2f in%n",
                onRealPose, onDriftedPose);
        assertTrue("stop-on-real settled " + onRealPose, onRealPose <= DISTANCE_THRESHOLD);
        assertTrue("stop-on-drift settled " + onDriftedPose, onDriftedPose <= DISTANCE_THRESHOLD);
    }

    /**
     * The park step at the end of the near routines asks for a quarter inch and a quarter degree.
     * The sqrt term's gain grows without bound as the error shrinks, so a threshold this tight is
     * where it would chatter instead of settling.
     */
    @Test
    public void theTightParkThresholdConverges() {
        DrivetrainSim sim = new DrivetrainSim(0, -24, Math.toRadians(-90));
        SimpleMatrix target = pose(0, -48, -90);
        PoseController controller = poseController();
        double tightDistance = 0.25;
        double tightAngle = Math.toRadians(0.25);

        int ticks = 0;
        for (; ticks < 1000; ticks++) {
            if (inStoppingZone(sim.pose(), target, tightDistance, tightAngle)) {
                break;
            }
            sim.step(controller.calculate(sim.driftedPose(), target), DT);
        }
        double error = Utils.calculateDistance(sim.x(), sim.y(), 0, -48);
        System.out.printf("tight park: %.2fs, error %.3f in%n", ticks * DT, error);
        assertTrue(String.format(
                        "never settled inside %.2f in; %.3f in after %.1fs",
                        tightDistance, error, ticks * DT),
                ticks < 1000);
    }

    /** How fast the robot is still moving when the stop test first passes. */
    @Test
    public void reportsApproachSpeedAtTheStopThreshold() {
        DrivetrainSim sim = new DrivetrainSim(0, 0, 0);
        SimpleMatrix target = pose(48, 0, 0);
        PoseController controller = poseController();
        for (int i = 0; i < 1000; i++) {
            if (inStoppingZone(sim.pose(), target, DISTANCE_THRESHOLD, ANGLE_THRESHOLD)) {
                System.out.printf(
                        "at stop threshold: speed %.2f in/s, coast ahead %.2f in%n",
                        sim.speed(),
                        Utils.calculateDistance(sim.x(), sim.y(),
                                sim.driftedPose().get(0, 0), sim.driftedPose().get(1, 0)));
                return;
            }
            sim.step(controller.calculate(sim.driftedPose(), target), DT);
        }
        throw new AssertionError("never reached the threshold");
    }
}
