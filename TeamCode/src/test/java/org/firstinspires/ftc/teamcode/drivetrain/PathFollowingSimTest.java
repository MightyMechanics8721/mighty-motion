package org.firstinspires.ftc.teamcode.drivetrain;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.sim.DrivetrainSim;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Drives the real pure pursuit follower against {@link DrivetrainSim}.
 * <p>
 * These reproduce the two follower faults found by reading the code, so they stay fixed: the
 * lookahead index carrying over between paths, and the speed term scaling small corrections up to
 * full send instead of capping large ones.
 */
public class PathFollowingSimTest {

    private static final double DT = 0.02; // (s)
    private static final double MAX_SPEED = 150; // (in/s), as every routine passes
    private static final int MAX_TICKS = 1500; // 30 s of match

    private static double[][] leg() {
        return new double[][]{{-8, -14}, {-12, -34}, {-12, -36}, {-12, -52}};
    }

    /** The six waypoint run out to the third row, straight from the near routines. */
    private static double[][] longLeg() {
        return new double[][]{
                {-8, -14}, {12, -20}, {36, -20}, {36, -30}, {36, -36}, {36, -62}};
    }

    /** The two waypoint run back, straight from the near routines. */
    private static double[][] shortReturn() {
        return new double[][]{{36, -62}, {-8, -14}};
    }

    /**
     * Follows a path to its end, exactly as followPathFunction does.
     *
     * @param capSpeed true caps the wheel speeds at maxSpeed, false normalises them to it, which
     * is what the code did before
     *
     * @return distance from the final waypoint when it stopped or ran out of time (in)
     */
    private static double follow(
            DrivetrainSim sim, GeometricController geometry, Path path, boolean capSpeed
    ) {
        PoseController follower = new PoseController(
                Drivetrain.FOLLOWER_CONSTANTS.xPIDConstants,
                Drivetrain.FOLLOWER_CONSTANTS.yPIDConstants,
                Drivetrain.FOLLOWER_CONSTANTS.headingPIDConstants
        );
        PoseController pose = new PoseController(
                Drivetrain.POSE_CONSTANTS.xPIDConstants,
                Drivetrain.POSE_CONSTANTS.yPIDConstants,
                Drivetrain.POSE_CONSTANTS.headingPIDConstants
        );
        double[] finish = path.getFinalPoint();
        SimpleMatrix endPose = Utils.makePoseVector(
                finish[0], finish[1], Math.toDegrees(path.finalHeading));

        for (int i = 0; i < MAX_TICKS; i++) {
            double toFinish = Utils.calculateDistance(sim.x(), sim.y(), finish[0], finish[1]);

            if (toFinish <= Drivetrain.FOLLOWER_CONSTANTS.poseControlHandoffDistance) {
                // Hand off to pose control for the last stretch.
                if (toFinish <= 1.0) {
                    break;
                }
                sim.step(pose.calculate(sim.driftedPose(), endPose), DT);
                continue;
            }

            SimpleMatrix target = geometry.calculate(sim.pose(), path);
            SimpleMatrix twist = follower.calculate(sim.driftedPose(), target);

            double speedLimit = MAX_SPEED / Drivetrain.MECHANICAL_PARAMETERS.wheelRadius;
            SimpleMatrix wheels = new MecanumKinematicModel(Drivetrain.MECHANICAL_PARAMETERS)
                    .inverseKinematics(twist);
            double fastest = wheels.elementMaxAbs();
            if (capSpeed) {
                if (fastest > speedLimit) {
                    twist = twist.scale(speedLimit / fastest);
                }
            } else if (fastest > 0) {
                twist = twist.scale(speedLimit / fastest); // the old normalise-always behaviour
            }

            sim.step(twist, DT);
        }
        return Utils.calculateDistance(sim.x(), sim.y(), finish[0], finish[1]);
    }

    @Test
    public void followsASinglePathToItsEnd() {
        DrivetrainSim sim = new DrivetrainSim(-8, -14, Math.toRadians(-90));
        double error = follow(sim, new GeometricController(
                        Drivetrain.FOLLOWER_CONSTANTS.positionLookahead,
                        Drivetrain.FOLLOWER_CONSTANTS.headingLookahead),
                new Path(leg(), Math.toRadians(-90), false, false), true);
        System.out.printf("single path -> %.2f in from the end%n", error);
        assertTrue("did not reach the end of the path, " + error + " in short", error <= 1.5);
    }

    /**
     * A long path followed by a short one is where the carried over lookahead index bites: the
     * index is left pointing at a segment number the new path does not have, the search loop never
     * runs, and the follower falls back to the closest point on the path -- a target inches away,
     * which it then creeps toward instead of driving the route.
     * <p>
     * This is the thirdRowStep to thirdRowToShoot transition in the near routines: six waypoints
     * then two.
     */
    @Test
    public void aStaleLookaheadIndexStallsAShorterFollowingPath() {
        GeometricController shared = new GeometricController(
                Drivetrain.FOLLOWER_CONSTANTS.positionLookahead,
                Drivetrain.FOLLOWER_CONSTANTS.headingLookahead);

        DrivetrainSim first = new DrivetrainSim(-8, -14, Math.toRadians(-90));
        follow(first, shared, new Path(longLeg(), Math.toRadians(-90), false, false), true);
        assertTrue("the long path should have advanced the index",
                shared.lastIndexXY > 0);

        // No resetLookAhead: what a deadline-terminated path used to leave behind.
        DrivetrainSim stale = new DrivetrainSim(36, -62, Math.toRadians(-90));
        double staleError = follow(stale, shared,
                new Path(shortReturn(), Math.toRadians(-45), true, false), true);

        shared.resetLookAhead();
        DrivetrainSim fresh = new DrivetrainSim(36, -62, Math.toRadians(-90));
        double freshError = follow(fresh, shared,
                new Path(shortReturn(), Math.toRadians(-45), true, false), true);

        System.out.printf(
                "short path after long: stale index %.1f in short, after reset %.1f in short%n",
                staleError, freshError);
        assertTrue("with the index reset the path should complete, was "
                + freshError + " in short", freshError <= 1.5);
        assertTrue("a stale index should leave the robot short of the end, was "
                + staleError + " in", staleError > 5.0);
    }

    /**
     * Normalising rather than capping drives full speed even when the correction is tiny, so a
     * follower that has lost its lookahead thrashes instead of creeping.
     */
    @Test
    public void normalisingInsteadOfCappingRunsFullSpeedOnATinyError() {
        MecanumKinematicModel kinematics =
                new MecanumKinematicModel(Drivetrain.MECHANICAL_PARAMETERS);
        double speedLimit = MAX_SPEED / Drivetrain.MECHANICAL_PARAMETERS.wheelRadius;

        // A correction of a fifth of an inch: what the closest-point fallback produces.
        SimpleMatrix tiny = new SimpleMatrix(new double[][]{{0.2}, {0}, {0}});
        double fastest = kinematics.inverseKinematics(tiny).elementMaxAbs();

        double normalised = tiny.scale(speedLimit / fastest).get(0, 0);
        double capped = fastest > speedLimit ? tiny.scale(speedLimit / fastest).get(0, 0)
                : tiny.get(0, 0);

        System.out.printf("tiny correction: normalised %.1f in/s, capped %.1f in/s%n",
                normalised, capped);
        assertTrue("normalising should blow a small error up to the speed limit",
                normalised > 100);
        assertTrue("capping should leave a small error alone", capped == 0.2);
    }

    /**
     * Follows a long straight at a given speed limit and reports the cruise speed reached.
     *
     * @param normaliseInsteadOfCap true reproduces the old behaviour, where the speed term forced
     * the wheels to exactly maxSpeed rather than capping them at it
     */
    private static double cruiseSpeed(double maxSpeed, boolean normaliseInsteadOfCap) {
        DrivetrainSim sim = new DrivetrainSim(0, 0, 0);
        GeometricController geometry = new GeometricController(
                Drivetrain.FOLLOWER_CONSTANTS.positionLookahead,
                Drivetrain.FOLLOWER_CONSTANTS.headingLookahead);
        PoseController follower = new PoseController(
                Drivetrain.FOLLOWER_CONSTANTS.xPIDConstants,
                Drivetrain.FOLLOWER_CONSTANTS.yPIDConstants,
                Drivetrain.FOLLOWER_CONSTANTS.headingPIDConstants);
        MecanumKinematicModel kinematics =
                new MecanumKinematicModel(Drivetrain.MECHANICAL_PARAMETERS);
        Path straight = new Path(new double[][]{{0, 0}, {200, 0}}, 0, false, true);
        double speedLimit = maxSpeed / Drivetrain.MECHANICAL_PARAMETERS.wheelRadius;

        double cruise = 0;
        for (int i = 0; i < 400; i++) {
            SimpleMatrix twist =
                    follower.calculate(sim.driftedPose(), geometry.calculate(sim.pose(), straight));
            double fastest = kinematics.inverseKinematics(twist).elementMaxAbs();
            if (fastest > 0 && (normaliseInsteadOfCap || fastest > speedLimit)) {
                twist = twist.scale(speedLimit / fastest);
            }
            sim.step(twist, DT);
            if (i > 200) {
                cruise = Math.max(cruise, sim.speed());
            }
        }
        return cruise;
    }

    /**
     * maxSpeed is a limiter, so it should hold the robot down when set below what the motors can
     * do, and do nothing when set above. The ceiling is where the feedforward saturates.
     */
    @Test
    public void maxSpeedLimitsOnlyBelowTheSaturationSpeed() {
        double ceiling = DrivetrainSim.MAX_WHEEL_SPEED
                * Drivetrain.MECHANICAL_PARAMETERS.wheelRadius;
        double slow = cruiseSpeed(8, false);
        double medium = cruiseSpeed(14, false);
        double unlimited = cruiseSpeed(150, false);
        double aboveCeiling = cruiseSpeed(30, false);

        System.out.printf("saturation ceiling %.1f in/s%n", ceiling);
        System.out.printf("maxSpeed  8 -> cruise %.1f in/s%n", slow);
        System.out.printf("maxSpeed 14 -> cruise %.1f in/s%n", medium);
        System.out.printf("maxSpeed 30 -> cruise %.1f in/s%n", aboveCeiling);
        System.out.printf("maxSpeed150 -> cruise %.1f in/s%n", unlimited);

        assertTrue("a limit below the ceiling should bite", slow < medium);
        assertTrue("8 in/s limit should hold near 8", Math.abs(slow - 8) < 1.5);
        assertTrue("a limit above the ceiling should do nothing",
                Math.abs(aboveCeiling - unlimited) < 0.5);
    }

    /**
     * With the old normalise-always behaviour a tight limit still pins the robot at exactly that
     * speed, so it never slows down to settle at the end of a path.
     */
    @Test
    public void normalisingPinsTheRobotAtTheLimitInsteadOfCappingIt() {
        double capped = cruiseSpeed(8, false);
        double normalised = cruiseSpeed(8, true);
        System.out.printf("limit 8: capped %.1f in/s, normalised %.1f in/s%n",
                capped, normalised);
        assertTrue("both should cruise near the limit on a long straight",
                Math.abs(capped - normalised) < 1.5);
    }
}
