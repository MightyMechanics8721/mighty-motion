package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Covers the trapezoidal and triangular motion profiles: the phase boundaries, continuity across
 * them, and behaviour past the end of the profile.
 */
public class MotionProfileTest {

    private static final double MAX_V = 30.0;
    private static final double MAX_A = 60.0;
    private static final double MAX_D = 60.0;

    private MotionProfile trapezoid() {
        return new MotionProfile(100, MAX_V, MAX_A, MAX_D, false);
    }

    @Test
    public void startsAtRest() {
        MotionProfile p = trapezoid();
        assertEquals(0.0, p.getPosition(0), 1e-9);
        assertEquals(0.0, p.getVelocity(0), 1e-9);
    }

    @Test
    public void endsAtTheTargetDistance() {
        MotionProfile p = trapezoid();
        assertEquals(100.0, p.getPosition(p.getTime()), 1e-6);
        assertEquals(100.0, p.getPosition(p.getTime() + 5), 1e-9);
    }

    @Test
    public void velocityIsZeroOnceFinished() {
        MotionProfile p = trapezoid();
        assertEquals(0.0, p.getVelocity(p.getTime() + 1), 1e-9);
        assertEquals(0.0, p.getAcceleration(p.getTime() + 1), 1e-9);
    }

    @Test
    public void velocityIsSaneAtTheAccelerationBoundary() {
        MotionProfile p = trapezoid();
        double tAccel = MAX_V / MAX_A;
        double v = p.getVelocity(tAccel);
        assertTrue("boundary must not fall through to a sentinel value, got " + v,
                v <= MAX_V + 1e-6);
        assertEquals(MAX_V, v, 1e-6);
    }

    @Test
    public void velocityNeverExceedsTheLimit() {
        MotionProfile p = trapezoid();
        for (double t = 0; t <= p.getTime() + 0.5; t += 0.005) {
            double v = p.getVelocity(t);
            assertTrue("velocity " + v + " at t=" + t + " exceeds the limit",
                    Math.abs(v) <= MAX_V + 1e-6);
        }
    }

    @Test
    public void positionIsMonotonicAndContinuousForwards() {
        MotionProfile p = trapezoid();
        double previous = p.getPosition(0);
        for (double t = 0; t <= p.getTime(); t += 0.005) {
            double current = p.getPosition(t);
            assertTrue("position went backwards at t=" + t, current >= previous - 1e-6);
            assertTrue("position jumped at t=" + t, current - previous < 1.0);
            previous = current;
        }
    }

    @Test
    public void reversedProfileIsTheMirrorOfTheForwardOne() {
        MotionProfile forward = new MotionProfile(100, MAX_V, MAX_A, MAX_D, false);
        MotionProfile backward = new MotionProfile(100, MAX_V, MAX_A, MAX_D, true);
        for (double t = 0; t <= forward.getTime(); t += 0.01) {
            assertEquals("reversed position must mirror the forward one at t=" + t,
                    -forward.getPosition(t), backward.getPosition(t), 1e-6);
        }
    }

    @Test
    public void reversedProfileIsContinuousThroughTheCruisePhase() {
        MotionProfile p = new MotionProfile(100, MAX_V, MAX_A, MAX_D, true);
        double previous = p.getPosition(0);
        for (double t = 0; t <= p.getTime(); t += 0.005) {
            double current = p.getPosition(t);
            assertTrue("reversed position jumped at t=" + t, Math.abs(current - previous) < 1.0);
            previous = current;
        }
    }

    @Test
    public void shortMoveFallsBackToATriangularProfile() {
        MotionProfile p = new MotionProfile(1, MAX_V, MAX_A, MAX_D, false);
        for (double t = 0; t <= p.getTime() + 0.2; t += 0.002) {
            double v = p.getVelocity(t);
            assertTrue("triangular profile should never reach the velocity cap", v <= MAX_V + 1e-6);
            assertTrue("velocity must stay finite", Double.isFinite(v));
        }
        assertEquals(1.0, p.getPosition(p.getTime()), 1e-6);
    }

    @Test
    public void zeroLengthMoveDoesNotProduceNaN() {
        MotionProfile p = new MotionProfile(0, MAX_V, MAX_A, MAX_D, false);
        assertTrue("total time must be finite", Double.isFinite(p.getTime()));
        assertTrue("position must be finite", Double.isFinite(p.getPosition(0.1)));
        assertTrue("velocity must be finite", Double.isFinite(p.getVelocity(0.1)));
        assertTrue("acceleration must be finite", Double.isFinite(p.getAcceleration(0.1)));
    }

    @Test
    public void accelerationIsPositiveThenZeroThenNegative() {
        MotionProfile p = trapezoid();
        double tAccel = MAX_V / MAX_A;
        assertEquals(MAX_A, p.getAcceleration(tAccel / 2), 1e-9);
        assertEquals(0.0, p.getAcceleration(p.getTime() / 2), 1e-9);
        assertEquals(-MAX_D, p.getAcceleration(p.getTime() - tAccel / 2), 1e-9);
    }

    @Test
    public void reverseFlagIsReported() {
        assertEquals(1, new MotionProfile(10, MAX_V, MAX_A, MAX_D, false).isReverse());
        assertEquals(-1, new MotionProfile(10, MAX_V, MAX_A, MAX_D, true).isReverse());
    }
}
