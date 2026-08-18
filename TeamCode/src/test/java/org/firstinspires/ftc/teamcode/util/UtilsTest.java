package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

/**
 * Covers the geometry helpers the whole control stack depends on: angle wrapping, distance, frame
 * rotation and pose construction.
 */
public class UtilsTest {

    private static final double EPS = 1e-9;

    @Test
    public void angleWrapLeavesAnglesInRangeAlone() {
        assertEquals(0.0, Utils.angleWrap(0.0), EPS);
        assertEquals(1.0, Utils.angleWrap(1.0), EPS);
        assertEquals(-1.0, Utils.angleWrap(-1.0), EPS);
        assertEquals(Math.PI, Utils.angleWrap(Math.PI), EPS);
    }

    @Test
    public void angleWrapFoldsAnglesIntoPlusMinusPi() {
        assertEquals(0.0, Utils.angleWrap(2 * Math.PI), EPS);
        assertEquals(0.0, Utils.angleWrap(-2 * Math.PI), EPS);
        assertEquals(-Math.PI / 2, Utils.angleWrap(3 * Math.PI / 2), EPS);
        assertEquals(Math.PI / 2, Utils.angleWrap(-3 * Math.PI / 2), EPS);
    }

    @Test
    public void angleWrapTakesTheShortWayRoundNearTheDiscontinuity() {
        double target = Math.toRadians(179);
        double current = Math.toRadians(-179);
        double error = Utils.angleWrap(target - current);
        assertTrue("should turn 2 degrees, not 358", Math.abs(error) < Math.toRadians(3));
    }

    @Test
    public void angleWrapHandlesManyRevolutions() {
        assertEquals(Math.PI / 4, Utils.angleWrap(Math.PI / 4 + 20 * Math.PI), 1e-6);
    }

    @Test
    public void calculateDistanceIsEuclidean() {
        assertEquals(5.0, Utils.calculateDistance(0, 0, 3, 4), EPS);
        assertEquals(0.0, Utils.calculateDistance(2, 2, 2, 2), EPS);
        assertEquals(5.0, Utils.calculateDistance(3, 4, 0, 0), EPS);
    }

    @Test
    public void rotatingToBodyAndBackIsIdentity() {
        SimpleMatrix v = new SimpleMatrix(new double[][]{{3}, {4}, {0.5}});
        for (double heading : new double[]{0, 0.3, Math.PI / 2, 2.5, -1.2}) {
            SimpleMatrix roundTrip = Utils.rotateBodyToGlobal(
                    Utils.rotateGlobalToBody(v, heading), heading);
            assertEquals(v.get(0, 0), roundTrip.get(0, 0), 1e-9);
            assertEquals(v.get(1, 0), roundTrip.get(1, 0), 1e-9);
            assertEquals(v.get(2, 0), roundTrip.get(2, 0), 1e-9);
        }
    }

    @Test
    public void rotatingNinetyDegreesSwapsAxes() {
        SimpleMatrix forward = new SimpleMatrix(new double[][]{{1}, {0}, {0}});
        SimpleMatrix rotated = Utils.rotateBodyToGlobal(forward, Math.PI / 2);
        assertEquals(0.0, rotated.get(0, 0), 1e-9);
        assertEquals(1.0, rotated.get(1, 0), 1e-9);
    }

    @Test
    public void rotationLeavesHeadingComponentUntouched() {
        SimpleMatrix v = new SimpleMatrix(new double[][]{{1}, {2}, {0.75}});
        assertEquals(0.75, Utils.rotateBodyToGlobal(v, 1.1).get(2, 0), 1e-9);
        assertEquals(0.75, Utils.rotateGlobalToBody(v, 1.1).get(2, 0), 1e-9);
    }
}
