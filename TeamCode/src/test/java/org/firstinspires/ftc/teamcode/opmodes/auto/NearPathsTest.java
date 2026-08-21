package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.field.Alliance;

/**
 * Locks the near routes against the waypoints the six per-alliance OpModes used before they were
 * merged, so the merge cannot quietly change where the robot drives.
 * <p>
 * RED_* below are copied from the red OpModes as they stood. They are not derived from the blue
 * ones, which is the point: the test fails if mirroring stops reproducing them.
 */
public class NearPathsTest {

    private static final double[][] RED_FIRST_STEP = {{-51, 51}, {-12, 12}};
    private static final double[][] RED_FIRST_HALF_STEP = {{-12, 12}, {12, 36}, {12, 59}};
    private static final double[][] RED_SECOND_ROW_TO_SHOOT =
            {{12, 55}, {12, 46}, {9, 36}, {-8, 14}};
    private static final double[][] RED_GATE_TO_SHOOT =
            {{13.5, 59}, {12, 46}, {9, 36}, {-8, 14}};
    private static final double[][] RED_THIRD_ROW_STEP =
            {{-8, 14}, {12, 20}, {36, 20}, {36, 30}, {36, 36}, {36, 62}};
    private static final double[][] RED_THIRD_ROW_TO_SHOOT = {{36, 62}, {-8, 14}};
    private static final double[][] RED_FIRST_ROW_STEP =
            {{-8, 14}, {-12, 34}, {-12, 36}, {-12, 52}};
    private static final double[][] RED_FIRST_ROW_TO_SHOOT =
            {{-12, 52}, {-12, 36}, {-12, 34}, {-8, 14}};

    private static void assertWaypointsEqual(double[][] expected, double[][] actual) {
        assertEquals("waypoint count", expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertArrayEquals("waypoint " + i, expected[i], actual[i], 0.0);
        }
    }

    @Test
    public void mirroringBlueReproducesTheRedRoutes() {
        Alliance red = Alliance.RED;
        assertWaypointsEqual(RED_FIRST_STEP, red.mirrorWaypoints(NearPaths.FIRST_STEP));
        assertWaypointsEqual(RED_FIRST_HALF_STEP, red.mirrorWaypoints(NearPaths.FIRST_HALF_STEP));
        assertWaypointsEqual(
                RED_SECOND_ROW_TO_SHOOT, red.mirrorWaypoints(NearPaths.SECOND_ROW_TO_SHOOT));
        assertWaypointsEqual(RED_GATE_TO_SHOOT, red.mirrorWaypoints(NearPaths.GATE_TO_SHOOT));
        assertWaypointsEqual(RED_THIRD_ROW_STEP, red.mirrorWaypoints(NearPaths.THIRD_ROW_STEP));
        assertWaypointsEqual(
                RED_THIRD_ROW_TO_SHOOT, red.mirrorWaypoints(NearPaths.THIRD_ROW_TO_SHOOT));
        assertWaypointsEqual(RED_FIRST_ROW_STEP, red.mirrorWaypoints(NearPaths.FIRST_ROW_STEP));
        assertWaypointsEqual(
                RED_FIRST_ROW_TO_SHOOT, red.mirrorWaypoints(NearPaths.FIRST_ROW_TO_SHOOT));
    }

    @Test
    public void blueIsLeftAlone() {
        assertWaypointsEqual(NearPaths.FIRST_STEP, Alliance.BLUE.mirrorWaypoints(NearPaths.FIRST_STEP));
    }

    @Test
    public void mirroringDoesNotWriteThroughToTheSharedArrays() {
        double[][] mirrored = Alliance.RED.mirrorWaypoints(NearPaths.FIRST_STEP);
        mirrored[0][1] = 999;
        assertEquals("shared blue waypoints must not be mutated", -51, NearPaths.FIRST_STEP[0][1], 0.0);
    }

    @Test
    public void headingsMirrorWithTheWaypoints() {
        assertEquals(-45, Alliance.RED.mirrorHeadingDeg(45), 0.0);
        assertEquals(90, Alliance.RED.mirrorHeadingDeg(-90), 0.0);
        assertEquals(45, Alliance.BLUE.mirrorHeadingDeg(45), 0.0);
    }
}
