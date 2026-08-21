package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.field.Alliance;

/**
 * Locks the far routes against the waypoints the four per-alliance OpModes used before they were
 * merged.
 * <p>
 * RED_* below are copied from RedFar as it stood, not derived from the blue ones. The far routines
 * were never a clean mirror of each other, so the ones that genuinely differ are asserted as
 * differing on purpose, and will fail if someone quietly "fixes" one to match the other.
 */
public class FarPathsTest {

    private static final double[][] RED_PRE_C = {{64, 24}, {60, 24}};
    private static final double[][] RED_HP_TO_SHOOT = {{60, 60}, {60, 48}, {60, 24}};
    private static final double[][] RED_SHOOT_TO_THIRD_ROW = {{60, 24}, {36, 15}, {36, 62}};
    private static final double[][] RED_THIRD_ROW_TO_SHOOT = {{36, 62}, {50, 30}, {60, 24}};
    private static final double[][] RED_SHOOT_TO_LINGERING =
            {{60, 24}, {45, 36}, {45, 48}, {55, 62}};
    private static final double[][] RED_LINGERING_TO_SHOOT = {{55, 60}, {60, 24}};
    private static final double[][] RED_PRE_CU = {{60, 24}, {60, 36}};

    private static void assertWaypointsEqual(double[][] expected, double[][] actual) {
        assertEquals("waypoint count", expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertArrayEquals("waypoint " + i, expected[i], actual[i], 0.0);
        }
    }

    @Test
    public void theSharedRoutesMirrorOntoRed() {
        Alliance red = Alliance.RED;
        assertWaypointsEqual(RED_PRE_C, red.mirrorWaypoints(FarPaths.PRE_C));
        assertWaypointsEqual(RED_HP_TO_SHOOT, red.mirrorWaypoints(FarPaths.HP_TO_SHOOT));
        assertWaypointsEqual(
                RED_SHOOT_TO_THIRD_ROW, red.mirrorWaypoints(FarPaths.SHOOT_TO_THIRD_ROW));
        assertWaypointsEqual(
                RED_THIRD_ROW_TO_SHOOT, red.mirrorWaypoints(FarPaths.THIRD_ROW_TO_SHOOT));
        assertWaypointsEqual(
                RED_SHOOT_TO_LINGERING, red.mirrorWaypoints(FarPaths.SHOOT_TO_LINGERING));
        assertWaypointsEqual(
                RED_LINGERING_TO_SHOOT, red.mirrorWaypoints(FarPaths.LINGERING_TO_SHOOT));
        assertWaypointsEqual(RED_PRE_CU, red.mirrorWaypoints(FarPaths.PRE_CU));
    }

    /**
     * The run out to the human player was tuned separately on each side. Held per alliance on
     * purpose; this fails if someone replaces one with a mirror of the other.
     */
    @Test
    public void theHumanPlayerRunIsNotAMirror() {
        double[][] mirroredBlue = Alliance.RED.mirrorWaypoints(FarPaths.SHOOT_TO_HP_BLUE);
        assertEquals("both routes have four waypoints",
                mirroredBlue.length, FarPaths.SHOOT_TO_HP_RED.length);
        boolean identical = true;
        for (int i = 0; i < mirroredBlue.length; i++) {
            if (mirroredBlue[i][0] != FarPaths.SHOOT_TO_HP_RED[i][0]
                    || mirroredBlue[i][1] != FarPaths.SHOOT_TO_HP_RED[i][1]) {
                identical = false;
                break;
            }
        }
        assertEquals("red's human player run is deliberately its own route", false, identical);
        assertWaypointsEqual(
                new double[][]{{60, 24}, {45, 36}, {45, 48}, {46, 63}}, FarPaths.SHOOT_TO_HP_RED);
        assertWaypointsEqual(
                new double[][]{{60, -24}, {60, -36}, {60, -48}, {60, -62}},
                FarPaths.SHOOT_TO_HP_BLUE);
    }

    /** Only red ever drove the corner leg; blue's copy was commented out. */
    @Test
    public void onlyRedHasTheCornerLeg() {
        assertNotNull(new FarPaths(Alliance.RED).humanPlayerCorner);
        assertNull(new FarPaths(Alliance.BLUE).humanPlayerCorner);
    }

    @Test
    public void buildingTheRoutesDoesNotMutateTheSharedArrays() {
        new FarPaths(Alliance.RED);
        new FarPaths(Alliance.BLUE);
        assertWaypointsEqual(new double[][]{{64, -24}, {60, -24}}, FarPaths.PRE_C);
        assertWaypointsEqual(
                new double[][]{{60, -24}, {60, -36}, {60, -48}, {60, -62}},
                FarPaths.SHOOT_TO_HP_BLUE);
    }
}
