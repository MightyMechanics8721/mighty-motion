package org.firstinspires.ftc.teamcode.storage;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

import org.ejml.simple.SimpleMatrix;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** Autonomous to TeleOp handoff. */
public class StaticVariablesTest {

    private static final double EPS = 1e-9;

    private SimpleMatrix pose(double x, double y, double heading) {
        return new SimpleMatrix(new double[][]{{x}, {y}, {heading}});
    }

    @Before
    public void clearBefore() {
        StaticVariables.clear();
    }

    @After
    public void clearAfter() {
        StaticVariables.clear();
    }

    @Test
    public void startsEmpty() {
        assertFalse(StaticVariables.hasRobotState());
        assertEquals(0.0, StaticVariables.turretAngle(), EPS);
    }

    @Test
    public void savedPoseComesBackUnchanged() {
        StaticVariables.saveRobotState(pose(12.5, -34.25, 1.0));
        assertTrue(StaticVariables.hasRobotState());

        SimpleMatrix out = StaticVariables.robotStateOr(pose(0, 0, 0));
        assertEquals(12.5, out.get(0, 0), EPS);
        assertEquals(-34.25, out.get(1, 0), EPS);
        assertEquals(1.0, out.get(2, 0), EPS);
    }

    @Test
    public void savedTurretAngleComesBackUnchanged() {
        StaticVariables.saveTurretAngle(37.5);
        assertEquals(37.5, StaticVariables.turretAngle(), EPS);
    }

    @Test
    public void fallbackIsUsedWhenNoAutonRan() {
        SimpleMatrix out = StaticVariables.robotStateOr(pose(-51, -51, 0.785));
        assertEquals(-51.0, out.get(0, 0), EPS);
        assertEquals(-51.0, out.get(1, 0), EPS);
        assertEquals(0.785, out.get(2, 0), EPS);
    }

    @Test
    public void storedPoseWinsOverTheFallback() {
        StaticVariables.saveRobotState(pose(1, 2, 0.5));
        SimpleMatrix out = StaticVariables.robotStateOr(pose(99, 99, 9));
        assertEquals(1.0, out.get(0, 0), EPS);
        assertEquals(2.0, out.get(1, 0), EPS);
    }

    @Test
    public void theStoredPoseIsACopy() {
        SimpleMatrix live = pose(1, 2, 3);
        StaticVariables.saveRobotState(live);
        live.set(0, 0, 999);

        SimpleMatrix out = StaticVariables.robotStateOr(pose(0, 0, 0));
        assertNotSame(live, out);
        assertEquals("later edits to the drivetrain state must not rewrite the stored pose",
                1.0, out.get(0, 0), EPS);
    }

    @Test
    public void readingDoesNotConsume() {
        StaticVariables.saveRobotState(pose(5, 6, 0.1));
        StaticVariables.robotStateOr(pose(0, 0, 0));
        assertTrue(StaticVariables.hasRobotState());
        assertEquals(5.0, StaticVariables.robotStateOr(pose(0, 0, 0)).get(0, 0), EPS);
    }

    @Test
    public void clearDiscardsBothValues() {
        StaticVariables.saveRobotState(pose(5, 5, 1));
        StaticVariables.saveTurretAngle(42);
        StaticVariables.clear();
        assertFalse(StaticVariables.hasRobotState());
        assertEquals(0.0, StaticVariables.turretAngle(), EPS);
    }
}
