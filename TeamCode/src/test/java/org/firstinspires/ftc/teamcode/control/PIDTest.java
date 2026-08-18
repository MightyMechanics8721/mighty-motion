package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Covers the PID maths, including the derivative-on-measurement overload the shooter and turret
 * use.
 * <p>
 * The controller reads its timestep from an internal clock, so the timing-sensitive assertions here
 * drive it through a short settle first and then check invariants that hold regardless of how fast
 * the test machine runs.
 */
public class PIDTest {

    private static final double EPS = 1e-9;

    private PID pid(double kP, double kI, double kD, PID.functionType type) {
        return new PID(new PIDConstants(kP, kI, kD), type);
    }

    @Test
    public void proportionalOnlyReturnsGainTimesError() {
        PID p = pid(2.0, 0, 0, PID.functionType.LINEAR);
        assertEquals(20.0, p.calculate(10, 0), EPS);
    }

    @Test
    public void firstCallSkipsDerivativeSoThereIsNoStartupKick() {
        PID p = pid(0, 0, 5.0, PID.functionType.LINEAR);
        assertEquals("D must not fire on the first sample", 0.0, p.calculate(100, 0), EPS);
    }

    @Test
    public void firstCallLeavesTheIntegralEmpty() {
        PID p = pid(0, 1.0, 0, PID.functionType.LINEAR);
        p.calculate(10, 0);
        assertEquals("the first sample seeds history only", 0.0, p.eIntegralSum, EPS);
    }

    @Test
    public void integralGrowsWithSustainedErrorAndKeepsItsSign() {
        PID p = pid(0, 1.0, 0, PID.functionType.LINEAR);
        for (int i = 0; i < 20; i++) {
            p.calculate(10, 0);
        }
        assertTrue("a sustained positive error must build a positive integral",
                p.eIntegralSum > 0);

        PID n = pid(0, 1.0, 0, PID.functionType.LINEAR);
        for (int i = 0; i < 20; i++) {
            n.calculate(-10, 0);
        }
        assertTrue("a sustained negative error must build a negative integral",
                n.eIntegralSum < 0);
    }

    @Test
    public void integralIsClampedAgainstWindup() {
        PID p = pid(0, 1.0, 0, PID.functionType.LINEAR);
        for (int i = 0; i < 200; i++) {
            p.calculate(1e9, 0);
        }
        assertTrue("integral must not run away",
                Math.abs(p.eIntegralSum) <= PID.maxIntegralSum + EPS);
    }

    @Test
    public void outputStaysFiniteAcrossManyRapidCalls() {
        PID p = pid(1.0, 1.0, 1.0, PID.functionType.LINEAR);
        for (int i = 0; i < 500; i++) {
            double out = p.calculate(5, 0);
            assertTrue("a zero or tiny dt must not produce NaN or infinity",
                    Double.isFinite(out));
        }
    }

    @Test
    public void resetClearsAccumulatedState() {
        PID p = pid(0, 1.0, 0, PID.functionType.LINEAR);
        for (int i = 0; i < 20; i++) {
            p.calculate(10, 0);
        }
        assertTrue(p.eIntegralSum > 0);

        p.reset();
        assertEquals(0.0, p.eIntegralSum, EPS);
        assertEquals(0.0, p.ePrev, EPS);
        p.calculate(10, 0);
        assertEquals("the first call after reset seeds history only", 0.0, p.eIntegralSum, EPS);
    }

    @Test
    public void sqrtModeKeepsTheSignOfTheError() {
        assertEquals(3.0, pid(1.0, 0, 0, PID.functionType.SQRT).calculate(9, 0), EPS);
        assertEquals(-3.0, pid(1.0, 0, 0, PID.functionType.SQRT).calculate(-9, 0), EPS);
    }

    @Test
    public void sqrtModeGivesMoreAuthorityPerUnitErrorWhenClose() {
        double out = pid(1.0, 0, 0, PID.functionType.SQRT).calculate(0.01, 0);
        assertTrue("sqrt response should exceed the raw error when error < 1", out > 0.01);
    }

    @Test
    public void zeroErrorProducesZeroOutput() {
        PID p = pid(5.0, 5.0, 5.0, PID.functionType.LINEAR);
        p.calculate(0, 0);
        assertEquals(0.0, p.calculate(0, 0), EPS);
    }

    @Test
    public void measuredSpeedOverloadDampsAgainstMotion() {
        PID p = pid(1.0, 0, 0.5, PID.functionType.LINEAR);
        p.calculate(10, 0, 0);
        double still = p.calculate(10, 0, 0);
        double moving = pid(1.0, 0, 0.5, PID.functionType.LINEAR).calculate(10, 0, 4);
        assertTrue("moving toward the target should reduce the effort", moving < still);
    }

    @Test
    public void measuredSpeedOverloadIsProportionalToSpeed() {
        PID p = pid(0, 0, 2.0, PID.functionType.LINEAR);
        assertEquals(-6.0, p.calculate(0, 0, 3.0), EPS);
    }

    @Test
    public void gainsAreReadLiveFromTheConstantsObject() {
        PIDConstants gains = new PIDConstants(1.0, 0, 0);
        PID p = new PID(gains, PID.functionType.LINEAR);
        assertEquals(4.0, p.calculate(4, 0), EPS);

        gains.kP = 3.0;
        assertEquals("dashboard edits must reach an already-built controller",
                12.0, p.calculate(4, 0), EPS);
    }
}
