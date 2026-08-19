package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

/** Flywheel speed curve. */
public class ShooterModelTest {

    private static final double EPS = 1e-9;

    @After
    public void restoreDefaults() {
        ShooterModel.quadCoeff = 0.103;
        ShooterModel.linearCoeff = -4.53;
        ShooterModel.constant = 1700;
    }

    @Test
    public void matchesTheOriginalPolynomial() {
        for (double d : new double[]{0, 12, 36, 60, 96, 144}) {
            assertEquals(0.103 * d * d - 4.53 * d + 1700,
                    ShooterModel.velocityForDistance(d), 1e-9);
        }
    }

    @Test
    public void atZeroDistanceReturnsTheConstant() {
        assertEquals(1700.0, ShooterModel.velocityForDistance(0), EPS);
    }

    @Test
    public void speedRisesAgainOnceTheQuadraticDominates() {
        double near = ShooterModel.velocityForDistance(20);
        double far = ShooterModel.velocityForDistance(120);
        assertTrue("a far shot must need more speed than a mid one", far > near);
    }

    @Test
    public void theCurveDipsBeforeItClimbs() {
        double atZero = ShooterModel.velocityForDistance(0);
        double atDip = ShooterModel.velocityForDistance(22);
        assertTrue("the linear term should pull the curve down first", atDip < atZero);
    }

    @Test
    public void coefficientsAreLiveSoDashboardEditsTakeEffect() {
        double before = ShooterModel.velocityForDistance(50);
        ShooterModel.constant += 100;
        assertEquals(before + 100, ShooterModel.velocityForDistance(50), EPS);
    }

    @Test
    public void everyOutputIsFinite() {
        for (double d = -200; d <= 200; d += 5) {
            assertTrue(Double.isFinite(ShooterModel.velocityForDistance(d)));
        }
    }
}
