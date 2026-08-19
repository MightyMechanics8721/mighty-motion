package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Coast distance curve. */
public class StoppingDistanceTest {

    private static final double EPS = 1e-9;
    private static final double LIN = 0.083;
    private static final double QUAD = 0.000075;

    @Test
    public void matchesTheOriginalExpression() {
        for (double v : new double[]{0, 10, 90, 250, 600}) {
            assertEquals(LIN * v + QUAD * v * v,
                    StoppingDistance.forVelocity(v, LIN, QUAD), 1e-9);
        }
    }

    @Test
    public void standingStillCoastsNowhere() {
        assertEquals(0.0, StoppingDistance.forVelocity(0, LIN, QUAD), EPS);
    }

    @Test
    public void reversingCoastsBackwardsByTheSameAmount() {
        double forward = StoppingDistance.forVelocity(120, LIN, QUAD);
        double backward = StoppingDistance.forVelocity(-120, LIN, QUAD);
        assertEquals("coast must be symmetric about zero", -forward, backward, EPS);
    }

    @Test
    public void coastGrowsWithSpeed() {
        double previous = 0;
        for (double v = 10; v <= 400; v += 10) {
            double d = StoppingDistance.forVelocity(v, LIN, QUAD);
            assertTrue("coast must increase with speed at v=" + v, d > previous);
            previous = d;
        }
    }

    @Test
    public void theQuadraticTermMattersAtSpeed() {
        double linearOnly = StoppingDistance.forVelocity(400, LIN, 0);
        double withQuad = StoppingDistance.forVelocity(400, LIN, QUAD);
        assertTrue("the squared term should add coast at high speed", withQuad > linearOnly);
    }

    @Test
    public void zeroCoefficientsMeanNoCoast() {
        assertEquals(0.0, StoppingDistance.forVelocity(500, 0, 0), EPS);
    }
}
