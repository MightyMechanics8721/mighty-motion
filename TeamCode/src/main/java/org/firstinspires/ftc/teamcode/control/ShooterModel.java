package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;

/** Flywheel speed needed to reach the goal from a given distance. */
@Config
public final class ShooterModel {

    /** Fitted coefficients of speed against distance. */
    public static double quadCoeff = 0.103;   // (rev/min per in^2)
    public static double linearCoeff = -4.53; // (rev/min per in)
    public static double constant = 1700;     // (rev/min)

    private ShooterModel() {
    }

    /**
     * @param distance distance to the goal (in)
     *
     * @return flywheel speed (rev/min)
     */
    public static double velocityForDistance(double distance) {
        return quadCoeff * distance * distance + linearCoeff * distance + constant;
    }
}
