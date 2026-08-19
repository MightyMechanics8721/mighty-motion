package org.firstinspires.ftc.teamcode.control;

/** How far a mechanism coasts after power is cut, fitted against speed. */
public final class StoppingDistance {

    private StoppingDistance() {
    }

    /**
     * Coast distance for a speed, keeping the sign of the speed.
     *
     * @param velocity current speed
     * @param linearCoeff coast per unit speed
     * @param quadCoeff coast per unit speed squared
     *
     * @return coast distance in the same distance unit the coefficients were fitted in
     */
    public static double forVelocity(double velocity, double linearCoeff, double quadCoeff) {
        double speed = Math.abs(velocity);
        return Math.signum(velocity) * (linearCoeff * speed + quadCoeff * speed * speed);
    }
}
