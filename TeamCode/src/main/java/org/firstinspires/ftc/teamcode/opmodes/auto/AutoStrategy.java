package org.firstinspires.ftc.teamcode.opmodes.auto;

/**
 * The near-side autonomous routines, and the timings each was tuned with.
 * <p>
 * All three drive the same waypoints; they differ in the order they work the rows and the gate,
 * and in a handful of timings. Held together here so a routine is one menu choice rather than one
 * OpMode per alliance per variant.
 */
public enum AutoStrategy {

    /**
     * Row, gate, row, gate, row, gate. Backs off the gate after each collection.
     * <p>
     * Was BlueNear / RedNear.
     */
    ROW_GATE("Row+Gate", 1.5, 1.75, 0.4, -57.5, -120, 1.0),

    /**
     * Same order as ROW_GATE, retuned.
     * <p>
     * Was BlueNearSolo / RedNearSolo.
     */
    ROW_GATE_SOLO("Row+Gate Solo", 1.35, 2.25, 0.45, -59, -115, 1.0),

    /**
     * Second row, then four gate cycles, then the first row. Does not back off the gate.
     * <p>
     * Was BlueNearWorlds / RedNearWorlds.
     */
    GATE_CYCLE("Gate Cycle", 1.35, 2.25, 0.45, -59, -115, 0.1),

    /**
     * Sit still for the whole period.
     * <p>
     * Last in the list so it is never one press away from a routine. Pick it when the robot is
     * hurt, or when an alliance partner's routine would collide with ours.
     */
    DO_NOTHING("Do Nothing", 0, 0, 0, -59, -115, 0);

    /** Deadline on the path into the gate. (s) */
    public final double followPathTime;
    /** How long the transfer runs while gathering a row. (s) */
    public final double rowTransferTime;
    /** How long the transfer runs to fire a volley. (s) */
    public final double shootTime;
    /** Gate Y on the blue side. (in) */
    public final double gateYBlue;
    /** Gate heading on the blue side. (deg) */
    public final double gateHeadingBlueDeg;
    /** Deadline on swinging the turret round before the preload volley. (s) */
    public final double preloadTurretTime;

    private final String label;

    AutoStrategy(
            String label,
            double followPathTime,
            double rowTransferTime,
            double shootTime,
            double gateYBlue,
            double gateHeadingBlueDeg,
            double preloadTurretTime
    ) {
        this.label = label;
        this.followPathTime = followPathTime;
        this.rowTransferTime = rowTransferTime;
        this.shootTime = shootTime;
        this.gateYBlue = gateYBlue;
        this.gateHeadingBlueDeg = gateHeadingBlueDeg;
        this.preloadTurretTime = preloadTurretTime;
    }

    @Override
    public String toString() {
        return label;
    }
}
