package org.firstinspires.ftc.teamcode.opmodes.auto;

/**
 * The far-side autonomous routines, and the timings each was tuned with.
 * <p>
 * Unlike the near routines these are genuinely different plans, not one plan mirrored. They share
 * most of their waypoints, which is why they live together, but each works the field its own way.
 */
public enum FarStrategy {

    /**
     * Third row, then six trips to the human player, shooting between each.
     * <p>
     * Was BlueFar. Only ever run on blue.
     */
    HUMAN_PLAYER_CYCLE("HP Cycle", 1.5, 1.5, 0.5),

    /**
     * Third row, one human player collection, then the lingering balls.
     * <p>
     * Was RedFar. Only ever run on red.
     */
    THIRD_ROW_LINGER("3rd Row + Linger", 2.5, 2.0, 1.5),

    /** Fire the preload from the start line, then nudge clear. Was BlueFarSimple. */
    SIMPLE_SHOOT("Simple Shoot", 2.0, 0.4, 0.5),

    /** Drive off the line and stop. Was BlueFarMove. */
    MOVE_ONLY("Move Only", 0, 0, 0),

    /**
     * Sit still for the whole period.
     * <p>
     * Last in the list so it is never one press away from a routine. Pick it when the robot is
     * hurt, or when an alliance partner's routine would collide with ours.
     */
    DO_NOTHING("Do Nothing", 0, 0, 0);

    /** How long the transfer runs while gathering a row. (s) */
    public final double rowTransferTime;
    /** Deadline on a shooting path. (s) */
    public final double shootPathTime;
    /** Deadline on the opening nudge off the wall. (s) */
    public final double openingPathTime;

    private final String label;

    FarStrategy(
            String label, double rowTransferTime, double shootPathTime, double openingPathTime
    ) {
        this.label = label;
        this.rowTransferTime = rowTransferTime;
        this.shootPathTime = shootPathTime;
        this.openingPathTime = openingPathTime;
    }

    @Override
    public String toString() {
        return label;
    }
}
