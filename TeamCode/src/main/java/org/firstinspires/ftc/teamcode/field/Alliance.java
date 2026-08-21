package org.firstinspires.ftc.teamcode.field;

/**
 * Which side of the field the robot is playing from.
 * <p>
 * The field is symmetric about the X axis, so every blue-side coordinate maps to its red-side
 * counterpart by flipping the sign of Y (and of a heading). Routines are written blue-side and
 * mirrored through here rather than duplicated per alliance.
 * <p>
 * <strong>This codebase puts blue at negative Y.</strong> The official FTC field coordinate system
 * is the other way round: the origin is the centre of the field, +Y runs away from the Red Wall,
 * so blue is at positive Y and red at negative Y. Every waypoint here is tuned to the inverted
 * convention, so it is not something to "fix" -- but anything arriving in official field
 * coordinates, such as an AprilTag or Limelight pose, needs its Y negated before it is compared
 * with a pose from this code.
 */
public enum Alliance {
    BLUE(1.0),
    RED(-1.0);

    private final double yMirror;

    Alliance(double yMirror) {
        this.yMirror = yMirror;
    }

    /**
     * @param blueY a Y coordinate written for the blue alliance (in)
     *
     * @return that coordinate on this alliance's side
     */
    public double mirrorY(double blueY) {
        return yMirror * blueY;
    }

    /**
     * @param blueHeadingDeg a heading written for the blue alliance (deg)
     *
     * @return that heading on this alliance's side
     */
    public double mirrorHeadingDeg(double blueHeadingDeg) {
        return yMirror * blueHeadingDeg;
    }

    /**
     * @param blueWaypoints a path written for the blue alliance
     *
     * @return a new array holding that path on this alliance's side
     */
    public double[][] mirrorWaypoints(double[][] blueWaypoints) {
        double[][] mirrored = new double[blueWaypoints.length][];
        for (int i = 0; i < blueWaypoints.length; i++) {
            mirrored[i] = new double[]{blueWaypoints[i][0], mirrorY(blueWaypoints[i][1])};
        }
        return mirrored;
    }
}
