package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.drivetrain.Path;
import org.firstinspires.ftc.teamcode.field.Alliance;

/**
 * The far-side routes.
 * <p>
 * Most are written blue-side and mirrored. Three are not, and are held per alliance instead:
 * the run out to the human player took a different line on each side, and two headings were
 * tuned to something other than their mirror. Those are the real divergences between the two
 * far routines, so they are stated here rather than hidden in two near-copies of one file.
 */
public final class FarPaths {

    // ----- mirrored blue-side waypoints -----
    static final double[][] PRE_C = {{64, -24}, {60, -24}};
    static final double[][] HP_TO_SHOOT = {{60, -60}, {60, -48}, {60, -24}};
    static final double[][] SHOOT_TO_THIRD_ROW = {{60, -24}, {36, -15}, {36, -62}};
    static final double[][] THIRD_ROW_TO_SHOOT = {{36, -62}, {50, -30}, {60, -24}};
    static final double[][] SHOOT_TO_LINGERING = {{60, -24}, {45, -36}, {45, -48}, {55, -62}};
    static final double[][] LINGERING_TO_SHOOT = {{55, -60}, {60, -24}};
    static final double[][] PRE_CU = {{60, -24}, {60, -36}};

    // ----- per alliance, because these were never mirrors of each other -----
    /** Blue ran straight down x = 60; red bent inboard to x = 45 and stopped a foot short. */
    static final double[][] SHOOT_TO_HP_BLUE = {{60, -24}, {60, -36}, {60, -48}, {60, -62}};
    static final double[][] SHOOT_TO_HP_RED = {{60, 24}, {45, 36}, {45, 48}, {46, 63}};
    /** Only red ever drove this leg; blue's copy was commented out. */
    static final double[][] HP_CORNER_RED = {{46, 63}, {64, 64}};

    /** Blue held -90 down this leg, red held 0. Not a mirror; both as tuned. */
    private static final double SHOOT_TO_HP_HEADING_BLUE = -90;
    private static final double SHOOT_TO_HP_HEADING_RED = 0;
    /** Blue held -140, red held 170. Not a mirror; both as tuned. */
    private static final double SHOOT_TO_LINGERING_HEADING_BLUE = -140;
    private static final double SHOOT_TO_LINGERING_HEADING_RED = 170;

    public final Path openingNudge;
    public final Path shootToThirdRow;
    public final Path thirdRowToShoot;
    public final Path shootToHumanPlayer;
    /** The corner leg after the human player. Null on blue, which never drove it. */
    public final Path humanPlayerCorner;
    public final Path humanPlayerToShoot;
    public final Path shootToLingering;
    public final Path lingeringToShoot;
    public final Path finishNudge;

    public FarPaths(Alliance alliance) {
        boolean red = alliance == Alliance.RED;

        this.openingNudge = staticHeading(alliance, PRE_C, 180);
        this.shootToThirdRow = staticHeading(alliance, SHOOT_TO_THIRD_ROW, -90);
        this.thirdRowToShoot = reversed(alliance, THIRD_ROW_TO_SHOOT, 180);
        this.humanPlayerToShoot = reversed(alliance, HP_TO_SHOOT, -90);
        this.lingeringToShoot = reversed(alliance, LINGERING_TO_SHOOT, 180);
        this.finishNudge = new Path(
                alliance.mirrorWaypoints(PRE_CU),
                Math.toRadians(180),
                false,
                false
        );

        this.shootToHumanPlayer = new Path(
                red ? SHOOT_TO_HP_RED : SHOOT_TO_HP_BLUE,
                Math.toRadians(red ? SHOOT_TO_HP_HEADING_RED : SHOOT_TO_HP_HEADING_BLUE),
                false,
                true
        );
        this.shootToLingering = new Path(
                alliance.mirrorWaypoints(SHOOT_TO_LINGERING),
                Math.toRadians(red
                        ? SHOOT_TO_LINGERING_HEADING_RED
                        : SHOOT_TO_LINGERING_HEADING_BLUE),
                false,
                true
        );
        this.humanPlayerCorner = red
                ? new Path(HP_CORNER_RED, Math.toRadians(90), false, false)
                : null;
    }

    private static Path staticHeading(
            Alliance alliance, double[][] blueWaypoints, double blueHeadingDeg
    ) {
        return new Path(
                alliance.mirrorWaypoints(blueWaypoints),
                Math.toRadians(alliance.mirrorHeadingDeg(blueHeadingDeg)),
                false,
                true
        );
    }

    private static Path reversed(
            Alliance alliance, double[][] blueWaypoints, double blueHeadingDeg
    ) {
        return new Path(
                alliance.mirrorWaypoints(blueWaypoints),
                Math.toRadians(alliance.mirrorHeadingDeg(blueHeadingDeg)),
                true,
                false
        );
    }
}
