package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.drivetrain.Path;
import org.firstinspires.ftc.teamcode.field.Alliance;

/**
 * The near-side routes, written once for the blue alliance and mirrored for red.
 * <p>
 * Every near routine drives these same nine paths; they only differ in the order they are run.
 * Holding one copy is what stops the two alliances drifting apart, which is how a red waypoint
 * ended up in the blue routine.
 */
public final class NearPaths {

    // Blue-side waypoints. Red is the same list with Y negated.
    static final double[][] FIRST_STEP = {{-51, -51}, {-12, -12}};
    static final double[][] FIRST_HALF_STEP = {{-12, -12}, {12, -36}, {12, -59}};
    static final double[][] SECOND_ROW_TO_SHOOT = {{12, -55}, {12, -46}, {9, -36}, {-8, -14}};
    static final double[][] SHOOT_TO_GATE = {{-8, -14}, {13.5, -30}, {13.5, -48}, {12, -59}};
    static final double[][] GATE_TO_SHOOT = {{13.5, -59}, {12, -46}, {9, -36}, {-8, -14}};
    static final double[][] THIRD_ROW_STEP =
            {{-8, -14}, {12, -20}, {36, -20}, {36, -30}, {36, -36}, {36, -62}};
    static final double[][] THIRD_ROW_TO_SHOOT = {{36, -62}, {-8, -14}};
    static final double[][] FIRST_ROW_STEP = {{-8, -14}, {-12, -34}, {-12, -36}, {-12, -52}};
    static final double[][] FIRST_ROW_TO_SHOOT = {{-12, -52}, {-12, -36}, {-12, -34}, {-8, -14}};

    // Blue-side final headings, in degrees.
    private static final double PRELOAD_HEADING = 45;
    private static final double FIRST_HALF_HEADING = -90;
    private static final double SECOND_TO_SHOOT_HEADING = -50;
    private static final double SHOOT_HEADING = -50;
    private static final double THIRD_ROW_HEADING = -90;
    private static final double THIRD_TO_SHOOT_HEADING = -45;
    private static final double FIRST_ROW_HEADING = -90;
    private static final double FIRST_TO_SHOOT_HEADING = -90;

    public final Path preload;
    public final Path firstHalf;
    public final Path secondToShoot;
    public final Path gate;
    public final Path shoot;
    public final Path thirdRow;
    public final Path thirdToShoot;
    public final Path firstRow;
    public final Path firstToShoot;

    /**
     * @param alliance side of the field to build the routes for
     * @param gateX gate X (in), the same on both alliances
     * @param gateYBlue gate Y written blue-side (in)
     * @param gateHeadingBlueDeg gate heading written blue-side (deg)
     */
    public NearPaths(
            Alliance alliance, double gateX, double gateYBlue, double gateHeadingBlueDeg
    ) {
        double[][] shootToGate = alliance.mirrorWaypoints(SHOOT_TO_GATE);
        shootToGate[shootToGate.length - 1] =
                new double[]{gateX, alliance.mirrorY(gateYBlue)};

        this.preload = path(alliance, FIRST_STEP, PRELOAD_HEADING, false);
        this.firstHalf = path(alliance, FIRST_HALF_STEP, FIRST_HALF_HEADING, false);
        this.secondToShoot =
                path(alliance, SECOND_ROW_TO_SHOOT, SECOND_TO_SHOOT_HEADING, true);
        this.gate = new Path(
                shootToGate,
                Math.toRadians(alliance.mirrorHeadingDeg(gateHeadingBlueDeg)),
                false,
                false
        );
        this.shoot = path(alliance, GATE_TO_SHOOT, SHOOT_HEADING, true);
        this.thirdRow = path(alliance, THIRD_ROW_STEP, THIRD_ROW_HEADING, false);
        this.thirdToShoot = path(alliance, THIRD_ROW_TO_SHOOT, THIRD_TO_SHOOT_HEADING, true);
        this.firstRow = path(alliance, FIRST_ROW_STEP, FIRST_ROW_HEADING, false);
        this.firstToShoot = path(alliance, FIRST_ROW_TO_SHOOT, FIRST_TO_SHOOT_HEADING, true);
    }

    private static Path path(
            Alliance alliance, double[][] blueWaypoints, double blueHeadingDeg, boolean reverse
    ) {
        return new Path(
                alliance.mirrorWaypoints(blueWaypoints),
                Math.toRadians(alliance.mirrorHeadingDeg(blueHeadingDeg)),
                reverse,
                false
        );
    }
}
