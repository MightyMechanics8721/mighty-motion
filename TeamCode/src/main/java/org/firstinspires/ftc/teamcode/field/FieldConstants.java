package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * The field points every OpMode aims at, in one place.
 * <p>
 * Values are written for the blue alliance and mirrored for red, so retuning the goal here moves
 * every autonomous and both TeleOps together. Tunable from the dashboard.
 */
@Config
public final class FieldConstants {

    /** Point the turret points at. */
    public static double turretAimX = -70;
    public static double turretAimYBlue = -70;

    /**
     * Point the shooter measures its range to.
     * <p>
     * Deliberately not the turret aim point: the turret tracks the goal itself, while the range
     * model was fitted against distance to this point.
     */
    public static double shooterRangeX = -60;
    public static double shooterRangeYBlue = -60;

    /** Where the near routines start, used as the TeleOp fallback when no Autonomous ran. */
    public static double nearStartX = -51;
    public static double nearStartYBlue = -51;
    public static double nearStartHeadingBlueDeg = 45;

    private FieldConstants() {
    }

    /** Goal point for the turret to aim at. */
    public static Vector2d turretAimPoint(Alliance alliance) {
        return new Vector2d(turretAimX, alliance.mirrorY(turretAimYBlue));
    }

    /** Point the shooter ranges to when choosing a flywheel speed. */
    public static Vector2d shooterRangePoint(Alliance alliance) {
        return new Vector2d(shooterRangeX, alliance.mirrorY(shooterRangeYBlue));
    }

    /** Starting pose of the near routines, as a 3x1 of x (in), y (in), heading (rad). */
    public static SimpleMatrix nearStartPose(Alliance alliance) {
        return Utils.makePoseVector(
                nearStartX,
                alliance.mirrorY(nearStartYBlue),
                alliance.mirrorHeadingDeg(nearStartHeadingBlueDeg)
        );
    }
}
