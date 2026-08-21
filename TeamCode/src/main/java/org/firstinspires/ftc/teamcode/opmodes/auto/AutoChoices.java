package org.firstinspires.ftc.teamcode.opmodes.auto;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.field.Alliance;

/**
 * What the driver picked on the init menu.
 *
 * @param <S> the routine enum this OpMode chooses between, such as {@link AutoStrategy} or
 * {@link FarStrategy}
 */
public class AutoChoices<S extends Enum<S>> {

    public Alliance alliance;
    public S strategy;
    /** How long to sit still before moving, to let a partner clear. (s) */
    public double startDelaySeconds;

    public AutoChoices(Alliance alliance, S strategy, double startDelaySeconds) {
        this.alliance = alliance;
        this.strategy = strategy;
        this.startDelaySeconds = startDelaySeconds;
    }

    @Override
    public String toString() {
        return String.format(
                Locale.US,
                "alliance=%s strategy=%s startDelay=%.1fs",
                alliance, strategy, startDelaySeconds
        );
    }
}
