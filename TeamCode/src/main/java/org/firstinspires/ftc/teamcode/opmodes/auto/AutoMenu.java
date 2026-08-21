package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.field.Alliance;

/**
 * The init-time menu that picks which autonomous runs.
 * <p>
 * Driven from gamepad 1 while the OpMode is initialised but not started: dpad up/down moves
 * between rows, dpad left/right changes the value on the selected row. One OpMode covering every
 * permutation beats one OpMode per permutation, which is how the alliance-mirrored routines
 * drifted apart from each other in the first place.
 * <p>
 * This works from FtcDashboard as well as the Driver Station. The dashboard forwards gamepad
 * state through the SDK's own Gamepad, so the WasPressed edge detection below behaves the same
 * either way, but it declines to do so while a physical gamepad is claimed. The caller is
 * expected to have pointed telemetry at both surfaces; see NearAuto.
 */
public final class AutoMenu {

    private static final int ROW_ALLIANCE = 0;
    private static final int ROW_STRATEGY = 1;
    private static final int ROW_DELAY = 2;
    private static final int ROW_COUNT = 3;

    private static final double DELAY_STEP = 0.5; // (s)
    private static final double DELAY_MAX = 15.0; // (s)

    private AutoMenu() {
    }

    /**
     * Blocks until the OpMode is started or stopped, letting the driver edit the choices.
     *
     * @param opMode the running OpMode, for gamepad, telemetry and start state
     * @param choices seeded with the defaults; edited in place and returned
     *
     * @return the same object, holding whatever the driver settled on
     */
    public static <S extends Enum<S>> AutoChoices<S> select(
            LinearOpMode opMode, AutoChoices<S> choices
    ) {
        int row = ROW_ALLIANCE;

        while (!opMode.isStarted() && !opMode.isStopRequested()) {
            if (opMode.gamepad1.dpadDownWasPressed()) {
                row = (row + 1) % ROW_COUNT;
            }
            if (opMode.gamepad1.dpadUpWasPressed()) {
                row = (row + ROW_COUNT - 1) % ROW_COUNT;
            }
            if (opMode.gamepad1.dpadRightWasPressed()) {
                change(choices, row, 1);
            }
            if (opMode.gamepad1.dpadLeftWasPressed()) {
                change(choices, row, -1);
            }

            render(opMode, choices, row);
            opMode.idle();
        }
        return choices;
    }

    private static <S extends Enum<S>> void change(
            AutoChoices<S> choices, int row, int direction
    ) {
        switch (row) {
            case ROW_ALLIANCE:
                choices.alliance = cycle(Alliance.values(), choices.alliance, direction);
                break;
            case ROW_STRATEGY:
                // The enum instance knows its own constants, so the menu works for any routine set.
                choices.strategy = cycle(
                        choices.strategy.getDeclaringClass().getEnumConstants(),
                        choices.strategy,
                        direction
                );
                break;
            case ROW_DELAY:
                double delay = choices.startDelaySeconds + direction * DELAY_STEP;
                choices.startDelaySeconds = Math.max(0.0, Math.min(DELAY_MAX, delay));
                break;
        }
    }

    private static <T> T cycle(T[] values, T current, int direction) {
        int i = 0;
        for (int k = 0; k < values.length; k++) {
            if (values[k] == current) {
                i = k;
                break;
            }
        }
        return values[((i + direction) % values.length + values.length) % values.length];
    }

    private static void render(LinearOpMode opMode, AutoChoices<?> choices, int row) {
        opMode.telemetry.addLine("dpad up/down to move, left/right to change, PLAY to run");
        opMode.telemetry.addLine(line(row == ROW_ALLIANCE, "Alliance", choices.alliance));
        opMode.telemetry.addLine(line(row == ROW_STRATEGY, "Routine", choices.strategy));
        opMode.telemetry.addLine(line(row == ROW_DELAY, "Start delay",
                String.format("%.1f s", choices.startDelaySeconds)));
        opMode.telemetry.addLine("");
        opMode.telemetry.addLine("Driving this from FtcDashboard? Leave the Driver Station");
        opMode.telemetry.addLine("gamepad unbound - the dashboard only supplies input when");
        opMode.telemetry.addLine("no physical gamepad is claimed.");
        opMode.telemetry.update();
    }

    private static String line(boolean selected, String name, Object value) {
        return String.format("%s %-12s %s", selected ? ">" : " ", name, value);
    }
}
