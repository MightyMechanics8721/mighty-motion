package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Wrappers that put a clock on an Action.
 * <p>
 * The clock starts on the first run(), not when the wrapper is built, so a routine can construct
 * its whole action tree up front.
 */
public final class Timed {

    private Timed() {
    }

    /**
     * Runs the action until it finishes or the deadline passes, whichever comes first.
     *
     * @param action the action to run
     * @param seconds deadline (s); zero or less means no deadline
     */
    public static Action deadline(Action action, double seconds) {
        return new Bounded(action, seconds, null, false);
    }

    /**
     * Runs the action until it finishes or the deadline passes, then runs onTimeout once if the
     * deadline was what ended it. Use onTimeout to cut motor power.
     *
     * @param action the action to run
     * @param seconds deadline (s); zero or less means no deadline
     * @param onTimeout run once when the deadline ends the action
     */
    public static Action deadline(Action action, double seconds, Runnable onTimeout) {
        return new Bounded(action, seconds, onTimeout, false);
    }

    /**
     * Runs the action for the full duration, ignoring whether the action reports itself finished.
     * A finished action keeps being run until the time is up.
     *
     * @param action the action to run
     * @param seconds how long to run (s)
     */
    public static Action forDuration(Action action, double seconds) {
        return new Bounded(action, seconds, null, true);
    }

    /**
     * Runs the action once, then finishes.
     */
    public static Action once(Action action) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                action.run(packet);
                return false;
            }
        };
    }

    /**
     * Does nothing for the given time, then finishes. Zero or less finishes straight away.
     * <p>
     * Note this differs from the deadline wrappers, where zero means "no deadline": a pause of no
     * time is a pause of no time, not a pause forever.
     * <p>
     * Named pause rather than wait because a static wait(double) sits alongside the inherited
     * Object.wait(long), and an int argument binds to the inherited one.
     *
     * @param seconds how long to pause (s)
     */
    public static Action pause(double seconds) {
        if (seconds <= 0) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    return false;
                }
            };
        }
        return forDuration(new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return true;
            }
        }, seconds);
    }

    private static class Bounded implements Action {
        private final Action action;
        private final double seconds;
        private final Runnable onTimeout;
        private final boolean ignoreActionFinish;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean started = false;
        private boolean timedOut = false;

        Bounded(Action action, double seconds, Runnable onTimeout, boolean ignoreActionFinish) {
            this.action = action;
            this.seconds = seconds;
            this.onTimeout = onTimeout;
            this.ignoreActionFinish = ignoreActionFinish;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                timer.reset();
                started = true;
            }
            boolean expired = seconds > 0 && timer.seconds() >= seconds;
            if (expired) {
                if (!timedOut) {
                    timedOut = true;
                    if (onTimeout != null) {
                        onTimeout.run();
                    }
                }
                return false;
            }
            boolean stillRunning = action.run(packet);
            return ignoreActionFinish || stillRunning;
        }
    }
}
