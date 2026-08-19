package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.junit.Test;

/** Action timer wrappers. */
public class TimedTest {

    private final TelemetryPacket packet = new TelemetryPacket();

    /** Counts its runs and reports finished after the given number of them. */
    private static class Counting implements Action {
        int runs = 0;
        final int finishAfter;

        Counting(int finishAfter) {
            this.finishAfter = finishAfter;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            runs++;
            return runs < finishAfter;
        }
    }

    @Test
    public void onceRunsTheActionExactlyOnceAndFinishes() {
        Counting inner = new Counting(100);
        Action a = Timed.once(inner);
        assertFalse("once() must report finished", a.run(packet));
        assertEquals(1, inner.runs);
    }

    @Test
    public void deadlineFinishesWhenTheActionFinishes() {
        Counting inner = new Counting(3);
        Action a = Timed.deadline(inner, 30);
        assertTrue(a.run(packet));
        assertTrue(a.run(packet));
        assertFalse("should end with the inner action, not wait for the clock", a.run(packet));
        assertEquals(3, inner.runs);
    }

    @Test
    public void forDurationKeepsRunningAFinishedAction() {
        Counting inner = new Counting(1);
        Action a = Timed.forDuration(inner, 30);
        assertTrue(a.run(packet));
        assertTrue("forDuration must ignore the inner action finishing", a.run(packet));
        assertTrue(a.run(packet));
        assertEquals(3, inner.runs);
    }

    @Test
    public void zeroSecondsMeansNoDeadline() {
        Counting inner = new Counting(2);
        Action a = Timed.deadline(inner, 0);
        assertTrue(a.run(packet));
        assertFalse(a.run(packet));
        assertEquals(2, inner.runs);
    }

    @Test
    public void theClockStartsOnFirstRunNotOnConstruction() throws InterruptedException {
        Counting inner = new Counting(100);
        Action a = Timed.deadline(inner, 0.15);
        Thread.sleep(250);
        assertTrue("an action built early must still get its full budget", a.run(packet));
        assertEquals(1, inner.runs);
    }

    @Test
    public void deadlineEndsTheActionWhenTimeRunsOut() throws InterruptedException {
        Counting inner = new Counting(100);
        Action a = Timed.deadline(inner, 0.1);
        assertTrue(a.run(packet));
        Thread.sleep(150);
        assertFalse("the deadline must end it", a.run(packet));
    }

    @Test
    public void onTimeoutRunsOnceAndOnlyOnTimeout() throws InterruptedException {
        int[] fired = {0};
        Action a = Timed.deadline(new Counting(100), 0.1, () -> fired[0]++);
        a.run(packet);
        assertEquals("must not fire before the deadline", 0, fired[0]);
        Thread.sleep(150);
        a.run(packet);
        a.run(packet);
        assertEquals("must fire exactly once", 1, fired[0]);
    }

    @Test
    public void onTimeoutDoesNotRunWhenTheActionFinishesFirst() {
        int[] fired = {0};
        Action a = Timed.deadline(new Counting(2), 30, () -> fired[0]++);
        a.run(packet);
        a.run(packet);
        assertEquals(0, fired[0]);
    }

    @Test
    public void theInnerActionIsNotRunAfterTheDeadline() throws InterruptedException {
        Counting inner = new Counting(100);
        Action a = Timed.deadline(inner, 0.1);
        a.run(packet);
        int before = inner.runs;
        Thread.sleep(150);
        a.run(packet);
        assertEquals("no further runs once expired", before, inner.runs);
    }

    @Test
    public void waitFinishesAfterItsTime() throws InterruptedException {
        Action a = Timed.wait(0.1);
        assertTrue(a.run(packet));
        Thread.sleep(150);
        assertFalse(a.run(packet));
    }
}
