package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer {

    private static Transfer instance;
    // Distance Sensors
    private final DigitalChannel laserInput1;
    private final DigitalChannel laserInput2;
    private final DigitalChannel laserInput3;
    private final Intake intake;
    private final Indexer indexer;
    public int ballCount = 0;
    boolean[] balls = new boolean[3];

    /** Takes the Intake and Indexer the OpMode already built. Do not initialise them here. */
    private Transfer(HardwareMap hardwareMap) {
        // Initialize digital laser sensors
        laserInput1 = hardwareMap.get(DigitalChannel.class, "bb1"); // BOTTOM
        laserInput2 = hardwareMap.get(DigitalChannel.class, "bb2"); // MIDDLE
        laserInput3 = hardwareMap.get(DigitalChannel.class, "bb3"); // TOP

        laserInput1.setMode(DigitalChannel.Mode.INPUT);
        laserInput2.setMode(DigitalChannel.Mode.INPUT);
        laserInput3.setMode(DigitalChannel.Mode.INPUT);
        intake = Intake.getInstance();
        indexer = Indexer.getInstance();
    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Transfer(hardwareMap);
    }

    public static Transfer getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Transfer not initialized!");
        }
        return instance;
    }

    /**
     * Powers Intake and Indexer based on balls detected in Intake System ----- INITIALIZE Intake
     * AND Indexer -----
     */
    public Action ballDetection() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (intake == null || indexer == null) {
                    throw new IllegalStateException("Subsystem not initialized");
                }
                updateBallCount();
                ballDetectionFunction();
                //                packet.put("Ball 0", balls[0]);
                //                packet.put("Ball 1", balls[1]);
                //                packet.put("Ball 2", balls[2]);
                //                packet.put("Ball Count", ballCount);
                return true;
            }
        };
    }

    /**
     * Powers Intake and Indexer based on balls detected in Intake System ----- INITIALIZE Intake
     * AND Indexer -----
     */
    public Action ballDetectionTimed(double seconds) {
        return new Action() {
            private double time = -1;
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (time < 0) {
                    timer.reset();
                }
                time = timer.seconds();
                if (time < seconds) {
                    updateBallCount();
                    ballDetectionFunction();
                } else {
                    setIntakeIndexerPowerFunction(0, 0);
                    return false;
                }
                return ballCount != 3;
            }
        };
    }

    /**
     * Powers Intake and Indexer based on balls detected in Intake System
     */
    public void ballDetectionFunction() {
        if (ballCount == 3) {
            setIntakeIndexerPowerFunction(0, 0);
        } else if (ballCount == 0) {
            setIntakeIndexerPowerFunction(1, 0);
        } else if (balls[0] && balls[1]) {
            setIntakeIndexerPowerFunction(1, 0.7);
        } else if (ballCount == 1) {
            setIntakeIndexerPowerFunction(1, 0);
        } else if (balls[1] && balls[2]) {
            setIntakeIndexerPowerFunction(1, 0);
        } else if (ballCount == 2) {
            setIntakeIndexerPowerFunction(1, 0.7);
        } else {
            setIntakeIndexerPowerFunction(0.3, 0.3);
        }
    }

    /**
     * Detect balls & update ballCount
     */
    public void updateBallCount() {
        // Read each sensor: true = object detected (HIGH), false = no object (LOW)
        balls[0] = laserInput1.getState();
        balls[1] = laserInput2.getState();
        balls[2] = laserInput3.getState();
        // Count balls
        ballCount = 0;
        if (balls[0]) ballCount++;
        if (balls[1]) ballCount++;
        if (balls[2]) ballCount++;
    }

    /**
     * Sets power of intake and indexer motor
     *
     * @param intakePower motor power, [-1, 1]
     * @param indexerPower motor power, [-1, 1]
     */
    public void setIntakeIndexerPowerFunction(double intakePower, double indexerPower) {
        intake.setIntakePowerFunction(intakePower);
        indexer.setIndexerPowerFunction(indexerPower);
    }

    /**
     * Sets power of  Intake and Indexer using Inputted values ----- INITIALIZE Intake AND Indexer
     * -----
     */
    public Action setIntakeIndexerPower(double intakePower, double indexerPower) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setIntakeIndexerPowerFunction(intakePower, indexerPower);
                return false;
            }
        };
    }
}