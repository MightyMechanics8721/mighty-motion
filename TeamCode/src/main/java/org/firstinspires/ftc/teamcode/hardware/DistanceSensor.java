package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Indexer;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

public class DistanceSensor {

    private static DistanceSensor instance;
    // Distance Sensors
    private final DigitalChannel laserInput1;
    private final DigitalChannel laserInput2;
    private final DigitalChannel laserInput3;
    private final Intake intake;
    private final Indexer indexer;
    boolean[] balls = new boolean[3];
    int ballCount = 0;

    /**
     * Builds the DistanceSensor from the beam-break channels, and takes the Intake and Indexer that the
     * OpMode has already initialised.
     * <p>
     * It deliberately does not initialise those two itself. Doing so would replace the singletons,
     * leaving any reference the OpMode had already captured pointing at a second object driving the
     * same motors with its own power-deduplication state, so commands would be silently dropped.
     */
    private DistanceSensor(HardwareMap hardwareMap) {
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
        instance = new DistanceSensor(hardwareMap);
    }

    public static DistanceSensor getInstance() {
        if (instance == null) {
            throw new IllegalStateException("DistanceSensor not initialized!");
        }
        return instance;
    }

    public Action ballDetection() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (intake == null || indexer == null) {
                    throw new IllegalStateException("Subsystem not initialized");
                }
                updateBallCount();
                ballDetectionFunction();
                packet.put("Ball 0", balls[0]);
                packet.put("Ball 1", balls[1]);
                packet.put("Ball 2", balls[2]);
                packet.put("Ball Count", ballCount);
                return true;
            }
        };
    }

    public void ballDetectionFunction() {
        if (ballCount == 3) {
            setIntakeSystemPower(0, 0);
        } else if (ballCount == 0) {
            setIntakeSystemPower(1, 0);
        } else if (balls[0] & balls[1]) {
            setIntakeSystemPower(1, 0.7);
        } else if (ballCount == 1) {
            setIntakeSystemPower(1, 0);
        } else if (balls[1] && balls[2]) {
            setIntakeSystemPower(1, 0);
        } else if (ballCount == 2) {
            setIntakeSystemPower(1, 0.7);
        } else {
            setIntakeSystemPower(0.3, 0.3);
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
     * @param intakePower  motor power, [-1, 1]
     * @param indexerPower motor power, [-1, 1]
     */
    public void setIntakeSystemPower(double intakePower, double indexerPower) {
        intake.setIntakePowerFunction(intakePower);
        indexer.setIndexerPowerFunction(indexerPower);
    }
}