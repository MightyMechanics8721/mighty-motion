package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.calculateDistance;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;

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

    private DistanceSensor(HardwareMap hardwareMap) {
        // Initialize digital laser sensors
        laserInput1 = hardwareMap.get(DigitalChannel.class, "bb1"); // BOTTOM
        laserInput2 = hardwareMap.get(DigitalChannel.class, "bb2"); // MIDDLE
        laserInput3 = hardwareMap.get(DigitalChannel.class, "bb3"); // TOP

        laserInput1.setMode(DigitalChannel.Mode.INPUT);
        laserInput2.setMode(DigitalChannel.Mode.INPUT);
        laserInput3.setMode(DigitalChannel.Mode.INPUT);
        Intake.initialize(hardwareMap);
        Indexer.initialize(hardwareMap);
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
//        if (ballCount == 0) {
//            setIntakeSystemPower(1, 0);
//        } else if (balls[0] && ballCount == 1) {
//            setIntakeSystemPower(1, 0.7);
//        } else if (balls[1] && ballCount == 1) {
//            setIntakeSystemPower(1, 0);
//        } else if (balls[0] && balls[1] && ballCount == 2) {
//            setIntakeSystemPower(1, 0.7);
//        } else if (balls[1] && balls[2] && ballCount == 2) {
//            setIntakeSystemPower(1, 0);
//        } else if (ballCount == 3) {
//            setIntakeSystemPower(0, 0);
//        } else {
//            setIntakeSystemPower(0.3, 0.3);
//        }
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
//        // Telemetry for debugging
//        packet.put("Ball 1 Detected", balls[0]);
//        packet.put("Ball 2 Detected", balls[1]);
//        packet.put("Ball 3 Detected", balls[2]);
//        packet.put("Ball Count", ballCount);
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