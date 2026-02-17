package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils.calculateDistance;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;

public class DistanceSensor {

    private static DistanceSensor instance;
    // Distance Sensors
    private final DigitalChannel laserInput1;
    private final DigitalChannel laserInput2;
    private final DigitalChannel laserInput3;
    public Intake intake;
    public Indexer indexer;
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
//        Intake.initialize(hardwareMap);
//        Indexer.initialize(hardwareMap);
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
                updateBallCount();
                ballDetectionFunction();
                indexer.setIndexerPower(1);
                packet.put("ZBall 0 Detected", balls[0]);
                packet.put("ZBall 1 Detected", balls[1]);
                packet.put("ZBall 2 Detected", balls[2]);
                packet.put("ZRaw 0", laserInput1.getState());
                packet.put("ZRaw 1", laserInput2.getState());
                packet.put("ZRaw 2", laserInput3.getState());
                packet.put("ZBall Count", ballCount);
                return false;
            }
        };
    }

    public void ballDetectionFunction() {
//        updateBallCount();
        intake.setIntakePower(1);

//        if (ballCount == 0) {
//            intake.setIntakePower(1);
//            indexer.setIndexerPower(0);
//        } else if (balls[0] && ballCount == 1) {
//            intake.setIntakePower(1);
//            indexer.setIndexerPower(0.7);
//        } else if (balls[1] && ballCount == 1) {
//            intake.setIntakePower(1);
//            indexer.setIndexerPower(0);
//        } else if (balls[0] && balls[1] && ballCount == 2) {
//            intake.setIntakePower(1);
//            indexer.setIndexerPower(0.7);
//        } else if (balls[1] && balls[2] && ballCount == 2) {
//            intake.setIntakePower(1);
//            indexer.setIndexerPower(0);
//        } else if (ballCount == 3) {
//            intake.setIntakePower(0);
//            indexer.setIndexerPower(0);
//        } else {
//            intake.setIntakePower(0.3);
//            indexer.setIndexerPower(0.3);
//        }
//        // Telemetry for debugging
//        packet.addData("Ball 1 Detected", balls[0]);
//        packet.addData("Ball 2 Detected", balls[1]);
//        packet.addData("Ball 3 Detected", balls[2]);
//        packet.addData("Ball Count", ballCount);
//        packet.update();
    }

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
}