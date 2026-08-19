package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

import static org.firstinspires.ftc.teamcode.drivetrain.Drivetrain.THRESHOLD_PARAMETERS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DcMotorAdvanced;

@Config
public class Indexer {
    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();
    private static Indexer instance;
    private final DcMotorAdvanced indexMotor;

    private Indexer(HardwareMap hardwareMap) {
        this.indexMotor = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, HardwareNames.INDEXER_MOTOR),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference
        );
        this.indexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Indexer(hardwareMap);
    }

    public static Indexer getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Indexer not initialized!");
        }
        return instance;
    }

    public Action setIndexerPower(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setIndexerPowerFunction(power);
                return false;
            }
        };
    }

    public void setIndexerPowerFunction(double power) {
        indexMotor.setPower(power);
    }

    public static class BatteryParameters {

        public double maxVoltage = 12.5; // (V)

    }

}
