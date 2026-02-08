package org.firstinspires.ftc.teamcode.Mechanisms.Indexer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain.THRESHOLD_PARAMETERS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
public class Indexer {
    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();
    private static Indexer instance;
    private final DcMotorAdvanced indexMotor;


    private Indexer(HardwareMap hardwareMap) {
        this.indexMotor = new DcMotorAdvanced(
                hardwareMap.get(DcMotorEx.class, "indexer"),
                THRESHOLD_PARAMETERS.maxVoltage,
                THRESHOLD_PARAMETERS.acceptablePowerDifference
        );

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
                indexMotor.setPower(power);
                return false;
            }
        };
    }

    public static class BatteryParameters {

        public double maxVoltage = 12.5; // (V)

    }


}
