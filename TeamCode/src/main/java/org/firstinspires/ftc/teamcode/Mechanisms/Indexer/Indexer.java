package org.firstinspires.ftc.teamcode.Mechanisms.Indexer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
public class Indexer {
    private static Indexer instance;
    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();
    public static ConfigurationNames CONFIGURATION_NAMES = new ConfigurationNames();
    public MotorController motorController;


    private Indexer(HardwareMap hardwareMap) {
        this.motorController = new MotorController(
                hardwareMap,
                new String[]{CONFIGURATION_NAMES.indexerMotorName},
                BATTERY_PARAMETERS.maxVoltage
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

    //    public void setIndexerPowerFunction(double power) {
    //        this.motorController.setPower(power);
    //    }
    public Action setIndexerPower(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                motorController.setPower(power);
                return false;
            }
        };
    }

    public static class ConfigurationNames {

        public String indexerMotorName = "indexer";


    }

    public static class BatteryParameters {

        public double maxVoltage = 12.5; // (V)

    }


}
