package org.firstinspires.ftc.teamcode.Mechanisms.Indexer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
public class Indexer {

    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();
    public static ConfigurationNames CONFIGURATION_NAMES = new ConfigurationNames();
    private Battery battery;
    public MotorController motorController;


    public Indexer(HardwareMap hardwareMap, Battery battery) {
        this.battery = battery;
        this.motorController = new MotorController(
                hardwareMap,
                new String[]{CONFIGURATION_NAMES.indexerMotorName},
                battery, BATTERY_PARAMETERS.maxVoltage
        );

    }

    //    public void setIndexerPower(double power) {
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
