package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.FFConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.Constants.PIDConstants;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
public class Indexer {

    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();
    public static ConfigurationNames CONFIGURATION_NAMES = new ConfigurationNames();
    private Battery battery;
    private MotorController motorController;


    public Indexer(HardwareMap hardwareMap, Battery battery) {

        this.motorController = new MotorController(hardwareMap,
                new String[]{this.CONFIGURATION_NAMES.indexerMotorName},
                battery, this.BATTERY_PARAMETERS.maxVoltage);

    }

    public void setPower(double power) {
        this.motorController.setPower(power);
    }

    public static class ConfigurationNames {

        public String indexerMotorName = "indexer";


    }

    public static class BatteryParameters {

        public double maxVoltage = 12.5; // (V)

    }


}
