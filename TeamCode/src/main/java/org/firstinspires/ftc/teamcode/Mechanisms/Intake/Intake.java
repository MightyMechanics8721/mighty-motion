package org.firstinspires.ftc.teamcode.Mechanisms.Intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;

import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
public class Intake {
    private static Intake instance;
    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();

    public static ConfigurationNames CONFIGURATION_NAMES = new ConfigurationNames();
    private final MotorController motorController;

    private DcMotorEx intake;

    private Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        this.motorController = new MotorController(
                hardwareMap,
                new String[]{CONFIGURATION_NAMES.intakeMotorName},
                BATTERY_PARAMETERS.maxVoltage
        );

    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Intake(hardwareMap);
    }

    public static Intake getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Intake not initialized!");
        }
        return instance;
    }

    public Action setIntakePower(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(power);

                return false;
            }
        };
    }

    public static class ConfigurationNames {

        public String intakeMotorName = "intake";


    }

    public static class BatteryParameters {

        public double maxVoltage = 12.5; // (V)

    }


}
