package org.firstinspires.ftc.teamcode.Mechanisms.Intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;

import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers.MotorController;

@Config
public class Intake {

    public static BatteryParameters BATTERY_PARAMETERS = new BatteryParameters();

    public static ConfigurationNames CONFIGURATION_NAMES = new ConfigurationNames();
    private MotorController motorController;

    private DcMotorEx intake;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        this.motorController = new MotorController(
                hardwareMap,
                new String[]{CONFIGURATION_NAMES.intakeMotorName},
                BATTERY_PARAMETERS.maxVoltage
        );

    }

    //        public void setIntakePower(double power) {
    //        this.motorController.setPower(power);
    //    }
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
