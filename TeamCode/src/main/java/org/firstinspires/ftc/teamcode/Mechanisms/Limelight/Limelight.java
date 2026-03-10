package org.firstinspires.ftc.teamcode.Mechanisms.Limelight;

import static org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain.THRESHOLD_PARAMETERS;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.DcMotorAdvanced;

public class Limelight {
    private static Limelight instance;
    private Limelight3A limelight;

    private Limelight(HardwareMap hardwareMap) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public static void initialize(HardwareMap hardwareMap) {
        instance = new Limelight(hardwareMap);
    }

    public Limelight getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Limelight not initialized!");
        }
        return instance;
    }

    public void startScan() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }
}

