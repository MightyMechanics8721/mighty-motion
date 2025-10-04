package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "BeakBreakTesting", group = "123123123")
public class BeamBreaker extends LinearOpMode {
    HardwareMap hardwareMap;

    DigitalChannel beamBreak;

    public BeamBreaker(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void runOpMode() {
        beamBreak = hardwareMap.get(DigitalChannel.class, "beam");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        waitForStart();

        while (opModeIsActive()) {
            if (beamBreak.getState()) {
                telemetryPacket.put("Beam", "Unbroken");
            } else {
                telemetryPacket.put("Beam", "Broken");
            }
            telemetry.update();
        }
    }

}
