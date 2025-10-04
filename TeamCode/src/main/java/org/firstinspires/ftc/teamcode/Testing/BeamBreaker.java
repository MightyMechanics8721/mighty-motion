package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "BeamBreakTesting", group = "123123123")
public class BeamBreaker extends LinearOpMode {

    @Override
    public void runOpMode() {
        DigitalChannel beamBreak;
        DigitalChannel beamBreak2;
        beamBreak = hardwareMap.get(DigitalChannel.class, "beam1");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        beamBreak.setState(true);
        beamBreak2 = hardwareMap.get(DigitalChannel.class, "beam2");
        beamBreak2.setMode(DigitalChannel.Mode.OUTPUT);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Beam", beamBreak.getState());
            telemetry.update();
        }
    }

}
