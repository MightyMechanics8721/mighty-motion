package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class TestOpMode extends OpMode {
//    public static class Parameters {
//        public double power = 0;
//    }
    public static Parameters parameters = new Parameters();


    private final FtcDashboard ftcDashboard = FtcDashboard.getInstance();



    @Override
    public void init() {
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Power", parameters.power);
        ftcDashboard.sendTelemetryPacket(packet);
    }



}
