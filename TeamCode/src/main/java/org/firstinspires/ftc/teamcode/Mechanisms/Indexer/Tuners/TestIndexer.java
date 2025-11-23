package org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;

@Config
@Autonomous(name = "TestIndexer", group = "5")
public class TestIndexer extends LinearOpMode {

    FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {
        Battery battery = new Battery(hardwareMap);
        Indexer indexer = new Indexer(hardwareMap, battery);

        double power;

        waitForStart();

        while (opModeIsActive()) {
            power = (double) gamepad1.right_trigger;
            Actions.runBlocking(indexer.setIndexerPower(power));
        }
    }
}
