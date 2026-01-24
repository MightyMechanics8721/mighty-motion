package org.firstinspires.ftc.teamcode.Prism;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Indexer.Indexer;

import java.util.concurrent.ExecutorService;

@Autonomous(name = "Test Prism")
public class TunePrism extends LinearOpMode {
    private ExecutorService backgroundExecutor = null;
    Prism prism;

    @Override
    public void runOpMode() throws InterruptedException {
        prism = new Prism(hardwareMap);
        this.initBackgroundThreads();

        waitForStart();
        while (opModeIsActive()) {
            sleep(1);
        }
        this.stopBackgroundThreads();
    }

    private final Runnable backgroundProcessingRunnable = () ->
    {
        while (opModeIsActive() || opModeInInit()) {
            prism.runCheck();
            sleep(1);
        }
    };

    public void initBackgroundThreads() {
        this.backgroundExecutor = ThreadPool.newSingleThreadExecutor("Background Thread");
        this.backgroundExecutor.submit(backgroundProcessingRunnable);
    }

    public void stopBackgroundThreads() {
        if (backgroundExecutor != null) {
            backgroundExecutor.shutdownNow();
            backgroundExecutor = null;
        }
    }
}
