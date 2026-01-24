package org.firstinspires.ftc.teamcode.Prism;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Testing.GoBildaPrismTest;

@Config
public class Prism {
    GoBildaPrismDriver prism;
    PrismAnimations.Solid solid = new PrismAnimations.Solid();
    int ballCount = 0;
    int prevBallCount = 0;
    private DigitalChannel laserInput1;
    private DigitalChannel laserInput2;
    private DigitalChannel laserInput3;

    public Prism(HardwareMap hardwareMap) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        // Initialize digital laser sensors
        laserInput1 = hardwareMap.get(DigitalChannel.class, "bb1");
        laserInput2 = hardwareMap.get(DigitalChannel.class, "bb2");
        laserInput3 = hardwareMap.get(DigitalChannel.class, "bb3");

        solid.setPrimaryColor(0, 255, 0);
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solid);
    }


    public Action ballCheck(boolean run) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ballCount = 0;
                ballCount += laserInput1.getState() ? 1 : 0;
                ballCount += laserInput2.getState() ? 1 : 0;
                ballCount += laserInput3.getState() ? 1 : 0;

                if (ballCount != prevBallCount) {
                    prevBallCount = ballCount;
                    prism.clearAllAnimations();
                    switch (ballCount) {
                        case 1:
                            solid.setPrimaryColor(Color.BLUE);
                            break;
                        case 2:
                            solid.setPrimaryColor(Color.ORANGE);
                            break;
                        case 3:
                            solid.setPrimaryColor(Color.GREEN);
                            break;
                        default:
                            solid.setPrimaryColor(Color.RED);
                            break;
                    }
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solid);
                }
                return !run;
            }

        };
    }
}