/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations.AnimationType;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations.PoliceLights;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Prism Test", group = "Linear OpMode")
@Disabled
public class GoBildaPrismTest extends LinearOpMode {

    public enum Layers {
        LAYER_0(AnimationType.NONE, 0, LayerHeight.LAYER_0),
        LAYER_1(AnimationType.NONE, 1, LayerHeight.LAYER_1),
        LAYER_2(AnimationType.NONE, 2, LayerHeight.LAYER_2),
        LAYER_3(AnimationType.NONE, 3, LayerHeight.LAYER_3),
        LAYER_4(AnimationType.NONE, 4, LayerHeight.LAYER_4),
        LAYER_5(AnimationType.NONE, 5, LayerHeight.LAYER_5),
        LAYER_6(AnimationType.NONE, 6, LayerHeight.LAYER_6),
        LAYER_7(AnimationType.NONE, 7, LayerHeight.LAYER_7),
        LAYER_8(AnimationType.NONE, 8, LayerHeight.LAYER_8),
        LAYER_9(AnimationType.NONE, 9, LayerHeight.LAYER_9);

        private final int index;
        private final LayerHeight layerHeight;
        private AnimationType animationType;

        Layers(AnimationType animationType, int index, LayerHeight layerHeight) {
            this.animationType = animationType;
            this.index = index;
            this.layerHeight = layerHeight;
        }
    }

    private ExecutorService backgroundExecutor = null;

    PrismAnimations.Solid solid = new PrismAnimations.Solid();
    GoBildaPrismDriver prism;
    DigitalChannel laserInput1;
    DigitalChannel laserInput2;
    DigitalChannel laserInput3;

    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        solid.setPrimaryColor(Color.WHITE);
        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);

        // Three digital laser sensors (beam-break style)
        laserInput1 = hardwareMap.get(DigitalChannel.class, "bb1");
        laserInput2 = hardwareMap.get(DigitalChannel.class, "bb2");
        laserInput3 = hardwareMap.get(DigitalChannel.class, "bb3");

        initBackgroundThreads();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        int prevBallCount = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int ballCount = 0;
            ballCount += laserInput1.getState() ? 1 : 0;
            ballCount += laserInput2.getState() ? 1 : 0;
            ballCount += laserInput3.getState() ? 1 : 0;

            if (ballCount != prevBallCount) {
                prevBallCount = ballCount;
                switch (ballCount) {
                    case 1:
                        solid.setPrimaryColor(Color.BLUE);
                        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);
                        break;
                    case 2:
                        prism.clearAllAnimations();
                        solid.setPrimaryColor(Color.ORANGE);
                        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);
                        break;
                    case 3:
                        prism.clearAllAnimations();
                        solid.setPrimaryColor(Color.GREEN);
                        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);
                        break;
                    default:
                        prism.clearAllAnimations();
                        solid.setPrimaryColor(Color.RED);
                        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);
                        break;
                }
            }
            sleep(1);
        }
    }

    public void initBackgroundThreads() {
        backgroundExecutor = ThreadPool.newSingleThreadExecutor("Background Thread");
        backgroundExecutor.submit(backgroundProcessingRunnable);
    }

    public void stopBackgroundThreads() {
        if (backgroundExecutor != null) {
            backgroundExecutor.shutdownNow();
            backgroundExecutor = null;
        }
    }

    private final Runnable backgroundProcessingRunnable = () ->
    {
        int prevBallCount = 0;

        while (!Thread.currentThread().isInterrupted() &&
                (opModeIsActive() || opModeInInit()) &&
                !isStopRequested()) {

            int ballCount = 0;
            ballCount += laserInput1.getState() ? 1 : 0;
            ballCount += laserInput2.getState() ? 1 : 0;
            ballCount += laserInput3.getState() ? 1 : 0;

            if (ballCount != prevBallCount) {
                prevBallCount = ballCount;
                switch (ballCount) {
                    case 1:
                        solid.setPrimaryColor(Color.BLUE);
                        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);
                        break;
                    case 2:
                        prism.clearAllAnimations();
                        solid.setPrimaryColor(Color.ORANGE);
                        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);
                        break;
                    case 3:
                        prism.clearAllAnimations();
                        solid.setPrimaryColor(Color.GREEN);
                        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);
                        break;
                    default:
                        prism.clearAllAnimations();
                        solid.setPrimaryColor(Color.RED);
                        prism.insertAndUpdateAnimation(Layers.LAYER_0.layerHeight, solid);
                        break;
                }
            }
        }
    };
}
