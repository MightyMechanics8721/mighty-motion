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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * goBILDA Laser Distance Sensor Example (Digital Mode)
 * <p>
 * This example shows how to read the digital output of the goBILDA
 * Laser Distance Sensor.
 * <p>
 * In Digital Mode, the sensor outputs either HIGH or LOW depending on
 * whether it detects an object in front of it. The onboard potentiometer
 * adjusts the detection distance from approximately 25mm up to 264mm.
 * <p>
 * This sensor is active-HIGH, meaning the output line goes HIGH (3.3V)
 * when an object is detected, and LOW (0V) when no object is present.
 * <p>
 * Wire the sensor to a Digital port on your Hub and name it "laserDigitalInput"
 * in your Robot Configuration.
 * <p>
 * Display:
 * The current detection state is displayed in telemetry.
 */
@TeleOp(name = "DistanceSensorSampleCode")
public class DistanceSensorSampleCode extends LinearOpMode {

    private DigitalChannel laserInput;
    private I2cDeviceSynch prismLED;

    @Override
    public void runOpMode() {
        // Get the digital sensor from the hardware map
        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        // Set the channel as an input
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        prismLED = hardwareMap.get(I2cDeviceSynch.class, "prismLED");
        prismLED.engage();
        // Wait for the driver to press PLAY
        waitForStart();

        // Loop while the OpMode is active
        while (opModeIsActive()) {
            // Read the sensor state (true = HIGH, false = LOW)
            boolean stateHigh = laserInput.getState();

            // Active-HIGH: HIGH means an object is detected
            boolean detected = stateHigh;

            // Display detection state
            if (detected) {
                setLEDColor(255, 0, 0);       // Red = no balls
                telemetry.addLine("Object detected!");
            } else {
                telemetry.addLine("No object detected");
                setLEDColor(0, 255, 0);     // Orange = 1 ball
            }

            // Display the raw HIGH/LOW signal for reference
            telemetry.addData("Raw (HIGH/LOW)", stateHigh);
            telemetry.update();
        }
    }

    private void setLEDColor(int r, int g, int b) {
        byte[] colorData = new byte[]{(byte) r, (byte) g, (byte) b};
        prismLED.write(colorData);  // Send RGB values to Prism LED
    }
}