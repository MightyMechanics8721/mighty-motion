package org.firstinspires.ftc.teamcode.opmodes.tuning;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

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

/**
 * Reads the analog sensor configured as HardwareNames.CURRENT_SENSOR and prints its voltage
 * alongside a linear conversion of it.
 * <p>
 * Adapted from goBILDA's analog laser distance sensor sample. The scale below is whatever was last
 * dialled in on this robot; confirm MAX_READING against the sensor actually plugged in before
 * trusting the converted number.
 */
@TeleOp(name = "current sensor")
public class CurrentSensor extends LinearOpMode {

    /** Full-scale analog output of the sensor. */
    private static final double MAX_VOLTS = 3.3;
    /** Reading that MAX_VOLTS corresponds to. */
    private static final double MAX_READING = 80;
    private AnalogInput laserAnalog;

    @Override
    public void runOpMode() {
        // Map the analog device from the hardware configuration
        laserAnalog = hardwareMap.get(AnalogInput.class, HardwareNames.CURRENT_SENSOR);

        // Wait for PLAY
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Read sensor voltage (0.0–3.3V)
            double volts = laserAnalog.getVoltage();

            double reading = (volts / MAX_VOLTS) * MAX_READING;

            telemetry.addData("Voltage (V)", "%.3f", volts);
            telemetry.addData("Scaled reading", "%.1f", reading);
            telemetry.update();
        }
    }
}