package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp(name = "LEDBeamBreak", group = "Testing")
public class LEDBeamBreak extends OpMode {

    boolean[] balls = new boolean[3];
    int ballCount = 0;
    // Three digital laser sensors (beam-break style)
    private DigitalChannel laserInput1;
    private DigitalChannel laserInput2;
    private DigitalChannel laserInput3;
    // Prism RGB LED (I2C)
    private I2cDeviceSynch prismLED;

    @Override
    public void init() {
        // Initialize digital laser sensors
        laserInput1 = hardwareMap.get(DigitalChannel.class, "bb1");
        laserInput2 = hardwareMap.get(DigitalChannel.class, "bb2");
        laserInput3 = hardwareMap.get(DigitalChannel.class, "bb3");

        laserInput1.setMode(DigitalChannel.Mode.INPUT);
        laserInput2.setMode(DigitalChannel.Mode.INPUT);
        laserInput3.setMode(DigitalChannel.Mode.INPUT);

        // Initialize Prism RGB LED (I2C)
        prismLED = hardwareMap.get(I2cDeviceSynch.class, "prismLED");
        prismLED.engage();
    }

    @Override
    public void loop() {
        // Read each sensor: true = object detected (HIGH), false = no object (LOW)

        balls[0] = laserInput1.getState();

        balls[1] = laserInput2.getState();

        balls[2] = laserInput3.getState();

        // Count the number of balls detected
        ballCount = 0;

        if (balls[0]) ballCount++;

        if (balls[1]) ballCount++;

        if (balls[2]) ballCount++;

        // Set Prism LED color based on number of balls detected
        switch (ballCount) {
            case 0:
                setLEDColor(255, 0, 0);       // Red = no balls
                break;
            case 1:
                setLEDColor(255, 165, 0);     // Orange = 1 ball
                break;
            case 2:
                setLEDColor(255, 255, 0);     // Yellow = 2 balls
                break;
            case 3:
                setLEDColor(0, 255, 0);       // Green = 3 balls
                break;
        }

        // Telemetry for debugging
        telemetry.addData("Ball 1 Detected", balls[0]);
        telemetry.addData("Ball 2 Detected", balls[1]);
        telemetry.addData("Ball 3 Detected", balls[2]);
        telemetry.addData("Ball Count", ballCount);
        telemetry.update();
    }

    // Method to set Prism LED color
    private void setLEDColor(int r, int g, int b) {
        byte[] colorData = new byte[]{(byte) r, (byte) g, (byte) b};
        prismLED.write(colorData);  // Send RGB values to Prism LED
    }
}
