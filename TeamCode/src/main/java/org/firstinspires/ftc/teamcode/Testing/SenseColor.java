package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.ColorSensorAdvanced;

@Config
public class SenseColor {
    public static double minSat = 0.2;
    public static double minVal = 0.1;
    public static double purpleLow = 260;
    public static double purpleHigh = 300;
    public static double greenLow = 120;
    public static double greenHigh = 160;
    public String detected = "NONE";
    ColorSensorAdvanced colorSensor;


    public SenseColor(HardwareMap hardwareMap) {
        colorSensor = new ColorSensorAdvanced(hardwareMap, "ColorSensor");

    }

    public Action ColorDetect() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int r = colorSensor.red();
                int g = colorSensor.green();
                int b = colorSensor.blue();
                int a = Math.max(1, colorSensor.alpha()); // prevent divide-by-zero

                // Normalize RGB by alpha (light intensity)
                double rNorm = (double) r / a;
                double gNorm = (double) g / a;
                double bNorm = (double) b / a;

                // Convert normalized RGB to HSV
                float[] hsv = new float[3];
                Color.RGBToHSV(
                        (int) (rNorm * 255),
                        (int) (gNorm * 255),
                        (int) (bNorm * 255),
                        hsv
                );

                float hue = hsv[0]; // 0–360
                float sat = hsv[1]; // 0–1
                float val = hsv[2]; // 0–1

                if (val < minVal || sat < minSat) {
                    detected = "NO COLOR";
                } else if (hue >= purpleLow && hue <= purpleHigh) {
                    detected = "PURPLE";
                } else if (hue >= greenLow && hue <= greenHigh) {
                    detected = "GREEN";
                } else {
                    detected = "UNKNOWN";
                }

                // Send data to FTC Dashboard
                packet.put("Raw R", r);
                packet.put("Raw G", g);
                packet.put("Raw B", b);
                packet.put("Alpha", a);
                packet.put("R norm", rNorm);
                packet.put("G norm", gNorm);
                packet.put("B norm", bNorm);
                packet.put("Hue", hue);
                packet.put("Sat", sat);
                packet.put("Val", val);
                packet.put("Detected", detected);

                // false = run once and finish
                return true;
            }
        };
    }
}
