package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;


public class TestingDetector {
    float chue;
    float csat;
    float cval;

    float[] hsv = new float[3];
    //ColorSensor sensor;

    public String Getcolor(ColorSensor sensor) {
        // Read raw RGB
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        int total = red + green + blue;
        if (total == 0) return null; // avoid divide-by-zero

        // Normalize RGB (brightness-independent)
        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        // Convert to HSV for hue-based detection
        int scaledR = (int) (rNorm * 255);
        int scaledG = (int) (gNorm * 255);
        int scaledB = (int) (bNorm * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(scaledR, scaledG, scaledB, hsv);

        chue = hsv[0];  // 0–360
        csat = hsv[1];  // 0–1
        cval = hsv[2];  // 0–1

        // Ignore dim or grayish readings
        if (cval < 0.15 || csat < 0.2) return null;

        // ----- Green Detection -----
        // HSV-based
        if (chue >= 80 && chue <= 170 && csat > 0.2 && cval > 0.2) {
            return "green";
        }
        // Relative RGB fallback
        if (gNorm > rNorm + 0.1 && gNorm > bNorm + 0.1) {
            return "green";
        }

        // ----- Purple Detection -----
        // HSV-based (typical violet/magenta)
        if ((chue >= 260 && chue <= 320) && csat > 0.2 && cval > 0.2) {
            return "purple";
        }
        // Relative RGB fallback
        if (rNorm > 0.2 && bNorm > 0.3 && gNorm < 0.3) {
            return "purple";
        }
        return null; // color not recognized
    }

    public float gethue() {
        return chue;
    }
    public float getsat() {
        return csat;
    }
    public float getval() {
        return cval;
    }
}