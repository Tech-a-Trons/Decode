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
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Normalize to remove brightness dependence
        int total = red + green + blue;
        if (total == 0) return null; // avoid divide-by-zero

        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        // Scale to 0–255 for HSV conversion
        int scaledR = (int) (rNorm * 255);
        int scaledG = (int) (gNorm * 255);
        int scaledB = (int) (bNorm * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(scaledR, scaledG, scaledB, hsv);

        chue = hsv[0];   // Hue angle (0–360)
        csat = hsv[1];   // Saturation (0–1)
        cval = hsv[2];   // Brightness (0–1)

        // Filter: ignore very dim or grayish readings
        if (cval < 0.15 || csat < 0.1) {
            return null;
        }

        // Green: hue around 80–170
        if (chue >= 80 && chue <= 170) {
            return "green";
        }

        // Purple: can wrap around (magenta-red area or violet area)
        // REV sensors often report purple around 260–320
        if ((chue >= 260 && chue <= 320) || (chue >= 280 && chue <= 300)) {
            return "purple";
        }
        return null;
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