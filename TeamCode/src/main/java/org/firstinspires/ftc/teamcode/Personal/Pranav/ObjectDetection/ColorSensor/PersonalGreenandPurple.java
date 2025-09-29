package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class PersonalGreenandPurple {
    float chue;
    float csat;
    float cval;

    float[] hsv = new float[3];
    //ColorSensor sensor;

    public String Getcolor(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        float max = Math.max(red, Math.max(green, blue));
        float scale = (max > 0) ? 255f / max : 1;
        int scaledR = (int) (red * scale);
        int scaledG = (int) (green * scale);
        int scaledB = (int) (blue * scale);

        // Convert RGB → HSV
        Color.RGBToHSV(scaledR, scaledG, scaledB, hsv);

        chue = hsv[0];   // Hue angle (0–360)
        csat = hsv[1];   // Saturation (0–1)
        cval = hsv[2];   // Brightness (0–1)

        // Typical green hue is ~80–160
        if (chue >= 90 && chue <= 170 && csat > 0.2 && cval > 0.2) {
            return "green";
        } else if (chue >= 260 && chue <= 320 && csat > 0.1 && cval > 0.2) {
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