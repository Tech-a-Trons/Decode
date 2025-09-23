package org.firstinspires.ftc.teamcode.Season;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class SeasonGreenandPurple {
    float phue;
    float psat;
    float pval;
    float ghue;
    float gsat;
    float gval;

    public boolean GCheck(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Convert RGB → HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);

        phue = hsv[0];   // Hue angle (0–360)
        psat = hsv[1];   // Saturation (0–1)
        pval = hsv[2];   // Brightness (0–1)

        // Typical green hue is ~80–160
        return (phue >= 80 && phue <= 160 && psat > 0.4 && pval > 0.2);
    }

    public boolean Pcheck(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);

        ghue = hsv[0];   // 0–360
        gsat = hsv[1];   // 0–1
        gval = hsv[2];   // 0–1

        // Purple usually ~260–300 hue
        return (ghue >= 260 && ghue <= 300 && gsat > 0.4 && gval > 0.2);

    }
    public float getPhue() {
        return phue;
    }
    public float getPsat() {
        return psat;
    }
    public float getPval() {
        return pval;
    }
    public float getGhue() {
        return ghue;
    }
    public float getGsat() {
        return gsat;
    }
    public float getGval() {
        return gval;
    }
}