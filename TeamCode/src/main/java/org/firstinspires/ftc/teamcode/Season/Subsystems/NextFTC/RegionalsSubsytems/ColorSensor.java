package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;

public class ColorSensor implements Subsystem {

    public static final ColorSensor INSTANCE = new ColorSensor();

    private RevColorSensorV3 colorsensor;
    private Servo rgbindicator;

    private boolean initialized = false;

    float chue, csat, cval;
    float red, green, blue, alpha;

    public static int artifactcounter = 0;

    public float current_sat = 0;
    public float current_hue = 0;
    public float current_val = 0;

    ElapsedTime IncountTimer = new ElapsedTime();

    private ColorSensor() {}

    // ================= INIT =================

    public void init(HardwareMap hardwareMap) {
        try {
            rgbindicator = hardwareMap.get(Servo.class, "rgbled");
            colorsensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
            initialized = true;
        } catch (Exception e) {
            initialized = false;
        }
    }

    // ================= COLOR DETECTION =================

    public String getColor() {

        if (!initialized || colorsensor == null) return null;

        red = colorsensor.red();
        green = colorsensor.green();
        blue = colorsensor.blue();
        alpha = colorsensor.alpha();

        float total = red + green + blue;
        if (total <= 0) return null;

        double rNorm = red / total;
        double gNorm = green / total;
        double bNorm = blue / total;

        int scaledR = (int) (rNorm * 255);
        int scaledG = (int) (gNorm * 255);
        int scaledB = (int) (bNorm * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(scaledR, scaledG, scaledB, hsv);

        chue = hsv[0];
        csat = hsv[1];
        cval = hsv[2];

        if (blue > red && blue > green && total > 50) {
            return "purple";
        }
        else if (green > red && green > blue && total > 50 && chue >= 50 && chue <= 170) {
            return "green";
        }

        return null;
    }

    // ================= BALL COUNTING =================

    public void IncountBalls() {

        if (!initialized || colorsensor == null) return;

        String color = getColor();
        current_sat = csat;
        current_hue = chue;
        current_val = cval;

        if (IncountTimer.milliseconds() > 200) {

            if (current_sat > 0.5 || current_hue > 175 && artifactcounter != 4) {
                artifactcounter += 1;
                light();
                IncountTimer.reset();
            }
        }
    }

    // ================= LIGHT CONTROL =================

    public void light() {

        if (!initialized || rgbindicator == null) return;

        if (artifactcounter == 0) {
            rgbindicator.setPosition(0);
        } else if (artifactcounter == 1) {
            rgbindicator.setPosition(0.3);
        } else if (artifactcounter == 2) {
            rgbindicator.setPosition(0.375);
        } else if (artifactcounter == 3) {
            rgbindicator.setPosition(0.5);
        } else {
            rgbindicator.setPosition(0.6);
        }
    }
        public void nolight() {
            rgbindicator.setPosition(0);
        }



    // ================= GETTERS =================

    public float getHue() { return chue; }
    public float getSat() { return csat; }
    public float getVal() { return cval; }
    public float getRed() { return red; }
    public float getGreen() { return green; }
    public float getBlue() { return blue; }
    public float getAlpha() { return alpha; }
}