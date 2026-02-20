package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;

public class ColorSensor implements Subsystem {

    // INSTANCE is now set via init() instead of at class load time
    public static final ColorSensor INSTANCE = new ColorSensor();

    float chue;
    float csat;
    float cval;

    float red;
    float blue;
    float alpha;

    public int green = 0;
    public int purple = 0;

    public static int artifactcounter = 0;

    public float current_sat = 0;
    public float current_hue = 0;
    public float current_val = 0;

    public int asc = 0;

    int[] motif = new int[3];

    ElapsedTime IncountTimer = new ElapsedTime();

    RevColorSensorV3 colorsensor;
    Servo rgbindicator;

    // Private constructor — call init() from your OpMode
    public ColorSensor() {}
    /**
     * Call this once at the top of runOpMode() before using INSTANCE.
     */
    public void init(HardwareMap hardwareMap) {
        rgbindicator = hardwareMap.get(Servo.class, "rgbled");
        colorsensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
    }

    public String Getcolor() {
        red = colorsensor.red();
        green = colorsensor.green();
        blue = colorsensor.blue();
        alpha = colorsensor.alpha();

        float total = red + green + blue;
        if (total == 0) return null; // avoid divide-by-zero

        // Normalize for brightness independence
        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        // Scale for HSV conversion
        int scaledR = (int) (rNorm * 255);
        int scaledG = (int) (gNorm * 255);
        int scaledB = (int) (bNorm * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(scaledR, scaledG, scaledB, hsv);

        chue = hsv[0]; // 0–360
        csat = hsv[1]; // 0–1
        cval = hsv[2]; // 0–1

        // Purple: blue dominates
        if (blue > red && blue > green && total > 50) {
            return "purple";
        }
        // Green: green dominates and hue in range
        else if (green > red && green > blue && total > 50 && chue >= 50 && chue <= 170) {
            return "green";
        }
        else {
            return "VALUE";
        }
    }

    public void IncountBalls() {
        String color = getColor();
        current_sat = getsat();
        current_hue = gethue();
        current_val = getval();

        if (IncountTimer.milliseconds() > 1000) {
            if (current_sat > 0.5) {
                artifactcounter += 1;
                asc += 5;
                light();
                IncountTimer.reset();
            } else if (current_hue > 180) {
                artifactcounter += 1;
                light();
                IncountTimer.reset();
            }
        }
    }

    public float gethue()   { return chue; }
    public float getsat()   { return csat; }
    public float getval()   { return cval; }
    public float getred()   { return red; }
    public float getblue()  { return blue; }
    public float getgreen() { return green; }
    public float getalpha() { return alpha; }

    public String getColor() {
        return Getcolor();
    }

    @Override
    public void periodic() {
        light();
    }

    public void light() {
        if (artifactcounter == 0) {
            rgbindicator.setPosition(0);
        } else if (artifactcounter == 1) {
            rgbindicator.setPosition(0.3);
        } else if (artifactcounter == 2) {
            rgbindicator.setPosition(0.375);
        } else if (artifactcounter == 3) {
            rgbindicator.setPosition(0.5);
        } else if (artifactcounter > 3) {
            rgbindicator.setPosition(0.6);
        }
    }
}