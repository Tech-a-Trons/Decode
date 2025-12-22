package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;

//Make changes here
//For color sensor

public class ColorSensor implements Subsystem {
    float chue;
    float csat;
    float cval;
    float chue2;
    float csat2;
    float cval2;
    float red;
    float blue;
    float alpha;
    float red2;
    float green2;
    float blue2;
    float alpha2;

    float distance;

    int green = 0;

    int purple = 0;

    public int artifactcounter = 0;

    public float current_sat = 0;

    public float current_hue = 0;


    public int activeslot = 0;
    public int asc = 0;
    // asc = active slot color
    int[] slots = new int[10];

    int[] motif = new int[3];


    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime IncountTimer = new ElapsedTime();

    RevColorSensorV3 colorsensor;
    RevColorSensorV3 colorsensor2;

    public ColorSensor(HardwareMap hardwareMap) {
        colorsensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        //colorsensor2 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor2");
    }
    Servo rgbindicator = hardwareMap.get(Servo.class, "rgbled");

    public String Getcolor() {
//        colorsensor.enableLed(e);
        red = colorsensor.red();
        green = colorsensor.green();
        blue = colorsensor.blue();
        alpha = colorsensor.alpha();

        //red2 = colorsensor2.red();
        //green2 = colorsensor2.green();
        //blue2 = colorsensor2.blue();
        //alpha2 = colorsensor2.alpha();

        float total = red + green + blue;
        if (total == 0) return null; // avoid divide-by-zero

        //float total2 = red2 + green2 + blue2;
        //if (total2 == 0) return null; // avoid divide-by-zero

        // Normalize for brightness independence
        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        //double rNorm2 = (double) red2 / total2;
        //double gNorm2 = (double) green2 / total2;
        //double bNorm2 = (double) blue2 / total2;

        // Scale for HSV conversion
        int scaledR = (int) (rNorm * 255);
        int scaledG = (int) (gNorm * 255);
        int scaledB = (int) (bNorm * 255);

        //int scaledR2 = (int) (rNorm2 * 255);
        //int scaledG2 = (int) (gNorm2 * 255);
        //int scaledB2 = (int) (bNorm2 * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(scaledR, scaledG, scaledB, hsv);

        float[] hsv2 = new float[3];
        //Color.RGBToHSV(scaledR2, scaledG2, scaledB2, hsv2);

        chue = hsv[0];   // 0–360
        csat = hsv[1];   // 0–1
        cval = hsv[2];   // 0–1

        //chue2 = hsv2[0];   // 0–360
        //csat2 = hsv2[1];   // 0–1
        //cval2 = hsv2[2];   // 0–1

        // ----------------------------
        // 1️⃣ Primary HSV detection
        // ----------------------------
        if (csat > 0.15 && cval > 0.15) { // ignore dim/gray
            // Green
            if (chue >= 80 && chue <= 150) {
                return "green";
            } else if (chue >= 250 && chue <= 300) {
                return "purple";
//            } else if (chue >= 142 && chue <= 155){
//                return "red";
            }else {
                return "VALUE";
            }
        }

        //if (csat2 > 0.15 && cval2 > 0.15) { // ignore dim/gray
        // Green
        //    if (chue2 >= 162 && chue2 <= 163) {
        //        return "green";
        //    } else if (chue2 >= 200 && chue2 <= 230) {
        //        return "purple";
//            } else if (chue >= 142 && chue <= 155){
//                return "red";
        //    }else {
        //        return "VALUE";
        //    }
        //}

        // ----------------------------
        // 2️⃣ Relative RGB fallback
        // ----------------------------
        // Green dominates
//        if (gNorm > rNorm + 0.15 && gNorm > bNorm + 0.15) return "green";
//
//        // Purple: red and blue strong, blue >= red, green not dominant
//        if (rNorm > 0.25 && bNorm > 0.25 && bNorm >= rNorm && gNorm < 0.7) return "purple";
//
//        int intblue = (int) blue;
//        int intred = (int) green;

        // ----------------------------
        // 3️⃣ Secondary heuristics
        // ----------------------------
        // If blue is highest and green moderate → likely purple
        if (blue > red && blue > green && total > 50) {
            return "purple";
        }
        // If green is clearly highest → likely green
        else if (green > red && green > blue && total > 50 && chue >= 50 && chue <= 170) {
            return "green";
        }
        //RED
//        else if (red > green && red > blue && green > blue && red > 50 && total > 50) {
//            return "red";
//        }
        else {
            return "VALUE";
        }


    }
    public void IncountBalls() {
        String color = getColor();
        current_sat = getsat();
        current_hue = gethue();
        int ColorSensing = 0;
        if (IncountTimer.milliseconds() > 800) {
            if (current_sat > 0.5) {
                artifactcounter += 1;
                asc += 5;
                light();
                AssignColors();
                activeslot += 1;
                IncountTimer.reset();

            } else if (current_hue > 167) {
                artifactcounter += 1;
                asc += 1;

                light();
                AssignColors();

                activeslot += 1;
                IncountTimer.reset();
            }
        }
        // remove activeslot = artifactcounter when new robot is made

        green = (slots[0] + slots[1] + slots[2]) / 5;
        purple = (slots[0] + slots[1] + slots[2]) % 5;
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

    public void AssignColors() {
        slots[activeslot] = asc;
        asc = 0;

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
    public float getred() {
        return red;
    }

    public float getblue() {
        return blue;
    }
    public float getgreen() {
        return green;
    }
    public float getalpha() {
        return alpha;
    }
    //public float getAlpha2(){return alpha2; }
    public String getColor() {return Getcolor();}
}