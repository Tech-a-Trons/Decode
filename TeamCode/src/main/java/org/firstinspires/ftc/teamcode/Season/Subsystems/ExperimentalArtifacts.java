package org.firstinspires.ftc.teamcode.Season.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

//Make changes here
//For color sensor

public class ExperimentalArtifacts {
    float chue;
    float csat;
    float cval;
    float chue2;
    float csat2;
    float cval2;
    float red;
    float green;
    float blue;
    float alpha;
    float red2;
    float green2;
    float blue2;
    float alpha2;
    RevColorSensorV3 colorsensor;
    RevColorSensorV3 colorsensor2;

    public ExperimentalArtifacts(HardwareMap hardwareMap) {
        colorsensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        //colorsensor2 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor2");
    }

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
            if (chue >= 162 && chue <= 163) {
                return "green";
            } else if (chue >= 200 && chue <= 230) {
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
        if (blue >= green | green - blue == 3 && green > red && total > 100 ) {
            return "purple";
        }
        // If green is clearly highest → likely green
        else if (green > red && green > blue && blue > red && green > 100 && chue > 162 && chue < 163 && total > 100) {
            return "green";
        }
        //RED
//        else if (red > green && red > blue && green > blue && red > 50 && total > 100) {
//            return "red";
//        }
        else {
            return "VALUE";
        }

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