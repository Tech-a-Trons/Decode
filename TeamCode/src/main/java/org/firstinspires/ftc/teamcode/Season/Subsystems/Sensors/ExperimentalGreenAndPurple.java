package org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

//Make changes here
//For color sensor

public class ExperimentalGreenAndPurple {
    float chue;
    float csat;
    float cval;
    ColorSensor colorsensor;

    public ExperimentalGreenAndPurple(HardwareMap hardwareMap) {
        colorsensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        colorsensor = new ColorSensor() {
            @Override
            public int red() {
                return 0;
            }

            @Override
            public int green() {
                return 0;
            }

            @Override
            public int blue() {
                return 0;
            }

            @Override
            public int alpha() {
                return 0;
            }

            @Override
            public int argb() {
                return 0;
            }

            @Override
            public void enableLed(boolean enable) {

            }

            @Override
            public void setI2cAddress(I2cAddr newAddress) {

            }

            @Override
            public I2cAddr getI2cAddress() {
                return null;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return "";
            }

            @Override
            public String getConnectionInfo() {
                return "";
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        };
    }

    public String Getcolor() {

        int red = colorsensor.red();
        int green = colorsensor.green();
        int blue = colorsensor.blue();

        int total = red + green + blue;
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

        chue = hsv[0];   // 0–360
        csat = hsv[1];   // 0–1
        cval = hsv[2];   // 0–1

        // ----------------------------
        // 1️⃣ Primary HSV detection
        // ----------------------------
        if (csat > 0.15 && cval > 0.15) { // ignore dim/gray
            // Green
            if (chue >= 80 && chue <= 170) return "green";
            // Purple
            if (chue >= 250 && chue <= 330) return "purple";
        }

        // ----------------------------
        // 2️⃣ Relative RGB fallback
        // ----------------------------
        // Green dominates
        if (gNorm > rNorm + 0.15 && gNorm > bNorm + 0.15) return "green";

        // Purple: red and blue strong, blue >= red, green not dominant
        if (rNorm > 0.25 && bNorm > 0.25 && bNorm >= rNorm && gNorm < 0.7) return "purple";

        int intblue = (int) blue;
        int intred = (int) green;

        // ----------------------------
        // 3️⃣ Secondary heuristics
        // ----------------------------
        // If blue is highest and green moderate → likely purple
        if (blue >= green | green - blue == 3 && green > red && total > 100) return "purple";

        // If green is clearly highest → likely green
        if (green > red && green > blue && total > 100) return "green";

        return null; // unknown color
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

    public String getColor() {return Getcolor();}
}