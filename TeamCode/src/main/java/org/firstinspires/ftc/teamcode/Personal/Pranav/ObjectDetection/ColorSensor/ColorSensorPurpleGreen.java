package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorPurpleGreen extends LinearOpMode {
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Map your color sensor (check config name)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while (opModeIsActive()) {
            boolean purpleSeen = isPurple(colorSensor);
            boolean greenSeen = isGreen(colorSensor);

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Purple?", purpleSeen);
            telemetry.addData("Detected Green?", greenSeen);
            telemetry.update();
        }
    }

    public boolean isPurple(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        int total = red + green + blue;
        if (total == 0) return false;

        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        return (rNorm > 0.30 && bNorm > 0.30 && gNorm < 0.25);
    }

    public boolean isGreen(ColorSensor sensor) {
        // Convert RGB → HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsv);

        float hue = hsv[0];   // Hue angle (0–360)
        float sat = hsv[1];   // Saturation (0–1)
        float val = hsv[2];   // Brightness (0–1)

        // Typical green hue is ~80–160
        return (hue >= 80 && hue <= 160 && sat > 0.4 && val > 0.2);
    }
}
