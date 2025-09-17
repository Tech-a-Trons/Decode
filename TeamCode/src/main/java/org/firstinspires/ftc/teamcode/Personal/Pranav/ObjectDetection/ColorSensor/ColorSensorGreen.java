package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorGreen extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Map your color sensor (check the config name in the Driver Hub)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while (opModeIsActive()) {
            boolean greenSeen = isGreen(colorSensor);

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Green?", greenSeen);
            telemetry.update();
        }
    }

    // Method to check if sensor sees green
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