package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorGreen extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Map your color sensor (check the config name in the Driver Hub)
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        int total = red + green + blue;
        if (total == 0) return false;

        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        // Green = dominant green, weaker red and blue
        return (gNorm > 0.40 && rNorm < 0.35 && bNorm < 0.35);
    }
}