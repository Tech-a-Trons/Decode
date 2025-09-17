package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorPurpleGreen extends LinearOpMode {
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Map your color sensor (check config name)
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        int total = red + green + blue;
        if (total == 0) return false;

        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        return (gNorm > 0.40 && rNorm < 0.35 && bNorm < 0.35);
    }
}
