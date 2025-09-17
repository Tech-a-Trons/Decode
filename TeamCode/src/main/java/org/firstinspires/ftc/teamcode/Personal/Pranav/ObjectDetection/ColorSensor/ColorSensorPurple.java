package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorPurple extends LinearOpMode {
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Map your color sensor (check your config name in the Driver Hub)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while (opModeIsActive()) {
            boolean purpleSeen = Pcheck(colorSensor);

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Purple?", purpleSeen);
            telemetry.update();
        }
    }

    public boolean Pcheck(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsv);

        float hue = hsv[0];   // 0–360
        float sat = hsv[1];   // 0–1
        float val = hsv[2];   // 0–1

        // Purple usually ~260–300 hue
        return (hue >= 260 && hue <= 300 && sat > 0.4 && val > 0.2);

    }
}

