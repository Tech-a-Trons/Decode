package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

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

        int total = red + green + blue;
        if (total == 0) return false;

        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        return (rNorm > 0.30 && bNorm > 0.30 && gNorm < 0.25);
    }
}

