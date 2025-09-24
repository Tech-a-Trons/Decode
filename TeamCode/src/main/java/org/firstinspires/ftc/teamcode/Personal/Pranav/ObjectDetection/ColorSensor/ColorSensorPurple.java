package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorPurple extends LinearOpMode {
    ColorSensor colorSensor;
    PersonalGreenandPurple personal;

    @Override
    public void runOpMode() {
        // Map your color sensor (check your config name in the Driver Hub)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while (opModeIsActive()) {
            float phue = personal.getPhue();
            float psat = personal.getPsat();
            float pval = personal.getPval();

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Purple?", personal.Pcheck(colorSensor));
            telemetry.addData("Hue: ", phue);
            telemetry.addData("Saturation: ",psat);
            telemetry.addData("Value: ", pval);
            telemetry.update();
        }
    }
}

