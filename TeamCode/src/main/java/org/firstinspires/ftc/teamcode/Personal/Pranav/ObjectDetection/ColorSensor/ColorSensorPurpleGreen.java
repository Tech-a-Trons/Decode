package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorPurpleGreen extends LinearOpMode {
    ColorSensor colorSensor;
    PersonalGreenandPurple personal;

    @Override
    public void runOpMode() {
        // Map your color sensor (check config name)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Purple?", personal.Pcheck(colorSensor));
            telemetry.addData("Detected Green?", personal.GCheck(colorSensor));
            telemetry.update();
        }
    }
}
