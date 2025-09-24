package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorGreen extends LinearOpMode {

    ColorSensor colorSensor;

    PersonalGreenandPurple personal;

    @Override
    public void runOpMode() {
        // Map your color sensor (check the config name in the Driver Hub)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while (opModeIsActive()) {
            float ghue = personal.getGhue();
            float gsat = personal.getGsat();
            float gval = personal.getGval();

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Green?", personal.GCheck(colorSensor));
            telemetry.addData("Hue: ", ghue);
            telemetry.addData("Saturation: ",gsat);
            telemetry.addData("Value: ", gval);
        }
    }
}