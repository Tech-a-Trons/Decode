package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorPurpleGreen extends LinearOpMode {
    ColorSensor colorSensor;
    PersonalGreenandPurple personal;
    String greenCheck;
    String purpleCheck;

    @Override
    public void runOpMode() {
        // Map your color sensor (check config name)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        if (personal.Getcolor(colorSensor) == "purple") {
            purpleCheck = "Yes";
            greenCheck = "No";
        } else if (personal.Getcolor(colorSensor) == "green"){
            purpleCheck = "No";
            greenCheck = "Yes";
        }

        while (opModeIsActive()) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Purple?: ", purpleCheck);
            telemetry.addData("Green?: ", greenCheck);
            telemetry.addData("Hue: ", personal.gethue());
            telemetry.addData("Saturation: ", personal.getsat());
            telemetry.addData("Value: ", personal.getval());
            telemetry.update();
        }
    }
}
