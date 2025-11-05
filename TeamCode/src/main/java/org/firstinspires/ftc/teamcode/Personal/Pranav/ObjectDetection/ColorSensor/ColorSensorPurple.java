package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@TeleOp
public class ColorSensorPurple extends LinearOpMode {
    ColorSensor colorSensor;
    PersonalGreenandPurple personal;
    String purpleCheck;

    @Override
    public void runOpMode() {
        // Map your color sensor (check your config name in the Driver Hub)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();
        if (personal.Getcolor(colorSensor) == "purple") {
            purpleCheck = "Yes";
        } else {
            purpleCheck = "No";
        }

        while (opModeIsActive()) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Object Color:", purpleCheck);
            telemetry.addData("Hue: ", personal.gethue());
            telemetry.addData("Saturation: ", personal.getsat());
            telemetry.addData("Value: ", personal.getval());
            telemetry.update();
        }
    }
}

