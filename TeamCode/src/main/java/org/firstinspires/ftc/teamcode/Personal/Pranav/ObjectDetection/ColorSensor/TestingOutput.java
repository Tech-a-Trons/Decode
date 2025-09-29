package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;
import android.graphics.SweepGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor.TestingDetector;

@TeleOp
public class TestingOutput extends LinearOpMode {
    ColorSensor colorSensor;
    TestingDetector personal= new TestingDetector();

    @Override
    public void runOpMode() {
        // Map your color sensor (check your config name in the Driver Hub)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Detected Object Color:", personal.Getcolor(colorSensor));
            telemetry.addData("Hue: ", personal.gethue());
            telemetry.addData("Saturation: ", personal.getsat());
            telemetry.addData("Value: ", personal.getval());
            telemetry.update();
        }
    }
}

