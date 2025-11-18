package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

@TeleOp
public class ColorSensorPurpleGreen extends LinearOpMode {
    ColorSensor colorSensor;
    PersonalGreenandPurple personal;
    String greenCheck;
    String purpleCheck;

    @Override
    public void runOpMode() {

        colorSensor = new ColorSensor() {
            @Override
            public int red() {
                return 0;
            }

            @Override
            public int green() {
                return 0;
            }

            @Override
            public int blue() {
                return 0;
            }

            @Override
            public int alpha() {
                return 0;
            }

            @Override
            public int argb() {
                return 0;
            }

            @Override
            public void enableLed(boolean enable) {

            }

            @Override
            public void setI2cAddress(I2cAddr newAddress) {

            }

            @Override
            public I2cAddr getI2cAddress() {
                return null;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return "";
            }

            @Override
            public String getConnectionInfo() {
                return "";
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        };

        // Map your color sensor (check config name)
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        waitForStart();

        while (opModeIsActive()) {

            String colorGet = personal.Getcolor(colorSensor);

            if (colorGet == "purple") {
                purpleCheck = "Yes";
                greenCheck = "No";
            } else if (colorGet == "green"){
                purpleCheck = "No";
                greenCheck = "Yes";
            } else if (colorGet == "VALUE") {
                purpleCheck = "No";
                greenCheck = "No";
            }

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
