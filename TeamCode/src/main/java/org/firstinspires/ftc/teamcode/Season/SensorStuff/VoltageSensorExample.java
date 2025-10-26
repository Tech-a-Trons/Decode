package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//For voltage sensor

@Disabled
@TeleOp
public class VoltageSensorExample extends LinearOpMode {
    private VoltageSensor myControlHubVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        waitForStart();

        double presentVoltage;
        presentVoltage = myControlHubVoltageSensor.getVoltage();
        telemetry.addData("Voltage: ",presentVoltage);

        double powerValue = 0.5;
        double offsetPower =  powerValue * (12.5/presentVoltage);
        telemetry.addData("offset Voltage: ",offsetPower);

    }
}