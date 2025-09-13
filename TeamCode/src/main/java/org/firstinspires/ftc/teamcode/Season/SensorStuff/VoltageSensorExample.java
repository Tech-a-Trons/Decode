package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageSensorExample extends LinearOpMode {
    private VoltageSensor myControlHubVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        waitForStart();

        double presentVoltage;
        presentVoltage = myControlHubVoltageSensor.getVoltage();

        telemetry.addData("Voltage: ",presentVoltage);

    }
}