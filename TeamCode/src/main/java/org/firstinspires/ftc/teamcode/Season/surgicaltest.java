package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class surgicaltest extends LinearOpMode {

    DcMotor activeintake = null;
    //DcMotor launchRight = null;
    private VoltageSensor myControlHubVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        activeintake = hardwareMap.get(DcMotor.class, "activeintake");

        activeintake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        activeintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double presentVoltage;
        presentVoltage = myControlHubVoltageSensor.getVoltage();
        telemetry.addData("Voltage: ", presentVoltage);

        double powerValue = 0.25;
        double offsetVoltage = powerValue * (12.5 / presentVoltage);
        telemetry.addData("offset Voltage: ", offsetVoltage);

        if (gamepad1.a) {
            activeintake.setTargetPosition(50);

            activeintake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            activeintake.setPower(-0.25);
        }
    }
}