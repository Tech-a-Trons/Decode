package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public class WheelLauncherTesting extends LinearOpMode {

    DcMotor launchLeft = null;
    //DcMotor launchRight = null;
    private VoltageSensor myControlHubVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        launchLeft = hardwareMap.get(DcMotor.class,"launchLeft");
        //launchRight = hardwareMap.get(DcMotor.class,"launch2");

        launchLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //launchRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double presentVoltage;
        presentVoltage = myControlHubVoltageSensor.getVoltage();
        telemetry.addData("Voltage: ",presentVoltage);

        double powerValue = 0.25;
        double offsetVoltage =  powerValue * (12.5/presentVoltage);
        telemetry.addData("offset Voltage: ",offsetVoltage);

        if (gamepad1.a) {
            launchLeft.setTargetPosition(50);
            //launchRight.setTargetPosition(50);

            launchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //launchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            launchLeft.setPower(-0.25);
            //launchRight.setPower(-0.25);
        }
    }
}
