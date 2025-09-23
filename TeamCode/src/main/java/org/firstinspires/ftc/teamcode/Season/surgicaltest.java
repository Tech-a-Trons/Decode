package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class surgicaltest extends LinearOpMode {
    DcMotor activeintake = null;

    @Override
    public void runOpMode() throws InterruptedException {

        activeintake = hardwareMap.get(DcMotor.class, "activeintake");

//        activeintake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        activeintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                activeintake.setPower(-0.6);
            }

            if (gamepad1.b) {
                activeintake.setPower(0.6);
            }

            if (gamepad1.x || gamepad1.y) {
                activeintake.setPower(0);
            }
        }
    }
}