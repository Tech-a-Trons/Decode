package org.firstinspires.ftc.teamcode.Season.TeleOp.Qual2Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestIntakeTransfer", group = "TeleOp")
public class TestIntakeTransfer extends LinearOpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    @Override
    public void runOpMode() {

        // Hardware map
        motor1 = hardwareMap.get(DcMotor.class, "transfer");
        motor2 = hardwareMap.get(DcMotor.class, "activeintake");

        waitForStart();

        while (opModeIsActive()) {

            // If A is pressed → run motors
            if (gamepad1.a) {
                motor1.setPower(1.0);
                motor2.setPower(-1.0);
            }

            // If X is pressed → stop motors
            if (gamepad1.x) {
                motor1.setPower(0.0);
                motor2.setPower(0.0);
            }
        }
    }
}