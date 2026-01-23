package org.firstinspires.ftc.teamcode.Season.Prototypes.PreRegionals;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name = "DualShooterOuttake")
public class ReyanshCode extends LinearOpMode {
//    double x = 0;
//    double y = 0;

    DcMotor motor2;
    DcMotor motor1;
    DcMotor motor3;

    DcMotor motor4;


    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                motor3.setPower(0.75);
            }

            if (gamepad1.b) {
                motor2.setPower(-0.75);
            }
            if (gamepad1.x) {
                motor1.setPower(-1);
            }
            if (gamepad1.a) {
                motor4.setPower(1);
            }
            if (gamepad1.dpad_right) {
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);
            }
        }

    }
}