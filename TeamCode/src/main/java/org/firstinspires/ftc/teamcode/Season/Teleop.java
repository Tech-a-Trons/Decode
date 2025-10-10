package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class Teleop extends LinearOpMode {
//    DcMotor activeintake = null;
    DcMotor out1 = null;
    DcMotor out2 = null;
    DcMotor ramp = null;

    @Override
    public void runOpMode() throws InterruptedException {
        out1 = hardwareMap.get(DcMotor.class,"outtake1");
        out2 = hardwareMap.get(DcMotor.class,"outtake2");
//        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
        ramp = hardwareMap.get(DcMotor.class,"ramp");
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        activeintake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        activeintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
//                activeintake.setPower(-1);
            }

            if (gamepad1.b) {
//                activeintake.setPower(1);
            }

            if (gamepad1.x ) {
//                activeintake.setPower(0);
                out1.setPower(0);
                out2.setPower(0);
            }
            if (gamepad1.dpad_up){
                ramp.setPower(.37);
            }
            if (gamepad1.dpad_down){
                ramp.setPower(.4);
            }
            if (gamepad1.dpad_left){
                ramp.setPower(.45);
            }
            if (gamepad1.dpad_right){
                ramp.setPower(.42);
            }



            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

        }
    }
}
