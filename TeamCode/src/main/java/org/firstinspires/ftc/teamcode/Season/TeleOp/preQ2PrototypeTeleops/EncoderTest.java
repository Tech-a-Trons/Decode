package org.firstinspires.ftc.teamcode.Season.TeleOp.preQ2PrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class EncoderTest extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotor.class, "outtakeleft");
        rightMotor = hardwareMap.get(DcMotor.class, "outtakeright");

        // Reverse right motor if needed
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // Simple manual control with gamepad sticks
            double power = -gamepad1.left_stick_y; // forward/back
            leftMotor.setPower(power);
            rightMotor.setPower(power);

            // Telemetry to check encoder values
            telemetry.addData("Left Motor Position", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Position", rightMotor.getCurrentPosition());
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.update();
        }
    }
}