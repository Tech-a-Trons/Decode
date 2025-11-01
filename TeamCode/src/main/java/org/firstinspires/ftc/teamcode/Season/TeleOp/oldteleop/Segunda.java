package org.firstinspires.ftc.teamcode.Season.TeleOp.oldteleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Season.Subsystems.VoltageGet;
@Disabled
@TeleOp(name = "Segunda")
public class Segunda extends LinearOpMode {

    VoltageGet volt = new VoltageGet();
    DcMotor activeintake = null;
    DcMotor out1 = null;
    DcMotor out2 = null;
    DcMotor ramp = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        out1 = hardwareMap.get(DcMotor.class, "outtake1");
        out2 = hardwareMap.get(DcMotor.class, "outtake2");
        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
        ramp = hardwareMap.get(DcMotor.class, "ramp");

        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "br");

        // Initialize voltage sensor system
        volt.init(hardwareMap);

        // Reverse left motors if needed
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // --- Mechanism Controls ---
            if (gamepad1.a) {
                activeintake.setPower(volt.regulate(1.0));
            }

            if (gamepad1.dpad_left) {
                ramp.setPower(volt.regulate(-1.0));
                sleep(100);
                out1.setPower(volt.regulate(0));
                out2.setPower(volt.regulate(0));
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(0));
//                sleep(300);


                sleep(100);
            }
            if (gamepad1.dpad_up){
                activeintake.setPower(volt.regulate(0));
                out1.setPower(volt.regulate(-0.36));
                out2.setPower(volt.regulate(0.36));
                sleep(1400);
                ramp.setPower(volt.regulate(-1.0));
                sleep(50);
                out1.setPower(volt.regulate(-0.1));
                out2.setPower(volt.regulate(0.1));
                ramp.setPower(volt.regulate(0));
            }

            if (gamepad1.b) {
                out1.setPower(volt.regulate(-0.36));
                out2.setPower(volt.regulate(0.36));
            }

            if (gamepad1.dpad_down) {
                out1.setPower(volt.regulate(-0.36));
                out2.setPower(volt.regulate(0.36));
                sleep(1400);
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(-1.0));
            }

            if (gamepad1.x) {
                activeintake.setPower(0);
                out1.setPower(0);
                out2.setPower(0);
                ramp.setPower(0);
            }

            if (gamepad1.right_trigger > 0.0) {
                ramp.setPower(volt.regulate(gamepad1.right_trigger));
            }
            if (gamepad1.right_bumper) {
                ramp.setPower(volt.regulate(-0.1));
            }
            if (gamepad1.left_bumper) {
                ramp.setPower(volt.regulate(-0.05));
            }

            // --- Drivetrain Controls ---
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x * 1.1; // strafe
            double rx = gamepad1.right_stick_x; // rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Apply voltage regulation to all drive motors
            frontLeftMotor.setPower(volt.regulate(frontLeftPower));
            backLeftMotor.setPower(volt.regulate(backLeftPower));
            frontRightMotor.setPower(volt.regulate(frontRightPower));
            backRightMotor.setPower(volt.regulate(backRightPower));

            // --- Telemetry ---
            telemetry.addData("Voltage", volt.getVoltage());
            telemetry.update();
        }
    }
}