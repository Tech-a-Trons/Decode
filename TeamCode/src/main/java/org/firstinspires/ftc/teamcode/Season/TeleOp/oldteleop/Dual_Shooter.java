package org.firstinspires.ftc.teamcode.Season.TeleOp.oldteleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "Dual Shooter", group = "Linear Opmode")
public class Dual_Shooter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Shooter motors
        DcMotor M1 = hardwareMap.get(DcMotor.class, "Outake Left");
        DcMotor M2 = hardwareMap.get(DcMotor.class, "Outake Right");
        DcMotor M3 = hardwareMap.get(DcMotor.class, "Intake");
//        DcMotor M4 = hardwareMap.get(DcMotor.class, "BootKicker1");

        // Drive motors
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        //Servos
        CRServo S2 = hardwareMap.get(CRServo.class, "BootKicker2.2");
        CRServo S1 = hardwareMap.get(CRServo.class, "BootKicker2.1");

        // Set directions
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Driving control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double flPower = (y + x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            // Shooter/Intake controls
             int outtakePower = (int) 0.4;  // read inside loop

            if (gamepad1.dpad_left) {
                M1.setPower(0.4);
            }
            if (gamepad1.dpad_right) {
                M2.setPower(0.4);
            }
            if (gamepad1.x) {
                M3.setPower(-1);
            }

            if (gamepad1.b) {
//                M4.setPower(-1);
                S1.setPower(-1);
            }
            if(gamepad1.y){
                S2.setPower(-1);
            }
            if (gamepad1.a) {
                // Stop all shooter mechanisms
                M1.setPower(0);
                M2.setPower(0);
                M3.setPower(0);
//              M4.setPower(0);
                S2.setPower(0);
                S1.setPower(0);
            }

            telemetry.addData("Outtake Power", outtakePower);
            telemetry.update();

            if(gamepad1.dpad_up){
                M1.setPower(-0.2);
                M2.setPower(-0.2);
                S2.setPower(0.3);
                S1.setPower(0.3);
            }
        }
    }
}