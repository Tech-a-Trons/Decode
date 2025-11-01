package org.firstinspires.ftc.teamcode.Season.TeleOp.oldteleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Disabled
public class Bankai extends LinearOpMode {
    private VoltageSensor myControlHubVoltageSensor;
    DcMotor activeintake = null;
    DcMotor out1 = null;
    DcMotor out2 = null;
    DcMotor ramp = null;
    //CRServo wheel = null ;
    @Override
    public void runOpMode() throws InterruptedException {
//        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "MyControlHub");
        out1 = hardwareMap.get(DcMotor.class,"outtake1");
        out2 = hardwareMap.get(DcMotor.class,"outtake2");
        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
        ramp = hardwareMap.get(DcMotor.class,"ramp");
//        wheel = hardwareMap.get(CRServo.class,"wheel");
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
//        double presentVoltage;
//        presentVoltage = myControlHubVoltageSensor.getVoltage();
//        telemetry.addData("Voltage: ",presentVoltage);
//
//        double powerValue = 0.5;
//        double offsetVoltage =  powerValue * (12.5/presentVoltage);
//        telemetry.addData("offset Voltage: ",offsetVoltage);
        while (opModeIsActive()) {

            if (gamepad1.a) {
                activeintake.setPower(1);

            }
            if (gamepad1.dpad_left){

//                out2.setPower(.45);
//                sleep(1000);
//                wheel.setPower(1);
                ramp.setPower(-1);
//                activeintake.setPower(1);
                ramp.setPower(-1);
                sleep(100);
                out1.setPower(-0);
                out2.setPower(0);
                activeintake.setPower(1);
//                wheel.setPower(0);
                ramp.setPower(-0);
                sleep(300);
                activeintake.setPower(0);
                out1.setPower(-.45);
                out2.setPower(.45);
                sleep(500);
//                wheel.setPower(1);
                ramp.setPower(-1);
//                activeintake.setPower(1);
                sleep(50);
//                wheel.setPower(0);
                out1.setPower(-.1);
                out2.setPower(.1);
                ramp.setPower(-0);
//                wheel.setPosition(0.7);
                sleep(100);
//                wheel.setPower(1);
                out1.setPower(-.45);
                out2.setPower(.45);
                sleep(700);
                activeintake.setPower(1);
//                wheel.setPosition(0.9);
                ramp.setPower(-1);
//                sleep(400);
////                wheel.setPower(0);
//                out1.setPower(-.1);
//                out2.setPower(.1);
//                ramp.setPower(-0);

            }

            if (gamepad1.b) {
//                activeintake.setPower(1);
                out1.setPower(-.4);
                out2.setPower(.4);

                //launching
            }
            if (gamepad1.dpad_down){
                out1.setPower(-.3);
                out2.setPower(.3);
            }

            if (gamepad1.x ) {
                activeintake.setPower(0);
                out1.setPower(0);
                out2.setPower(0);
                ramp.setPower(0);
            }
            if (gamepad1.right_trigger>0.0 || gamepad1.right_trigger < 0){
                ramp.setPower(gamepad1.right_trigger);
                //intake
            }
            if (gamepad1.right_bumper){
                ramp.setPower(-.1);
                //intake
            }

            if (gamepad1.left_bumper){
                ramp.setPower(-.05);
                //holding
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
