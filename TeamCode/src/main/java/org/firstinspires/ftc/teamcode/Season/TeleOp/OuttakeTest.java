package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OuttakeTest extends LinearOpMode {
    DcMotor outleft = null;
    DcMotor outright = null;
    Servo hood = null;
    @Override
    public void runOpMode() throws InterruptedException {
        outright=hardwareMap.get(DcMotor.class,"outtakeright");
        outleft=hardwareMap.get(DcMotor.class,"outtakeleft");
        hood = hardwareMap.get(Servo.class,"hood");
        outleft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
//            if (gamepad1.a){
//                outright.setPower(0.5);
//                outleft.setPower(0.5);
//                hood.setPosition(0.25);
//            }
//
//            if (gamepad1.y){
//                outright.setPower(0.4);
//                outleft.setPower(0.4);
//                hood.setPosition(0.35);
//            }
            if (gamepad1.b){
                outright.setPower(0.6);
                outleft.setPower(0.6);
                //0.6 power for close
            }
            if (gamepad1.a){
                outright.setPower(0.8);
                outleft.setPower(0.8);
                //0.8 power for far
            }
//            if (gamepad1.a){
//                hood.setPosition(0.3);
//            }
            if (gamepad1.y){
                hood.setPosition(-0.5);
            }
            if (gamepad1.dpad_left){
                hood.setPosition(1);
            }
            if (gamepad1.x){
                outleft.setPower(0);
                outright.setPower(0);
                hood.setPosition(0);
            }

        }



    }
}
