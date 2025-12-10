package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OuttakeTest extends LinearOpMode {
    DcMotor outleft = null;
    DcMotor outright = null;
    CRServo hood = null;
    @Override
    public void runOpMode() throws InterruptedException {
        outright=hardwareMap.get(DcMotor.class,"outtakeright");
        outleft=hardwareMap.get(DcMotor.class,"outtakeleft");
        hood = hardwareMap.get(CRServo.class,"hood");
        outleft.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(CRServo.Direction.REVERSE);
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
                outright.setPower(0.72);
                outleft.setPower(0.72);
                //0.8 power for far
            }
//            if (gamepad1.a){
//                hood.setPosition(0.3);
//            }
            if (gamepad1.y){
                hood.setPower(0.3);
//                sleep(100);
//                hood.setPower(-0);
            }
            if (gamepad1.dpad_left){
                hood.setPower(-0.5);
//                sleep(100);
//                hood.setPower(-0);
            }
            if (gamepad1.x){
                outleft.setPower(0);
                outright.setPower(0);
                hood.setPower(-1);
//                sleep(100);
//                hood.setPower(-0);
            }

        }



    }
}
