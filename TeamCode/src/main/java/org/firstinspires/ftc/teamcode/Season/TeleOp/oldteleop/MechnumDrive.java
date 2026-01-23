//package org.firstinspires.ftc.teamcode.Season.TeleOp.oldteleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@TeleOp
//public class MechnumDrive extends LinearOpMode{
//    DcMotor fl;
//
//    DcMotor fr;
//
//    DcMotor bl;
//
//    DcMotor br;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotor frontleftmotor = hardwareMap.get(DcMotor.class,"fl");
//        DcMotor frontrightmotor = hardwareMap.get(DcMotor.class,"fr");
//        DcMotor backrightmotor =  hardwareMap.get(DcMotor.class,"bl");
//        DcMotor backleftmotor =  hardwareMap.get(DcMotor.class,"br");
//
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//
//        if (isStopRequested()) {
//            return;
//        }
//
//        while (opModeIsActive()) {
//
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x * 1.1;
//            double rx = gamepad1.right_stick_x;
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.75);
//
//            double flPower = (y+x+rx) / denominator;
//            double frPower = (y-x-rx) / denominator;
//            double blPower = (y-x+rx) / denominator;
//            double brPower = (y+x-rx) / denominator;
//
//            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            fl.setPower(flPower);
//            fr.setPower(frPower);
//            bl.setPower(blPower);
//            br.setPower(brPower);
//
//
//        }
//    }
//}
