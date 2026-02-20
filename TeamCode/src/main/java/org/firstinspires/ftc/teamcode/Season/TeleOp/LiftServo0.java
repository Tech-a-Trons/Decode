package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LiftServo0 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Servo liftservo1 = null;
        Servo liftservo2 = null;

        liftservo1 = hardwareMap.get(Servo.class, "lift1");

        liftservo2 = hardwareMap.get(Servo.class, "lift2");

        waitForStart();

        while (opModeIsActive()) {
            liftservo1.setPosition(0);
            liftservo2.setPosition(0);
            telemetry.addData("Servo1", liftservo1.getPosition());
            telemetry.addData("Servo2", liftservo2.getPosition());

        }
    }
}
