package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LiftServo0 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Servo hood = null;

        hood = hardwareMap.get(Servo.class, "hood");


        waitForStart();

        while (opModeIsActive()) {
            hood.setPosition(0);


        }
    }
}
