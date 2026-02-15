package org.firstinspires.ftc.teamcode.Season.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoSetter0 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretServo1 = null;
        Servo turretServo2 = null;

        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");

        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

        waitForStart();

        while (opModeIsActive()) {
            turretServo1.setPosition(0);
            turretServo2.setPosition(0);
            telemetry.addData("Servo1", turretServo1.getPosition());
            telemetry.addData("Servo2", turretServo2.getPosition());

        }
    }
}
