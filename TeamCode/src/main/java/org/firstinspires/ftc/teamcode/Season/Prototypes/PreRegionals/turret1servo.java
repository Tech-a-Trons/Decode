package org.firstinspires.ftc.teamcode.Season.Prototypes.PreRegionals;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class turret1servo extends LinearOpMode {

    Servo leftservo;
    float pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftservo = hardwareMap.get(Servo.class,"turretleftservo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
//                pos += 0.1;
//                if (pos > 1) {
//                    pos = 1;
//                }
                leftservo.setPosition(0.5); //pos
            }
            if (gamepad1.left_bumper) {
//                pos -= 0.1;
//                if (pos < -1) {
//                    pos = -1;
//                }
                leftservo.setPosition(-0.5); //pos
            }
        }
    }
}
