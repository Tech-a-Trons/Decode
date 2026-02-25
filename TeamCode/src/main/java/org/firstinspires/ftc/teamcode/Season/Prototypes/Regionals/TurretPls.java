package org.firstinspires.ftc.teamcode.Season.Prototypes.Regionals;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class TurretPls extends LinearOpMode {

    Servo turretServo1;
    Servo turretServo2;

    @Override
    public void runOpMode() throws InterruptedException {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

        double currentServoPos = 0;
        turretServo1.setPosition(currentServoPos);
        turretServo2.setPosition(currentServoPos);
        telemetry.addData("Pos: ", currentServoPos);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                currentServoPos = Math.min(1.0, currentServoPos + 0.1);
            }

            if (gamepad1.b) {
                currentServoPos = Math.max(0.0, currentServoPos - 0.1);
            }

            turretServo1.setPosition(currentServoPos);
            turretServo2.setPosition(currentServoPos);

            telemetry.addData("Pos: ", currentServoPos);
            telemetry.update();

            sleep(200); // prevents rapid spamming
        }
    }
}
