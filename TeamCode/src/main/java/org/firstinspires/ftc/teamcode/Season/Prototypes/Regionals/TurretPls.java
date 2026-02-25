package org.firstinspires.ftc.teamcode.Season.Prototypes.Regionals;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

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
                turretServo1.setPosition(currentServoPos + 0.1);
                turretServo2.setPosition(currentServoPos + 0.1);
                currentServoPos = turretServo1.getPosition();
                telemetry.addData("Pos: ", currentServoPos);
                telemetry.update();
            }

            if (gamepad1.b) {
                turretServo1.setPosition(currentServoPos - 0.1);
                turretServo2.setPosition(currentServoPos - 0.1);
                currentServoPos = turretServo1.getPosition();
                telemetry.addData("Pos: ", currentServoPos);
                telemetry.update();
            }
        }
    }
}
