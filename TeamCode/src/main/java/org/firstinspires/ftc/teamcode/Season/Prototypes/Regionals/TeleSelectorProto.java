package org.firstinspires.ftc.teamcode.Season.Prototypes.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TeleSelectorProto extends LinearOpMode {
    boolean isred = true;
    boolean isblue = false;
    @Override
    public void runOpMode() throws InterruptedException {

        //init will be here

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                isred = true;
                isblue = false;
                telemetry.addLine("Red TeleOp");
            } else if (gamepad1.dpad_left) {
                isred = false;
                isblue = true;
                telemetry.addLine("Blue TeleOp");
            }
        }
    }
}
