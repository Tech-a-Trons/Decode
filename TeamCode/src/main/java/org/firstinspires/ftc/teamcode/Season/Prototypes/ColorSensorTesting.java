package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Season.Subsystems.ExperimentalArtifacts;

@TeleOp
public class ColorSensorTesting extends LinearOpMode {

    ExperimentalArtifacts colorparser;
    Servo rgbindicator;
    public int artifactcounter = 0;
    public float last_alphavalue = 32;
    public float current_alphavalue = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        colorparser = new ExperimentalArtifacts(hardwareMap);
        rgbindicator = hardwareMap.get(Servo.class, "rgbled");



        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Ball count: ", artifactcounter);
            telemetry.addData("Alpha: ", colorparser.getalpha());
            telemetry.addData("Current Alpha: ", current_alphavalue);
            telemetry.addData("Last Alpha: ", last_alphavalue);
            telemetry.update();
            countBalls();

        }
        stop();
    }
    public void countBalls() {
        String color = colorparser.getColor();
        current_alphavalue = colorparser.getalpha();
        if (artifactcounter < 3) {
            if (current_alphavalue > 40 && last_alphavalue < 40) {
                artifactcounter += 1;
            }
            last_alphavalue = current_alphavalue;
        } else if (artifactcounter == 3) {
            telemetry.addLine("3 BALLS!");
            last_alphavalue = current_alphavalue;
            rgbindicator.setPosition(0.5);
        }
    }
}
