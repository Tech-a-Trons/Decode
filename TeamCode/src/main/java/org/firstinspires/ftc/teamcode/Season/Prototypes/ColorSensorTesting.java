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
    public float last_alphavalue2 = 32;

    public float current_alphavalue = 0;
    public float current_alphavalue2 = 0;



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
            telemetry.addData("Alpha2: ", colorparser.getAlpha2());
            telemetry.addData("Current Alpha: ", current_alphavalue);
            telemetry.addData("Last Alpha: ", last_alphavalue);
            telemetry.update();
            IncountBalls();
            light();

        }
        stop();
    }
    public void IncountBalls() {
        String color = colorparser.getColor();
        current_alphavalue = colorparser.getalpha();
        if (artifactcounter < 3) {
            if (current_alphavalue > 40 && last_alphavalue < 40) {
                artifactcounter += 1;
            }
            last_alphavalue = current_alphavalue;
        } else if (artifactcounter == 3) {
            telemetry.addLine("3 BALLS!");
            if (current_alphavalue > 40 && last_alphavalue < 40) {
                artifactcounter += 1;
            }
            last_alphavalue = current_alphavalue;
            //rgbindicator.setPosition(0.5);
        } else if (artifactcounter > 3) {
            if (current_alphavalue > 40 && last_alphavalue < 40) {
                artifactcounter += 1;
            }
            last_alphavalue = current_alphavalue;
        }
    }

    public void light() {
        if (artifactcounter == 0) {
            rgbindicator.setPosition(0);
        } else if (artifactcounter == 1) {
            rgbindicator.setPosition(0.3);
        } else if (artifactcounter == 2) {
            rgbindicator.setPosition(0.375);
        } else if (artifactcounter == 3) {
            rgbindicator.setPosition(0.5);
        } else if (artifactcounter > 3) {
            rgbindicator.setPosition(0.6);
        }
    }

    //DO NOT USE

//    public void OutcountBalls() {
//        String color = colorparser.getColor();
//        current_alphavalue2 = colorparser.getalpha();
//        if (artifactcounter > 0) {
//            if (current_alphavalue2 > 36 && last_alphavalue2 < 36) {
//                artifactcounter -= 1;
//                rgbindicator.setPosition(0);
//            }
//            last_alphavalue = current_alphavalue;
//        } else if (artifactcounter == 0) {
//            telemetry.addLine("0 BALLS!");
//            last_alphavalue = current_alphavalue;
//            rgbindicator.setPosition(0.75);
//        }
//    }
}
