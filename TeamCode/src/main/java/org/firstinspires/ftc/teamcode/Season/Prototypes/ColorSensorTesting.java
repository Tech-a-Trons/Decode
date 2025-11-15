package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.ExperimentalGreenAndPurple;

@TeleOp
public class ColorSensorTesting extends LinearOpMode {

    ExperimentalGreenAndPurple colorparser;
    int artifactcounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        colorparser = new ExperimentalGreenAndPurple();

        waitForStart();

        countBalls();
        telemetry.addData("Ball count: ", artifactcounter);
    }
    public void countBalls() {
        while (artifactcounter < 3) {
            if (colorparser.getColor() == "purple" || colorparser.getColor() == "green") {
                artifactcounter +=1;
            }
        }

        if (artifactcounter == 3) {
        }
    }
}
