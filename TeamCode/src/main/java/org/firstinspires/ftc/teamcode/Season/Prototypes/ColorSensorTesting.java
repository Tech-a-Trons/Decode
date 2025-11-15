package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.ExperimentalGreenAndPurple;

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
    public int countBalls() {
        while (artifactcounter < 3) {
            if (colorparser.getColor() == "purple" || colorparser.getColor() == "green") {
                artifactcounter +=1;
            }
        }

        if (artifactcounter == 3) {
            return artifactcounter;
        }
        return artifactcounter;
    }
}
