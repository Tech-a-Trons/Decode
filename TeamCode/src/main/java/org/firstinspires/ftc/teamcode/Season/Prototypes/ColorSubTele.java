package org.firstinspires.ftc.teamcode.Season.Prototypes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ColorSensor;

public class ColorSubTele extends LinearOpMode {
    ColorSensor colorSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Color", colorSensor.artifactcounter);
            telemetry.addData("Green", colorSensor.green);
            telemetry.addData("Purple", colorSensor.purple);


            telemetry.update();
            colorSensor.IncountBalls();
        }

    }
}
