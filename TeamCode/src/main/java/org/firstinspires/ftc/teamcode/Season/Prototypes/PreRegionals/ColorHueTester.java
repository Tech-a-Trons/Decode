package org.firstinspires.ftc.teamcode.Season.Prototypes.PreRegionals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ColorSensor;

@TeleOp
public class ColorHueTester extends LinearOpMode {

    public float current_sat = 0;
    public float current_hue = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // FIX: Initialize ColorSensor here so hardwareMap is available
        ColorSensor.init(hardwareMap);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // FIX: Read from ColorSensor.artifactcounter, not a local variable
            telemetry.addData("Ball count: ", ColorSensor.artifactcounter);
            telemetry.addData("Alpha: ", ColorSensor.INSTANCE.getalpha());
            telemetry.addData("Hue: ", ColorSensor.INSTANCE.gethue());
            telemetry.addData("Sat: ", ColorSensor.INSTANCE.getsat());
            telemetry.addData("Val: ", ColorSensor.INSTANCE.getval());
            telemetry.addData("Color: ", ColorSensor.INSTANCE.getColor());

            telemetry.update();

            current_sat = ColorSensor.INSTANCE.getsat();
            current_hue = ColorSensor.INSTANCE.gethue();

            ColorSensor.INSTANCE.IncountBalls();
        }
    }
}