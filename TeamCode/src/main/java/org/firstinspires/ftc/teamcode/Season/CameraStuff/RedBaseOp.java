package org.firstinspires.ftc.teamcode.Season.CameraStuff;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.ArrayList;
import java.util.Objects;

//This is a basic auton for the red goal and it first gets the obelisk and then goes on - 9/08

@Autonomous
public class RedBaseOp extends LinearOpMode {
    ColorSensor colorSensor;
    Limelight3A limelight;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        limelight.pipelineSwitch(0);

        waitForStart();

        while (opModeIsActive()) {
            TagData tag = AprilTagExtractor.getAprilTagData();
            // Map your color sensor (check config name)
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

            waitForStart();

            boolean purpleSeen = PCheck(colorSensor);
            boolean greenSeen = GCheck(colorSensor);

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            int id = tag.id;
            double tx = tag.tx;
            double ty = tag.ty;
            double tz = tag.tz;

            //Telemetry is just for testing

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Detected Purple?", purpleSeen);
            telemetry.addData("Detected Green?", greenSeen);
            telemetry.update();

            if (tag.valid) {
                telemetry.addData("Tag ID", id);
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("tz", tz);
            } else {
                telemetry.addLine("No AprilTags detected");
            }

            ArrayList<String> colorIndex = new ArrayList<>();

            telemetry.update();

            //pedro here to get into position to find oblitag

            if (id == 21) {
                colorIndex.set(0,"GPP");

                limelight.pipelineSwitch(2);

            } else if (id == 22) {
                colorIndex.set(0,"PGP");

                limelight.pipelineSwitch(2);

            } else if (id == 23) {
                colorIndex.set(0,"PPG");

                limelight.pipelineSwitch(2);
            } else {
                telemetry.addLine("Searching for Oblitag!");
            }

            limelight.pipelineSwitch(2);

            if (id == 24) {
                if (Objects.equals(colorIndex.get(0), "GPP")) {
                    //pedro
                } else if (Objects.equals(colorIndex.get(0), "PGP")) {
                    //pedro
                } else if (Objects.equals(colorIndex.get(0), "PPG")) {
                    //pedro
                } else {
                    telemetry.addLine("Oblitag wasn't found!");
                }
            } else {
                telemetry.addLine("Searching for goal!");
            }
        }
    }

    public boolean PCheck(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        int total = red + green + blue;
        if (total == 0) return false;

        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        return (rNorm > 0.30 && bNorm > 0.30 && gNorm < 0.25);
    }

    public boolean GCheck(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        int total = red + green + blue;
        if (total == 0) return false;

        double rNorm = (double) red / total;
        double gNorm = (double) green / total;
        double bNorm = (double) blue / total;

        return (gNorm > 0.40 && rNorm < 0.35 && bNorm < 0.35);
    }
}