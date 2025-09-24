package org.firstinspires.ftc.teamcode.Season;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Season.SensorStuff.AprilTagExtractor;
import org.firstinspires.ftc.teamcode.Season.SensorStuff.TagData;

import java.util.ArrayList;
import java.util.Objects;

//This is a basic auton for the blue goal and it first gets the obelisk and then goes on - 9/08
//Added Voltage Sensor - 9/13

@Autonomous
public class BlueBaseOp extends LinearOpMode {
    ColorSensor colorSensor;
    Limelight3A limelight;
    private VoltageSensor myControlHubVoltageSensor;
    SeasonGreenandPurple color;

    @Override
    public void runOpMode() {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        limelight.pipelineSwitch(0);

        waitForStart();

        while (opModeIsActive()) {
            TagData tag = AprilTagExtractor.getAprilTagData();
            // Map your color sensor (check config name)
            double presentVoltage;
            presentVoltage = myControlHubVoltageSensor.getVoltage();
            double powerValue = 0.5;
            double offsetVoltage =  powerValue * (12.5/presentVoltage);

            float phue = color.getPhue();
            float psat = color.getPsat();
            float pval = color.getPval();
            float ghue = color.getGhue();
            float gsat = color.getGsat();
            float gval = color.getGval();

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            int id = tag.id;
            double tx = tag.tx;
            double ty = tag.ty;
            double tz = tag.tz;

            //Telemetry is just for testing
            telemetry.addData("Voltage: ",presentVoltage);
            telemetry.addData("Offset Voltage: ",offsetVoltage);

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Detected Purple?", color.PCheck(colorSensor));
            telemetry.addData("Detected Green?", color.GCheck(colorSensor));
            telemetry.addData("PHue: ", phue);
            telemetry.addData("PSaturation: ",psat);
            telemetry.addData("PValue: ", pval);
            telemetry.addData("GHue: ", ghue);
            telemetry.addData("GSaturation: ",gsat);
            telemetry.addData("GValue: ", gval);
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

            } else if (id == 22) {
                colorIndex.set(0,"PGP");

            } else if (id == 23) {
                colorIndex.set(0,"PPG");

            } else {
                telemetry.addLine("Searching for Oblitag!");
            }

            //then pedro to detect goal, from this we will plan out what to do

            if (id == 20) {
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
}