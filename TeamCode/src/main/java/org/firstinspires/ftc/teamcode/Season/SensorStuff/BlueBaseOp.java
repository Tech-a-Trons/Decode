package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.StableGreenandPurple;

import java.util.ArrayList;
import java.util.Objects;

//This is a basic auton for the blue goal and it first gets the obelisk and then goes on - 9/08
//Added Voltage Sensor - 9/13

@Disabled
@Autonomous
public class BlueBaseOp extends LinearOpMode {
    ColorSensor colorSensor;
    Limelight3A limelight;
    private VoltageSensor myControlHubVoltageSensor;
    StableGreenandPurple color;
    String purpleCheck;
    String greenCheck;

    @Override
    public void runOpMode() {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        limelight.pipelineSwitch(0);

        waitForStart();
        if (color.Getcolor(colorSensor) == "purple") {
            purpleCheck = "Yes";
            greenCheck = "No";
        } else if (color.Getcolor(colorSensor) == "green"){
            purpleCheck = "No";
            greenCheck = "Yes";
        }

        while (opModeIsActive()) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Purple?: ", purpleCheck);
            telemetry.addData("Green?: ", greenCheck);
            telemetry.addData("Hue: ", color.gethue());
            telemetry.addData("Saturation: ", color.getsat());
            telemetry.addData("Value: ", color.getval());
            TagData tag = AprilTagExtractor.getAprilTagData();
            // Map your color sensor (check config name)
            double presentVoltage;
            presentVoltage = myControlHubVoltageSensor.getVoltage();
            double powerValue = 0.5;
            double offsetVoltage =  powerValue * (12.5/presentVoltage);

            int id = tag.id;
            double tx = tag.tx;
            double ty = tag.ty;
            double tz = tag.tz;

            //Telemetry is just for testing
            telemetry.addData("Voltage: ",presentVoltage);
            telemetry.addData("Offset Voltage: ",offsetVoltage);

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