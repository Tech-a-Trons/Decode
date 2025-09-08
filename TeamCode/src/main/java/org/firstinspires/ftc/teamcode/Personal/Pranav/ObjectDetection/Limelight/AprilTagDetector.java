package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.Limelight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.AprilTagExtractor;
import org.firstinspires.ftc.teamcode.Season.TagData;

@TeleOp
public class AprilTagDetector extends LinearOpMode {

    //Example AprilTag code that uses my methods

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            TagData tag = AprilTagExtractor.getAprilTagData();

            if (tag.valid) {
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("tx", tag.tx);
                telemetry.addData("ty", tag.ty);
                telemetry.addData("tz", tag.tz);
            } else {
                telemetry.addLine("No AprilTags detected");
            }

            telemetry.update();
        }
    }
}


