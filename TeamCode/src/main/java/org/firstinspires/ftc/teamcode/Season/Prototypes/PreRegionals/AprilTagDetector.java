package org.firstinspires.ftc.teamcode.Season.Prototypes.PreRegionals;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Season.SensorStuff.AprilTagExtractor;
import org.firstinspires.ftc.teamcode.Season.SensorStuff.TagData;


//Old Apriltag code

@Disabled
@TeleOp
public class AprilTagDetector extends LinearOpMode {

    //Example AprilTag code that uses my methods

    Limelight3A limelight;
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        limelight.pipelineSwitch(0);

        telemetry.addLine("Connecting to Limelight...");
        telemetry.update();

        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            if (tag.id == 21) {

                fl.setTargetPosition(25);
                fr.setTargetPosition(25);
                bl.setTargetPosition(25);
                br.setTargetPosition(25);

                fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                fl.setPower(0.05);
                fr.setPower(0.05);
                bl.setPower(0.05);
                br.setPower(0.05);

            } else if (tag.id == 22) {

                fl.setTargetPosition(25);
                fr.setTargetPosition(25);
                bl.setTargetPosition(25);
                br.setTargetPosition(25);

                fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                fl.setPower(0.05);
                fr.setPower(0.05);
                bl.setPower(0.05);
                br.setPower(0.05);

            } else if (tag.id == 23) {

                fl.setTargetPosition(25);
                fr.setTargetPosition(25);
                bl.setTargetPosition(25);
                br.setTargetPosition(25);

                fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                fl.setPower(0.05);
                fr.setPower(0.05);
                bl.setPower(0.05);
                br.setPower(0.05);

            } else {
                telemetry.addLine("Searching!");
            }
        }
    }
}


