package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class VideoCodeTester extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"Limelight");
        limelight.pipelineSwitch(1);

        //Remember to change values based on control hub position
        limelight.start();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose();
            telemetry.addData("tx: ",result.getTx());
            telemetry.addData("ty: ",result.getTy());
            telemetry.addData("ta: ",result.getTa());
            telemetry.addData("Pose: ",pose);
        }

    }
}
