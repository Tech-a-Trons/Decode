package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.Limelight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@Disabled
@TeleOp (name = "P0Example")

public class P0LimelightExample extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        LimelightExtractor ll = new LimelightExtractor();

        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        limelight.pipelineSwitch(0);

        telemetry.addLine("Connecting to Limelight...");
        telemetry.update();

        if (!ll.connect()) {
            telemetry.addLine("Failed to connect!");
            telemetry.update();
            return;
        }

        // Subscribe to vision values
        ll.subscribe("tx");
        ll.subscribe("ty");
        ll.subscribe("ta");

        waitForStart();

        while (opModeIsActive()) {
            String msg = ll.readMessage();
            if (msg != null) {
                telemetry.addData("Raw NT4 msg", msg);
            } else {
                telemetry.addLine("No data yet...");
            }
            telemetry.update();
        }

        ll.close();
    }
}