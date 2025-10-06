package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.Limelight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp (name = "P1Example")

public class P1LimelightExample extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        LimelightExtractor ll = new LimelightExtractor();

        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        limelight.pipelineSwitch(1);

        telemetry.addLine("Connecting to Limelight...");
        telemetry.update();

        if (!ll.connect()) {
            telemetry.addLine("Failed to connect!");
            telemetry.update();
            sleep(100000);
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