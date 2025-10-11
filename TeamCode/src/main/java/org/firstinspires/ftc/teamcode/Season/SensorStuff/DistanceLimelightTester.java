package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.DistanceLimelightExtractor;

@TeleOp
public class DistanceLimelightTester extends LinearOpMode {

    public Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        DistanceLimelightExtractor ll = new DistanceLimelightExtractor(limelight,telemetry);

        telemetry.addLine("Connecting to Limelight...");
        telemetry.update();

        ll.setPipeline(1);

        ll.startReading();

        waitForStart();

        while (opModeIsActive()) {
            ll.update();
            telemetry.update();
        }

        ll.stopReading();
    }
}
