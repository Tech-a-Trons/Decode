package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.StableDistanceLExtractor;

//To test

@Disabled
@TeleOp
public class DistanceLimelightTester extends LinearOpMode {

    public Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        StableDistanceLExtractor ll = new StableDistanceLExtractor(hardwareMap);

        telemetry.addLine("Connecting to Limelight...");
        telemetry.update();

        ll.setTelemetry(telemetry); // pass telemetry reference
        ll.startReading();

        Double tx = ll.getTx();
        if (tx == null) {tx = 0.0;}
        Double ty = ll.getTy();
        if (ty == null) {ty = 0.0;}
        Double ta = ll.getTa();
        if (ta == null) {ta = 0.0;}

        waitForStart();

        while (opModeIsActive()) {
            ll.update();
            telemetry.update();

            double hAngle = ll.getHorizontalAngle();
            double vAngle = ll.getVerticalAngle();
            boolean visible = ll.isTargetVisible();

            // Optional: use values in your robot logic
            if (visible) {
                telemetry.addData("Info", "Target detected! H: %.2f, V: %.2f", hAngle, vAngle);
            }
        }

        ll.stopReading();
    }
}
