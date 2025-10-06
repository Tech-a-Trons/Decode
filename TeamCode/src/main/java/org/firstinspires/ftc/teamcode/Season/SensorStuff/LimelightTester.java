package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimelightTester extends LinearOpMode {

    public Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        SeasonLimelightExtractor ll = new SeasonLimelightExtractor();
        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        limelight.pipelineSwitch(1);

        ll.setTelemetry(telemetry); // pass telemetry reference
        ll.startReading();

        //just getting values. this is a framework to base my other codes on
        Double tx = ll.getTx();
        if (tx == null) {tx = 0.0;}
        Double ty = ll.getTy();
        if (ty == null) {ty = 0.0;}
        Double ta = ll.getTa();
        if (ta == null) {ta = 0.0;}
        Double tl = ll.getTl();
        if (tl == null) {tl = 0.0;}
        String status = ll.getStatus();
        Double fps = ll.getFps();

        // Telemetry-safe: use fallback text if null
//        telemetry.addData("tx", tx != null ? String.format("%.2f", tx) : "N/A");

        telemetry.addLine("Connecting to Limelight...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ll.update();
            telemetry.update();
        }
        ll.stopReading();
    }
}
