package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LimelightTester extends LinearOpMode {

    public Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        SeasonLimelightExtractor ll = new SeasonLimelightExtractor();
        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        limelight.pipelineSwitch(1);

//        Double tx = ll.getTx();
//        if (tx == null) {tx = 0.0;}
//        Double ty = ll.getTy();
//        if (ty == null) {ty = 0.0;}
//        Double ta = ll.getTa();
//        if (ta == null) {ta = 0.0;}
//        Double tl = ll.getTl();
//        if (tl == null) {tl = 0.0;}
//        String status = ll.getStatus();

        // Telemetry-safe: use fallback text if null
//        telemetry.addData("tx", tx != null ? String.format("%.2f", tx) : "N/A");

        telemetry.addLine("Connecting to Limelight...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ll.update();
            ll.addTelemetry();
            telemetry.update();
            sleep(35);
        }
        ll.close();
    }
}
