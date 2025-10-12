package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.DistanceLimelightExtractor;

public class MoveToDistance extends LinearOpMode {
    Limelight3A limelight;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;
    DistanceLimelightExtractor ll;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");

        ll = new DistanceLimelightExtractor(hardwareMap);
        ll.setTelemetry(telemetry);
        ll.startReading();

        double target = 10.0;

        waitForStart();

        while (opModeIsActive()) {
            if (ll.getDistance() >= target) {
                fl.setPower(-0.01);
                fr.setPower(-0.01);
                bl.setPower(-0.01);
                br.setPower(-0.01);
            } else if (ll.getDistance() <= target) {
                fl.setPower(0.01);
                fr.setPower(0.01);
                bl.setPower(0.01);
                br.setPower(0.01);
            } else if (target - ll.getDistance() < 0.1 || target - ll.getDistance() < -0.1) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            } else if (ll.getDistance() == null) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            } else {
                telemetry.addLine("Smth sus happened");
            }
        }
        ll.stopReading();
    }
}
