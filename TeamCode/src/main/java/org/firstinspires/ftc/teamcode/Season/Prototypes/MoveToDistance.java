package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Season.Subsystems.StableDistanceLExtractor;

//Pranav's code, distance + rotation
@TeleOp
public class MoveToDistance extends LinearOpMode {
    Limelight3A limelight;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;
    StableDistanceLExtractor ll;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);


        ll = new StableDistanceLExtractor(hardwareMap);
        ll.setTelemetry(telemetry);
        ll.startReading();

        double target = 10.0;

        Double tx = ll.getTx();
        if (tx == null) {
            tx = 0.0;
        }

        Double distance = ll.getDistance();
        if (distance == null) {
            distance = 0.0;
        }

        waitForStart();

        while (opModeIsActive()) {
            ll.startReading();
            ll.update();

            tx = ll.getTx();
            if (tx == null) {
                tx = 0.0;
            }

            distance = ll.getDistance();
            if (distance == null) {
                distance = 0.0;
            }

            if (distance >= target) {
                fl.setPower(-0.15);
                fr.setPower(-0.15);
                bl.setPower(-0.15);
                br.setPower(-0.15);
            } else if (distance <= target) {
                fl.setPower(0.15);
                fr.setPower(0.15);
                bl.setPower(0.15);
                br.setPower(0.15);
            } else if (target - distance < 0.1 && target - distance < -0.1) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                break;
            } else if (distance == 0.0) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                break;
            } else {
                telemetry.addLine("Smth sus happened");
                telemetry.update();
            }

            if (tx > 1.0) {
                fl.setPower(0.15);
                fr.setPower(-0.15);
                bl.setPower(0.15);
                br.setPower(-0.15);
            } else if (tx < -1.0) {
                fl.setPower(-0.15);
                fr.setPower(0.15);
                bl.setPower(-0.15);
                br.setPower(0.15);
            } else if (tx < 1.0 && tx > -1.0) {
                fl.setPower(0.0);
                fr.setPower(0.0);
                bl.setPower(0.0);
                br.setPower(0.0);
                break;
            } else if (tx == 0.0){
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                break;
            } else {
                telemetry.addLine("JUNK");
                telemetry.update();
            }
        }
        ll.stopReading();
    }
}