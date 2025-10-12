package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Season.Subsystems.DistanceLimelightExtractor;

@TeleOp(name = "AutoAlignToAprilTag", group = "Subsystems")
public class AutoAlignToAprilTag extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private DistanceLimelightExtractor limelightExtractor;

    private final double TARGET_DISTANCE = 10.0; // inches
    private final double DISTANCE_TOLERANCE = 1.0;
    private final double ANGLE_TOLERANCE = 2.0;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        limelightExtractor = new DistanceLimelightExtractor(hardwareMap);
        limelightExtractor.setTelemetry(telemetry);
        limelightExtractor.startReading();

        waitForStart();

        while (opModeIsActive()) {
            limelightExtractor.update();

            Double distance = limelightExtractor.getEuclideanDistance();
            Double tx = limelightExtractor.getTx();

            if (distance == null || tx == null) {
                stopDrive();
                telemetry.addLine("No AprilTag detected");
                telemetry.update();
                continue;
            }

            double distanceError = distance - TARGET_DISTANCE;
            double angleError = tx;

            double forwardPower = (distanceError * 0.05) * 0.2;
            double strafePower = (-angleError * 0.03) * 0.2;
            double turnPower = (-angleError * 0.02) * 0.2;

            forwardPower = clamp(forwardPower, -0.4, 0.4);
            strafePower = clamp(strafePower, -0.4, 0.4);
            turnPower = clamp(turnPower, -0.3, 0.3);

            if (Math.abs(distanceError) <= DISTANCE_TOLERANCE && Math.abs(angleError) <= ANGLE_TOLERANCE) {
                stopDrive();
                telemetry.addLine("Aligned with AprilTag");
            } else {
                moveMecanum(forwardPower, strafePower, turnPower);
            }

            telemetry.addData("Distance", distance);
            telemetry.addData("TX", tx);
            telemetry.addData("ForwardPower", forwardPower);
            telemetry.addData("StrafePower", strafePower);
            telemetry.addData("TurnPower", turnPower);
            telemetry.update();
        }

        limelightExtractor.stopReading();
        stopDrive();
    }

    private void moveMecanum(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}