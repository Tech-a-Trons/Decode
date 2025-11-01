package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.StableDistanceLExtractor;

//Same thing as SetDistanceLauncher, but w/Gamepad
@Disabled
@TeleOp(name = "GamepadSetDistanceLauncher")
public class GamepadSetDistanceLauncher extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotor l1,l2;
    private StableDistanceLExtractor limelightExtractor;

    private final double TARGET_DISTANCE = 48.0; // inches
    private final double DISTANCE_TOLERANCE = 1.0;
    private final double ANGLE_TOLERANCE = 2.0;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        l1 = hardwareMap.get(DcMotor.class,"l1");
        l2 = hardwareMap.get(DcMotor.class,"l2");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        limelightExtractor = new StableDistanceLExtractor(hardwareMap);
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

            double forwardPower = (-distanceError * 0.05) * 0.5;
            double strafePower = (-angleError * 0.03) * 0.5;
            double turnPower = (angleError * 0.02) * 0.5;

            forwardPower = clamp(forwardPower, -0.4, 0.4);
            strafePower = clamp(strafePower, -0.4, 0.4);
            turnPower = clamp(turnPower, -0.3, 0.3);

            //I dont think the distance part of this works...
//            if (Math.abs(distanceError) <= DISTANCE_TOLERANCE && Math.abs(angleError) <= ANGLE_TOLERANCE) {
//                stopDrive();
//                telemetry.addLine("Aligned with AprilTag");
//            } else {
//                moveMecanum(forwardPower, strafePower, turnPower);
//            }

            //So I made a new one...
            if (Math.abs(distanceError) == 0 && Math.abs(angleError) <= ANGLE_TOLERANCE) {
                stopDrive();
                telemetry.addLine("Aligned with AprilTag");

                if (gamepad1.a) {
                    //Left Motor
                    l1.setPower(-1);
                    //Right Motor
                    l2.setPower(1);

                    sleep(1000);
                }

                if (gamepad1.x) {
                    l1.setPower(0);
                    l2.setPower(0);
                }
            } else if (Math.abs(distanceError) < 0){
                moveMecanum(-forwardPower,strafePower,turnPower);
            } else {
                moveMecanum(forwardPower,strafePower,turnPower);
            }

            telemetry.addData("Distance: ", distance);
            telemetry.addData("Tx: ", tx);
            telemetry.addData("ForwardPower: ", forwardPower);
            telemetry.addData("StrafePower: ", strafePower);
            telemetry.addData("TurnPower: ", turnPower);
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