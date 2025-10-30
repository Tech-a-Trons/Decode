package org.firstinspires.ftc.teamcode.Season.TeleOp;

import static java.lang.Math.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.ExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.VoltageGet;

@TeleOp(name = "TsFastSoloPls")
public class TsFastSolo extends LinearOpMode {

    VoltageGet volt = new VoltageGet();
    DcMotor activeintake = null;
    DcMotor out1 = null;
    DcMotor out2 = null;
    DcMotor ramp = null;
    DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
    private final double TARGET_DISTANCE = 48.0; // inches
    private final double ANGLE_TOLERANCE = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        out1 = hardwareMap.get(DcMotor.class, "outtake1");
        out2 = hardwareMap.get(DcMotor.class, "outtake2");
        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
        ramp = hardwareMap.get(DcMotor.class, "ramp");

        ExperimentalDistanceLExtractor ll = new ExperimentalDistanceLExtractor(hardwareMap);
        ll.startReading();
        ll.setTelemetry(telemetry);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");

        // Initialize voltage sensor system
        volt.init(hardwareMap);

        // Reverse left motors if needed
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Double tx = ll.getTx();
        if (tx == null) {
            tx = 0.0;
        }
        Double distance = ll.getEuclideanDistance();
        if (distance == null) {
            distance = 0.0;
        }

        waitForStart();

        while (opModeIsActive()) {
            ll.update();

            distance = ll.getEuclideanDistance();
            tx = ll.getTx();
            if (tx == null) {
                tx = 0.0;
            }
            if (distance == null) {
                distance = 0.0;
            }

            double distanceError = distance - TARGET_DISTANCE;
            double angleError = tx;

            double forwardPower = (-distanceError * 0.05) * 1;
            double strafePower = (-angleError * 0.03) * 1;
            double turnPower = (angleError * 0.02) * 1;

            forwardPower = clamp(forwardPower, -0.4, 0.4);
            strafePower = clamp(strafePower, -0.4, 0.4);
            turnPower = clamp(turnPower, -0.3, 0.3);


            // --- Mechanism Controls ---
            if (gamepad1.a) {
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(0.3);
            }

            if (gamepad1.dpad_left) {
                ramp.setPower(volt.regulate(-1.0));
                sleep(100);
                out1.setPower(volt.regulate(0));
                out2.setPower(volt.regulate(0));
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(0));
                sleep(300);
                activeintake.setPower(volt.regulate(0));
                out1.setPower(volt.regulate(-0.36));
                out2.setPower(volt.regulate(0.36));
                sleep(800);
                ramp.setPower(volt.regulate(-1.0));
                sleep(50);
                out1.setPower(volt.regulate(-0.1));
                out2.setPower(volt.regulate(0.1));
                ramp.setPower(volt.regulate(0));
                sleep(100);
                out1.setPower(volt.regulate(-0.36));
                out2.setPower(volt.regulate(0.36));
                sleep(500);
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(-1.0));
            }

            if (gamepad1.b) {
                out1.setPower(volt.regulate(-0.32));
                out2.setPower(volt.regulate(0.32));
            }

            if (gamepad1.dpad_down) {
                out1.setPower(volt.regulate(0.3));
                out2.setPower(volt.regulate(-0.3));
            }
            if(gamepad1.left_bumper){
                out1.setPower(volt.regulate(-0.45));
                out2.setPower(volt.regulate(0.45));
                sleep(800);
                ramp.setPower(volt.regulate(-1.0));
                sleep(100);
                out1.setPower(volt.regulate(0));
                out2.setPower(volt.regulate(0));
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(0));
                sleep(300);
                activeintake.setPower(volt.regulate(0));
                out1.setPower(volt.regulate(-0.45));
                out2.setPower(volt.regulate(0.45));
                sleep(800);
                ramp.setPower(volt.regulate(-1.0));
                sleep(50);
                out1.setPower(volt.regulate(-0.1));
                out2.setPower(volt.regulate(0.1));
                ramp.setPower(volt.regulate(0));
                sleep(100);
                out1.setPower(volt.regulate(-0.45));
                out2.setPower(volt.regulate(0.45));
                sleep(500);
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(-1.0));

            }
            if (gamepad1.x) {
                activeintake.setPower(0);
                out1.setPower(0);
                out2.setPower(0);
                ramp.setPower(0);
            }

            if (gamepad1.right_trigger > 0.0) {
                ramp.setPower(volt.regulate(gamepad1.right_trigger));
            }

            //Niranjan auto align code is here! - Pranav 10/27

            if (gamepad1.y) {
                if (Math.abs(distanceError) == 0 && Math.abs(angleError) <= ANGLE_TOLERANCE) {
                    frontLeftMotor.setPower(volt.regulate(0.0));
                    frontRightMotor.setPower(volt.regulate(0.0));
                    backLeftMotor.setPower(volt.regulate(0.0));
                    backRightMotor.setPower(volt.regulate(0.0));
                } else {
                    moveMecanum(forwardPower, strafePower, turnPower);
                }
            }

            // --- Drivetrain Controls ---
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x * 1.1; // strafe
            double rx = gamepad1.right_stick_x; // rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Apply voltage regulation to all drive motors
            frontLeftMotor.setPower(volt.regulate(frontLeftPower));
            backLeftMotor.setPower(volt.regulate(backLeftPower));
            frontRightMotor.setPower(volt.regulate(frontRightPower));
            backRightMotor.setPower(volt.regulate(backRightPower));

            // --- Telemetry ---
            telemetry.addData("Voltage", volt.getVoltage());
            telemetry.update();
        }
        ll.stopReading();
        frontLeftMotor.setPower(volt.regulate(0.0));
        frontRightMotor.setPower(volt.regulate(0.0));
        backLeftMotor.setPower(volt.regulate(0.0));
        backRightMotor.setPower(volt.regulate(0.0));
    }

    //I added these bc they worked rly well for rotations - Pranav 10/27

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    private void moveMecanum(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        frontLeftMotor.setPower(flPower);
        frontRightMotor.setPower(frPower);
        backLeftMotor.setPower(blPower);
        backRightMotor.setPower(brPower);
    }
}