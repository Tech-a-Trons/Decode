package org.firstinspires.ftc.teamcode.Season.TeleOp;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.Outtake.outtake;
import static java.lang.Math.clamp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.ExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.VoltageGet;
@Disabled
@TeleOp(name = "BlackTubeDuoFinal")
public class BlackTubeDuoFinal extends LinearOpMode {

    VoltageGet volt = new VoltageGet();
    DcMotor activeintake = null;
    DcMotor out1 = null;
    DcMotor out2 = null;
    DcMotor ramp = null;
    DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
    private final double STARGET_DISTANCE = 40; // inches
    private final double SANGLE_TOLERANCE = 1.57;
//    private final double FTARGET_DISTANCE = 96.5;
//    private final double FANGLE_TOLERANCE = 27.0;

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

            double sdistanceError = distance - STARGET_DISTANCE;
            //double fdistanceError = distance - FTARGET_DISTANCE;
            double sangleError = tx;
            //double fangleError = tx;

            double sforwardPower = (-sdistanceError * 0.05) * 1;
            double shstrafePower = (-sangleError * 0.03) * 1;
            double sturnPower = (sangleError * 0.02) * 1;

//            double farforwardPower = (-fdistanceError * 0.05) * 1;
//            double fstrafePower = (-fangleError * 0.03) * 1;
//            double fturnPower = (fangleError * 0.02) * 1;

            sforwardPower = clamp(sforwardPower, -0.4, 0.4);
            shstrafePower = clamp(shstrafePower, -0.4, 0.4);
            sturnPower = clamp(sturnPower, -0.3, 0.3);

//            farforwardPower = clamp(farforwardPower, -0.4, 0.4);
//            fstrafePower = clamp(fstrafePower, -0.4, 0.4);
//            fturnPower = clamp(fturnPower, -0.3, 0.3);


            // --- Mechanism Controls ---
            if (gamepad2.a) {
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(0.3);
            }

            if (gamepad2.dpad_left) {
                out1.setPower(volt.regulate(-0.37));
                out2.setPower(volt.regulate(0.37));
                sleep(1000);

                ramp.setPower(volt.regulate(-1.0));
                sleep(100);

//        Outtake.outtake.setPower(volt.regulate(0.1));
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(0));
                sleep(100);

                activeintake.setPower(volt.regulate(0));
                out1.setPower(volt.regulate(-0.34));
                out2.setPower(volt.regulate(0.34));
                sleep(100);

                ramp.setPower(volt.regulate(-1));
//        sleep(50);

//        Outtake.outtake.setPower(volt.regulate(0.1));
//        midtake.newtake.setPower(volt.regulate(0));
                sleep(100);
                out1.setPower(volt.regulate(-0.34));
                out2.setPower(volt.regulate(0.34));
                sleep(100);
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(-1));
                sleep(1000);

                // Stop all
                out1.setPower(volt.regulate(0));
                out2.setPower(volt.regulate(0));
                ramp.setPower(volt.regulate(0));
                activeintake.setPower(volt.regulate(0));
            }

            if (gamepad2.b) {
                out1.setPower(volt.regulate(-0.36));
                out2.setPower(volt.regulate(0.36));
            }

            if (gamepad2.dpad_down) {
                out1.setPower(volt.regulate(0.3));
                out2.setPower(volt.regulate(-0.3));
            }
            if(gamepad2.left_bumper){
                out1.setPower(volt.regulate(-0.3));
                out2.setPower(volt.regulate(0.3));
                sleep(800);
                ramp.setPower(volt.regulate(-1.0));
                sleep(100);
                out1.setPower(volt.regulate(0));
                out2.setPower(volt.regulate(0));
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(0));
                sleep(300);
                activeintake.setPower(volt.regulate(0));
                out1.setPower(volt.regulate(-0.3));
                out2.setPower(volt.regulate(0.3));
                sleep(800);
                ramp.setPower(volt.regulate(-1.0));
                sleep(50);
                out1.setPower(volt.regulate(-0.1));
                out2.setPower(volt.regulate(0.1));
                ramp.setPower(volt.regulate(0));
                sleep(100);
                out1.setPower(volt.regulate(-0.3));
                out2.setPower(volt.regulate(0.3));
                sleep(500);
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(-1.0));

            }
            if (gamepad2.x) {
                activeintake.setPower(0);
                out1.setPower(0);
                out2.setPower(0);
                ramp.setPower(0);
            }

            if (gamepad2.right_trigger > 0.0) {
                ramp.setPower(volt.regulate(gamepad1.right_trigger));
            }

            //Niranjan auto align code is here! - Pranav 10/27

            if (gamepad2.y) {
                if (Math.abs(sdistanceError) == 0 && Math.abs(sangleError) <= SANGLE_TOLERANCE) {
                    frontLeftMotor.setPower(volt.regulate(0.0));
                    frontRightMotor.setPower(volt.regulate(0.0));
                    backLeftMotor.setPower(volt.regulate(0.0));
                    backRightMotor.setPower(volt.regulate(0.0));

                } else {
                    moveMecanum(sforwardPower, shstrafePower, sturnPower);
                }
            }

//            if (gamepad1.dpad_right) {
//                if (Math.abs(fdistanceError) == 0 && Math.abs(fangleError) <= FANGLE_TOLERANCE) {
//                    frontLeftMotor.setPower(volt.regulate(0.0));
//                    frontRightMotor.setPower(volt.regulate(0.0));
//                    backLeftMotor.setPower(volt.regulate(0.0));
//                    backRightMotor.setPower(volt.regulate(0.0));
//                } else {
//                    moveMecanum(farforwardPower, fstrafePower, fturnPower);
//                }
//            }

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