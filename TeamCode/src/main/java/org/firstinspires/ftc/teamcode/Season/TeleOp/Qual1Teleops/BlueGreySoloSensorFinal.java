package org.firstinspires.ftc.teamcode.Season.TeleOp.Qual1Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.ExperimentalGreenAndPurple;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;
@Disabled
@TeleOp(name = "BlueGreySoloSensorFinal")
public class BlueGreySoloSensorFinal extends LinearOpMode {

    ExperimentalGreenAndPurple colorparser;
    VoltageGet volt = new VoltageGet();
    DcMotor activeintake = null;
    DcMotor out1 = null;
    DcMotor out2 = null;
    DcMotor ramp = null;
    //ColorSensor sensor;
    DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
    int artifactcounter = 0;
    private final double STARGET_DISTANCE = 42.97; // inches
    private final double SANGLE_TOLERANCE = -1.8;
//    private final double MTARGET_DISTANCE = 2838; // PLACEHOLDER
//    private final double MANGLE_TOLERANCE = 134; // PLACEHOLDER
    private final double FTARGET_DISTANCE = 112.21;
    private final double FANGLE_TOLERANCE = 3.47;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        out1 = hardwareMap.get(DcMotor.class, "outtake1");
        out2 = hardwareMap.get(DcMotor.class, "outtake2");
        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
        ramp = hardwareMap.get(DcMotor.class, "ramp");

        //sensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor(hardwareMap);
        ll.startReading();
        ll.setTelemetry(telemetry);

        colorparser = new ExperimentalGreenAndPurple(hardwareMap);

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
            //double mdistanceError = distance - MTARGET_DISTANCE;
            double fdistanceError = distance - FTARGET_DISTANCE;
            double sangleError = tx;
            //double mangleError = tx;
            double fangleError = tx;

            double sforwardPower = (-sdistanceError * 0.05) * 1;
            double shstrafePower = (-sangleError * 0.03) * 1;
            double sturnPower = (sangleError * 0.02) * 1;

            double farforwardPower = (-fdistanceError * 0.05) * 1;
            double fstrafePower = (-fangleError * 0.03) * 1;
            double fturnPower = (fangleError * 0.02) * 1;

//            double mforwardPower = (-mdistanceError * 0.05) * 1;
//            double mstrafePower = (-mangleError * 0.03) * 1;
//            double mturnPower = (mangleError * 0.02) * 1;

            sforwardPower = clamp(sforwardPower, -0.4, 0.4);
            shstrafePower = clamp(shstrafePower, -0.4, 0.4);
            sturnPower = clamp(sturnPower, -0.3, 0.3);

            farforwardPower = clamp(farforwardPower, -0.4, 0.4);
            fstrafePower = clamp(fstrafePower, -0.4, 0.4);
            fturnPower = clamp(fturnPower, -0.3, 0.3);

//            mforwardPower = clamp(mforwardPower, -0.4, 0.4);
//            mstrafePower = clamp(mstrafePower, -0.4, 0.4);
//            mturnPower = clamp(mturnPower, -0.3, 0.3);

            // --- Mechanism Controls ---
            if (gamepad1.a) {
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(0.4);
                //countBalls();
            }

            if (gamepad1.dpad_left) {
                artifactcounter -= 3;
                if (artifactcounter < 0) {
                    artifactcounter = 0;
                }
                out1.setPower(volt.regulate(-0.41));
                out2.setPower(volt.regulate(0.41));
                sleep(1000);

                ramp.setPower(volt.regulate(-1.0));
                sleep(100);

//        Outtake.outtake.setPower(volt.regulate(0.1));
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(0));
                sleep(100);

                activeintake.setPower(volt.regulate(0));
                out1.setPower(volt.regulate(-0.41));
                out2.setPower(volt.regulate(0.41));
                sleep(100);

                ramp.setPower(volt.regulate(-1));
//        sleep(50);

//        Outtake.outtake.setPower(volt.regulate(0.1));
//        midtake.newtake.setPower(volt.regulate(0));
                sleep(100);
                out1.setPower(volt.regulate(-0.43));
                out2.setPower(volt.regulate(0.43));
                sleep(100);
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(-1));
            }

            if (gamepad1.b) {
                artifactcounter -= 1;
                if (artifactcounter < 0) {
                    artifactcounter = 0;
                }
                out1.setPower(volt.regulate(-0.36));
                out2.setPower(volt.regulate(0.36));
            }

            if (gamepad1.dpad_down) {
                artifactcounter -= 1;
                if (artifactcounter < 0) {
                    artifactcounter = 0;
                }
                out1.setPower(volt.regulate(0.3));
                out2.setPower(volt.regulate(-0.3));
            }
            if(gamepad1.left_bumper){
                out1.setPower(volt.regulate(-0.6));
                out2.setPower(volt.regulate(0.6));
                sleep(1000);

                ramp.setPower(volt.regulate(-1.0));
                sleep(100);

//        Outtake.outtake.setPower(volt.regulate(0.1));
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(0));
                sleep(100);

                activeintake.setPower(volt.regulate(0));
                out1.setPower(volt.regulate(-0.6));
                out2.setPower(volt.regulate(0.6));
                sleep(100);

                ramp.setPower(volt.regulate(-1));
//        sleep(50);

//        Outtake.outtake.setPower(volt.regulate(0.1));
//        midtake.newtake.setPower(volt.regulate(0));
                sleep(100);
                out1.setPower(volt.regulate(-0.6));
                out2.setPower(volt.regulate(0.6));
                sleep(100);
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(volt.regulate(-1));

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
                if (Math.abs(sdistanceError) == 0 && Math.abs(sangleError) <= SANGLE_TOLERANCE) {
                    frontLeftMotor.setPower(volt.regulate(0.0));
                    frontRightMotor.setPower(volt.regulate(0.0));
                    backLeftMotor.setPower(volt.regulate(0.0));
                    backRightMotor.setPower(volt.regulate(0.0));

                } else {
                    moveMecanum(sforwardPower, shstrafePower, sturnPower);
                }
            }

            if (gamepad1.dpad_right) {
                if (Math.abs(fdistanceError) == 0 && Math.abs(fangleError) <= FANGLE_TOLERANCE) {
                    frontLeftMotor.setPower(volt.regulate(0.0));
                    frontRightMotor.setPower(volt.regulate(0.0));
                    backLeftMotor.setPower(volt.regulate(0.0));
                    backRightMotor.setPower(volt.regulate(0.0));
                } else {
                    moveMecanum(farforwardPower, fstrafePower, fturnPower);
                }
            }

//            if (gamepad1.right_bumper) {
//                if (Math.abs(mdistanceError) == 0 && Math.abs(mangleError) <= MANGLE_TOLERANCE) {
//                    frontLeftMotor.setPower(volt.regulate(0.0));
//                    frontRightMotor.setPower(volt.regulate(0.0));
//                    backLeftMotor.setPower(volt.regulate(0.0));
//                    backRightMotor.setPower(volt.regulate(0.0));
//                } else {
//                    moveMecanum(mforwardPower, mstrafePower, mturnPower);
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

//    public int countBalls() {
//        String colorGet = colorparser.getColor();
//        if (colorGet == null) {
//            colorGet = "VALUE";
//        }
//        while (artifactcounter < 3) {
//            if (colorGet == "purple" || colorGet == "green") {
//                artifactcounter +=1;
//            } else if (colorGet == "VALUE") {
//
//            } else {
//
//            }
//        }
//
//        if (artifactcounter == 3) {
//            activeintake.setPower(0);
//            //ramp.setPower(0);
//        }
//        return artifactcounter;
//    }
}