package org.firstinspires.ftc.teamcode.Season.TeleOp.Qual1Teleops;

import static java.lang.Math.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.ColorHueFix;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

@TeleOp(name = "SatHueSolo")
public class HueSatSolo extends LinearOpMode {

    VoltageGet volt = new VoltageGet();
    DcMotor activeintake = null;
    DcMotor out1 = null;
    double y;
    double x;
    double rx;
    DcMotor out2 = null;
    DcMotor ramp = null;

    ColorHueFix colorparser;
    Servo rgbindicator;
    public int artifactcounter = 0;
    public float last_alphavalue = 32;
    public float last_alphavalue2 = 32;

    public float current_alphavalue = 0;
    public float current_alphavalue2 = 0;
    public float current_sat = 0;

    public float current_hue = 0;

    double outakeslot = 1.5;

    public int activeslot = 0;
    public int asc = 0;
    // asc = active slot color
    int[] slots = new int[3];

    int[] motif = new int[3];

    int motifadvancement = 1;

    DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
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

        BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor(hardwareMap);
        ll.startReading();
//        ll.setTelemetry(telemetry);

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

        colorparser = new ColorHueFix(hardwareMap);
        rgbindicator = hardwareMap.get(Servo.class, "rgbled");

        telemetry.addLine("Ready!");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

                telemetry.addData("Ball count: ", artifactcounter);
                telemetry.addData("Alpha: ", colorparser.getalpha());
                //telemetry.addData("Alpha2: ", colorparser.getAlpha2());
                telemetry.addData("Current Alpha: ", current_alphavalue);
                telemetry.addData("Last Alpha: ", last_alphavalue);
                telemetry.addData("Hue: ", colorparser.gethue());
                telemetry.addData("Sat: ", colorparser.getsat());
                telemetry.addData("Val: ", colorparser.getval());
                telemetry.addData("Color: ", colorparser.getColor());
                telemetry.update();

                current_sat = colorparser.getsat();
                current_hue = colorparser.gethue();


            ll.update();

            distance = ll.getEuclideanDistance();
            tx = ll.getTx();
            if (tx == null) {
                tx = 0.0;
            }
            if (distance == null) {
                distance = 0.0;
            }

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
            // ---Kill Switches--
            if (gamepad2.a){
                activeintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                activeintake.setPower(0);
            }
            if (gamepad2.b){
                out1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                out2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                out1.setPower(0);
                out2.setPower(0);
                ramp.setPower(0);
            }
            if (gamepad2.y){
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                sleep(100);
            }
            // --- Mechanism Controls ---
            if (gamepad1.a) {
                activeintake.setPower(volt.regulate(1.0));
                ramp.setPower(0.3);
                IncountBalls();
            }
            IncountBalls();

            if (gamepad1.dpad_left && !gamepad2.b) {
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
                out1.setPower(volt.regulate(-0.36));
                out2.setPower(volt.regulate(0.36));
            }

            if (gamepad1.dpad_down) {
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

            if (gamepad1.right_trigger > 0.0 || !gamepad2.b) {
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

            // --- Drivetrain Controls ---
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0 || gamepad2.left_stick_x != 0 || !gamepad2.y) {
                y = -gamepad1.left_stick_y; // forward/back
                x = gamepad1.left_stick_x * 1.1; // strafe
                rx = gamepad1.right_stick_x; // rotation
            } else {
                y = -gamepad2.left_stick_y; // forward/back
                x = gamepad2.left_stick_x * 1.1; // strafe
                rx = gamepad2.right_stick_x; // rotation
            }
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
//            telemetry.addData("Voltage", volt.getVoltage());
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
        double blPower  = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        frontLeftMotor.setPower(flPower);
        frontRightMotor.setPower(frPower);
        backLeftMotor.setPower(blPower);
        backRightMotor.setPower(brPower);
    }

    public void IncountBalls() {
        String color = colorparser.getColor();
        current_sat = colorparser.getsat();
        current_hue = colorparser.gethue();


            if (current_sat > 0.5) {
                artifactcounter += 1;
                asc += 5;
                light();
                sleep(800);
            } else if (current_hue > 167) {
                artifactcounter += 1;
                asc += 1;
                light();
                sleep(800);
            }

//         last_alphavalue = current_alphavalue;
    }
    public void light() {
        if (artifactcounter == 0) {
            rgbindicator.setPosition(0);
        } else if (artifactcounter == 1) {
            rgbindicator.setPosition(0.3);
        } else if (artifactcounter == 2) {
            rgbindicator.setPosition(0.375);
        } else if (artifactcounter == 3) {
            rgbindicator.setPosition(0.5);
        } else if (artifactcounter > 3) {
            rgbindicator.setPosition(0.6);
        }
    }

    public void AssignColors() {
        slots[activeslot] = asc;
        asc = 0;
    }

    public void spindexe() {
        //motor spin to next slot
        activeslot += 1;
        if (activeslot > 2) {
            activeslot = 0;
        }
        if (activeslot < 0) {
                activeslot = 2;
        }
    }

        int green = (slots[0] + slots[1] + slots[2])/5;
        int purple = (slots[0] + slots[1] + slots[2])%5;

        public void Outake_Transfer() {
            double RightNumber = 0;
            double LeftNumber = 0;
            double RightColor = 0;
            double LeftColor = 0;
            int RightPlacement = 0;
            int LeftPlacement = 0;


            motif[1] = 5;
            motif[2] = 1;
            motif[3] = 1;
            outakeslot = activeslot += 1.5;
            if (outakeslot > 2.5){
                outakeslot = 0.5;
            }
            
            RightPlacement = (int) (outakeslot += 0.5);
            if (RightPlacement > 2.5) {
                RightPlacement = 0;
            }
            while (RightColor != motif[motifadvancement]) {
                RightNumber += 1;
                RightColor = slots[RightPlacement];
                RightPlacement = (int) (outakeslot += 1);
                if (RightPlacement > 2.5) {
                    RightPlacement = 0;
                }
            }
            LeftPlacement = (int) (outakeslot -= 0.5);
            while (LeftColor != motif[motifadvancement]) {
                LeftNumber += 1;
                LeftColor = slots[LeftPlacement];
                LeftPlacement = (int) (outakeslot -= 1);
                if (LeftPlacement < 0) {
                    LeftPlacement = 2;
                }
            }
            if (LeftNumber > RightNumber) {
                //move sorter right
            } else if (LeftNumber < RightNumber){
                //move sorter left
            } else if (LeftNumber == RightNumber){
                // move sorter right
            }

            }

    }



