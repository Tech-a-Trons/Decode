//package org.firstinspires.ftc.teamcode.Season.TeleOp;
//
//import static java.lang.Math.clamp;
//
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.Season.SensorStuff.LimelightTester;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.ColorHueFix;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.ExperimentalArtifacts;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.LLHood;
//
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.core.units.Distance;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//@TeleOp(name = "ColorSensingTele")
//public class Q2Tele extends LinearOpMode implements Subsystem {
//    ColorHueFix colorparser;
//    Servo rgbindicator;
//    public int artifactcounter = 0;
//
//    public float current_alphavalue = 0;
//    public float current_alphavalue2 = 0;
//    private Follower follower;
//    private final Pose startPose = new Pose(123.13, 122.08, Math.toRadians(220)); //See ExampleAuto to understand how to use this
//    private boolean automatedDrive;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
//
//
//    VoltageGet volt = new VoltageGet();
//    DcMotor activeintake = null;
//    DcMotor out1 = null;
//    DcMotor out2 = null;
//    DcMotor ramp = null;
//    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
//
//    int green = 0;
//
//    int purple = 0;
//
//    public float current_sat = 0;
//
//    public float current_hue = 0;
//
//    double outakeslot = 1.5;
//
//    public int activeslot = 0;
//    public int asc = 0;
//    // asc = active slot color
//    int[] slots = new int[10];
//
//    int[] motif = new int[3];
//
//
//    int motifadvancement = 1;
//
//    private final double STARGET_DISTANCE = 58; // inches 42.97
//    private final double SANGLE_TOLERANCE = -1.8;
//    //    private final double MTARGET_DISTANCE = 2838; // PLACEHOLDER
////    private final double MANGLE_TOLERANCE = 134; // PLACEHOLDER
//    private final double FTARGET_DISTANCE = 124; //115
//    private final double FANGLE_TOLERANCE = 1; //3.47
//    ElapsedTime shootTimer = new ElapsedTime();
//    ElapsedTime IncountTimer = new ElapsedTime();
//
//    boolean shooting = false;
//    int shootStep = 0;
//    ElapsedTime shootSlowTimer = new ElapsedTime();
//    boolean slowshooting = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        colorparser = new ColorHueFix(hardwareMap);
//        rgbindicator = hardwareMap.get(Servo.class, "rgbled");
//
//        telemetry.addLine("Ready!");
//        telemetry.update();
//        // Initialize hardware
//        out1 = hardwareMap.get(DcMotor.class, "outtake1");
//        out2 = hardwareMap.get(DcMotor.class, "outtake2");
//        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
//        ramp = hardwareMap.get(DcMotor.class, "ramp");
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        follower.update();
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor(hardwareMap);
//        ll.startReading();
////        ll.setTelemetry(telemetry);
////
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
//        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
//        backRightMotor = hardwareMap.get(DcMotor.class, "br");
//
//        // Initialize voltage sensor system
//        volt.init(hardwareMap);
//
//        // Reverse left motors if needed
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        Double tx = ll.getTx();
//        if (tx == null) {
//            tx = 0.0;
//        }
//        Double distance = ll.getEuclideanDistance();
//        if (distance == null) {
//            distance = 0.0;
//        }
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            IncountBalls();
//            telemetry.addData("Ball count: ", artifactcounter);
////            telemetry.addData("Alpha: ", colorparser.getalpha());
//            //telemetry.addData("Alpha2: ", colorparser.getAlpha2());
////                telemetry.addData("Current Alpha: ", current_alphavalue);
////                telemetry.addData("Last Alpha: ", last_alphavalue);
//            telemetry.addData("Hue: ", colorparser.gethue());
//            telemetry.addData("Sat: ", colorparser.getsat());
////            telemetry.addData("Val: ", colorparser.getval());
////                telemetry.addData("Color: ", colorparser.getColor());
//            telemetry.addData("green", green);
//            telemetry.addData("purple", purple);
//            telemetry.addData("slot 0", slots[0]);
//            telemetry.addData("slot 1", slots[1]);
//            telemetry.addData("slot 2", slots[2]);
//            telemetry.addData("distance", distance);
//            telemetry.update();
//
//            current_sat = colorparser.getsat();
//            current_hue = colorparser.gethue();
//
//
//            follower.update();
//            telemetryM.update();
//            ll.update();
//
//            distance = ll.getEuclideanDistance();
//            tx = ll.getTx();
//            if (tx == null) {
//                tx = 0.0;
//            }
//            if (distance == null) {
//                distance = 0.0;
//            }
//
//            ll.update();
//
//            distance = ll.getEuclideanDistance();
//            tx = ll.getTx();
//            if (tx == null) {
//                tx = 0.0;
//            }
//            if (distance == null) {
//                distance = 0.0;
//            }
//
//            double sdistanceError = distance - STARGET_DISTANCE;
//            //double mdistanceError = distance - MTARGET_DISTANCE;
//            double fdistanceError = distance - FTARGET_DISTANCE;
//            double sangleError = tx;
//            //double mangleError = tx;
//            double fangleError = tx;
//
//            double sforwardPower = (-sdistanceError * 0.05) * 1; //1
//            double shstrafePower = (-sangleError * 0.03) * 1;//1
//            double sturnPower = (sangleError * 0.02) * 1;//1
//
//            double farforwardPower = (-fdistanceError * 0.05) * 1;//1
//            double fstrafePower = (-fangleError * 0.03) * 1;//1
//            double fturnPower = (fangleError * 0.02) * 1;//1
//
////            double mforwardPower = (-mdistanceError * 0.05) * 1;
////            double mstrafePower = (-mangleError * 0.03) * 1;
////            double mturnPower = (mangleError * 0.02) * 1;
//
//            sforwardPower = clamp(sforwardPower, -1, 1);
//            shstrafePower = clamp(shstrafePower, -1, 1);
//            sturnPower = clamp(sturnPower, -1, 1);
//
//            farforwardPower = clamp(farforwardPower, -1, 1);
//            fstrafePower = clamp(fstrafePower, -1, 1);
//            fturnPower = clamp(fturnPower, -1, 1);
//
////            mforwardPower = clamp(mforwardPower, -0.4, 0.4);
////            mstrafePower = clamp(mstrafePower, -0.4, 0.4);
////            mturnPower = clamp(mturnPower, -0.3, 0.3);
//            // ---Kill Switches--
//            if (gamepad2.a) {
//                activeintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                activeintake.setPower(0);
//            }
////            if (gamepad1.b){
////                out1.setPower(volt.regulate(-0.38));
////                out2.setPower(volt.regulate(0.38));
////            }
//            if (gamepad2.x) {
//                ramp.setPower(0);
////                out1.setPower(volt.regulate(-0.38));
////                out2.setPower(volt.regulate(0.38));
//            }
//
//            if (gamepad2.y) {
//                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                frontLeftMotor.setPower(0);
//                backRightMotor.setPower(0);
//                frontRightMotor.setPower(0);
//                backLeftMotor.setPower(0);
//                sleep(100);
//            }
//            // --- Mechanism Controls ---
//            if (gamepad1.a) {
//                activeintake.setPower(volt.regulate(1.0));
//                ramp.setPower(0.4);
//
//            }
//
//            // Start sequence
//            if (gamepad1.dpad_left ) {
//                shooting = true;
//                shootStep = 0;
//                shootTimer.reset();
//            }
//
//// Shooting state machine
//            if (shooting == true) {
//                switch (shootStep) {
//                    case 0:
//                        LLHood.HoodPower();
//                        out1.setPower(volt.regulate(-0.38));
//                        out2.setPower(volt.regulate(0.38));
//
//                        if (shootTimer.milliseconds() > 750) {
//
//                            shootStep++;
//                            shootTimer.reset();
//                        }
//                        break;
//
//                    case 1:
//                        ramp.setPower(volt.regulate(-1.0));
//                        artifactcounter = 0;
//                        slots[0] = 0;
//                        slots[1] = 1;
//                        slots[2] = 2;
//                        light();
//
//                        if (shootTimer.milliseconds() > 100) {
//
//                            shootStep++;
//                            shootTimer.reset();
//                        }
//                        break;
//
//                    case 2:
//
//                        activeintake.setPower(volt.regulate(1.0));
//                        ramp.setPower(0);
//
//                        if (shootTimer.milliseconds() > 100) {
//
//                            shootStep++;
//                            shootTimer.reset();
//                        }
//                        break;
//
//                    case 3:
//                        activeintake.setPower(0);
//                        LLHood.HoodPower();
//
//                        if (shootTimer.milliseconds() > 100) {
//
//                            shootStep++;
//                            shootTimer.reset();
//                        }
//                        break;
//
//                    case 4:
//                        ramp.setPower(volt.regulate(-1));
//
//
//
//                        if (shootTimer.milliseconds() > 100) {
//                            follower.update();
//                            shootStep++;
//                            shootTimer.reset();
//                        }
//                        break;
//
//                    case 5:
//                        out1.setPower(volt.regulate(-0.42));
//                        out2.setPower(volt.regulate(0.42));
//
//                        if (shootTimer.milliseconds() > 100) {
//                            follower.update();
//                            shootStep++;
//                            shootTimer.reset();
//                        }
//                        break;
//
//                    case 6:
//                        activeintake.setPower(volt.regulate(1.0));
//                        ramp.setPower(volt.regulate(-1));
//                        shootStep=0;
//                        // Done
//                        shooting = false;
//                        break;
//                }
//            }
//
//            if (gamepad1.left_bumper) {
//                out1.setPower(volt.regulate(-0.6));
//                out2.setPower(volt.regulate(0.6));
//                sleep(1000);
//
//                ramp.setPower(volt.regulate(-1.0));
//                sleep(100);
//
//                activeintake.setPower(volt.regulate(1.0));
//                ramp.setPower(volt.regulate(0));
//                sleep(100);
//
//                activeintake.setPower(volt.regulate(0));
//                out1.setPower(volt.regulate(-0.6));
//                out2.setPower(volt.regulate(0.6));
//                sleep(100);
//
//                ramp.setPower(volt.regulate(-1));
//
//                sleep(100);
//                out1.setPower(volt.regulate(-0.6));
//                out2.setPower(volt.regulate(0.6));
//                sleep(100);
//                activeintake.setPower(volt.regulate(1.0));
//                ramp.setPower(volt.regulate(-1));
//
//            }
//            if (gamepad1.x) {
//                activeintake.setPower(0);
//                out1.setPower(0);
//                out2.setPower(0);
//                ramp.setPower(0);
//            }
//
//            if (gamepad1.right_trigger > 0.0) {
//                ramp.setPower(0.3);
//                activeintake.setPower(-1);
//                sleep(100);
//                activeintake.setPower(0);
//                ramp.setPower(0);
//            }
//
//            //Niranjan auto align code is here! - Pranav 10/27
//
//            if (gamepad1.y) {
//                if (Math.abs(sdistanceError) == 0 && Math.abs(sangleError) <= SANGLE_TOLERANCE) {
//                    frontLeftMotor.setPower(volt.regulate(0.0));
//                    frontRightMotor.setPower(volt.regulate(0.0));
//                    backLeftMotor.setPower(volt.regulate(0.0));
//                    backRightMotor.setPower(volt.regulate(0.0));
//
//                } else {
//                    moveMecanum(sforwardPower, shstrafePower, sturnPower);
//                }
//            }
//            if (gamepad2.left_bumper) {
//                out1.setPower(volt.regulate(0));
//                out2.setPower(volt.regulate(0));
//            }
//
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
//
////             --- Drivetrain Controls ---
////            double y = -gamepad1.left_stick_y; // forward/back
////            double x = gamepad1.left_stick_x * 1.1; // strafe
////            double rx = gamepad1.right_stick_x; // rotation
//            double y = 0;
//            double x = 0;
//            double rx = 0;
//            if (!gamepad2.y) {
//                y = -gamepad1.left_stick_y * 0.75;
//                x = gamepad1.left_stick_x * 0.75;
//                rx = gamepad1.right_stick_x * 0.75;
//            }
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            // Apply voltage regulation to all drive motors
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//
////             --- Telemetry ---
////            telemetry.addData("Voltage", volt.getVoltage());
//            telemetry.update();
//        }
//        ll.stopReading();
//        frontLeftMotor.setPower(volt.regulate(0.0));
//        frontRightMotor.setPower(volt.regulate(0.0));
//        backLeftMotor.setPower(volt.regulate(0.0));
//        backRightMotor.setPower(volt.regulate(0.0));
//    }
//
//    //I added these bc they worked rly well for rotations - Pranav 10/27
//
//
//    private double clamp(double val, double min, double max) {
//        return Math.max(min, Math.min(max, val));
//    }
//    private void moveMecanum(double forward, double strafe, double turn) {
//        double flPower = forward + strafe + turn;
//        double frPower = forward - strafe - turn;
//        double blPower = forward - strafe + turn;
//        double brPower = forward + strafe - turn;
//
//        frontLeftMotor.setPower(flPower);
//        frontRightMotor.setPower(frPower);
//        backLeftMotor.setPower(blPower);
//        backRightMotor.setPower(brPower);
//    }
//    public void IncountBalls() {
//        String color = colorparser.getColor();
//        current_sat = colorparser.getsat();
//        current_hue = colorparser.gethue();
//        int ColorSensing = 0;
//        if (IncountTimer.milliseconds() > 800) {
//            if (current_sat > 0.5) {
//                artifactcounter += 1;
//                asc += 5;
//                light();
//                AssignColors();
//                activeslot += 1;
//                IncountTimer.reset();
//
//            } else if (current_hue > 167) {
//                artifactcounter += 1;
//                asc += 1;
//
//                light();
//                AssignColors();
//
//                activeslot += 1;
//                IncountTimer.reset();
//            }
//        }
//        // remove activeslot = artifactcounter when new robot is made
//
//        green = (slots[0] + slots[1] + slots[2]) / 5;
//        purple = (slots[0] + slots[1] + slots[2]) % 5;
//    }
//    public void light() {
//        if (artifactcounter == 0) {
//            rgbindicator.setPosition(0);
//        } else if (artifactcounter == 1) {
//            rgbindicator.setPosition(0.3);
//        } else if (artifactcounter == 2) {
//            rgbindicator.setPosition(0.375);
//        } else if (artifactcounter == 3) {
//            rgbindicator.setPosition(0.5);
//        } else if (artifactcounter > 3) {
//            rgbindicator.setPosition(0.6);
//        }
//    }
//
//    public void AssignColors() {
//        slots[activeslot] = asc;
//        asc = 0;
//
//    }
//
//
////    public void spindexe() {
////    motor spin to next slot
////        activeslot = 1;
////        if (activeslot > 2) {
////            activeslot = 0;
////        }
////        if (activeslot < 0) {
////                activeslot = 2;
////     put commented code after new robot is made
//
//}