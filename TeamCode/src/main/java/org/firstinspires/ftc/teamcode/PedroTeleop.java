//package org.firstinspires.ftc.teamcode;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import static org.firstinspires.ftc.teamcode.Season.Pedro.Tuning.follower;
//import static java.lang.Math.clamp;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.VoltageGet;
//import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.VoltageGet;
//
//import java.util.function.Supplier;
//
//@Configurable
//@TeleOp
//public class PedroTestTeleop extends OpMode {
//    private Follower follower;
//    public Pose startingPose; //See ExampleAuto to understand how to use this
//    private boolean automatedDrive;
//    private Supplier<PathChain> pathChain;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
//    VoltageGet volt = new VoltageGet();
//    DcMotor activeintake = null;
//    DcMotor out1 = null;
//    DcMotor out2 = null;
//    DcMotor ramp = null;
//    DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
//    private final double STARGET_DISTANCE = 42.97; // inches
//    private final double SANGLE_TOLERANCE = -1.8;
//    //    private final double MTARGET_DISTANCE = 2838; // PLACEHOLDER
////    private final double MANGLE_TOLERANCE = 134; // PLACEHOLDER
//    private final double FTARGET_DISTANCE = 112.21;
//    private final double FANGLE_TOLERANCE = 3.47;
//
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        out1 = hardwareMap.get(DcMotor.class, "outtake1");
//        out2 = hardwareMap.get(DcMotor.class, "outtake2");
//        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
//        ramp = hardwareMap.get(DcMotor.class, "ramp");
//        BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor(hardwareMap);
//        ll.startReading();
//        ll.setTelemetry(telemetry);
//
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
//
//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
//    }
//
//    @Override
//    public void start() {
//        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
//        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
//        //If you don't pass anything in, it uses the default (false)
//        follower.startTeleopDrive();
//    }
//
//    @Override
//    public void loop() {
//        Double tx = ll.getTx();
//        if (tx == null) {
//            tx = 0.0;
//        }
//        Double distance = ll.getEuclideanDistance();
//        if (distance == null) {
//            distance = 0.0;
//        }
//        //Call this once per loop
//        follower.update();
//        telemetryM.update();
//        ll.update();
//
//        distance = ll.getEuclideanDistance();
//        tx = ll.getTx();
//        if (tx == null) {
//            tx = 0.0;
//        }
//        if (distance == null) {
//            distance = 0.0;
//        }
//
//        ll.update();
//
//        distance = ll.getEuclideanDistance();
//        tx = ll.getTx();
//        if (tx == null) {
//            tx = 0.0;
//        }
//        if (distance == null) {
//            distance = 0.0;
//        }
//
//        double sdistanceError = distance - STARGET_DISTANCE;
//        //double mdistanceError = distance - MTARGET_DISTANCE;
//        double fdistanceError = distance - FTARGET_DISTANCE;
//        double sangleError = tx;
//        //double mangleError = tx;
//        double fangleError = tx;
//
//        double sforwardPower = (-sdistanceError * 0.05) * 1;
//        double shstrafePower = (-sangleError * 0.03) * 1;
//        double sturnPower = (sangleError * 0.02) * 1;
//
//        double farforwardPower = (-fdistanceError * 0.05) * 1;
//        double fstrafePower = (-fangleError * 0.03) * 1;
//        double fturnPower = (fangleError * 0.02) * 1;
//
////            double mforwardPower = (-mdistanceError * 0.05) * 1;
////            double mstrafePower = (-mangleError * 0.03) * 1;
////            double mturnPower = (mangleError * 0.02) * 1;
//
//        sforwardPower = clamp(sforwardPower, -0.4, 0.4);
//        shstrafePower = clamp(shstrafePower, -0.4, 0.4);
//        sturnPower = clamp(sturnPower, -0.3, 0.3);
//
//        farforwardPower = clamp(farforwardPower, -0.4, 0.4);
//        fstrafePower = clamp(fstrafePower, -0.4, 0.4);
//        fturnPower = clamp(fturnPower, -0.3, 0.3);
//
////            mforwardPower = clamp(mforwardPower, -0.4, 0.4);
////            mstrafePower = clamp(mstrafePower, -0.4, 0.4);
////            mturnPower = clamp(mturnPower, -0.3, 0.3);
//        // ---Kill Switches--
//        if (gamepad2.a){
//            activeintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            activeintake.setPower(0);
//        }
////            if (gamepad2.b){
////                ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////                ramp.setPower(0);
////            }
//
//        if (gamepad2.y) {
//            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            frontLeftMotor.setPower(0);
//            backRightMotor.setPower(0);
//            frontRightMotor.setPower(0);
//            backLeftMotor.setPower(0);
//            sleep(100);
//        }
//        // --- Mechanism Controls ---
//        if (gamepad1.a) {
//            activeintake.setPower(volt.regulate(1.0));
//            ramp.setPower(0.3);
//        }
//
//        if (gamepad1.dpad_left) {
//            follower.holdPoint(follower.getPose());
//            out1.setPower(volt.regulate(-0.41));
//            out2.setPower(volt.regulate(0.41));
//            sleep(1000);
//
//            ramp.setPower(volt.regulate(-1.0));
//            sleep(100);
//
////        Outtake.outtake.setPower(volt.regulate(0.1));
//            activeintake.setPower(volt.regulate(1.0));
//            ramp.setPower(volt.regulate(0));
//            sleep(100);
//
//            activeintake.setPower(volt.regulate(0));
//            out1.setPower(volt.regulate(-0.41));
//            out2.setPower(volt.regulate(0.41));
//            sleep(100);
//            ramp.setPower(volt.regulate(-1));
////        sleep(50);
//
////        Outtake.outtake.setPower(volt.regulate(0.1));
////        midtake.newtake.setPower(volt.regulate(0));
//            sleep(100);
//            out1.setPower(volt.regulate(-0.43));
//            out2.setPower(volt.regulate(0.43));
//            sleep(100);
//            activeintake.setPower(volt.regulate(1.0));
//            ramp.setPower(volt.regulate(-1));
//            follower.setTeleOpDrive(1,1,1,true);
//        }
//
//        if (gamepad1.b) {
//            out1.setPower(volt.regulate(-0.36));
//            out2.setPower(volt.regulate(0.36));
//        }
//
//        if (gamepad1.dpad_down) {
//            out1.setPower(volt.regulate(0.3));
//            out2.setPower(volt.regulate(-0.3));
//        }
//        if (gamepad1.left_bumper) {
//            out1.setPower(volt.regulate(-0.6));
//            out2.setPower(volt.regulate(0.6));
//            sleep(1000);
//
//            ramp.setPower(volt.regulate(-1.0));
//            sleep(100);
//
////        Outtake.outtake.setPower(volt.regulate(0.1));
//            activeintake.setPower(volt.regulate(1.0));
//            ramp.setPower(volt.regulate(0));
//            sleep(100);
//
//            activeintake.setPower(volt.regulate(0));
//            out1.setPower(volt.regulate(-0.6));
//            out2.setPower(volt.regulate(0.6));
//            sleep(100);
//
//            ramp.setPower(volt.regulate(-1));
////        sleep(50);
//
////        Outtake.outtake.setPower(volt.regulate(0.1));
////        midtake.newtake.setPower(volt.regulate(0));
//            sleep(100);
//            out1.setPower(volt.regulate(-0.6));
//            out2.setPower(volt.regulate(0.6));
//            sleep(100);
//            activeintake.setPower(volt.regulate(1.0));
//            ramp.setPower(volt.regulate(-1));
//
//        }
//        if (gamepad1.x) {
//            activeintake.setPower(0);
//            out1.setPower(0);
//            out2.setPower(0);
//            ramp.setPower(0);
//        }
//
//        if (gamepad1.right_trigger > 0.0) {
//            ramp.setPower(0.3);
//            activeintake.setPower(-1);
//            sleep(100);
//            activeintake.setPower(0);
//            ramp.setPower(0);
//        }
//
//        //Niranjan auto align code is here! - Pranav 10/27
//
//        if (gamepad1.y) {
//            if (Math.abs(sdistanceError) == 0 && Math.abs(sangleError) <= SANGLE_TOLERANCE) {
//                frontLeftMotor.setPower(volt.regulate(0.0));
//                frontRightMotor.setPower(volt.regulate(0.0));
//                backLeftMotor.setPower(volt.regulate(0.0));
//                backRightMotor.setPower(volt.regulate(0.0));
//
//            } else {
//                moveMecanum(sforwardPower, shstrafePower, sturnPower);
//            }
//        }
//
//        if (gamepad1.dpad_right) {
//            if (Math.abs(fdistanceError) == 0 && Math.abs(fangleError) <= FANGLE_TOLERANCE) {
//                frontLeftMotor.setPower(volt.regulate(0.0));
//                frontRightMotor.setPower(volt.regulate(0.0));
//                backLeftMotor.setPower(volt.regulate(0.0));
//                backRightMotor.setPower(volt.regulate(0.0));
//            } else {
//                moveMecanum(farforwardPower, fstrafePower, fturnPower);
//            }
//        }
//
////             --- Drivetrain Controls ---
////            double y = -gamepad1.left_stick_y; // forward/back
////            double x = gamepad1.left_stick_x * 1.1; // strafe
////            double rx = gamepad1.right_stick_x; // rotation
//        double y = 0;
//        double x = 0;
//        double rx = 0;
//        if (!automatedDrive) {
//            //Make the last parameter false for field-centric
//            //In case the drivers want to use a "slowMode" you can scale the vectors
//            //This is the normal version to use in the TeleOp
//            if (!slowMode) follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    true // Robot Centric
//            );
//                //This is how it looks with slowMode on
//            else follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y * slowModeMultiplier,
//                    -gamepad1.left_stick_x * slowModeMultiplier,
//                    -gamepad1.right_stick_x * slowModeMultiplier,
//                    true // Robot Centric
//            );
//        }
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
////            if (!gamepad2.y) {
////                y = -gamepad1.left_stick_y;
////                x = gamepad1.left_stick_x * 1.1;
////                rx = gamepad1.right_stick_x;
////            }
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        // Apply voltage regulation to all drive motors
//        frontLeftMotor.setPower(volt.regulate(frontLeftPower));
//        backLeftMotor.setPower(volt.regulate(backLeftPower));
//        frontRightMotor.setPower(volt.regulate(frontRightPower));
//        backRightMotor.setPower(volt.regulate(backRightPower));
//
////             --- Telemetry ---
//        telemetry.addData("Voltage", volt.getVoltage());
//        telemetry.update();
//    }
//        ll.stopReading();
//        frontLeftMotor.setPower(volt.regulate(0.0));
//        frontRightMotor.setPower(volt.regulate(0.0));
//        backLeftMotor.setPower(volt.regulate(0.0));
//        backRightMotor.setPower(volt.regulate(0.0));
//}
//
////I added these bc they worked rly well for rotations - Pranav 10/27
//
//private double clamp(double val, double min, double max) {
//    return Math.max(min, Math.min(max, val));
//}
//private void moveMecanum(double forward, double strafe, double turn) {
//    double flPower = forward + strafe + turn;
//    double frPower = forward - strafe - turn;
//    double blPower = forward - strafe + turn;
//    double brPower = forward + strafe - turn;
//
//    frontLeftMotor.setPower(flPower);
//    frontRightMotor.setPower(frPower);
//    backLeftMotor.setPower(blPower);
//    backRightMotor.setPower(brPower);
//}
//    }
//}