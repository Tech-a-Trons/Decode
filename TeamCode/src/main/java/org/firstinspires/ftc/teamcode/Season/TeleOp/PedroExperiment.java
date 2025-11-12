//package org.firstinspires.ftc.teamcode.Season.TeleOp;
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
//import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.VoltageGet;
//
//import java.util.function.Supplier;
//
//@Configurable
//@TeleOp
//public class PedroExperiment extends OpMode {
//    VoltageGet volt = new VoltageGet();
//    DcMotor activeintake = null;
//    DcMotor out1 = null;
//    DcMotor out2 = null;
//    DcMotor ramp = null;
//    DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
//    private final double STARGET_DISTANCE = 40.1; // inches
//    private final double SANGLE_TOLERANCE = 1.57;
//    private Follower follower;
//    public static Pose startingPose; //See ExampleAuto to understand how to use this
//    private boolean automatedDrive;
//    private Supplier<PathChain> pathChain;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
//
//    @Override
//    public void init() {
//        out1 = hardwareMap.get(DcMotor.class, "outtake1");
//        out2 = hardwareMap.get(DcMotor.class, "outtake2");
//        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
//        ramp = hardwareMap.get(DcMotor.class, "ramp");
//
//        RedExperimentalDistanceLExtractor ll = new RedExperimentalDistanceLExtractor(hardwareMap);
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
//        Double tx = ll.getTx();
//        if (tx == null) {
//            tx = 0.0;
//        }
//        Double distance = ll.getEuclideanDistance();
//        if (distance == null) {
//            distance = 0.0;
//        }
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
//        //Call this once per loop
//        follower.update();
//        telemetryM.update();
//
//        if (!automatedDrive) {
//            //Make the last parameter false for field-centric
//            //In case the drivers want to use a "slowMode" you can scale the vectors
//
//            //This is the normal version to use in the TeleOp
//            if (!slowMode) follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    true // Robot Centric
//            );
//
//                //This is how it looks with slowMode on
////            else follower.setTeleOpDrive(
////                    -gamepad1.left_stick_y * slowModeMultiplier,
////                    -gamepad1.left_stick_x * slowModeMultiplier,
////                    -gamepad1.right_stick_x * slowModeMultiplier,
////                    true // Robot Centric
////            );
//        }
//
//        //Automated PathFollowing
//        if (gamepad1.a) {
//            activeintake.setPower(volt.regulate(1.0));
//            ramp.setPower(0.3);
//        }
//
//        if (gamepad1.dpad_left) {
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
//
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
//        if(gamepad1.left_bumper){
//            out1.setPower(volt.regulate(-0.45));
//            out2.setPower(volt.regulate(0.45));
//            sleep(800);
//            ramp.setPower(volt.regulate(-1.0));
//            sleep(100);
//            out1.setPower(volt.regulate(0));
//            out2.setPower(volt.regulate(0));
//            activeintake.setPower(volt.regulate(1.0));
//            ramp.setPower(volt.regulate(0));
//            sleep(300);
//            activeintake.setPower(volt.regulate(0));
//            out1.setPower(volt.regulate(-0.45));
//            out2.setPower(volt.regulate(0.45));
//            sleep(800);
//            ramp.setPower(volt.regulate(-1.0));
//            sleep(50);
//            out1.setPower(volt.regulate(-0.1));
//            out2.setPower(volt.regulate(0.1));
//            ramp.setPower(volt.regulate(0));
//            sleep(100);
//            out1.setPower(volt.regulate(-0.45));
//            out2.setPower(volt.regulate(0.45));
//            sleep(500);
//            activeintake.setPower(volt.regulate(1.0));
//            ramp.setPower(volt.regulate(-1.0));
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
//            ramp.setPower(volt.regulate(gamepad1.right_trigger));
//        }
//
//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);
//    }
//}