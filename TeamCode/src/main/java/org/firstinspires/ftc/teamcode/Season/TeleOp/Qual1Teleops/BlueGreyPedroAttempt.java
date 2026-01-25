package org.firstinspires.ftc.teamcode.Season.TeleOp.Qual1Teleops;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

import java.util.function.Supplier;
@Disabled
@Configurable
@TeleOp
public class BlueGreyPedroAttempt extends OpMode {
    private Follower follower;
//    private final Pose startPose = new Pose(123.13, 122.08, Math.toRadians(220)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    VoltageGet volt = new VoltageGet();
    DcMotor activeintake = null;
    DcMotor out1 = null;
    DcMotor out2 = null;
    DcMotor ramp = null;
    DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
    private final double STARGET_DISTANCE = 42.97; // inches
    private final double SANGLE_TOLERANCE = -1.8;
    //    private final double MTARGET_DISTANCE = 2838; // PLACEHOLDER
//    private final double MANGLE_TOLERANCE = 134; // PLACEHOLDER
    private final double FTARGET_DISTANCE = 112.21;
    private final double FANGLE_TOLERANCE = 3.47;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean shooting = false;
    int shootStep = 0;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Supplier<PathChain> pathChain;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        out1 = hardwareMap.get(DcMotor.class, "outtake1");
        out2 = hardwareMap.get(DcMotor.class, "outtake2");
        activeintake = hardwareMap.get(DcMotor.class, "activeintake");
        ramp = hardwareMap.get(DcMotor.class, "ramp");
        BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor(hardwareMap);
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

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

    }
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