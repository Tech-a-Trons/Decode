//package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;
//
//
//import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext.lastPose;
//import static dev.nextftc.bindings.Bindings.button;
//
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ColorSensor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.RGBled;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdo;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;
//
//import java.util.function.Supplier;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.ftc.Gamepads;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//import dev.nextftc.hardware.driving.MecanumDriverControlled;
//import dev.nextftc.hardware.impl.MotorEx;
//import dev.nextftc.extensions.pedro.PedroComponent;
//
//@TeleOp (name = "OdoTest1")
//public class RegionalSolo1st extends NextFTCOpMode {
//
//    // Limelight and alignment controller
//    private ElapsedTime elapsedtime;
//    private RedExperimentalDistanceLExtractor limelight;
//    private ColorSensor colorSensor;
//    private RGBled rgBled;
//
//    public RegionalSolo1st() {
//        addComponents(
//                new SubsystemComponent(
//                        TurretPID.INSTANCE,
//                        Hood.INSTANCE,
//                        CompliantIntake.INSTANCE,
//                        Transfer.INSTANCE, rgBled.INSTANCE,
//                        TurretOdoAi.INSTANCE
//                ),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE,
//                new PedroComponent(Constants::createFollower)
//        );
//    }
//
//    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
//    private final MotorEx frontRightMotor = new MotorEx("fr");
//    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
//    private final MotorEx backRightMotor = new MotorEx("br");
//    private boolean turretLimelightEnabled = true;
//
//    private boolean automatedDrive;
//    private Supplier<PathChain> farscore;
//    private Supplier<PathChain> closescore;
//    private Supplier<PathChain> Parkpath;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 1;
//
//    // Target position for turret (adjust to your field coordinates)
//    private static final double TARGET_X = 121;  // Example: center of field
//    private static final double TARGET_Y = 121;  // Example: center of field
//
//    // Distance threshold for switching between close and far align (tune this value)
//    private static final double DISTANCE_THRESHOLD = 90.0; // Adjust based on testing
//
//    private Pose targetPose = new Pose(TARGET_X, TARGET_Y);
//    private Pose Middle = new Pose(72,35,180);
//    private Pose Park = new Pose(38.74532374100719,33.358273381294964,90);
//    private boolean intakeToggle = false;
//    private long intakeStartTime = 0;
//    private VoltageGet voltageGet;
//    @Override
//    public void onStartButtonPressed() {
//        // Initialize Limelight and turret alignment
//        limelight = new RedExperimentalDistanceLExtractor(hardwareMap);
//        colorSensor = new ColorSensor();
//        rgBled = new RGBled();
//
//        limelight.startReading();
////        turretAlignment.setTelemetry(telemetry);
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        farscore = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(83.17241379310344, 12.620689655172416))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
//                .build();
//        closescore = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(72, 72))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
//                .build();
//        Parkpath = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(38.74532374100719,33.358273381294964,0))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(0), 0.8))
//                .build();
//        if (lastPose != null){
//            PedroComponent.follower().setPose(lastPose);
//        }
//        else {
//            PedroComponent.follower().setPose(new Pose(0,0,0));
//        }
//
//        PedroComponent.follower().startTeleopDrive();
//
//        // Setup driving
//        Command driverControlled = new MecanumDriverControlled(
//                frontLeftMotor,
//                frontRightMotor,
//                backLeftMotor,
//                backRightMotor,
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX()
//        );
//
//        // ========== LIMELIGHT TURRET CONTROLS ==========
//
//        // B Button: Toggle Limelight auto-alignment on/off
//        Gamepads.gamepad1().b()
//                .whenBecomesTrue(() -> {
//                    turretLimelightEnabled = !turretLimelightEnabled;
//
//                    if (!turretLimelightEnabled) {
////                        turretAlignment.stopTurret();
//                        telemetryM.addData("Limelight Turret", "Auto-Align Disabled");
//                    } else {
//                        telemetryM.addData("Limelight Turret", "Auto-Align Enabled");
//                    }
//                });
//        Gamepads.gamepad1().dpadLeft()
//                .whenTrue(() -> {
//                    turretLimelightEnabled = false; // manual override
////                    turretAlignment.turretLeft();
//                })
//                .whenBecomesFalse(() -> {
//                    turretLimelightEnabled = true;  // re-enable auto align
////                    turretAlignment.stopAndEnableAlign();
//                });
//
//// DPAD RIGHT — hold to rotate right
//        Gamepads.gamepad1().dpadRight()
//                .whenTrue(() -> {
//                    turretLimelightEnabled = false; // manual override
//                })
//                .whenBecomesFalse(() -> {
//                    turretLimelightEnabled = true;  // re-enable auto align
//                });
//        // A Button: Stop turret and reset intake
//        Gamepads.gamepad1().a()
//                .whenBecomesTrue(() -> {
//                    turretLimelightEnabled = false;
//                    CompliantIntake.INSTANCE.repel();
//                    telemetryM.addData("Turret", "Stopped & Intake Reset");
//                });
//
//        // ========== EXISTING CONTROLS ==========
//
//        // Right Bumper: Toggle intake with color sensor ball counting
//        Gamepads.gamepad1().rightBumper()
//                .whenBecomesTrue(() -> {
//                    intakeToggle = !intakeToggle;
//
//                    if (intakeToggle) {
//                        // Starting intake
//                        intakeStartTime = System.currentTimeMillis();
//                        CompliantIntake.INSTANCE.on();
//                        Transfer.INSTANCE.slight();
//
//                        // Reset ball counter when starting intake
//                        colorSensor.artifactcounter = 0;
//
//                        telemetryM.addData("Intake", "ON - Counting Balls");
//                    } else {
//                        // Manually stopping intake
//                        intakeStartTime = 0;
//                        CompliantIntake.INSTANCE.off();
//                        Transfer.INSTANCE.off();
//
//                        // Reset ball counter
//                        colorSensor.artifactcounter = 0;
//
//                        telemetryM.addData("Intake", "OFF - Manual Stop");
//                    }
//                });
//
//        // Right Trigger: Shoot
//        button(() -> gamepad1.right_trigger > 0.05)
//                .whenTrue(() -> {
//                    Pose pose = PedroComponent.follower().getPose();
//                    double d = Math.hypot(
//                            TARGET_X - pose.getX(),
//                            TARGET_Y - pose.getY()
//                    );
//
//                    TurretPID.INSTANCE.newshooterdistance(d).schedule();
//                    TurretPID.shootRequested = true;
//                    TurretPID.hasShot = false;
//                });
//
//        // D-pad Down: Reset position to corner
//        Gamepads.gamepad1().dpadDown()
//                .whenBecomesTrue(() -> {
//                    PedroComponent.follower().setPose(Middle);
//                });
//
//        // Left Trigger: Run intake and transfer
//        Gamepads.gamepad1().leftTrigger()
//                .greaterThan(0.05)
//                .whenBecomesTrue(() -> {
//                    Transfer.INSTANCE.on();
//                    CompliantIntake.INSTANCE.on();
//                });
//
//        // Left Bumper: Reset shooter
//        Gamepads.gamepad1().leftBumper()
//                .whenBecomesTrue(() -> {
//                    TurretPID.INSTANCE.resetShooter().schedule();
//                    Hood.INSTANCE.midopen();
//                    CompliantIntake.INSTANCE.off();
//                    Transfer.INSTANCE.off();
//                });
//
//        // Y Button: Automated park path
//        Gamepads.gamepad1().y()
//                .whenBecomesTrue(() -> {
//                    PedroComponent.follower().followPath(Parkpath.get());
//                    automatedDrive = true;
//                });
//
//    }
//
//    @Override
//    public void onUpdate() {
//        if (!automatedDrive) {
//            //Make the last parameter false for field-centric
//            //In case the drivers want to use a "slowMode" you can scale the vectors
//            //This is the normal version to use in the TeleOp
//            PedroComponent.follower().setTeleOpDrive(
//                    -gamepad1.left_stick_y * slowModeMultiplier,
//                    -gamepad1.left_stick_x * slowModeMultiplier,
//                    -gamepad1.right_stick_x * slowModeMultiplier,
//                    true // Robot Centric
//            );
//        }
//
//        PedroComponent.follower().update();
//        double targetVel = TurretPID.activeTargetVelocity;
//        double actualVel = TurretPID.turret.getVelocity();
//        Pose robotPose = PedroComponent.follower().getPose();
//        double distanceToTarget = Math.hypot(TARGET_X - robotPose.getX(), TARGET_Y - robotPose.getY());
//
//        // Distance threshold for close hood mode (in inches, tune this value)
//        final double CLOSE_HOOD_DISTANCE = 20.0;
//
//
//        if (distanceToTarget <= CLOSE_HOOD_DISTANCE) {
//            // Close to goal: Use close hood mode, NO velocity correction
//            Hood.INSTANCE.close();
//        } else {
//            // Far from goal: Use velocity correction if shooter is spinning
//            if (targetVel > 500) {
//                Hood.INSTANCE.compensateFromVelocity(targetVel, actualVel);
//            }
//        }
//
//        // Calculate distance to target for alignment mode selection
//
//        // Update Limelight turret alignment when enabled with automatic mode selection
//
//
//        // ========== COLOR SENSOR BALL COUNTING ==========
//        // Only count balls when intake is active
//        if (intakeToggle) {
//            colorSensor.IncountBalls();
//            if (colorSensor.artifactcounter>0){
////                RGBled.INSTANCE.open();
//            }
//            if (colorSensor.artifactcounter >= 2) {
//                Transfer.INSTANCE.advance();
////                RGBled.INSTANCE.midopen();
//                // Reset toggle state
//            }
//            // Auto-stop when 3 balls are collected
//            if (colorSensor.artifactcounter >= 3) {
////                RGBled.INSTANCE.close();
//                // Turn off intake and transfer
//                CompliantIntake.INSTANCE.off();
//                Transfer.INSTANCE.off();
//
//                // Reset toggle state
//                intakeToggle = false;
//                intakeStartTime = 0;
//
//                // Optional: Rumble controller to alert driver
//                gamepad1.rumble(500);
//
//                // Reset ball counter
//                colorSensor.artifactcounter = 0;
//            }
//        }
//
//        limelight.update();
//
//
//        // ========== TELEMETRY ==========
//
//    }
//}