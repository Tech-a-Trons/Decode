//package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;
//
//
//import static dev.nextftc.bindings.Bindings.button;
//
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ColorSensor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.CompliantIntake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Hood;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Transfer;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.SimpleLL;
//
//import java.util.function.Supplier;
//
//import dev.nextftc.bindings.BindingManager;
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
//@Disabled
//public class OldLLCode extends NextFTCOpMode {
//
//    // Limelight and alignment controller
//    private RedExperimentalDistanceLExtractor limelight;
//    private SimpleLL turretAlignment;
//
//    public OldLLCode() {
//        addComponents(
//                new SubsystemComponent(
//                        TurretPID.INSTANCE,
//                        Hood.INSTANCE,
//                        Turret.INSTANCE,
//                        CompliantIntake.INSTANCE,
//                        Transfer.INSTANCE
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
//    private boolean turretLimelightEnabled = false;
//    private ColorSensor colorSensor;
//
//    private boolean automatedDrive;
//    private Supplier<PathChain> farscore;
//    private Supplier<PathChain> closescore;
//    private Supplier<PathChain> Parkpath;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
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
//    private Pose Park = new Pose(38.74532374100719,33.358273381294964,0);
//    private boolean intakeToggle = false;
//    private long intakeStartTime = 0;
//
//    @Override
//    public void onStartButtonPressed() {
//        // Initialize Limelight and turret alignment
//        limelight = new RedExperimentalDistanceLExtractor(hardwareMap);
//        turretAlignment = new SimpleLL(hardwareMap, limelight);
//
//        limelight.startReading();
//        turretAlignment.setTelemetry(telemetry);
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
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
//                .build();
//
//        PedroComponent.follower().setPose(new Pose(120.587, 69.410, Math.toRadians(0)));
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
//                        turretAlignment.stopTurret();
//                        telemetryM.addData("Limelight Turret", "Auto-Align Disabled");
//                    } else {
//                        telemetryM.addData("Limelight Turret", "Auto-Align Enabled");
//                    }
//                });
//
//        // A Button: Stop turret and reset intake
//        Gamepads.gamepad1().a()
//                .whenBecomesTrue(() -> {
//                    turretLimelightEnabled = false;
//                    turretAlignment.stopTurret();
//                    CompliantIntake.INSTANCE.repel();
//                    telemetryM.addData("Turret", "Stopped & Intake Reset");
//                });
//
//        // ========== EXISTING CONTROLS ==========
//
//        // Right Bumper: Toggle intake
//        Gamepads.gamepad1().rightBumper()
//                .whenBecomesTrue(() -> {
//                    intakeToggle = !intakeToggle;
//
//                    if (intakeToggle) {
//                        intakeStartTime = System.currentTimeMillis();
//                        CompliantIntake.INSTANCE.on();
//                        Transfer.INSTANCE.slight();
//                    } else {
//                        intakeStartTime = 0;
//                        CompliantIntake.INSTANCE.off();
//                        Transfer.INSTANCE.off();
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
//        if (targetVel > 500) {
//            Hood.INSTANCE.compensateFromVelocity(targetVel, actualVel);
//        }
//
//        // Calculate distance to target for alignment mode selection
//        Pose robotPose = PedroComponent.follower().getPose();
//        double distanceToTarget = Math.hypot(TARGET_X - robotPose.getX(), TARGET_Y - robotPose.getY());
//
//        // Update Limelight turret alignment when enabled with automatic mode selection
//        if (turretLimelightEnabled) {
//            // Automatically choose alignment mode based on distance
//            if (distanceToTarget > DISTANCE_THRESHOLD) {
//                turretAlignment.farAlign();  // Use far alignment (kP = 0.004)
//            } else {
//                turretAlignment.closeAlign(); // Use close alignment (kP = 0.01)
//            }
//        }
//
//        limelight.update();
//
//        telemetryM.update();
//
//        // ========== TELEMETRY ==========
//        telemetryM.debug("position", robotPose);
//        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);
//
//        // Limelight Turret telemetry
//        telemetryM.debug("LL_turret_enabled", turretLimelightEnabled);
//        telemetryM.debug("LL_target_visible", limelight.isTargetVisible());
//        telemetryM.debug("LL_tx", limelight.getTx() != null ? String.format("%.2f°", limelight.getTx()) : "N/A");
//        telemetryM.debug("LL_ty", limelight.getTy() != null ? String.format("%.2f°", limelight.getTy()) : "N/A");
//        telemetryM.debug("LL_tag_id", limelight.getTagId() != null ? limelight.getTagId() : "None");
//        telemetryM.debug("LL_distance", limelight.getDistance() != null ? String.format("%.2f in", limelight.getDistance()) : "N/A");
//        telemetryM.debug("LL_aligned", turretAlignment.isAligned());
//
//        // Show distance to odometry target and which alignment mode is active
//        telemetryM.debug("distance_to_odom_target", String.format("%.2f", distanceToTarget));
//        telemetryM.debug("alignment_mode", distanceToTarget > DISTANCE_THRESHOLD ? "FAR (kP=0.004)" : "CLOSE (kP=0.01)");
//        telemetryM.debug("distance_threshold", DISTANCE_THRESHOLD);
//    }
//}