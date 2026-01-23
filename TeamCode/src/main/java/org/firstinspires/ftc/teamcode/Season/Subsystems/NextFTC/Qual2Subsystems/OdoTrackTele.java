//package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;
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
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretTuff;
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
//@TeleOp
//public class OdoTrackTele extends NextFTCOpMode {
//
//    // Odometry-based turret subsystem
//    private TurretSubsystem turretSubsystem;
//
//    public OdoTrackTele() {
//        addComponents(
//                new SubsystemComponent(
//                        TurretPID.INSTANCE,
//                        Hood.INSTANCE,
//                        Turret.INSTANCE,
//                        CompliantIntake.INSTANCE,
//                        Transfer.INSTANCE,
//                        TurretTuff.INSTANCE
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
//    private boolean turretOdometryEnabled = false;
//
//
//    private boolean automatedDrive;
//    private Supplier<PathChain> farscore;
//    private Supplier<PathChain> closescore;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
//
//    // Target position for turret (adjust to your field coordinates)
//    private static final double TARGET_X = 122;
//    private static final double TARGET_Y = 122;
//    private Pose targetPose = new Pose(TARGET_X, TARGET_Y);
//    private Pose Corner = new Pose(8.7,8.7,180);
//    private boolean intakeToggle = false;
//    private long intakeStartTime = 0;
//
//    @Override
//    public void onStartButtonPressed() {
//        // Initialize odometry-based turret subsystem
//        new TurretTuff().INSTANCE.init(hardwareMap);
//        TurretTuff.setTarget(TARGET_X, TARGET_Y);
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
//
//        PedroComponent.follower().setPose(new Pose(72, 35, Math.toRadians(90)));
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
//        // ========== ODOMETRY TURRET CONTROLS ==========
//
//        // B Button: Toggle Odometry-based auto-alignment on/off
//        Gamepads.gamepad1().b()
//                .whenBecomesTrue(() -> {
//                    turretOdometryEnabled = !turretOdometryEnabled;
//
//                    if (!turretOdometryEnabled) {
//                        TurretTuff.INSTANCE.stop();
//                        telemetryM.addData("Odometry Turret", "Auto-Track Disabled");
//                    } else {
//                        TurretTuff.INSTANCE.resetPID();
//                        telemetryM.addData("Odometry Turret", "Auto-Track Enabled");
//                    }
//                });
//
//        // A Button: Stop turret and reset intake
//        Gamepads.gamepad1().a()
//                .whenBecomesTrue(() -> {
//                    turretOdometryEnabled = false;
//                    TurretTuff.INSTANCE.stop();
//                    CompliantIntake.INSTANCE.repel();
//                    telemetryM.addData("Turret", "Stopped & Intake Reset");
//                });
//
//        // X Button: Manually set new target position (example: current robot position)
//        Gamepads.gamepad1().x()
//                .whenBecomesTrue(() -> {
//                    Pose currentPose = PedroComponent.follower().getPose();
//                    TurretTuff.INSTANCE.setTarget(currentPose.getX(), currentPose.getY());
//                    telemetryM.addData("Target Updated", String.format("(%.1f, %.1f)",
//                            currentPose.getX(), currentPose.getY()));
//                });
//
//        // Y Button: Reset target to default (122, 122)
//        Gamepads.gamepad1().y()
//                .whenBecomesTrue(() -> {
//                    TurretTuff.INSTANCE.setTarget(TARGET_X, TARGET_Y);
//                    telemetryM.addData("Target Reset", String.format("(%.1f, %.1f)",
//                            TARGET_X, TARGET_Y));
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
//                .whenBecomesTrue(() -> {
//                    Pose pose = PedroComponent.follower().getPose();
//                    double d = Math.hypot(
//                            TARGET_X - pose.getX(),
//                            TARGET_Y - pose.getY()
//                    );
//                    TurretPID.INSTANCE.setShooterFromDistance(d).schedule();
//                    TurretPID.shootRequested = true;
//                    TurretPID.hasShot = false;
//                });
//
//        // D-pad Down: Reset position to corner
//        Gamepads.gamepad1().dpadDown()
//                .whenBecomesTrue(() -> {
//                    PedroComponent.follower().setPose(Corner);
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
//        // D-pad Right: Calibrate turret encoder
//        Gamepads.gamepad1().dpadRight()
//                .whenBecomesTrue(() -> {
//                    TurretTuff.INSTANCE.calibrateEncoder();
//                    telemetryM.addData("Turret", "Encoder Calibrated");
//                });
//    }
//
//    @Override
//    public void onUpdate() {
//        if (!automatedDrive) {
//            //Make the last parameter false for field-centric
//            //In case the drivers want to use a "slowMode" you can scale the vectors
//            //This is the normal version to use in the TeleOp
//            if (!slowMode) PedroComponent.follower().setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    true // Robot Centric
//            );
//                //This is how it looks with slowMode on
//            else PedroComponent.follower().setTeleOpDrive(
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
//        // Update odometry-based turret alignment when enabled
//        if (turretOdometryEnabled) {
//            TurretTuff.INSTANCE.update();
//        }
//
//
//    }
//}