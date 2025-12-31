package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretSubsystem;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.extensions.pedro.PedroComponent;

@TeleOp
public class newPIDTest extends NextFTCOpMode {

    // OdometryTracker instance
    private TurretSubsystem odometryTracker;

    public newPIDTest() {
        addComponents(
                new SubsystemComponent(
                        TurretPID.INSTANCE,
                        Hood.INSTANCE,
                        Turret.INSTANCE,
                        CompliantIntake.INSTANCE,
                        Transfer.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");
    private boolean turretEnabled = false;


    private boolean automatedDrive;
    private Supplier<PathChain> farscore;
    private Supplier<PathChain> closescore;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Target position for odometry tracker (adjust to your field coordinates)
    private static final double TARGET_X = 121;  // Example: center of field
    private static final double TARGET_Y = 121;  // Example: center of field
    private Pose targetPose = new Pose(TARGET_X, TARGET_Y);


    @Override
    public void onStartButtonPressed() {
        // Initialize OdometryTracker
        odometryTracker = new TurretSubsystem(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        farscore = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(83.17241379310344, 12.620689655172416))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();
        closescore = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();

        PedroComponent.follower().setPose(new Pose(72, 35, Math.toRadians(90)));
        PedroComponent.follower().startTeleopDrive();

        // Setup driving
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        // ========== ODOMETRY TRACKER CONTROLS ==========

        // Left Bumper: Toggle auto-tracking on/off
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    if (odometryTracker.enabled) {
                        odometryTracker.setEnabled(false);
                        telemetryM.addData("OdometryTracker", "Disabled");
                    } else {
                        odometryTracker.setEnabled(true);
                        telemetryM.addData("OdometryTracker", "Auto-Tracking Enabled");
                    }
                });

        // D-pad Left: Rotate tracker counterclockwise (when not auto-tracking)
        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> {
                    if (!odometryTracker.enabled) {
                        odometryTracker.addAngle(Math.toRadians(30));
                        telemetryM.addData("OdometryTracker", "Manual Rotate Left");
                    }
                });

        // D-pad Right: Rotate tracker clockwise (when not auto-tracking)
        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    if (!odometryTracker.enabled) {
                        odometryTracker.addAngle(Math.toRadians(-30));
                        telemetryM.addData("OdometryTracker", "Manual Rotate Right");
                    }
                });

        // Right Bumper: Reset tracker to center and run intake
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    odometryTracker.reset();
                    CompliantIntake.INSTANCE.repel();
                    telemetryM.addData("OdometryTracker", "Reset to Center");
                });

        // ========== EXISTING CONTROLS ==========

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> {
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.advance();
                });

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setCloseShooterSpeed().schedule());

        Gamepads.gamepad1().a()
//                .whenBecomesTrue(() -> TurretPID.INSTANCE.setFarShooterSpeed().schedule());
                .whenBecomesTrue(() -> {
                    Pose pose = PedroComponent.follower().getPose();
                    double d = Math.hypot(
                            TARGET_X - pose.getX(),
                            TARGET_Y - pose.getY()
                    );
                    TurretPID.INSTANCE.setShooterFromDistance(d).schedule();
                    TurretPID.shootRequested = true;
                    TurretPID.hasShot = false;
                });

        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.resetShooter().schedule();
                    Hood.INSTANCE.midopen();
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();
                });

        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    Transfer.INSTANCE.on();
                    CompliantIntake.INSTANCE.on();
                });
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
        double targetVel = TurretPID.activeTargetVelocity;
        double actualVel = TurretPID.turret.getVelocity();
        if (targetVel > 500) {
            Hood.INSTANCE.compensateFromVelocity(targetVel, actualVel);
        }

        // Update odometry tracker with current robot pose
        Pose robotPose = PedroComponent.follower().getPose();
        if (odometryTracker.enabled) {
            odometryTracker.trackTarget(targetPose, robotPose);
        }
        odometryTracker.periodic();

        telemetryM.update();

        if (!automatedDrive) {
            if (!slowMode) {
                PedroComponent.follower().setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot Centric
                );
            } else {
                PedroComponent.follower().setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        true // Robot Centric
                );
            }
        }

        // Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !PedroComponent.follower().isBusy())) {
            PedroComponent.follower().startTeleopDrive();
            automatedDrive = false;
        }

        // ========== TELEMETRY ==========
        telemetryM.debug("position", robotPose);
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        // Odometry tracker telemetry
        telemetryM.debug("tracker_angle", Math.toDegrees(odometryTracker.getAngle()));
        telemetryM.debug("tracker_target", Math.toDegrees(odometryTracker.getTargetAngle()));
        telemetryM.debug("tracker_error", Math.toDegrees(odometryTracker.getError()));
        telemetryM.debug("tracker_enabled", odometryTracker.enabled);
        telemetryM.debug("tracker_ready", odometryTracker.isReady());

        // Show angle to target
        double angleToTarget = Math.atan2(TARGET_Y - robotPose.getY(), TARGET_X - robotPose.getX());
        double relativeAngle = TurretSubsystem.normalizeAngle(angleToTarget - robotPose.getHeading());
        telemetryM.debug("angleToTarget", Math.toDegrees(relativeAngle));
    }
}