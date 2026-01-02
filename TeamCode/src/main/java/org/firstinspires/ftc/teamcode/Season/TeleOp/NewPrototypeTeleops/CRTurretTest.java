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
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.CRTurretSubsystem;

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
public class CRTurretTest extends NextFTCOpMode {

    // CR Turret instance
    private CRTurretSubsystem crTurret;

    public CRTurretTest() {
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

    // Target position for turret (adjust to your field coordinates)
    private static final double TARGET_X = 122;  // Example: center of field
    private static final double TARGET_Y = 122;  // Example: center of field
    private Pose targetPose = new Pose(TARGET_X, TARGET_Y);


    @Override
    public void onStartButtonPressed() {
        // Initialize CR Turret
        crTurret = new CRTurretSubsystem(hardwareMap);

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

        // ========== CR TURRET CONTROLS ==========

        // Left Bumper: Toggle auto-tracking on/off
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    if (crTurret.enabled) {
                        crTurret.setEnabled(false);
                        crTurret.stop(); // Stop the CR servo
                        telemetryM.addData("CRTurret", "Disabled");
                    } else {
                        crTurret.setEnabled(true);
                        telemetryM.addData("CRTurret", "Auto-Tracking Enabled");
                    }
                });

        // D-pad Left: Rotate turret counterclockwise (manual adjustment)
        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> {
                    crTurret.addAngle(Math.toRadians(45)); // Larger increment for testing
                    telemetryM.addData("CRTurret", "Manual Rotate Left +45°");
                });

        // D-pad Right: Rotate turret clockwise (manual adjustment)
        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    crTurret.addAngle(Math.toRadians(-45)); // Larger decrement for testing
                    telemetryM.addData("CRTurret", "Manual Rotate Right -45°");
                });

        // Right Bumper: Reset turret to center position
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    crTurret.reset();
                    CompliantIntake.INSTANCE.repel();
                    telemetryM.addData("CRTurret", "Reset to Zero Position");
                });

        // D-pad Up: Calibrate turret to current position as zero
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> {
                    crTurret.calibrateToAngle(0);
                    telemetryM.addData("CRTurret", "Calibrated to Zero");
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.advance();
                });

        // D-pad Down: Emergency stop turret
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    crTurret.stop();
                    telemetryM.addData("CRTurret", "Emergency Stop");
                });

        // Left Trigger: Manual control - rotate CCW while held (handled in onUpdate)
        // Right Trigger: Manual control - rotate CW while held (handled in onUpdate)

        // ========== EXISTING CONTROLS ==========

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setCloseShooterSpeed().schedule());

        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setFarShooterSpeed().schedule());

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

        // Handle trigger-based manual control
        if (gamepad1.left_trigger > 0.1) {
            crTurret.setManualMode(true, -gamepad1.left_trigger * 0.8); // CCW
        } else if (gamepad1.right_trigger > 0.1) {
            crTurret.setManualMode(true, gamepad1.right_trigger * 0.8); // CW
        } else if (crTurret.manualMode) {
            crTurret.setManualMode(false, 0); // Exit manual mode when triggers released
        }

        // Update CR turret with current robot pose
        Pose robotPose = PedroComponent.follower().getPose();
        if (crTurret.enabled) {
            crTurret.continuousTracking(targetPose, robotPose);
        }
        crTurret.periodic();

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

        // CR Turret telemetry
        telemetryM.debug("turret_angle_deg", Math.toDegrees(crTurret.getAngle()));
        telemetryM.debug("turret_target_deg", Math.toDegrees(crTurret.getTargetAngle()));
        telemetryM.debug("turret_error_deg", Math.toDegrees(crTurret.getError()));
        telemetryM.debug("turret_power", crTurret.getPower());
        telemetryM.debug("turret_enabled", crTurret.enabled);
        telemetryM.debug("turret_ready", crTurret.isReady());
        telemetryM.debug("turret_manual_mode", crTurret.manualMode);

        // Show angle to target
        double angleToTarget = Math.atan2(TARGET_Y - robotPose.getY(), TARGET_X - robotPose.getX());
        double relativeAngle = CRTurretSubsystem.normalizeAngle(angleToTarget - robotPose.getHeading());
        telemetryM.debug("angleToTarget_deg", Math.toDegrees(relativeAngle));

        // Show if turret is within acceptable range
        telemetryM.debug("turret_in_range",
                Math.abs(crTurret.getAngle()) <= Math.abs(CRTurretSubsystem.angleMax));
    }
}