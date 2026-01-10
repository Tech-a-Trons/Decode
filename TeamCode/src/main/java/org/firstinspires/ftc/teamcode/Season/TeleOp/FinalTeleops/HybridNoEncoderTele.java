package org.firstinspires.ftc.teamcode.Season.TeleOp.FinalTeleops;


import static dev.nextftc.bindings.Bindings.button;

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
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.HybridTurretNoEncoder;

import java.util.function.Supplier;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.extensions.pedro.PedroComponent;

@TeleOp(name = "Hybrid No Encoder TeleOp")
public class HybridNoEncoderTele extends NextFTCOpMode {

    // Limelight and hybrid turret alignment WITHOUT ENCODER
    private RedExperimentalDistanceLExtractor limelight;
    private HybridTurretNoEncoder turretAlignment;
    private ColorSensor colorSensor;

    public HybridNoEncoderTele() {
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
    private boolean turretAutoAlign = false;

    private boolean automatedDrive;
    private Supplier<PathChain> farscore;
    private Supplier<PathChain> closescore;
    private Supplier<PathChain> Parkpath;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Target position for turret (adjust to your field coordinates)
    private static final double TARGET_X = 121;
    private static final double TARGET_Y = 121;

    // Distance threshold for switching between close and far align
    private static final double DISTANCE_THRESHOLD = 90.0;

    private Pose targetPose = new Pose(TARGET_X, TARGET_Y);
    private Pose Middle = new Pose(72,35,180);
    private Pose Park = new Pose(38.74532374100719,33.358273381294964,90);
    private boolean intakeToggle = false;
    private long intakeStartTime = 0;

    @Override
    public void onStartButtonPressed() {
        // Initialize Limelight and Hybrid Turret Alignment (NO ENCODER)
        limelight = new RedExperimentalDistanceLExtractor(hardwareMap);
        turretAlignment = new HybridTurretNoEncoder(hardwareMap, limelight);
        colorSensor = new ColorSensor(hardwareMap);

        limelight.startReading();
        turretAlignment.setTelemetry(telemetry);
        turretAlignment.setTarget(TARGET_X, TARGET_Y);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        farscore = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(83.17241379310344, 12.620689655172416))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();
        closescore = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();
        Parkpath = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(38.74532374100719,33.358273381294964,0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(0), 0.8))
                .build();

        PedroComponent.follower().setPose(new Pose(120.587, 69.410, Math.toRadians(0)));
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

        // ========== HYBRID TURRET CONTROLS (NO ENCODER) ==========

        // B Button: Toggle hybrid auto-alignment on/off
        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    turretAutoAlign = !turretAutoAlign;

                    if (!turretAutoAlign) {
                        turretAlignment.stopTurret();
                        telemetryM.addData("Hybrid Turret", "Auto-Align Disabled");
                    } else {
                        turretAlignment.resetPID();
                        telemetryM.addData("Hybrid Turret", "Auto-Align Enabled (Seeking→Limelight)");
                    }
                });

        // A Button: Stop turret and reset intake
        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
                    turretAutoAlign = false;
                    turretAlignment.stopTurret();
                    CompliantIntake.INSTANCE.repel();
                    telemetryM.addData("Turret", "Stopped & Intake Reset");
                });

        // X Button: Update target to current position (for testing)
        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    Pose currentPose = PedroComponent.follower().getPose();
                    turretAlignment.setTarget(currentPose.getX(), currentPose.getY());
                    telemetryM.addData("Turret Target", String.format("Updated to (%.1f, %.1f)",
                            currentPose.getX(), currentPose.getY()));
                });

        // ========== EXISTING CONTROLS ==========

        // Right Bumper: Toggle intake with color sensor ball counting
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    intakeToggle = !intakeToggle;

                    if (intakeToggle) {
                        intakeStartTime = System.currentTimeMillis();
                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.slight();
                        colorSensor.artifactcounter = 0;
                        telemetryM.addData("Intake", "ON - Counting Balls");
                    } else {
                        intakeStartTime = 0;
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();
                        colorSensor.artifactcounter = 0;
                        telemetryM.addData("Intake", "OFF - Manual Stop");
                    }
                });

        // Right Trigger: Shoot
        button(() -> gamepad1.right_trigger > 0.05)
                .whenTrue(() -> {
                    Pose pose = PedroComponent.follower().getPose();
                    double d = Math.hypot(
                            TARGET_X - pose.getX(),
                            TARGET_Y - pose.getY()
                    );

                    TurretPID.INSTANCE.newshooterdistance(d).schedule();
                    TurretPID.shootRequested = true;
                    TurretPID.hasShot = false;
                });

        // D-pad Down: Reset position to middle
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().setPose(Middle);
                });

        // Left Trigger: Run intake and transfer
        Gamepads.gamepad1().leftTrigger()
                .greaterThan(0.05)
                .whenBecomesTrue(() -> {
                    Transfer.INSTANCE.on();
                    CompliantIntake.INSTANCE.on();
                });

        // Left Bumper: Reset shooter
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.resetShooter().schedule();
                    Hood.INSTANCE.midopen();
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();
                });

        // Y Button: Automated park path
        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().followPath(Parkpath.get());
                    automatedDrive = true;
                });
    }

    @Override
    public void onUpdate() {
        if (!automatedDrive) {
            PedroComponent.follower().setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        PedroComponent.follower().update();

        // Calculate distance to target for hood control and alignment mode selection
        Pose robotPose = PedroComponent.follower().getPose();
        double distanceToTarget = Math.hypot(TARGET_X - robotPose.getX(), TARGET_Y - robotPose.getY());

        // Distance threshold for close hood mode
        final double CLOSE_HOOD_DISTANCE = 20.0;

        // Hood control based on distance
        double targetVel = TurretPID.activeTargetVelocity;
        double actualVel = TurretPID.turret.getVelocity();

        if (distanceToTarget <= CLOSE_HOOD_DISTANCE) {
            Hood.INSTANCE.close();
        } else {
            if (targetVel > 500) {
                Hood.INSTANCE.compensateFromVelocity(targetVel, actualVel);
            }
        }

        // Update Hybrid turret alignment when enabled (NO ENCODER VERSION)
        if (turretAutoAlign) {
            // Automatically choose alignment mode based on distance
            if (distanceToTarget > DISTANCE_THRESHOLD) {
                turretAlignment.farAlign();
            } else {
                turretAlignment.closeAlign();
            }
        }

        // ========== COLOR SENSOR BALL COUNTING ==========
        if (intakeToggle) {
            colorSensor.IncountBalls();
            if (colorSensor.artifactcounter >= 2) {
                Transfer.INSTANCE.advance();
            }
            if (colorSensor.artifactcounter >= 3) {
                CompliantIntake.INSTANCE.off();
                Transfer.INSTANCE.off();
                intakeToggle = false;
                intakeStartTime = 0;
                gamepad1.rumble(500);
                colorSensor.artifactcounter = 0;
            }
        }

        limelight.update();
        telemetryM.update();

        // ========== TELEMETRY ==========
        telemetryM.debug("position", robotPose);
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        // Hybrid Turret telemetry (NO ENCODER)
        telemetryM.debug("turret_auto_align", turretAutoAlign);
        telemetryM.debug("turret_info", turretAlignment.getTelemetry());
        telemetryM.debug("LL_target_visible", limelight.isTargetVisible());
        telemetryM.debug("LL_tx", limelight.getTx() != null ? String.format("%.2f°", limelight.getTx()) : "N/A");
        telemetryM.debug("turret_aligned", turretAlignment.isAligned());

        // Distance and mode info
        telemetryM.debug("distance_to_target", String.format("%.2f", distanceToTarget));
        telemetryM.debug("alignment_mode", distanceToTarget > DISTANCE_THRESHOLD ? "FAR" : "CLOSE");
        telemetryM.debug("hood_mode", distanceToTarget <= CLOSE_HOOD_DISTANCE ? "CLOSE" : "FAR");

        // Color sensor telemetry
        telemetryM.debug("intake_active", intakeToggle);
        telemetryM.debug("ball_count", colorSensor.artifactcounter);
        telemetryM.debug("color_detected", colorSensor.getColor() != null ? colorSensor.getColor() : "None");
    }
}