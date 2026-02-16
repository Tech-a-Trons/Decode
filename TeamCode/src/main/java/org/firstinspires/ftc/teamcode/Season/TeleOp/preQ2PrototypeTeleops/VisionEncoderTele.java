package org.firstinspires.ftc.teamcode.Season.TeleOp.preQ2PrototypeTeleops;


import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ColorSensor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.VisionEncoderTurret;

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
@Disabled
@TeleOp(name = "VisionEncoderTele")
public class VisionEncoderTele extends NextFTCOpMode {

    // Limelight and alignment controller with encoder feedback
    private RedExperimentalDistanceLExtractor limelight;
    private VisionEncoderTurret turretAlignment;
    private ColorSensor colorSensor;

    public VisionEncoderTele() {
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
    private boolean turretLimelightEnabled = false;

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

    // Distance threshold for switching between close and far align (tune this value)
    private static final double DISTANCE_THRESHOLD = 90.0; // Adjust based on testing

    private Pose targetPose = new Pose(TARGET_X, TARGET_Y);
    private Pose Middle = new Pose(72,35,180);
    private Pose Park = new Pose(38.74532374100719,33.358273381294964,90);
    private boolean intakeToggle = false;
    private long intakeStartTime = 0;

    @Override
    public void onStartButtonPressed() {
        // Initialize Limelight and Vision+Encoder turret alignment
        limelight = new RedExperimentalDistanceLExtractor(hardwareMap);
        turretAlignment = new VisionEncoderTurret(hardwareMap, limelight);
        colorSensor = new ColorSensor();

        limelight.startReading();
        turretAlignment.setTelemetry(telemetry);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        farscore = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(83.17241379310344, 12.620689655172416))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();
        closescore = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();
        Parkpath = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
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

        // ========== VISION+ENCODER TURRET CONTROLS ==========

        // B Button: Toggle Vision+Encoder auto-alignment on/off
        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    turretLimelightEnabled = !turretLimelightEnabled;

                    if (!turretLimelightEnabled) {
                        turretAlignment.stopTurret();
                        telemetryM.addData("Vision+Encoder Turret", "Auto-Align Disabled");
                    } else {
                        turretAlignment.resetPID();
                        telemetryM.addData("Vision+Encoder Turret", "Auto-Align Enabled");
                    }
                });

        // A Button: Stop turret and reset intake
        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
                    turretLimelightEnabled = false;
                    turretAlignment.stopTurret();
                    CompliantIntake.INSTANCE.repel();
                    telemetryM.addData("Turret", "Stopped & Intake Reset");
                });

        // X Button: Calibrate encoder (set current position as 0°)
        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    turretAlignment.calibrateEncoder();
                    telemetryM.addData("Turret", "Encoder Calibrated");
                });

        // ========== EXISTING CONTROLS ==========

        // Right Bumper: Toggle intake with color sensor ball counting
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    intakeToggle = !intakeToggle;

                    if (intakeToggle) {
                        // Starting intake
                        intakeStartTime = System.currentTimeMillis();
                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.slight();

                        // Reset ball counter when starting intake
                        colorSensor.artifactcounter = 0;

                        telemetryM.addData("Intake", "ON - Counting Balls");
                    } else {
                        // Manually stopping intake
                        intakeStartTime = 0;
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();

                        // Reset ball counter
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
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
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

        // Distance threshold for close hood mode (in inches, tune this value)
        final double CLOSE_HOOD_DISTANCE = 20.0;

        // Hood control based on distance
        double targetVel = TurretPID.activeTargetVelocity;
        double actualVel = TurretPID.turret.getVelocity();

        if (distanceToTarget <= CLOSE_HOOD_DISTANCE) {
            // Close to goal: Use close hood mode, NO velocity correction
            Hood.INSTANCE.close();
        } else {
            // Far from goal: Use velocity correction if shooter is spinning
            if (targetVel > 500) {
                Hood.INSTANCE.compensateFromVelocity(targetVel, actualVel);
            }
        }

        // Update Vision+Encoder turret alignment when enabled with automatic mode selection
        if (turretLimelightEnabled) {
            // Automatically choose alignment mode based on distance
            if (distanceToTarget > DISTANCE_THRESHOLD) {
                turretAlignment.farAlign();  // Standard PID (kP = 0.08)
            } else {
                turretAlignment.closeAlign(); // More aggressive (kP = 0.12)
            }
        }

        // ========== COLOR SENSOR BALL COUNTING ==========
        // Only count balls when intake is active
        if (intakeToggle) {
            colorSensor.IncountBalls();
            if (colorSensor.artifactcounter >= 2) {
                Transfer.INSTANCE.advance();
            }
            // Auto-stop when 3 balls are collected
            if (colorSensor.artifactcounter >= 3) {
                // Turn off intake and transfer
                CompliantIntake.INSTANCE.off();
                Transfer.INSTANCE.off();

                // Reset toggle state
                intakeToggle = false;
                intakeStartTime = 0;

                // Optional: Rumble controller to alert driver
                gamepad1.rumble(500);

                // Reset ball counter
                colorSensor.artifactcounter = 0;
            }
        }

        limelight.update();

        telemetryM.update();

        // ========== TELEMETRY ==========
        telemetryM.debug("position", robotPose);
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        // Vision+Encoder Turret telemetry
        telemetryM.debug("turret_enabled", turretLimelightEnabled);
        telemetryM.debug("LL_target_visible", limelight.isTargetVisible());
        telemetryM.debug("LL_tx", limelight.getTx() != null ? String.format("%.2f°", limelight.getTx()) : "N/A");
        telemetryM.debug("turret_current_angle", String.format("%.2f°", turretAlignment.getCurrentAngle()));
        telemetryM.debug("turret_aligned", turretAlignment.isAligned());
        telemetryM.debug("encoder_voltage", String.format("%.3fV", hardwareMap.get(com.qualcomm.robotcore.hardware.AnalogInput.class, "turret_encoder").getVoltage()));

        // Show distance to target and which modes are active
        telemetryM.debug("distance_to_target", String.format("%.2f", distanceToTarget));
        telemetryM.debug("alignment_mode", distanceToTarget > DISTANCE_THRESHOLD ? "FAR (kP=0.08)" : "CLOSE (kP=0.12)");
        telemetryM.debug("hood_mode", distanceToTarget <= CLOSE_HOOD_DISTANCE ? "CLOSE (No Correction)" : "FAR (Velocity Correction)");

        // Color sensor telemetry
        telemetryM.debug("intake_active", intakeToggle);
        telemetryM.debug("ball_count", colorSensor.artifactcounter);
        telemetryM.debug("color_detected", colorSensor.getColor() != null ? colorSensor.getColor() : "None");
    }
}