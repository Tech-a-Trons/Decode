package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ColorSensor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.NewHood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "Regionals Teleop CRASH-HARDENED")
public class TeleOpProgram extends NextFTCOpMode {

    private boolean intakeToggle = false;
    public double SlowModeMultiplier = 1.0;

    // Turret hybrid control
    private ElapsedTime turretRightHoldTimer = new ElapsedTime();
    private ElapsedTime turretLeftHoldTimer = new ElapsedTime();
    private static final double HOLD_THRESHOLD = 0.3;
    private static final double CONTINUOUS_SPEED = 0.8;

    // ========== CRASH PREVENTION: Telemetry throttling ==========
    private ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_UPDATE_INTERVAL = 0.1;

    // ========== CRASH PREVENTION: Pose caching with null safety ==========
    private Pose cachedPose = null;
    private double lastPoseUpdateTime = 0;
    private static final double POSE_CACHE_TIME = 0.02;

    // ========== CRASH PREVENTION: Initialization tracking ==========
    private boolean motorsInitialized = false;
    private boolean followerInitialized = false;

    // ========== CRASH PREVENTION: Watchdog timer ==========
    private ElapsedTime watchdogTimer = new ElapsedTime();
    private static final double WATCHDOG_TIMEOUT = 0.5; // 500ms max per loop iteration

    // ========== CRASH PREVENTION: Error recovery state ==========
    private int consecutiveErrors = 0;
    private static final int MAX_CONSECUTIVE_ERRORS = 5;
    private boolean emergencyStopEngaged = false;

    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(
                        CompliantIntake.INSTANCE,
                        Transfer.INSTANCE,
                        TurretPID.INSTANCE,
                        TurretOdoAi.INSTANCE,
                        NewHood.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private static final double TARGET_X = 121;
    private static final double TARGET_Y = 121;
    private Pose Middle = new Pose(72, 72, Math.toRadians(270));

    // ========== CRASH PREVENTION: Nullable motor references ==========
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    @Override
    public void onStartButtonPressed() {
        try {
            // ========== CRASH PREVENTION: Safe motor initialization ==========
            initializeMotors();

            // ========== CRASH PREVENTION: Safe turret initialization ==========
            initializeTurret();

            // ========== CRASH PREVENTION: Safe follower initialization ==========
            initializeFollower();

            // Start timers
            telemetryTimer.reset();
            watchdogTimer.reset();

            // ========== CRASH PREVENTION: Only bind controls if initialization succeeded ==========
            if (followerInitialized) {
                bindControls();
            } else {
                telemetry.addData("ERROR", "Follower failed to initialize - limited functionality");
                telemetry.update();
            }

        } catch (Exception e) {
            // ========== CRASH PREVENTION: Top-level exception handler ==========
            telemetry.addData("CRITICAL ERROR", "Initialization failed: " + e.getMessage());
            telemetry.update();
            emergencyStopEngaged = true;
        }
    }

    /**
     * ========== CRASH PREVENTION: Safe motor initialization ==========
     */
    private void initializeMotors() {
        try {
            frontLeftMotor = new MotorEx("fl").reversed();
            frontRightMotor = new MotorEx("fr");
            backLeftMotor = new MotorEx("bl").reversed();
            backRightMotor = new MotorEx("br");

            // Set safe defaults
            setMotorsBrakeMode(false);

            motorsInitialized = true;
        } catch (Exception e) {
            motorsInitialized = false;
            telemetry.addData("WARNING", "Motor init failed: " + e.getMessage());
        }
    }

    /**
     * ========== CRASH PREVENTION: Safe brake mode setting ==========
     */
    private void setMotorsBrakeMode(boolean brake) {
        if (!motorsInitialized) return;

        try {
            DcMotor.ZeroPowerBehavior mode = brake ?
                    DcMotor.ZeroPowerBehavior.BRAKE :
                    DcMotor.ZeroPowerBehavior.FLOAT;

            if (frontLeftMotor != null) frontLeftMotor.setZeroPowerBehavior(mode);
            if (frontRightMotor != null) frontRightMotor.setZeroPowerBehavior(mode);
            if (backLeftMotor != null) backLeftMotor.setZeroPowerBehavior(mode);
            if (backRightMotor != null) backRightMotor.setZeroPowerBehavior(mode);
        } catch (Exception e) {
            // Silent failure - don't crash on brake mode
        }
    }

    /**
     * ========== CRASH PREVENTION: Safe turret initialization ==========
     */
    private void initializeTurret() {
        try {
            TurretOdoAi.INSTANCE.init(hardwareMap);
            NewHood.INSTANCE.init(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("WARNING", "Turret init failed: " + e.getMessage());
        }
    }

    /**
     * ========== CRASH PREVENTION: Safe follower initialization ==========
     */
    private void initializeFollower() {
        try {
            if (PedroComponent.follower() != null) {
                PedroComponent.follower().setPose(new Pose(72, 72, Math.toRadians(270)));
                PedroComponent.follower().startTeleopDrive();
                followerInitialized = true;
            } else {
                followerInitialized = false;
                telemetry.addData("WARNING", "Follower is null");
            }
        } catch (Exception e) {
            followerInitialized = false;
            telemetry.addData("WARNING", "Follower init failed: " + e.getMessage());
        }
    }

    /**
     * ========== CRASH PREVENTION: Bind controls with error handling ==========
     */
    private void bindControls() {
        try {
            // === TURRET MODE CONTROL (Gamepad 1) ===
            Gamepads.gamepad1().dpadRight()
                    .whenBecomesTrue(() -> safeExecute(() -> {
                        TurretOdoAi.INSTANCE.setManualMode();
                        if (gamepad1 != null) gamepad1.rumble(500);
                    }));

            Gamepads.gamepad1().dpadLeft()
                    .whenBecomesTrue(() -> safeExecute(() -> {
                        TurretOdoAi.INSTANCE.setAutoMode();
                        if (gamepad1 != null) gamepad1.rumble(200);
                    }));

            // === MANUAL TURRET CONTROL - SINGLE TAP (Gamepad 2) ===
            Gamepads.gamepad2().dpadRight()
                    .whenBecomesTrue(() -> safeExecute(() -> {
                        if (TurretOdoAi.INSTANCE.isManualMode()) {
                            TurretOdoAi.INSTANCE.turnRight();
                            turretRightHoldTimer.reset();
                        }
                    }));

            Gamepads.gamepad2().dpadLeft()
                    .whenBecomesTrue(() -> safeExecute(() -> {
                        if (TurretOdoAi.INSTANCE.isManualMode()) {
                            TurretOdoAi.INSTANCE.turnLeft();
                            turretLeftHoldTimer.reset();
                        }
                    }));

            // === POSE RESET ===
            Gamepads.gamepad1().dpadDown()
                    .whenBecomesTrue(() -> safeExecute(() -> {
                        if (followerInitialized && PedroComponent.follower() != null) {
                            PedroComponent.follower().setPose(Middle);
                            cachedPose = null; // Invalidate cache
                        }
                    }));

            // === SHOOT ===
            button(() -> gamepad1 != null && gamepad1.right_trigger > 0.05)
                    .whenTrue(() -> safeExecute(() -> {
                        Pose pose = getCachedPose();
                        if (pose != null) {
                            double d = Math.hypot(
                                    TARGET_X - pose.getX(),
                                    TARGET_Y - pose.getY()
                            );
                            TurretPID.INSTANCE.newshooterdistance(d).schedule();
                            TurretPID.shootRequested = true;
                            TurretPID.hasShot = false;
                        }
                    }));

            // === INTAKE TOGGLE ===
            Gamepads.gamepad1().rightBumper()
                    .whenBecomesTrue(() -> safeExecute(() -> {
                        intakeToggle = !intakeToggle;

                        if (intakeToggle) {
                            CompliantIntake.INSTANCE.on();
                            Transfer.INSTANCE.nice();
                        } else {
                            CompliantIntake.INSTANCE.off();
                            Transfer.INSTANCE.off();
                        }
                    }));

            // === MANUAL INTAKE ===
            Gamepads.gamepad1().leftTrigger()
                    .greaterThan(0.05)
                    .whenBecomesTrue(() -> safeExecute(() -> {
                        Transfer.INSTANCE.on();
                        CompliantIntake.INSTANCE.on();
                    }));

            // === EMERGENCY STOP ===
            Gamepads.gamepad1().leftBumper()
                    .whenBecomesTrue(() -> safeExecute(() -> {
                        TurretPID.INSTANCE.resetShooter().schedule();
                        Hood.INSTANCE.midopen();
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();
                        intakeToggle = false;
                    }));

        } catch (Exception e) {
            telemetry.addData("ERROR", "Control binding failed: " + e.getMessage());
        }
    }

    /**
     * ========== CRASH PREVENTION: Safe execution wrapper ==========
     * Wraps any operation in try-catch to prevent crashes
     */
    private void safeExecute(Runnable operation) {
        try {
            operation.run();
            consecutiveErrors = 0; // Reset error counter on success
        } catch (Exception e) {
            consecutiveErrors++;
            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                emergencyStopEngaged = true;
                telemetry.addData("CRITICAL", "Too many errors - emergency stop");
            }
        }
    }

    /**
     * ========== CRASH PREVENTION: Get cached pose with null safety ==========
     */
    private Pose getCachedPose() {
        try {
            if (!followerInitialized || PedroComponent.follower() == null) {
                return null;
            }

            double currentTime = telemetryTimer.seconds();

            // Return cached pose if still fresh
            if (cachedPose != null && (currentTime - lastPoseUpdateTime) < POSE_CACHE_TIME) {
                return cachedPose;
            }

            // Update cache
            Pose newPose = PedroComponent.follower().getPose();
            if (newPose != null) {
                cachedPose = newPose;
                lastPoseUpdateTime = currentTime;
            }

            return cachedPose;

        } catch (Exception e) {
            // Return last known good pose on error
            return cachedPose;
        }
    }

    @Override
    public void onUpdate() {
        // ========== CRASH PREVENTION: Watchdog reset ==========
        watchdogTimer.reset();

        // ========== CRASH PREVENTION: Emergency stop check ==========
        if (emergencyStopEngaged) {
            safeStopAllMotion();
            if (telemetryTimer.seconds() >= TELEMETRY_UPDATE_INTERVAL) {
                telemetry.addData("⚠⚠⚠ EMERGENCY STOP ⚠⚠⚠", "Restart OpMode");
                telemetry.update();
                telemetryTimer.reset();
            }
            return;
        }

        try {
            // ========== CRASH PREVENTION: Safe gamepad access ==========
            boolean killswitchActive = (gamepad2 != null && gamepad2.y);

            // === KILLSWITCH - Hold Y to stop robot ===
            double currentMultiplier = SlowModeMultiplier;
            if (killswitchActive) {
                currentMultiplier = 0;
                setMotorsBrakeMode(true);
            }

            // ========== CRASH PREVENTION: Safe drive control ==========
            if (followerInitialized && PedroComponent.follower() != null && gamepad1 != null) {
                PedroComponent.follower().setTeleOpDrive(
                        -gamepad1.left_stick_y * currentMultiplier,
                        -gamepad1.left_stick_x * currentMultiplier,
                        -gamepad1.right_stick_x * currentMultiplier,
                        true
                );
                PedroComponent.follower().update();
            }

            // === CONTINUOUS TURRET CONTROL (Hold Mode) ===
            if (gamepad2 != null && TurretOdoAi.INSTANCE.isManualMode()) {
                // Right - continuous movement after holding
                if (gamepad2.dpad_right && turretRightHoldTimer.seconds() > HOLD_THRESHOLD) {
                    safeExecute(() -> TurretOdoAi.INSTANCE.continuousTurnRight(CONTINUOUS_SPEED));
                } else if (!gamepad2.dpad_right) {
                    turretRightHoldTimer.reset();
                }

                // Left - continuous movement after holding
                if (gamepad2.dpad_left && turretLeftHoldTimer.seconds() > HOLD_THRESHOLD) {
                    safeExecute(() -> TurretOdoAi.INSTANCE.continuousTurnLeft(CONTINUOUS_SPEED));
                } else if (!gamepad2.dpad_left) {
                    turretLeftHoldTimer.reset();
                }
            }

            // ========== CRASH PREVENTION: Throttled telemetry with safety ==========
            if (telemetryTimer.seconds() >= TELEMETRY_UPDATE_INTERVAL) {
                updateTelemetry(killswitchActive);
                telemetryTimer.reset();
            }

            // ========== CRASH PREVENTION: Watchdog check ==========
            if (watchdogTimer.seconds() > WATCHDOG_TIMEOUT) {
                consecutiveErrors++;
                if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                    emergencyStopEngaged = true;
                }
            }

        } catch (Exception e) {
            // ========== CRASH PREVENTION: Top-level loop exception handler ==========
            consecutiveErrors++;
            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                emergencyStopEngaged = true;
                safeStopAllMotion();
            }
        }
    }

    /**
     * ========== CRASH PREVENTION: Safe telemetry update ==========
     */
    private void updateTelemetry(boolean killswitchActive) {
        try {
            Pose pose = getCachedPose();

            if (pose == null) {
                telemetry.addData("WARNING", "Pose unavailable");
                telemetry.update();
                return;
            }

            // === POSE TELEMETRY ===
            telemetry.addData("Status", "Running");
            telemetry.addData("X", String.format("%.1f", pose.getX()));
            telemetry.addData("Y", String.format("%.1f", pose.getY()));
            telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(pose.getHeading())));

            // Show killswitch status
            if (killswitchActive) {
                telemetry.addData("⚠ KILLSWITCH", "ACTIVE");
            }

            // === TURRET TELEMETRY ===
            if (TurretOdoAi.INSTANCE.hardwareInitialized) {
                telemetry.addData("Mode", TurretOdoAi.INSTANCE.isManualMode() ? "MANUAL" : "AUTO");

                // Show hold status in manual mode
                if (gamepad2 != null && TurretOdoAi.INSTANCE.isManualMode()) {
                    if (gamepad2.dpad_right && turretRightHoldTimer.seconds() > HOLD_THRESHOLD) {
                        telemetry.addData("Control", "SWEEP RIGHT >>>");
                    } else if (gamepad2.dpad_left && turretLeftHoldTimer.seconds() > HOLD_THRESHOLD) {
                        telemetry.addData("Control", "<<< SWEEP LEFT");
                    }
                }

                telemetry.addData("Turret", String.format("%.1f° → %.1f°",
                        TurretOdoAi.INSTANCE.getTurretAngleDeg(),
                        TurretOdoAi.INSTANCE.getTargetAngleDeg()));
                telemetry.addData("Error", String.format("%.1f°", TurretOdoAi.INSTANCE.getLastError()));

                double loopTime = TurretOdoAi.INSTANCE.getLoopTime();
                if (loopTime > 0.001) {
                    telemetry.addData("Rate", String.format("%.0f Hz", 1.0 / loopTime));
                }
            }

            // ========== CRASH PREVENTION: Error counter display ==========
            if (consecutiveErrors > 0) {
                telemetry.addData("⚠ Errors", consecutiveErrors);
            }

            telemetry.update();

        } catch (Exception e) {
            // Don't let telemetry errors crash the robot
        }
    }

    /**
     * ========== CRASH PREVENTION: Safe emergency stop ==========
     */
    private void safeStopAllMotion() {
        try {
            // Stop drivetrain
            if (followerInitialized && PedroComponent.follower() != null) {
                PedroComponent.follower().setTeleOpDrive(0, 0, 0, true);
            }

            // Stop intake/transfer
            try {
                CompliantIntake.INSTANCE.off();
            } catch (Exception e) {}

            try {
                Transfer.INSTANCE.off();
            } catch (Exception e) {}

            // Set motors to brake
            setMotorsBrakeMode(true);

        } catch (Exception e) {
            // Last resort - just let it fail gracefully
        }
    }
}