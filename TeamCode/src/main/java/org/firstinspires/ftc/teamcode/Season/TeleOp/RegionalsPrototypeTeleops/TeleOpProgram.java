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

@TeleOp(name = "Regionals Teleop FINAL w/Manual")
public class TeleOpProgram extends NextFTCOpMode {

    private boolean intakeToggle = false;
    public double SlowModeMultiplier = 1.0;

    // Turret hybrid control
    private ElapsedTime turretRightHoldTimer = new ElapsedTime();
    private ElapsedTime turretLeftHoldTimer = new ElapsedTime();
    private static final double HOLD_THRESHOLD = 0.3;
    private static final double CONTINUOUS_SPEED = 0.8;

    // Telemetry throttling
    private ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_UPDATE_INTERVAL = 0.05; // 50ms = 20 Hz

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
    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");

    @Override
    public void onStartButtonPressed() {
        // Initialize turret safely
        TurretOdoAi.INSTANCE.init(hardwareMap);
        NewHood.INSTANCE.init(hardwareMap);

        // Set initial pose
        PedroComponent.follower().setPose(new Pose(72, 72, Math.toRadians(270)));
        PedroComponent.follower().startTeleopDrive();

        // Start telemetry timer
        telemetryTimer.reset();

        // === MANUAL TURRET CONTROL - SINGLE TAP (Gamepad 2) ===
        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(() -> {
                    TurretOdoAi.INSTANCE.turnRight();

                });

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(() -> {
                    TurretOdoAi.INSTANCE.turnLeft();

                });

        // === POSE RESET ===
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().setPose(Middle);
                });

        // === SHOOT ===
        button(() -> gamepad1.right_trigger > 0.05)
                .whenTrue(() -> {
                    Pose pose = PedroComponent.follower().getPose();
                    if (pose != null) {
                        double d = Math.hypot(
                                TARGET_X - pose.getX(),
                                TARGET_Y - pose.getY()
                        );
                        TurretPID.INSTANCE.regionalsshooterdistance(d).schedule();
                        TurretPID.shootRequested = true;
                        TurretPID.hasShot = false;
                    }
                });

        // === INTAKE TOGGLE ===
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    intakeToggle = !intakeToggle;

                    if (intakeToggle) {
                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.nice();
                    } else {
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();
                    }
                });

        // === MANUAL INTAKE ===
        Gamepads.gamepad1().leftTrigger()
                .greaterThan(0.05)
                .whenBecomesTrue(() -> {
                    Transfer.INSTANCE.on();
                    CompliantIntake.INSTANCE.on();
                });

        // === EMERGENCY STOP ===
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.resetShooter().schedule();
                    Hood.INSTANCE.midopen();
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();
                    intakeToggle = false;
                });
    }

    @Override
    public void onUpdate() {
        // === KILLSWITCH - Hold Y to stop robot ===
        double currentMultiplier = SlowModeMultiplier;
        if (gamepad2.y) {
            currentMultiplier = 0;
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Drive control - runs every loop for responsive driving
        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y * currentMultiplier,
                -gamepad1.left_stick_x * currentMultiplier,
                -gamepad1.right_stick_x * currentMultiplier,
                true
        );
        PedroComponent.follower().update();


        // Throttled telemetry - updates every 50ms
        if (telemetryTimer.seconds() >= TELEMETRY_UPDATE_INTERVAL) {
            updateTelemetry();
            telemetryTimer.reset();
        }
    }

    /**
     * Telemetry update - called every 50ms (20 Hz)
     */
    private void updateTelemetry() {
        // Get fresh pose each telemetry update
        Pose pose = PedroComponent.follower().getPose();
        if (pose == null) {
            telemetry.addData("ERROR", "Pose is null");
            telemetry.update();
            return;
        }

        // === POSE TELEMETRY ===
        telemetry.addData("Status", "Running");
        telemetry.addData("X", String.format("%.1f", pose.getX()));
        telemetry.addData("Y", String.format("%.1f", pose.getY()));
        telemetry.addData("Heading", String.format("%.1f°", (Math.toDegrees(pose.getHeading())+360) % 360));
        telemetry.addData("Kp", TurretOdoAi.INSTANCE.kP);

        // Show killswitch status
        if (gamepad2.y) {
            telemetry.addData("⚠ KILLSWITCH", "ACTIVE");
        }

        // === TURRET TELEMETRY ===
        if (TurretOdoAi.INSTANCE.hardwareInitialized) {
      // Show hold status in manual mode

            telemetry.addData("Turret", String.format("%.1f° → %.1f°",
                    TurretOdoAi.INSTANCE.getTurretAngleDeg(),
                    TurretOdoAi.INSTANCE.getTargetAngleDeg()));
            telemetry.addData("Distance", String.format("%.1f", TurretOdoAi.INSTANCE.getDistanceToTarget()));
            telemetry.addData("Skipped Loops", TurretOdoAi.INSTANCE.getSkippedLoops());
            telemetry.addData("Angle Adjust", TurretOdoAi.INSTANCE.AngleAdjust());
            telemetry.addData("Error", String.format("%.1f°", TurretOdoAi.INSTANCE.getLastError()));

            // Only show update rate if relevant
            double loopTime = TurretOdoAi.INSTANCE.getLoopTime();
            if (loopTime > 0.001) {
                telemetry.addData("Rate", String.format("%.0f Hz", 1.0 / loopTime));
            }
        }

        telemetry.update();
    }
}