package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID.newvelo;
import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ColorSensor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.NewHood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "Red Teleop")
public class RegionalRed extends NextFTCOpMode {

    private boolean intakeToggle = false;
    public double SlowModeMultiplier = 1.0;

    // Telemetry throttling
    private static final double TELEMETRY_UPDATE_INTERVAL = 0.05; // 50ms = 20 Hz

    public RegionalRed() {
        addComponents(
                new SubsystemComponent(
                        CompliantIntake.INSTANCE,
                        Transfer.INSTANCE,
                        TurretPID.INSTANCE,
                        TurretOdoAi.INSTANCE,
                        NewHood.INSTANCE,
                        ColorSensor.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower), new LoopTimeComponent()
        );
    }

    private static final double TARGET_X = 130;
    private static final double TARGET_Y = 130;
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
        ColorSensor.INSTANCE.init(hardwareMap);

        NewHood.INSTANCE.setAlliance("red");
        TurretOdoAi.INSTANCE.setAlliance("red");
        TurretOdoAi.INSTANCE.AngleAdjust = 0;
        TurretOdoAi.INSTANCE.ManualAngleAdjust= 0;

        // Reseters
        ColorSensor.artifactcounter = 0;


        // Set initial pose
        PedroComponent.follower().setPose(new Pose(72, 72, Math.toRadians(270)));
        PedroComponent.follower().startTeleopDrive();
//
//
//        Gamepads.gamepad1().dpadRight()
//                .whenBecomesTrue(TurretOdoAi.INSTANCE::relocalize);
//
//        Gamepads.gamepad1().dpadUp()
//                .whenBecomesTrue(TurretOdoAi.INSTANCE::correctWithLimelight);

        // === POSE RESET ===
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().setPose(Middle);
                });

        // === SHOOT ===
        button(() -> gamepad1.right_trigger > 0.05)
                .whenTrue(() -> {
                    ColorSensor.artifactcounter = 0;
                    Pose pose = PedroComponent.follower().getPose();
                    if (pose != null) {
                        double d = Math.hypot(
                                TARGET_X - pose.getX(),
                                TARGET_Y - pose.getY()
                        );
                        TurretPID.INSTANCE.regionalsshooterdistance(d).schedule();
                        double actualRPM = TurretPID.INSTANCE.getActualVelocity();
                        NewHood.INSTANCE.adjustForDistanceAndVelocity(d, newvelo, actualRPM);
                        TurretOdoAi.INSTANCE.correctWithLimelight();
                        TurretPID.shootRequested = true;
                        TurretPID.hasShot = false;
                    }
                });
        Gamepads.gamepad1().dpadLeft()
                .whenTrue(() -> {
                    Transfer.INSTANCE.repel();
                });
        // === INTAKE TOGGLE ===
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {

                    intakeToggle = !intakeToggle;

                    if (intakeToggle) {

                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.slight();

                        // Reset ball counter when starting intake


                    } else {
                        // Manually stopping intake
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();

                        // Reset ball counter


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
//                    NewHood.INSTANCE.midopen();
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();
                    intakeToggle = false;
                });

// === MANUAL TURRET CONTROL - SINGLE TAP (Gamepad 2) ===
        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(() -> {
                    TurretOdoAi.INSTANCE.turnRight();

                });

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(() -> {
                    TurretOdoAi.INSTANCE.turnLeft();

                });

        // Artifact Count Updater (Gamepad 2)
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(() -> {
                    ColorSensor.INSTANCE.artifactcounter += 1;

                });

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(() -> {
                    ColorSensor.INSTANCE.artifactcounter -= 1;

                });
        // Emergency Backup Drivetrain Stop

        button(() -> gamepad2.right_trigger > 0.2)
                .whenTrue(() -> {
                    SlowModeMultiplier = 0;
                    gamepad1.rumble(500);
                })
                .whenFalse(() -> {
                    SlowModeMultiplier = 1;
                });
        Gamepads.gamepad2().b()
                .whenBecomesTrue(() -> {
                    TurretPID.shootRequested = false;
                });
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(TurretOdoAi.INSTANCE::relocalize);

        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(TurretOdoAi.INSTANCE::correctWithLimelight);

    }
    @Override
    public void onUpdate() {
        NewHood.INSTANCE.adjustForCurrentDistance();
        if (intakeToggle) {
            ColorSensor.INSTANCE.IncountBalls();
            if (ColorSensor.artifactcounter == 0) {
            }
            if (ColorSensor.artifactcounter == 1) {
                Transfer.INSTANCE.advance();
                // Reset toggle state
            }
            if (ColorSensor.artifactcounter == 2) {
                Transfer.INSTANCE.off();
                // Reset toggle state
            }
            // Auto-stop when 3 balls are collected
            if (ColorSensor.artifactcounter == 3) {
                // Turn off intake and transfer
                CompliantIntake.INSTANCE.off();
                Transfer.INSTANCE.off();
                // Reset toggle state
                intakeToggle = false;
                // Optional: Rumble controller to alert driver
                gamepad1.rumble(250);
                gamepad2.rumble(250);
            }
        }
        // Drive control - runs every loop for responsive driving
        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y * SlowModeMultiplier,
                -gamepad1.left_stick_x * SlowModeMultiplier,
                -gamepad1.right_stick_x * SlowModeMultiplier,
                true
        );
        PedroComponent.follower().update();
    }
    }
