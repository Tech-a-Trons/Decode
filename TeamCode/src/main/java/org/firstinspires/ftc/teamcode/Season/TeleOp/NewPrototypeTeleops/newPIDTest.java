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
    public newPIDTest() {
        addComponents(
                new SubsystemComponent(
                        TurretPID.INSTANCE,
                        Hood.INSTANCE,
                        Turret.INSTANCE,
                        CompliantIntake.INSTANCE,
                        Transfer.INSTANCE,
                        TurretSubsystem.INSTANCE  // Added turret subsystem
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
    private static final double TARGET_X = 121;  // Example: center of field
    private static final double TARGET_Y = 121;  // Example: center of field


    @Override
    public void onStartButtonPressed() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        farscore = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(83.17241379310344, 12.620689655172416))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();
        closescore = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();
        TurretSubsystem.INSTANCE.init();
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
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    if (TurretSubsystem.INSTANCE.isAutoAimEnabled()) {
                        TurretSubsystem.INSTANCE.disableAutoAim();
                        telemetryM.addData("Turret", "Manual Mode");
                    } else {
                        TurretSubsystem.INSTANCE.enableAutoAim(TARGET_X, TARGET_Y);
                        telemetryM.addData("Turret", "Auto-Tracking Enabled");
                    }
                });
        // ========== TURRET ALIGNMENT CONTROLS ==========

        // Right Trigger: Toggle auto-tracking on/off
        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> {
                    if (!TurretSubsystem.INSTANCE.isAutoAimEnabled()) {
                        double currentAngle = TurretSubsystem.INSTANCE.getTurretAngle();
                        TurretSubsystem.INSTANCE.setTurretAngle(currentAngle + Math.toRadians(30));
                    }
                });

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    if (!TurretSubsystem.INSTANCE.isAutoAimEnabled()) {
                        double currentAngle = TurretSubsystem.INSTANCE.getTurretAngle();
                        TurretSubsystem.INSTANCE.setTurretAngle(currentAngle - Math.toRadians(30));
                    }
                });

        // Right Bumper: Center turret
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    CompliantIntake.INSTANCE.repel();
                });

        // D-pad Left/Right: Manual turret control when auto-aim is off




        // ========== EXISTING CONTROLS ==========

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> {
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.advance();
                });

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setCloseShooterSpeed().schedule());
//                .whenBecomesTrue(() -> PedroComponent.follower().followPath(closescore.get()))
//                .whenBecomesTrue(() -> automatedDrive = true);

        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setFarShooterSpeed().schedule());
//                .whenBecomesTrue(() -> Transfer.INSTANCE.on());
//            .whenBecomesTrue(() -> PedroComponent.follower().followPath(farscore.get()))
//                .whenBecomesTrue(() -> automatedDrive = true);

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
                    // Reserved for future use
                });




    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
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

        // Automated PathFollowing


        // Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !PedroComponent.follower().isBusy())) {
            PedroComponent.follower().startTeleopDrive();
            automatedDrive = false;
        }

        // ========== TELEMETRY ==========
        Pose robotPose = PedroComponent.follower().getPose();
        telemetryM.debug("position", robotPose);
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("turretAngle", Math.toDegrees(TurretSubsystem.INSTANCE.getTurretAngle()));
        telemetryM.debug("turretAutoAim", TurretSubsystem.INSTANCE.isAutoAimEnabled());

        // Show angle to target
        double angleToTarget = Math.atan2(TARGET_Y - robotPose.getY(), TARGET_X - robotPose.getX());
        telemetryM.debug("angleToTarget", Math.toDegrees(angleToTarget));
    }
}