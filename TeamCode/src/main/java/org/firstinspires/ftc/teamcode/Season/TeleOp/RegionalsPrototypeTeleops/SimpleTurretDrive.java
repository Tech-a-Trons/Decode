package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext.lastPose;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

@TeleOp(name = "Simple Turret Drive")
public class SimpleTurretDrive extends NextFTCOpMode {

    public SimpleTurretDrive() {
        addComponents(
                new SubsystemComponent(
                        TurretOdoAi.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private double slowModeMultiplier = 1;
    private boolean turretEnabled = true;

    @Override
    public void onStartButtonPressed() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        try {
            // Initialize turret hardware
            TurretOdoAi.INSTANCE.init(hardwareMap);
            telemetry.addData("Turret", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Turret Init Error", e.getMessage());
        }

        try {
            // Initialize Pedro follower
            if (lastPose != null) {
                PedroComponent.follower().setPose(lastPose);
            } else {
                PedroComponent.follower().setPose(new Pose(0, 0, 0));
            }
            PedroComponent.follower().startTeleopDrive();
            telemetry.addData("Pedro", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Pedro Init Error", e.getMessage());
        }

        telemetry.addData("Status", "Ready!");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        try {
            // === TURRET CONTROL ===
            // Toggle turret on/off with B button
            if (gamepad1.b) {
                turretEnabled = !turretEnabled;
                while (gamepad1.b) {
                    // Wait for button release
                }
            }

            // Manual turret override with dpad
            if (gamepad1.dpad_left) {
                turretEnabled = false;
                // Manual left control would go here
            } else if (gamepad1.dpad_right) {
                turretEnabled = false;
                // Manual right control would go here
            }

            // Note: TurretOdoAi.periodic() is called automatically by SubsystemComponent
            // We don't need to call it manually here

            // === GET POSE ===
            Pose pose = PedroComponent.follower().getPose();
            if (pose == null) {
                telemetry.addData("ERROR", "Pose is null");
                telemetry.update();
                return;
            }

            // === TELEMETRY ===
            telemetry.addData("Status", "Running");
            telemetry.addData("X", String.format("%.1f", pose.getX()));
            telemetry.addData("Y", String.format("%.1f", pose.getY()));
            telemetry.addData("Heading", String.format("%.1f", Math.toDegrees(pose.getHeading())));
            telemetry.addData("Turret", turretEnabled ? "ENABLED" : "DISABLED");

            if (TurretOdoAi.INSTANCE.isHardwareInitialized()) {
                telemetry.addData("Turret Angle", String.format("%.1f°", TurretOdoAi.INSTANCE.getTurretAngleDeg()));
                telemetry.addData("Target Angle", String.format("%.1f°", TurretOdoAi.INSTANCE.getTargetAngleDeg()));
                telemetry.addData("Error", String.format("%.1f°", TurretOdoAi.INSTANCE.getLastError()));
                telemetry.addData("Distance", String.format("%.1f", TurretOdoAi.INSTANCE.getDistanceToTarget()));
            }

            telemetry.update();

            // === DRIVE CONTROL ===
            PedroComponent.follower().setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true  // field-centric
            );

            // === CRITICAL: Update follower ONCE per loop ===
            PedroComponent.follower().update();

        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.addData("Type", e.getClass().getSimpleName());
            if (e.getStackTrace().length > 0) {
                telemetry.addData("At", e.getStackTrace()[0].toString());
            }
            telemetry.update();
        }
    }
}