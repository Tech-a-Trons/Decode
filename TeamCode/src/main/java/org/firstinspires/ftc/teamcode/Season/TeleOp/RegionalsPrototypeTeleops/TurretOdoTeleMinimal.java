package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext.lastPose;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

@TeleOp(name = "TurretOdoTele_MINIMAL_TEST")
public class TurretOdoTeleMinimal extends NextFTCOpMode {

    public TurretOdoTeleMinimal() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private double slowModeMultiplier = 0.5;

    @Override
    public void onStartButtonPressed() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        try {
            if (lastPose != null) {
                PedroComponent.follower().setPose(lastPose);
            } else {
                PedroComponent.follower().setPose(new Pose(0, 0, 0));
            }

            PedroComponent.follower().startTeleopDrive();

            telemetry.addData("Status", "Initialized successfully");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("INIT ERROR", e.getMessage());
            telemetry.addData("Error Type", e.getClass().getSimpleName());
            telemetry.update();
        }
    }

    @Override
    public void onUpdate() {
        try {
            // Get pose
            Pose pose = PedroComponent.follower().getPose();

            if (pose == null) {
                telemetry.addData("ERROR", "Pose is null");
                telemetry.update();
                return;
            }

            // Simple telemetry
            telemetry.addData("X", String.format("%.1f", pose.getX()));
            telemetry.addData("Y", String.format("%.1f", pose.getY()));
            telemetry.addData("Heading", String.format("%.1f", Math.toDegrees(pose.getHeading())));
            telemetry.addData("Status", "Running OK");
            telemetry.update();

            // Drive control
            PedroComponent.follower().setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true
            );

            // Update follower
            PedroComponent.follower().update();

        } catch (Exception e) {
            telemetry.addData("CRITICAL ERROR", e.getMessage());
            telemetry.addData("Error Type", e.getClass().getSimpleName());
            if (e.getStackTrace().length > 0) {
                telemetry.addData("Location", e.getStackTrace()[0].toString());
            }
            telemetry.update();
        }
    }
}