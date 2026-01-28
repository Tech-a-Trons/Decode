package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Turret Odo Tele (Manual)")
public class TurretOdoTele extends NextFTCOpMode {

    private boolean robotCentric = false; // Start with field-centric
    private boolean togglePressed = false; // Debounce for toggle button

    public TurretOdoTele() {
        addComponents(
                new SubsystemComponent(TurretOdoAi.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onStartButtonPressed() {
        // Initialize turret subsystem hardware
        TurretOdoAi.INSTANCE.init(hardwareMap);

        // Reset robot pose
        PedroComponent.follower().setPose(new Pose(0, 0, 0));

        telemetry.addData("Status", "Manual Turret TeleOp Started");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        // Toggle drive mode with 'A' button (with debounce)
        if (gamepad1.a && !togglePressed) {
            robotCentric = !robotCentric;
            togglePressed = true;
        } else if (!gamepad1.a) {
            togglePressed = false;
        }

        // Let subsystem handle odometry update
        TurretOdoAi.INSTANCE.periodic();

        // Get pose
        Pose pose = PedroComponent.follower().getPose();

        // Null check to prevent crash
        if (pose == null) {
            telemetry.addData("ERROR", "Pose is null - follower not initialized");
            telemetry.update();
            return;
        }

        // Update telemetry
        telemetry.addData("Drive Mode", robotCentric ? "Robot Centric" : "Field Centric");
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Turret Angle (deg)", TurretOdoAi.INSTANCE.getTurretAngleDeg());
        telemetry.addData("Target", "(" + TurretOdoAi.xt + ", " + TurretOdoAi.yt + ")");
        telemetry.update();

        // Set drive with toggleable mode
        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robotCentric
        );
    }
}