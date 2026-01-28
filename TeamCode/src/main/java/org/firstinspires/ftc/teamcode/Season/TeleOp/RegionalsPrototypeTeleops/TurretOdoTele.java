package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Turret Odo Tele (Manual)")
public class TurretOdoTele extends NextFTCOpMode {

    private boolean robotCentric = false;
    private boolean togglePressed = false;

    public TurretOdoTele() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onStartButtonPressed() {
        TurretOdoAi.INSTANCE.init(hardwareMap);

        // Wait for follower to be ready
        int waitCount = 0;
        while (PedroComponent.follower() == null && waitCount < 100) {
            telemetry.addData("Status", "Waiting for follower...");
            telemetry.update();
            waitCount++;
            try { Thread.sleep(10); } catch (InterruptedException e) {}
        }

        if (PedroComponent.follower() != null) {
            PedroComponent.follower().setPose(new Pose(0, 0, 0));
            telemetry.addData("Status", "Manual Turret TeleOp Started");
        } else {
            telemetry.addData("ERROR", "Follower failed to initialize");
        }
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        // Toggle drive mode
        if (gamepad1.a && !togglePressed) {
            robotCentric = !robotCentric;
            togglePressed = true;
        } else if (!gamepad1.a) {
            togglePressed = false;
        }

        // Check if follower is ready
        if (PedroComponent.follower() == null) {
            telemetry.addData("ERROR", "Follower not initialized");
            telemetry.update();
            return;
        }

        // Update follower and get pose
        PedroComponent.follower().update();
        Pose pose = PedroComponent.follower().getPose();

        if (pose == null) {
            telemetry.addData("ERROR", "Pose is null");
            telemetry.update();
            return;
        }

        // IMMEDIATELY extract values from pose to avoid null issues
        double x = pose.getX();
        double y = pose.getY();
        double headingRad = pose.getHeading();
        double headingDeg = Math.toDegrees(headingRad);

        // Now call periodic() - it will update again internally but that's okay
        TurretOdoAi.INSTANCE.periodic();

        // Telemetry using stored values
        telemetry.addData("Drive Mode", robotCentric ? "Robot Centric" : "Field Centric");
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading (deg)", headingDeg);
        telemetry.addData("Turret Angle (deg)", TurretOdoAi.INSTANCE.getTurretAngleDeg());
        telemetry.addData("Target", "(" + TurretOdoAi.xt + ", " + TurretOdoAi.yt + ")");
        telemetry.update();

        // Drive
        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robotCentric
        );
    }
}