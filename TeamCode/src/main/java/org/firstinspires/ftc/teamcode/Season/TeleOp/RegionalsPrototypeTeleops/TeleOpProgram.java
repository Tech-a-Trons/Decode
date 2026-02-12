package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "Regionals Teleop")
public class TeleOpProgram extends NextFTCOpMode {

    // Target position for turret (adjust to your field coordinates)
    private static final double TARGET_X = 121;
    private static final double TARGET_Y = 121;

    private boolean intakeToggle = false;

    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(CompliantIntake.INSTANCE, Transfer.INSTANCE, TurretPID.INSTANCE, Hood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)  // Add Pedro for localization only
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");

    @Override
    public void onStartButtonPressed() {
        // Set initial pose if needed
        PedroComponent.follower().setPose(new Pose(38, 22.5, 180));

        // Use NextFTC's MecanumDriverControlled instead of Pedro's teleop drive
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        // Right Trigger: Shoot (uses Pedro's pose tracking)
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

        // Right Bumper: Toggle intake and transfer
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    intakeToggle = !intakeToggle;

                    if (intakeToggle) {
                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.slight();
                    } else {
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();
                    }
                });

        // Left Trigger: Run intake and transfer
        Gamepads.gamepad1().leftTrigger()
                .greaterThan(0.05)
                .whenBecomesTrue(() -> {
                    Transfer.INSTANCE.on();
                    CompliantIntake.INSTANCE.on();
                });

        // Left Bumper: Turn everything off
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
        // Update Pedro follower for localization/odometry tracking only
        // This updates the pose estimate without controlling the motors
        PedroComponent.follower().update();

        // Optional: Add telemetry to see current pose
        Pose currentPose = PedroComponent.follower().getPose();
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }
}