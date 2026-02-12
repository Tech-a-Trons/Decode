package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAiFixed;
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
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Regionals Teleop")
public class TeleOpProgram extends NextFTCOpMode {

    private boolean intakeToggle = false;

    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(

                        CompliantIntake.INSTANCE,
                        Transfer.INSTANCE,
                        TurretPID.INSTANCE,
                        Hood.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    } private static final double TARGET_X = 121;  // Example: center of field
    private static final double TARGET_Y = 121;
    private Pose Middle = new Pose(72,35,180);
    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");

    @Override
    public void onStartButtonPressed() {

        // Initialize turret safely

        // Set initial pose
        PedroComponent.follower().setPose(new Pose(38, 22.5, Math.toRadians(180)));

        PedroComponent.follower().startTeleopDrive();
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().setPose(Middle);
                });
//        Command driverControlled = new MecanumDriverControlled(
//                frontLeftMotor,
//                frontRightMotor,
//                backLeftMotor,
//                backRightMotor,
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX()
//        );
//        driverControlled.schedule();

        // Shoot
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

        // Intake toggle
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    intakeToggle = !intakeToggle;

                    if (intakeToggle) {
                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.off();
                    } else {
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();
                    }
                });

        // Manual intake
        Gamepads.gamepad1().leftTrigger()
                .greaterThan(0.05)
                .whenBecomesTrue(() -> {
                    Transfer.INSTANCE.on();
                    CompliantIntake.INSTANCE.on();
                });

        // Emergency stop
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
        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x ,
                -gamepad1.right_stick_x ,
                true // Robot Centric
        );
        PedroComponent.follower().update();

        Pose currentPose = PedroComponent.follower().getPose();
        if (currentPose == null) return;

        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }
}