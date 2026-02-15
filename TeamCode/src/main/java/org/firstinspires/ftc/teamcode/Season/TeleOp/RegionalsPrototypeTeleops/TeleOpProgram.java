package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.NewHood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAiFixed;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;

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

@TeleOp(name = "Regionals Teleop Turret")
public class TeleOpProgram extends NextFTCOpMode {

    private boolean intakeToggle = false;
    private boolean turretManualMode = false;

    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(

                        CompliantIntake.INSTANCE,
                        Transfer.INSTANCE,
                        TurretPID.INSTANCE,
                        TurretOdoAi.INSTANCE,  // Added TurretOdoAiFixed
                        NewHood.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private static final double TARGET_X = 121;  // Example: center of field
    private static final double TARGET_Y = 121;
    private Pose Middle = new Pose(72,72,Math.toRadians(270));
    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");

    @Override
    public void onStartButtonPressed() {

        // Initialize turret safely
        TurretOdoAi.INSTANCE.init(hardwareMap);  // Initialize TurretOdoAiFixed
        NewHood.INSTANCE.init(hardwareMap);
        // Set target position for turret auto-aiming
        // Set initial pose
        PedroComponent.follower().setPose(new Pose(72, 72, Math.toRadians(270)));

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

        // Toggle between manual and auto turret mode with D-pad Up
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> {
                    turretManualMode = !turretManualMode;
                    TurretOdoAiFixed.INSTANCE.setManualMode(turretManualMode);
                });

        // Manual turret control - D-pad Left (turn left, decrease position)
        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> {
                    if (turretManualMode) {
                        TurretOdoAiFixed.INSTANCE.turnLeft();
                    }
                });

        // Manual turret control - D-pad Right (turn right, increase position)
        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    if (turretManualMode) {
                        TurretOdoAiFixed.INSTANCE.turnRight();
                    }
                });
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x ,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
        PedroComponent.follower().update();

        Pose pose = PedroComponent.follower().getPose();
        if (pose == null) {
            telemetry.addData("ERROR", "Pose is null");
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("X", String.format("%.1f", pose.getX()));
        telemetry.addData("Y", String.format("%.1f", pose.getY()));
        telemetry.addData("Heading", String.format("%.1f", Math.toDegrees(pose.getHeading())));
        telemetry.addData("Turret", "ENABLED" );

        if (TurretOdoAi.INSTANCE.hardwareInitialized) {
            telemetry.addData("Turret Angle", String.format("%.1f°", TurretOdoAi.INSTANCE.getTurretAngleDeg()));
            telemetry.addData("Target Angle", String.format("%.1f°", TurretOdoAi.INSTANCE.getTargetAngleDeg()));
            telemetry.addData("Error", String.format("%.1f°", TurretOdoAi.INSTANCE.getLastError()));
            telemetry.addData("Distance", String.format("%.1f", TurretOdoAi.INSTANCE.getDistanceToTarget()));
        }
        telemetry.update();
    }
}