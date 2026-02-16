package org.firstinspires.ftc.teamcode.Season.Prototypes.Regionals;

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

@TeleOp(name = "Moving while shooting")
public class TuffShootTele extends NextFTCOpMode {

    private boolean intakeToggle = false;
    private boolean turretManualMode = false;
    public double SlowModeMultipler = 0;

    public TuffShootTele() {
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

    double lastTime = System.nanoTime() / 1e9;
    double lastX = TurretOdoAi.INSTANCE.getX();
    double lastY = TurretOdoAi.INSTANCE.getY();

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


        // Shoot
        button(() -> gamepad1.right_trigger > 0.05)
                .whenTrue(() -> {
                    double currentTime = System.nanoTime() / 1e9;
                    double dt = currentTime - lastTime;
                    lastTime = currentTime;

                    double robotX = TurretOdoAi.INSTANCE.getX();
                    double robotY = TurretOdoAi.INSTANCE.getY();
                    double robotHeadingDeg = TurretOdoAi.INSTANCE.getHeading();
                    double robotHeadingRad = Math.toRadians(robotHeadingDeg);

                    // ===== Compute field velocity (inches/sec) =====
                    double robotVx = (robotX - lastX) / dt;
                    double robotVy = (robotY - lastY) / dt;

                    lastX = robotX;
                    lastY = robotY;
//                    Pose pose = PedroComponent.follower().getPose();
//                    double d = Math.hypot(
//                            TARGET_X - pose.getX(),
//                            TARGET_Y - pose.getY()
//                    );
//                    TurretPID.INSTANCE.newshooterdistance(d).schedule();
                    TurretPID.INSTANCE.tuffshot(
                            robotX, robotY, robotHeadingRad,
                            robotVx, robotVy,
                            121.0, 121.0   // goal
                    ).schedule();
                    TurretPID.shootRequested = true;
                    TurretPID.hasShot = false;
                });

        // Intake toggle
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

    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
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