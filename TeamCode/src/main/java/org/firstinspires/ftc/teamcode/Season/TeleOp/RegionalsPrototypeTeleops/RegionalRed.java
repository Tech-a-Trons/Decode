package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static org.firstinspires.ftc.teamcode.Season.Auto.Tuning.follower;
import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext.lastPose;
import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID.newvelo;
import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID.turret;
import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ColorSensor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.NewHood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.NewTurret;
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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Final Red Teleop")
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
//                        NewTurret.INSTANCE,
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

    ElapsedTime InttakeStopper = new ElapsedTime();

    @Override
    public void onStartButtonPressed() {

        // Initialize turret safely
        TurretOdoAi.INSTANCE.init(hardwareMap);

        NewHood.INSTANCE.init(hardwareMap);
        //ColorSensor.INSTANCE.init(hardwareMap);
ColorSensor.INSTANCE.init(hardwareMap);
        NewHood.INSTANCE.setAlliance("red");
        TurretOdoAi.INSTANCE.setAlliance("red");
        TurretOdoAi.INSTANCE.AngleAdjust = 0;
        TurretOdoAi.INSTANCE.ManualAngleAdjust= 0;
        

        // Reseters
        ColorSensor.artifactcounter = 0;


        // Set initial pose
        if (lastPose != null){
    PedroComponent.follower().setPose(lastPose);
        }
else {
    PedroComponent.follower().setPose(new Pose(0,0,0));
}

        PedroComponent.follower().startTeleopDrive();

//        Gamepads.gamepad1().dpadLeft()
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
                        TurretPID.shootRequested = true;
                        TurretPID.hasShot = false;
                    }
                });
        Gamepads.gamepad1().leftStickButton()
                .whenTrue(() -> {
                    Transfer.INSTANCE.repel();
                });

        // In your TeleOp periodic/loop:
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
            Pose limelightPose = TurretOdoAi.INSTANCE.getRobotPosFromTarget();

            if (limelightPose != null) {
                // Preserve the current odometry heading — only correct X/Y
                double currentHeading = PedroComponent.follower().getPose().getHeading();

                double dx = limelightPose.getX() - PedroComponent.follower().getPose().getX();
                double dy = limelightPose.getY() - PedroComponent.follower().getPose().getY();
                double correctionMag = Math.hypot(dx, dy);

                if (correctionMag < 12.0) { // reject if >12 inches off — likely a bad read
                    PedroComponent.follower().setPose(new Pose(
                            limelightPose.getX(), limelightPose.getY(), currentHeading
                    ));
                }

                Pose correctedPose = new Pose(
                        limelightPose.getX(),
                        limelightPose.getY(),
                        currentHeading
                );

                PedroComponent.follower().setPose(correctedPose);
                telemetry.addLine("Relocalized via Limelight!");
            } else {
                telemetry.addLine("Relocalization failed — no valid tag in view.");
            }

            telemetry.update();
        });

        // === INTAKE TOGGLE ===
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {

                    intakeToggle = !intakeToggle;

                    if (intakeToggle) {
                        ColorSensor.INSTANCE.light();
                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.repel();

                        // Reset ball counter when starting intake
                        ColorSensor.artifactcounter = 0;

                    } else {
                        // Manually stopping intake
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();
//                        ColorSensor.INSTANCE.nolight();
                        // Reset ball counter

                        ColorSensor.artifactcounter = 0;

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



        button(() -> gamepad2.right_trigger > 0.2)
                .whenTrue(() -> {
                    SlowModeMultiplier = 0;
                    gamepad1.rumble(500);
                })
                .whenFalse(() -> {
                    SlowModeMultiplier = 1;
                });

    }
    @Override
    public void onUpdate() {
        NewHood.INSTANCE.adjustForCurrentDistance();
        if (intakeToggle) {
            ColorSensor.INSTANCE.IncountBalls();
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
    public void onStop(){
        Transfer.INSTANCE.off();
        CompliantIntake.INSTANCE.off();
        TurretPID.INSTANCE.setShooterSpeed(0);
        RobotContext.lastPose = PedroComponent.follower().getPose();
    }
}
