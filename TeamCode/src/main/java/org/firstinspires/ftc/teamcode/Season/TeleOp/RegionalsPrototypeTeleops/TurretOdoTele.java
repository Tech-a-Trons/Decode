package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext.lastPose;
import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.RGBled;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.RedLL;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

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
public class TurretOdoTele extends NextFTCOpMode {

    private RedExperimentalDistanceLExtractor limelight;
    private RedLL turretAlignment;
    private ColorSensor colorSensor;
    private RGBled rgBled;

    public TurretOdoTele() {
        addComponents(
                new SubsystemComponent(
                        TurretOdoAi.INSTANCE,
                        TurretPID.INSTANCE,
                        Hood.INSTANCE,
                        Turret.INSTANCE,
                        CompliantIntake.INSTANCE,
                        Transfer.INSTANCE,
                        rgBled.INSTANCE
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
    private boolean turretLimelightEnabled = true;
    private boolean automatedDrive;
    private Supplier<PathChain> farscore;
    private Supplier<PathChain> closescore;
    private Supplier<PathChain> Parkpath;
    private TelemetryManager telemetryM;
    private double slowModeMultiplier = 1;

    private static final double TARGET_X = 60;
    private static final double TARGET_Y = 60;
    private static final double DISTANCE_THRESHOLD = 90.0;
    private static final double CLOSE_HOOD_DISTANCE = 20.0;

    private Pose targetPose = new Pose(TARGET_X, TARGET_Y);
    private Pose Middle = new Pose(72, 35, 180);
    private Pose Park = new Pose(38.74532374100719, 33.358273381294964, 90);
    private boolean intakeToggle = false;
    private long intakeStartTime = 0;
    private VoltageGet voltageGet;

    private boolean robotCentric = false;
    private boolean togglePressed = false;

    @Override
    public void onStartButtonPressed() {
        limelight = new RedExperimentalDistanceLExtractor(hardwareMap);
        turretAlignment = new RedLL(hardwareMap, limelight, voltageGet);
        colorSensor = new ColorSensor(hardwareMap);
        rgBled = new RGBled();

        limelight.startReading();
        turretAlignment.setTelemetry(telemetry);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        farscore = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose,
                        new Pose(83.17241379310344, 12.620689655172416))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();

        closescore = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose,
                        new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();

        Parkpath = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose,
                        new Pose(38.74532374100719, 33.358273381294964, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        PedroComponent.follower()::getHeading, Math.toRadians(0), 0.8))
                .build();

        if (lastPose != null) {
            PedroComponent.follower().setPose(lastPose);
        } else {
            PedroComponent.follower().setPose(new Pose(0, 0, 0));
        }

        PedroComponent.follower().startTeleopDrive();

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        // ========== BUTTON BINDINGS ==========

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    turretLimelightEnabled = !turretLimelightEnabled;
                    if (!turretLimelightEnabled) {
                        turretAlignment.stopTurret();
                        telemetryM.addData("Limelight Turret", "Auto-Align Disabled");
                    } else {
                        telemetryM.addData("Limelight Turret", "Auto-Align Enabled");
                    }
                });

        Gamepads.gamepad1().dpadLeft()
                .whenTrue(() -> {
                    turretLimelightEnabled = false;
                    turretAlignment.turretLeft();
                })
                .whenBecomesFalse(() -> {
                    turretLimelightEnabled = true;
                    turretAlignment.stopAndEnableAlign();
                });

        Gamepads.gamepad1().dpadRight()
                .whenTrue(() -> {
                    turretLimelightEnabled = false;
                    turretAlignment.turretRight();
                })
                .whenBecomesFalse(() -> {
                    turretLimelightEnabled = true;
                    turretAlignment.stopAndEnableAlign();
                });

        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
                    turretLimelightEnabled = false;
                    turretAlignment.stopTurret();
                    CompliantIntake.INSTANCE.repel();
                    telemetryM.addData("Turret", "Stopped & Intake Reset");
                });

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    intakeToggle = !intakeToggle;
                    if (intakeToggle) {
                        intakeStartTime = System.currentTimeMillis();
                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.slight();
                        colorSensor.artifactcounter = 0;
                        telemetryM.addData("Intake", "ON - Counting Balls");
                    } else {
                        intakeStartTime = 0;
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();
                        colorSensor.artifactcounter = 0;
                        telemetryM.addData("Intake", "OFF - Manual Stop");
                    }
                });

        button(() -> gamepad1.right_trigger > 0.05)
                .whenTrue(() -> {
                    Pose pose = PedroComponent.follower().getPose();
                    double d = Math.hypot(TARGET_X - pose.getX(), TARGET_Y - pose.getY());
                    TurretPID.INSTANCE.newshooterdistance(d).schedule();
                    TurretPID.shootRequested = true;
                    TurretPID.hasShot = false;
                });

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().setPose(Middle);
                });

        Gamepads.gamepad1().leftTrigger()
                .greaterThan(0.05)
                .whenBecomesTrue(() -> {
                    Transfer.INSTANCE.on();
                    CompliantIntake.INSTANCE.on();
                });

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.resetShooter().schedule();
                    Hood.INSTANCE.midopen();
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();
                });

        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    PedroComponent.follower().followPath(Parkpath.get());
                    automatedDrive = true;
                });
    }

    @Override
    public void onUpdate() {
        // Drive mode toggle - FIXED: check dpad_up for release
        if (gamepad1.dpad_up && !togglePressed) {
            robotCentric = !robotCentric;
            togglePressed = true;
        } else if (!gamepad1.dpad_up) {
            togglePressed = false;
        }

        // Let subsystem handle odometry update

        // Get pose ONCE and cache it
        Pose pose = PedroComponent.follower().getPose();

        if (pose == null) {
            telemetry.addData("ERROR", "Pose is null - follower not initialized");
            telemetry.update();
            return;
        }

        // Cache all frequently-used values at the start
        double x = pose.getX();
        double y = pose.getY();
        double heading = TurretOdoAi.INSTANCE.getHeading();
        double turretAngle = TurretOdoAi.INSTANCE.getTurretAngleDeg();
        double distanceToTarget = Math.hypot(TurretOdoAi.xt - x, TurretOdoAi.yt - y);

        // DETAILED DEBUG TELEMETRY
        telemetry.addData("Drive Mode", robotCentric ? "Robot Centric" : "Field Centric");
        telemetry.addData("Robot X", x);
        telemetry.addData("Robot Y", y);
        telemetry.addData("Robot Heading", heading);
        telemetry.addData("Target X", TurretOdoAi.xt);
        telemetry.addData("Target Y", TurretOdoAi.yt);
        telemetry.addData("Delta X", TurretOdoAi.xt - x);
        telemetry.addData("Delta Y", TurretOdoAi.yt - y);
        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Distance", distanceToTarget);
        telemetry.update();

        // ========== DRIVE CONTROL ==========
        if (!automatedDrive) {
            PedroComponent.follower().setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    robotCentric
            );
        }

        // CRITICAL: Only ONE follower update per loop
        PedroComponent.follower().update();

        // Cache velocity values
        double targetVel = TurretPID.activeTargetVelocity;
        double actualVel = TurretPID.turret.getVelocity();

        // ========== HOOD CONTROL ==========
        if (distanceToTarget <= CLOSE_HOOD_DISTANCE) {
            Hood.INSTANCE.close();
        } else {
            if (targetVel > 500) {
                Hood.INSTANCE.compensateFromVelocity(targetVel, actualVel);
            }
        }

        // ========== TURRET LIMELIGHT ALIGNMENT ==========
        if (turretLimelightEnabled) {
            if (distanceToTarget > DISTANCE_THRESHOLD) {
                turretAlignment.farAlign();
            } else {
                turretAlignment.closeAlign();
            }
        }

        // ========== COLOR SENSOR BALL COUNTING ==========
        if (intakeToggle) {
            colorSensor.IncountBalls();

            // Cache ball count
            int ballCount = colorSensor.artifactcounter;

            if (ballCount > 0) {
                RGBled.INSTANCE.open();
            }
            if (ballCount >= 2) {
                Transfer.INSTANCE.advance();
                RGBled.INSTANCE.midopen();
            }
            if (ballCount >= 3) {
                RGBled.INSTANCE.close();
                CompliantIntake.INSTANCE.off();
                Transfer.INSTANCE.off();
                intakeToggle = false;
                intakeStartTime = 0;
                gamepad1.rumble(500);
                colorSensor.artifactcounter = 0;
            }
        }

        // Single limelight update
        limelight.update();
    }
}