//package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;
//
//import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext.lastPose;
//import static dev.nextftc.bindings.Bindings.button;
//
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ColorSensor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.RGBled;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;
//
//import java.util.function.Supplier;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.ftc.Gamepads;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//import dev.nextftc.hardware.driving.MecanumDriverControlled;
//import dev.nextftc.hardware.impl.MotorEx;
//import dev.nextftc.extensions.pedro.PedroComponent;
//
//@TeleOp(name = "TurretOdo3")
//public class TurretOdoTele extends NextFTCOpMode {
//
//    private ColorSensor colorSensor;
//    private RGBled rgBled;
//
//    public TurretOdoTele() {
//        addComponents(
//                new SubsystemComponent(
//                        TurretOdoAi.INSTANCE,
//                        TurretPID.INSTANCE,
//                        Hood.INSTANCE,
//                        Turret.INSTANCE,
//                        CompliantIntake.INSTANCE,
//                        Transfer.INSTANCE,
//                        ColorSensor.INSTANCE
//                ),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE,
//                new PedroComponent(Constants::createFollower)
//        );
//    }
//
//    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
//    private final MotorEx frontRightMotor = new MotorEx("fr");
//    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
//    private final MotorEx backRightMotor = new MotorEx("br");
//    private boolean turretAutoAlignEnabled = true;
//    private boolean automatedDrive;
//    private Supplier<PathChain> farscore;
//    private Supplier<PathChain> closescore;
//    private Supplier<PathChain> Parkpath;
//    private TelemetryManager telemetryM;
//    private double slowModeMultiplier = 0.5;
//
//    private static final double DISTANCE_THRESHOLD = 90.0;
//    private static final double CLOSE_HOOD_DISTANCE = 20.0;
//
//    private Pose Middle = new Pose(72, 35, 180);
//    private Pose Park = new Pose(38.74532374100719, 33.358273381294964, 90);
//    private boolean intakeToggle = false;
//    private long intakeStartTime = 0;
//
//    private boolean robotCentric = false;
//    private boolean togglePressed = false;
//
//    @Override
//    public void onStartButtonPressed() {
//        colorSensor = new ColorSensor();
//        rgBled = new RGBled();
//
//        // Initialize TurretOdoAi hardware with error handling
//        try {
//            TurretOdoAi.INSTANCE.init(hardwareMap);
//            telemetry.addData("TurretOdoAi", "Initialized successfully");
//        } catch (Exception e) {
//            telemetry.addData("TurretOdoAi Init Error", e.getMessage());
//            telemetry.addData("Error Type", e.getClass().getSimpleName());
//        }
//
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        farscore = () -> PedroComponent.follower().pathBuilder()
//                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose,
//                        new Pose(83.17241379310344, 12.620689655172416))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
//                        PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
//                .build();
//
//        closescore = () -> PedroComponent.follower().pathBuilder()
//                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose,
//                        new Pose(72, 72))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
//                        PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
//                .build();
//
//        Parkpath = () -> PedroComponent.follower().pathBuilder()
//                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose,
//                        new Pose(38.74532374100719, 33.358273381294964, 0))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
//                        PedroComponent.follower()::getHeading, Math.toRadians(0), 0.8))
//                .build();
//
//        if (lastPose != null) {
//            PedroComponent.follower().setPose(lastPose);
//        } else {
//            PedroComponent.follower().setPose(new Pose(0, 0, 0));
//        }
//
//        PedroComponent.follower().startTeleopDrive();
//
//        Command driverControlled = new MecanumDriverControlled(
//                frontLeftMotor,
//                frontRightMotor,
//                backLeftMotor,
//                backRightMotor,
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX()
//        );
//
//        // ========== BUTTON BINDINGS ==========
//
//        Gamepads.gamepad1().b()
//                .whenBecomesTrue(() -> {
//                    turretAutoAlignEnabled = !turretAutoAlignEnabled;
//                    if (!turretAutoAlignEnabled) {
//                        telemetryM.addData("Turret", "Auto-Align Disabled");
//                    } else {
//                        telemetryM.addData("Turret", "Auto-Align Enabled");
//                    }
//                });
//
//        Gamepads.gamepad1().dpadLeft()
//                .whenTrue(() -> {
//                    turretAutoAlignEnabled = false;
//                })
//                .whenBecomesFalse(() -> {
//                    turretAutoAlignEnabled = true;
//                });
//
//        Gamepads.gamepad1().dpadRight()
//                .whenTrue(() -> {
//                    turretAutoAlignEnabled = false;
//                })
//                .whenBecomesFalse(() -> {
//                    turretAutoAlignEnabled = true;
//                });
//
//        Gamepads.gamepad1().a()
//                .whenBecomesTrue(() -> {
//                    turretAutoAlignEnabled = false;
//                    CompliantIntake.INSTANCE.repel();
//                    telemetryM.addData("Turret", "Stopped & Intake Reset");
//                });
//
//        Gamepads.gamepad1().rightBumper()
//                .whenBecomesTrue(() -> {
//                    intakeToggle = !intakeToggle;
//                    if (intakeToggle) {
//                        intakeStartTime = System.currentTimeMillis();
//                        CompliantIntake.INSTANCE.on();
//                        Transfer.INSTANCE.slight();
//                        colorSensor.artifactcounter = 0;
//                        telemetryM.addData("Intake", "ON - Counting Balls");
//                    } else {
//                        intakeStartTime = 0;
//                        CompliantIntake.INSTANCE.off();
//                        Transfer.INSTANCE.off();
//                        colorSensor.artifactcounter = 0;
//                        telemetryM.addData("Intake", "OFF - Manual Stop");
//                    }
//                });
//
//        button(() -> gamepad1.right_trigger > 0.05)
//                .whenTrue(() -> {
//                    double d = TurretOdoAi.INSTANCE.getDistanceToTarget();
//                    TurretPID.INSTANCE.newshooterdistance(d).schedule();
//                    TurretPID.shootRequested = true;
//                    TurretPID.hasShot = false;
//                });
//
//        Gamepads.gamepad1().dpadDown()
//                .whenBecomesTrue(() -> {
//                    PedroComponent.follower().setPose(new Pose(0, 0, 0));
//                });
//
//        Gamepads.gamepad1().leftTrigger()
//                .greaterThan(0.05)
//                .whenBecomesTrue(() -> {
//                    Transfer.INSTANCE.on();
//                    CompliantIntake.INSTANCE.on();
//                });
//
//        Gamepads.gamepad1().leftBumper()
//                .whenBecomesTrue(() -> {
//                    TurretPID.INSTANCE.resetShooter().schedule();
//                    Hood.INSTANCE.midopen();
//                    CompliantIntake.INSTANCE.off();
//                    Transfer.INSTANCE.off();
//                });
//
//        Gamepads.gamepad1().y()
//                .whenBecomesTrue(() -> {
//                    PedroComponent.follower().followPath(Parkpath.get());
//                    automatedDrive = true;
//                });
//    }
//
//    @Override
//    public void onUpdate() {
//        try {
//            // Drive mode toggle
//            if (gamepad1.dpad_up && !togglePressed) {
//                robotCentric = !robotCentric;
//                togglePressed = true;
//            } else if (!gamepad1.dpad_up) {
//                togglePressed = false;
//            }
//
//            // Let subsystem handle odometry update with error handling
//            try {
//                TurretOdoAi.INSTANCE.periodic();
//            } catch (Exception e) {
//                telemetry.addData("TurretOdoAi Error", e.getMessage());
//            }
//
//            // Get pose ONCE and cache it
//            Pose pose = PedroComponent.follower().getPose();
//
//            if (pose == null) {
//                telemetry.addData("ERROR", "Pose is null - follower not initialized");
//                telemetry.update();
//                return;
//            }
//
//            // Cache all values from subsystem
//            double x = TurretOdoAi.INSTANCE.getX();
//            double y = TurretOdoAi.INSTANCE.getY();
//            double heading = TurretOdoAi.INSTANCE.getHeading();
//            double targetTurretAngle = TurretOdoAi.INSTANCE.getTargetAngleDeg();
//            double distanceToTarget = TurretOdoAi.INSTANCE.getDistanceToTarget();
//            double turretAngle1 = TurretOdoAi.INSTANCE.getTurretAngleDeg();
//            double Lasterror = TurretOdoAi.INSTANCE.getLastError();
//
//            // ========== ALL TELEMETRY DATA ==========
//            telemetry.addData("Drive Mode", robotCentric ? "Robot Centric" : "Field Centric");
//            telemetry.addData("X", String.format("%.1f", x));
//            telemetry.addData("Y", String.format("%.1f", y));
//            telemetry.addData("Heading (deg)", String.format("%.1f", heading));
//            telemetry.addData("Target Turret Angle (deg)", String.format("%.1f", targetTurretAngle));
//            telemetry.addData("Target", "(" + TurretOdoAi.xt + ", " + TurretOdoAi.yt + ")");
//            telemetry.addData("Distance", String.format("%.1f", distanceToTarget));
//            telemetry.addData("Turret Angle", String.format("%.1f", turretAngle1));
//            telemetry.addData("Error", String.format("%.1f°", Lasterror));
//            telemetry.addData("Auto-Align", turretAutoAlignEnabled ? "Enabled" : "Disabled");
//            telemetry.update();
//
//            // ========== DRIVE CONTROL ==========
//            if (!automatedDrive) {
//                PedroComponent.follower().setTeleOpDrive(
//                        -gamepad1.left_stick_y * slowModeMultiplier,
//                        -gamepad1.left_stick_x * slowModeMultiplier,
//                        -gamepad1.right_stick_x * slowModeMultiplier,
//                        true
//                );
//            }
//
//            // CRITICAL: Only ONE follower update per loop
//            PedroComponent.follower().update();
//
//            // Cache velocity values
//            double targetVel = TurretPID.activeTargetVelocity;
//            double actualVel = TurretPID.turret.getVelocity();
//
//            // ========== HOOD CONTROL ==========
//            if (distanceToTarget <= CLOSE_HOOD_DISTANCE) {
//                Hood.INSTANCE.close();
//            } else {
//                if (targetVel > 500) {
//                    Hood.INSTANCE.compensateFromVelocity(targetVel, actualVel);
//                }
//            }
//
//            // ========== TURRET AUTO ALIGNMENT ==========
//            //        if (turretAutoAlignEnabled) {
//            //            // Use odometry-based turret alignment
//            //            TurretOdoAi.INSTANCE.alignTurret();
//            //        }
//
//            // ========== COLOR SENSOR BALL COUNTING ==========
//            if (intakeToggle) {
//                colorSensor.IncountBalls();
//
//                int ballCount = colorSensor.artifactcounter;
//
//                if (ballCount > 0) {
//                    // RGBled.INSTANCE.open();
//                }
//                if (ballCount >= 2) {
//                    Transfer.INSTANCE.advance();
//                    // RGBled.INSTANCE.midopen();
//                }
//                if (ballCount >= 3) {
//                    // RGBled.INSTANCE.close();
//                    CompliantIntake.INSTANCE.off();
//                    Transfer.INSTANCE.off();
//                    intakeToggle = false;
//                    intakeStartTime = 0;
//                    gamepad1.rumble(500);
//                    colorSensor.artifactcounter = 0;
//                }
//            }
//        } catch (Exception e) {
//            telemetry.addData("CRITICAL ERROR", e.getMessage());
//            telemetry.addData("Error Type", e.getClass().getSimpleName());
//            telemetry.addData("Stack", e.getStackTrace()[0].toString());
//            telemetry.update();
//        }
//    }
//}