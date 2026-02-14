//package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;
//
//import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext.lastPose;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ColorSensor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;
//
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//import dev.nextftc.extensions.pedro.PedroComponent;
//
//@TeleOp(name = "TurretOdoTele_TEST_SUBSYSTEMS")
//public class TurretOdoTeleTest extends NextFTCOpMode {
//
//    // Flags to enable/disable subsystems for testing
//    private static final boolean ENABLE_TURRET_PID = false;
//    private static final boolean ENABLE_HOOD = true;
//    private static final boolean ENABLE_TURRET = false;
//    private static final boolean ENABLE_INTAKE = false;
//    private static final boolean ENABLE_TRANSFER = false;
//    private static final boolean ENABLE_COLOR_SENSOR = false;
//
//    public TurretOdoTeleTest() {
//        try {
//            // Test each subsystem individually by enabling ONE flag at a time
//            // Start with all false, then enable ONE to find which crashes
//
//            if (ENABLE_TURRET_PID && ENABLE_HOOD && ENABLE_TURRET &&
//                    ENABLE_INTAKE && ENABLE_TRANSFER && ENABLE_COLOR_SENSOR) {
//                // All enabled
//                addComponents(
//                        new SubsystemComponent(
//                                TurretPID.INSTANCE,
//                                Hood.INSTANCE,
//                                Turret.INSTANCE,
//                                CompliantIntake.INSTANCE,
//                                Transfer.INSTANCE,
//                                ColorSensor.INSTANCE
//                        ),
//                        BulkReadComponent.INSTANCE,
//                        BindingsComponent.INSTANCE,
//                        new PedroComponent(Constants::createFollower)
//                );
//            } else if (ENABLE_TURRET_PID && !ENABLE_HOOD && !ENABLE_TURRET &&
//                    !ENABLE_INTAKE && !ENABLE_TRANSFER && !ENABLE_COLOR_SENSOR) {
//                // Only TurretPID
//                addComponents(
//                        new SubsystemComponent(TurretPID.INSTANCE),
//                        BulkReadComponent.INSTANCE,
//                        BindingsComponent.INSTANCE,
//                        new PedroComponent(Constants::createFollower)
//                );
//            } else if (!ENABLE_TURRET_PID && ENABLE_HOOD && !ENABLE_TURRET &&
//                    !ENABLE_INTAKE && !ENABLE_TRANSFER && !ENABLE_COLOR_SENSOR) {
//                // Only Hood
//                addComponents(
//                        new SubsystemComponent(Hood.INSTANCE),
//                        BulkReadComponent.INSTANCE,
//                        BindingsComponent.INSTANCE,
//                        new PedroComponent(Constants::createFollower)
//                );
//            } else if (!ENABLE_TURRET_PID && !ENABLE_HOOD && ENABLE_TURRET &&
//                    !ENABLE_INTAKE && !ENABLE_TRANSFER && !ENABLE_COLOR_SENSOR) {
//                // Only Turret
//                addComponents(
//                        new SubsystemComponent(Turret.INSTANCE),
//                        BulkReadComponent.INSTANCE,
//                        BindingsComponent.INSTANCE,
//                        new PedroComponent(Constants::createFollower)
//                );
//            } else if (!ENABLE_TURRET_PID && !ENABLE_HOOD && !ENABLE_TURRET &&
//                    ENABLE_INTAKE && !ENABLE_TRANSFER && !ENABLE_COLOR_SENSOR) {
//                // Only CompliantIntake
//                addComponents(
//                        new SubsystemComponent(CompliantIntake.INSTANCE),
//                        BulkReadComponent.INSTANCE,
//                        BindingsComponent.INSTANCE,
//                        new PedroComponent(Constants::createFollower)
//                );
//            } else if (!ENABLE_TURRET_PID && !ENABLE_HOOD && !ENABLE_TURRET &&
//                    !ENABLE_INTAKE && ENABLE_TRANSFER && !ENABLE_COLOR_SENSOR) {
//                // Only Transfer
//                addComponents(
//                        new SubsystemComponent(Transfer.INSTANCE),
//                        BulkReadComponent.INSTANCE,
//                        BindingsComponent.INSTANCE,
//                        new PedroComponent(Constants::createFollower)
//                );
//            } else if (!ENABLE_TURRET_PID && !ENABLE_HOOD && !ENABLE_TURRET &&
//                    !ENABLE_INTAKE && !ENABLE_TRANSFER && ENABLE_COLOR_SENSOR) {
//                // Only ColorSensor
//                addComponents(
//                        new SubsystemComponent(ColorSensor.INSTANCE),
//                        BulkReadComponent.INSTANCE,
//                        BindingsComponent.INSTANCE,
//                        new PedroComponent(Constants::createFollower)
//                );
//            } else {
//                // No subsystems (baseline test)
//                addComponents(
//                        BulkReadComponent.INSTANCE,
//                        BindingsComponent.INSTANCE,
//                        new PedroComponent(Constants::createFollower)
//                );
//            }
//        } catch (Exception e) {
//            // Constructor error - will show in driver station logs
//        }
//    }
//
//    private double slowModeMultiplier = 0.5;
//
//    @Override
//    public void onStartButtonPressed() {
//        telemetry.addData("Status", "Initializing...");
//        telemetry.addData("TurretPID", ENABLE_TURRET_PID ? "ENABLED" : "disabled");
//        telemetry.addData("Hood", ENABLE_HOOD ? "ENABLED" : "disabled");
//        telemetry.addData("Turret", ENABLE_TURRET ? "ENABLED" : "disabled");
//        telemetry.addData("Intake", ENABLE_INTAKE ? "ENABLED" : "disabled");
//        telemetry.addData("Transfer", ENABLE_TRANSFER ? "ENABLED" : "disabled");
//        telemetry.addData("ColorSensor", ENABLE_COLOR_SENSOR ? "ENABLED" : "disabled");
//        telemetry.update();
//
//        try {
//            if (lastPose != null) {
//                PedroComponent.follower().setPose(lastPose);
//            } else {
//                PedroComponent.follower().setPose(new Pose(0, 0, 0));
//            }
//
//            PedroComponent.follower().startTeleopDrive();
//
//            telemetry.addData("Status", "✓ Initialized successfully");
//            telemetry.update();
//
//        } catch (Exception e) {
//            telemetry.addData("INIT ERROR", e.getMessage());
//            telemetry.addData("Error Type", e.getClass().getSimpleName());
//            telemetry.update();
//        }
//    }
//
//    @Override
//    public void onUpdate() {
//        try {
//            // Get pose
//            Pose pose = PedroComponent.follower().getPose();
//
//            if (pose == null) {
//                telemetry.addData("ERROR", "Pose is null");
//                telemetry.update();
//                return;
//            }
//
//            // Telemetry
//            telemetry.addData("X", String.format("%.1f", pose.getX()));
//            telemetry.addData("Y", String.format("%.1f", pose.getY()));
//            telemetry.addData("Heading", String.format("%.1f", Math.toDegrees(pose.getHeading())));
//            telemetry.addData("Status", "✓ Running OK");
//            telemetry.addData("Subsystems Active",
//                    (ENABLE_TURRET_PID ? "PID " : "") +
//                            (ENABLE_HOOD ? "Hood " : "") +
//                            (ENABLE_TURRET ? "Turret " : "") +
//                            (ENABLE_INTAKE ? "Intake " : "") +
//                            (ENABLE_TRANSFER ? "Transfer " : "") +
//                            (ENABLE_COLOR_SENSOR ? "Color " : "")
//            );
//            telemetry.update();
//
//            // Drive control
//            PedroComponent.follower().setTeleOpDrive(
//                    -gamepad1.left_stick_y * slowModeMultiplier,
//                    -gamepad1.left_stick_x * slowModeMultiplier,
//                    -gamepad1.right_stick_x * slowModeMultiplier,
//                    true
//            );
//
//            // Update follower
//            PedroComponent.follower().update();
//
//        } catch (Exception e) {
//            telemetry.addData("CRITICAL ERROR", e.getMessage());
//            telemetry.addData("Error Type", e.getClass().getSimpleName());
//            if (e.getStackTrace().length > 0) {
//                telemetry.addData("Location", e.getStackTrace()[0].toString());
//            }
//            telemetry.update();
//        }
//    }
//}