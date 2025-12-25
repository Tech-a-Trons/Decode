package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.OdoTurretSubsystem;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;

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
public class newPIDTest extends NextFTCOpMode {
    public newPIDTest() {
        addComponents(
                new SubsystemComponent(
                        TurretPID.INSTANCE,
                        Hood.INSTANCE,
                        Turret.INSTANCE,
                        CompliantIntake.INSTANCE
//                        OdoTurretSubsystem.INSTANCE  // This enables periodic() to be called
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
    private boolean turretEnabled = false;


    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;


    @Override
    public void onStartButtonPressed() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> PedroComponent.follower().pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, new Pose(83.17241379310344, 12.620689655172416))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, Math.toRadians(90), 0.8))
                .build();
//        OdoTurretSubsystem.INSTANCE.init(); // lazy init after hardware map is ready
//        OdoTurretSubsystem.INSTANCE.toggleAutoTrack();

        // Initialize limelight
//
        PedroComponent.follower().setPose(new Pose(72,35,90));
// in onStartButtonPressed()
        PedroComponent.follower().startTeleopDrive();

        // Setup driving
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        // ========== TURRET ALIGNMENT CONTROLS ==========

        // Right Trigger: Hold to enable auto-alignment

        // ========== EXISTING CONTROLS ==========

//        Gamepads.gamepad1().dpadRight()
//                .whenBecomesTrue(() -> Turret.INSTANCE.close());
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> CompliantIntake.INSTANCE.on());

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setCloseShooterSpeed().schedule());

        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setFarShooterSpeed().schedule());

        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.resetShooter().schedule();
                    Hood.INSTANCE.close();
                    CompliantIntake.INSTANCE.off();
                });

        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {

                });

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> Hood.INSTANCE.open());

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> Hood.INSTANCE.midopen());

        // ========== TELEMETRY ==========

        // Add a repeating command to update telemetry

//
//        driverControlled.schedule();
    }
    @Override public void onUpdate() {
        PedroComponent.follower().update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) PedroComponent.follower().setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else PedroComponent.follower().setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            PedroComponent.follower().followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !PedroComponent.follower().isBusy())) {
            PedroComponent.follower().startTeleopDrive();
            automatedDrive = false;
        }
        //Slow Mode
        telemetryM.debug("position", PedroComponent.follower().getPose());
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

    }
}