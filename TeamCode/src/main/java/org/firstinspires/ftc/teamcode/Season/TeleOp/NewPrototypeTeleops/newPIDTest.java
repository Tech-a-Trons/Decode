package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID.turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.OdoTurretSubsystem;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class newPIDTest extends NextFTCOpMode {
    public newPIDTest() {
        addComponents(
                new SubsystemComponent(
                        TurretPID.INSTANCE,
                        Hood.INSTANCE,
                        Turret.INSTANCE,
                        OdoTurretSubsystem.INSTANCE  // This enables periodic() to be called
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");
    private boolean turretEnabled = false;



    @Override
    public void onStartButtonPressed() {
        // Initialize limelight


// in onStartButtonPressed()
        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> OdoTurretSubsystem.INSTANCE.toggleAutoTrack());

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

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> Turret.INSTANCE.open());

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setCloseShooterSpeed().schedule());

        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> TurretPID.INSTANCE.setFarShooterSpeed().schedule());

        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.resetShooter().schedule();
                    Hood.INSTANCE.close();
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


        driverControlled.schedule();
    }
}