package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Disabled
@TeleOp(name = "OuttakePID")

public class OuttakePID extends NextFTCOpMode {
    public OuttakePID() {
        addComponents(
                new SubsystemComponent( TurretPID.INSTANCE, Hood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
//    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
//    private final MotorEx frontRightMotor = new MotorEx("fr");
//    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
//    private final MotorEx backRightMotor = new MotorEx("br");
    private boolean intakeOn = false;

    @Override
    public void onStartButtonPressed() {
//        Command driverControlled = new MecanumDriverControlled(
//                frontLeftMotor,
//                frontRightMotor,
//                backLeftMotor,
//                backRightMotor,
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX()
//        );

// Toggle all off when left bumper is pressed

//        Gamepads.gamepad1().dpadUp()
//                .whenTrue(() -> Turret.INSTANCE.open())
//                .whenTrue(() -> Hood.INSTANCE.open())
//                // run intake while held
//                .whenFalse(() -> Turret.INSTANCE.resetToStart())
//        .whenFalse(() -> Hood.INSTANCE.close());// stop intake when released
//
//        // Inside your periodic loop or command scheduler
//        Gamepads.gamepad1().dpadDown()
//                .whenTrue(() -> Turret.INSTANCE.close())   // run intake while held
//                .whenFalse(() -> Turret.INSTANCE.resetToStart());

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
//                    Turret.INSTANCE.resetToStart();
//                    ;// stop outtake
                    Hood.INSTANCE.close();    // stop ramp/midtake
                    TurretPID.INSTANCE.resetShooter();     // stop intake
                });
        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
//                    Turret.INSTANCE.resetToStart();
                    ;// stop outtake
                    Hood.INSTANCE.close();    // stop ramp/midtake
                    TurretPID.INSTANCE.setCloseShooterSpeed();     // stop intake
                });
        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
//                    Turret.INSTANCE.resetToStart();
                    ;// stop outtake
                    Hood.INSTANCE.close();    // stop ramp/midtake
                    TurretPID.INSTANCE.setFarShooterSpeed();     // stop intake
                });
//        driverControlled.schedule();
    }
}