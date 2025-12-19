package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Midtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "TurretTest")

public class TurretTest extends NextFTCOpMode {
    public TurretTest() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
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

        Gamepads.gamepad1().dpadUp()
                .whenTrue(() -> Turret.INSTANCE.open())   // run intake while held
                .whenFalse(() -> Turret.INSTANCE.resetToStart()); // stop intake when released

        // Inside your periodic loop or command scheduler
        Gamepads.gamepad1().dpadDown()
                .whenTrue(() -> Turret.INSTANCE.close())   // run intake while held
                .whenFalse(() -> Turret.INSTANCE.resetToStart());


//        driverControlled.schedule();
    }
}