package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Midtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Outtake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "TeleopNext")
public class TeleopNext extends NextFTCOpMode {
    public TeleopNext() {
        addComponents(
                new SubsystemComponent(Outtake.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");

    @Override
    public void onStartButtonPressed() {
        button(() -> gamepad1.left_bumper)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(new InstantCommand(() -> {
                    Outtake.INSTANCE.setShooterSpeed();
                    Midtake.INSTANCE.midtake.setPower(-1);
                }))
                .whenBecomesFalse(new InstantCommand(() -> {
                    Outtake.INSTANCE.outtake.setPower(0);
                    Midtake.INSTANCE.midtake.setPower(0);
                }));
        button(() -> gamepad1.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(new InstantCommand(() -> Intake.INSTANCE.activeintake.setPower(1)))
                .whenBecomesFalse(new InstantCommand(() -> Intake.INSTANCE.activeintake.setPower(0)));
        button(() -> gamepad1.x)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(new InstantCommand(() -> Intake.INSTANCE.activeintake.setPower(0)))
                .whenBecomesTrue(new InstantCommand(() -> Outtake.INSTANCE.outtake.setPower(0)))
                .whenBecomesTrue(new InstantCommand(() -> Midtake.INSTANCE.midtake.setPower(0)));
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();



    }
    @Override public void onUpdate() {
        telemetry.addData("velocity", Outtake.outtake.getVelocity());
        telemetry.addData("power", Outtake.outtake.getPower());
        telemetry.update();
    }
}